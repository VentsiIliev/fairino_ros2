#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include "erob_moveit_runtime/srv/compute_contour_ik.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using ComputeContourIK = erob_moveit_runtime::srv::ComputeContourIK;

namespace
{
constexpr int32_t ERROR_NONE = 1;
constexpr int32_t ERROR_INVALID_REQUEST = -1;
constexpr int32_t ERROR_IK_FAILED = -2;
constexpr int32_t ERROR_FK_ERROR = -3;
constexpr int32_t ERROR_JOINT_STEP = -4;
constexpr int32_t ERROR_JOINT_SPAN = -5;

double nearestEquivalentAngle(double reference, double value)
{
    double adjusted = value;
    constexpr double two_pi = 2.0 * M_PI;
    while (adjusted - reference > M_PI)
    {
        adjusted -= two_pi;
    }
    while (adjusted - reference < -M_PI)
    {
        adjusted += two_pi;
    }
    return adjusted;
}

double durationFromRequest(double value, double fallback)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        return fallback;
    }
    return value;
}

double steadySeconds()
{
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

double positionErrorMm(const geometry_msgs::msg::Pose& expected, const Eigen::Isometry3d& actual)
{
    const Eigen::Vector3d expected_position(
        expected.position.x,
        expected.position.y,
        expected.position.z);
    return (expected_position - actual.translation()).norm() * 1000.0;
}

double orientationErrorDeg(const geometry_msgs::msg::Pose& expected, const Eigen::Isometry3d& actual)
{
    Eigen::Quaterniond expected_q(
        expected.orientation.w,
        expected.orientation.x,
        expected.orientation.y,
        expected.orientation.z);
    Eigen::Quaterniond actual_q(actual.rotation());

    if (expected_q.norm() <= 1e-12 || actual_q.norm() <= 1e-12)
    {
        return 180.0;
    }

    expected_q.normalize();
    actual_q.normalize();
    const double dot = std::clamp(std::abs(expected_q.dot(actual_q)), -1.0, 1.0);
    return std::acos(dot) * 2.0 * 180.0 / M_PI;
}

double maxAbsDelta(const std::vector<double>& a, const std::vector<double>& b)
{
    double max_delta = 0.0;
    const std::size_t count = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        max_delta = std::max(max_delta, std::abs(a[i] - b[i]));
    }
    return max_delta;
}

double maxAbsCurvature(
    const std::vector<double>& before,
    const std::vector<double>& middle,
    const std::vector<double>& after)
{
    double max_curvature = 0.0;
    const std::size_t count = std::min({before.size(), middle.size(), after.size()});
    for (std::size_t i = 0; i < count; ++i)
    {
        max_curvature = std::max(max_curvature, std::abs(after[i] - 2.0 * middle[i] + before[i]));
    }
    return max_curvature;
}

double trajectoryMaxCurvature(const std::vector<std::vector<double>>& points)
{
    double max_curvature = 0.0;
    if (points.size() < 3)
    {
        return max_curvature;
    }
    for (std::size_t i = 1; i + 1 < points.size(); ++i)
    {
        max_curvature = std::max(max_curvature, maxAbsCurvature(points[i - 1], points[i], points[i + 1]));
    }
    return max_curvature;
}
}

class ContourIKHelperNode : public rclcpp::Node
{
public:
    ContourIKHelperNode() : Node("contour_ik_helper")
    {
        RCLCPP_INFO(this->get_logger(), "Contour IK helper starting...");
        service_ = this->create_service<ComputeContourIK>(
            "/compute_contour_ik",
            std::bind(
                &ContourIKHelperNode::computeContourIK,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service '/compute_contour_ik' created");
    }

    void initialize()
    {
        auto node_ptr = shared_from_this();
        loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_ptr);
        model_ = loader_->getModel();
        if (!model_)
        {
            throw std::runtime_error("Robot model load failed");
        }
        RCLCPP_INFO(this->get_logger(), "Robot model cached for contour IK");
    }

private:
    struct TimingStats
    {
        double ik_s = 0.0;
        double candidate_eval_s = 0.0;
        double per_point_fk_s = 0.0;
        double smoothing_candidate_s = 0.0;
        double smoothing_fk_s = 0.0;
        double final_validate_s = 0.0;
        double response_pack_s = 0.0;
        std::size_t candidate_attempts = 0;
        std::size_t candidate_successes = 0;
        std::size_t smoothing_candidates = 0;
        std::size_t smoothing_fk_checks = 0;
        std::size_t fast_points = 0;
        std::size_t full_score_points = 0;
        std::size_t rollback_replays = 0;
    };

    rclcpp::Service<ComputeContourIK>::SharedPtr service_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
    moveit::core::RobotModelPtr model_;

    void fail(
        const std::shared_ptr<ComputeContourIK::Response>& response,
        int32_t error_code,
        const std::string& message,
        uint32_t failed_index = 0)
    {
        response->success = false;
        response->error_code = error_code;
        response->message = message;
        response->failed_index = failed_index;
        RCLCPP_WARN(this->get_logger(), "Contour IK rejected: %s", message.c_str());
    }

    bool fkWithinTolerance(
        moveit::core::RobotState& state,
        const std::vector<std::string>& joint_names,
        const std::vector<double>& positions,
        const std::string& link_name,
        const geometry_msgs::msg::Pose& target_pose,
        double position_tolerance_mm,
        double orientation_tolerance_deg,
        double* position_error_mm = nullptr,
        double* orientation_error_deg = nullptr)
    {
        state.setVariablePositions(joint_names, positions);
        state.update();
        const auto& actual_tf = state.getGlobalLinkTransform(link_name);
        const double pos_error = positionErrorMm(target_pose, actual_tf);
        const double ori_error = orientationErrorDeg(target_pose, actual_tf);
        if (position_error_mm)
        {
            *position_error_mm = pos_error;
        }
        if (orientation_error_deg)
        {
            *orientation_error_deg = ori_error;
        }
        return pos_error <= position_tolerance_mm && ori_error <= orientation_tolerance_deg;
    }

    std::size_t smoothSolvedPoints(
        const std::shared_ptr<ComputeContourIK::Request>& request,
        const std::vector<std::string>& joint_names,
        const std::string& link_name,
        double max_step_rad,
        moveit::core::RobotState& state,
        std::vector<std::vector<double>>& solved_points,
        TimingStats* timing_stats = nullptr)
    {
        if (!request->smoothing_enabled || solved_points.size() < 3)
        {
            return 0;
        }

        const std::size_t iterations = std::min<std::size_t>(
            static_cast<std::size_t>(request->smoothing_iterations),
            8);
        if (iterations == 0)
        {
            return 0;
        }

        const double alpha = std::clamp(request->smoothing_alpha, 0.0, 1.0);
        if (alpha <= 1e-9)
        {
            return 0;
        }

        const double smoothing_pos_tol_mm = std::min(
            durationFromRequest(request->smoothing_fk_position_tolerance_mm, 0.05),
            durationFromRequest(request->fk_position_tolerance_mm, 0.15));
        const double smoothing_ori_tol_deg = std::min(
            durationFromRequest(request->smoothing_fk_orientation_tolerance_deg, 0.10),
            durationFromRequest(request->fk_orientation_tolerance_deg, 0.25));

        std::size_t accepted = 0;
        double max_joint_adjustment = 0.0;
        double max_position_error_mm = 0.0;
        double max_orientation_error_deg = 0.0;

        for (std::size_t iter = 0; iter < iterations; ++iter)
        {
            std::size_t accepted_this_iter = 0;
            for (std::size_t i = 1; i + 1 < solved_points.size(); ++i)
            {
                const double smoothing_candidate_started_s = steadySeconds();
                std::vector<double> candidate = solved_points[i];
                double candidate_adjustment = 0.0;
                bool step_ok = true;
                for (std::size_t j = 0; j < candidate.size(); ++j)
                {
                    const double midpoint = 0.5 * (solved_points[i - 1][j] + solved_points[i + 1][j]);
                    const double original = candidate[j];
                    candidate[j] = original + alpha * (midpoint - original);
                    candidate_adjustment = std::max(candidate_adjustment, std::abs(candidate[j] - original));
                    if (std::abs(candidate[j] - solved_points[i - 1][j]) > max_step_rad ||
                        std::abs(solved_points[i + 1][j] - candidate[j]) > max_step_rad)
                    {
                        step_ok = false;
                        break;
                    }
                }
                if (timing_stats)
                {
                    timing_stats->smoothing_candidate_s += steadySeconds() - smoothing_candidate_started_s;
                    timing_stats->smoothing_candidates += 1;
                }
                if (!step_ok || candidate_adjustment <= 1e-9)
                {
                    continue;
                }

                double pos_error_mm = 0.0;
                double ori_error_deg = 0.0;
                const double smoothing_fk_started_s = steadySeconds();
                if (!fkWithinTolerance(
                        state,
                        joint_names,
                        candidate,
                        link_name,
                        request->poses[i],
                        smoothing_pos_tol_mm,
                        smoothing_ori_tol_deg,
                        &pos_error_mm,
                        &ori_error_deg))
                {
                    if (timing_stats)
                    {
                        timing_stats->smoothing_fk_s += steadySeconds() - smoothing_fk_started_s;
                        timing_stats->smoothing_fk_checks += 1;
                    }
                    continue;
                }
                if (timing_stats)
                {
                    timing_stats->smoothing_fk_s += steadySeconds() - smoothing_fk_started_s;
                    timing_stats->smoothing_fk_checks += 1;
                }

                solved_points[i] = candidate;
                accepted += 1;
                accepted_this_iter += 1;
                max_joint_adjustment = std::max(max_joint_adjustment, candidate_adjustment);
                max_position_error_mm = std::max(max_position_error_mm, pos_error_mm);
                max_orientation_error_deg = std::max(max_orientation_error_deg, ori_error_deg);
            }
            if (accepted_this_iter == 0)
            {
                break;
            }
        }

        if (accepted > 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Contour IK smoothing accepted %zu point updates: max_adjust=%.5frad fk_max=%.4fmm/%.4fdeg",
                accepted,
                max_joint_adjustment,
                max_position_error_mm,
                max_orientation_error_deg);
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Contour IK smoothing made no changes within FK tolerance %.4fmm/%.4fdeg",
                smoothing_pos_tol_mm,
                smoothing_ori_tol_deg);
        }
        return accepted;
    }

    bool validateFinalTrajectory(
        const std::shared_ptr<ComputeContourIK::Request>& request,
        const std::shared_ptr<ComputeContourIK::Response>& response,
        const std::vector<std::string>& joint_names,
        const std::string& link_name,
        const std::unordered_set<std::string>& full_turn_joints,
        double fk_pos_tol_mm,
        double fk_ori_tol_deg,
        double max_step_rad,
        double max_span_rad,
        double max_endpoint_delta_rad,
        double full_turn_span_rad,
        double full_turn_endpoint_delta_rad,
        moveit::core::RobotState& state,
        const std::vector<std::vector<double>>& solved_points)
    {
        response->max_fk_position_error_mm = 0.0;
        response->max_fk_orientation_error_deg = 0.0;
        response->max_joint_step_rad = 0.0;
        response->max_joint_span_rad = 0.0;
        response->max_endpoint_delta_rad = 0.0;

        for (std::size_t i = 0; i < solved_points.size(); ++i)
        {
            double pos_error_mm = 0.0;
            double ori_error_deg = 0.0;
            if (!fkWithinTolerance(
                    state,
                    joint_names,
                    solved_points[i],
                    link_name,
                    request->poses[i],
                    fk_pos_tol_mm,
                    fk_ori_tol_deg,
                    &pos_error_mm,
                    &ori_error_deg))
            {
                std::ostringstream msg;
                msg << "FK validation failed after smoothing at index " << i
                    << " pos_error_mm=" << pos_error_mm
                    << " ori_error_deg=" << ori_error_deg;
                fail(response, ERROR_FK_ERROR, msg.str(), static_cast<uint32_t>(i));
                return false;
            }
            response->max_fk_position_error_mm =
                std::max(response->max_fk_position_error_mm, pos_error_mm);
            response->max_fk_orientation_error_deg =
                std::max(response->max_fk_orientation_error_deg, ori_error_deg);

            if (i > 0)
            {
                double max_step = 0.0;
                for (std::size_t j = 0; j < solved_points[i].size(); ++j)
                {
                    max_step = std::max(
                        max_step,
                        std::abs(solved_points[i][j] - solved_points[i - 1][j]));
                }
                response->max_joint_step_rad =
                    std::max(response->max_joint_step_rad, max_step);
                if (max_step > max_step_rad)
                {
                    std::ostringstream msg;
                    msg << "joint step exceeded after smoothing at index " << i
                        << " max_step_rad=" << max_step
                        << " limit_rad=" << max_step_rad;
                    fail(response, ERROR_JOINT_STEP, msg.str(), static_cast<uint32_t>(i));
                    return false;
                }
            }
        }

        if (!solved_points.empty())
        {
            for (std::size_t joint_index = 0; joint_index < joint_names.size(); ++joint_index)
            {
                double min_value = solved_points.front()[joint_index];
                double max_value = solved_points.front()[joint_index];
                for (const auto& positions : solved_points)
                {
                    min_value = std::min(min_value, positions[joint_index]);
                    max_value = std::max(max_value, positions[joint_index]);
                }
                const double span = max_value - min_value;
                const double endpoint_delta =
                    std::abs(solved_points.back()[joint_index] - solved_points.front()[joint_index]);
                const bool full_turn_allowed =
                    full_turn_joints.find(joint_names[joint_index]) != full_turn_joints.end();
                const double joint_span_limit = full_turn_allowed ? full_turn_span_rad : max_span_rad;
                const double joint_endpoint_limit = full_turn_allowed
                    ? full_turn_endpoint_delta_rad
                    : max_endpoint_delta_rad;
                response->max_joint_span_rad = std::max(response->max_joint_span_rad, span);
                response->max_endpoint_delta_rad =
                    std::max(response->max_endpoint_delta_rad, endpoint_delta);
                if (span > joint_span_limit)
                {
                    std::ostringstream msg;
                    msg << "joint span exceeded for " << joint_names[joint_index]
                        << " span_rad=" << span
                        << " limit_rad=" << joint_span_limit;
                    fail(response, ERROR_JOINT_SPAN, msg.str());
                    return false;
                }
                if (endpoint_delta > joint_endpoint_limit)
                {
                    std::ostringstream msg;
                    msg << "endpoint delta exceeded for " << joint_names[joint_index]
                        << " endpoint_delta_rad=" << endpoint_delta
                        << " limit_rad=" << joint_endpoint_limit;
                    fail(response, ERROR_JOINT_SPAN, msg.str());
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<double> buildPredictiveSeed(
        const std::vector<double>& previous_previous_positions,
        const std::vector<double>& previous_positions,
        double max_step_rad) const
    {
        std::vector<double> seed = previous_positions;
        for (std::size_t j = 0; j < seed.size(); ++j)
        {
            const double velocity = previous_positions[j] - previous_previous_positions[j];
            const double clamped_velocity = std::clamp(velocity, -max_step_rad, max_step_rad);
            seed[j] = previous_positions[j] + clamped_velocity;
        }
        return seed;
    }

    bool solveLocalIKCandidate(
        moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* joint_model_group,
        const geometry_msgs::msg::Pose& pose,
        const std::string& link_name,
        const std::vector<std::string>& joint_names,
        const std::vector<double>& previous_positions,
        const std::vector<double>& seed_positions,
        double timeout_s,
        double max_step_rad,
        std::vector<double>& candidate_positions,
        TimingStats* timing_stats = nullptr)
    {
        state.setVariablePositions(joint_names, seed_positions);
        state.update();

        moveit::core::GroupStateValidityCallbackFn local_continuity_cb =
            [&](moveit::core::RobotState* candidate_state,
                const moveit::core::JointModelGroup*,
                const double*) -> bool
            {
                double max_delta = 0.0;

                for (std::size_t j = 0; j < joint_names.size(); ++j)
                {
                    double value = candidate_state->getVariablePosition(joint_names[j]);
                    value = nearestEquivalentAngle(previous_positions[j], value);
                    max_delta = std::max(max_delta, std::abs(value - previous_positions[j]));
                }

                // Reject this IK candidate and let solver keep searching.
                return max_delta <= max_step_rad;
            };

        if (timing_stats)
        {
            timing_stats->candidate_attempts += 1;
        }
        const double ik_started_s = steadySeconds();
        const bool ok = state.setFromIK(
            joint_model_group,
            pose,
            link_name,
            timeout_s,
            local_continuity_cb);
        if (timing_stats)
        {
            timing_stats->ik_s += steadySeconds() - ik_started_s;
        }
        if (!ok)
        {
            return false;
        }

        candidate_positions.clear();
        candidate_positions.reserve(joint_names.size());
        for (std::size_t j = 0; j < joint_names.size(); ++j)
        {
            const double value = nearestEquivalentAngle(
                previous_positions[j],
                state.getVariablePosition(joint_names[j]));
            if (std::abs(value - previous_positions[j]) > max_step_rad)
            {
                return false;
            }
            candidate_positions.push_back(value);
        }
        if (timing_stats)
        {
            timing_stats->candidate_successes += 1;
        }
        return true;
    }

    bool solveLocalIK(
        moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* joint_model_group,
        const geometry_msgs::msg::Pose& pose,
        const std::string& link_name,
        const std::vector<std::string>& joint_names,
        const std::vector<double>& previous_positions,
        const std::vector<double>* previous_previous_positions,
        double timeout_s,
        double max_step_rad,
        TimingStats* timing_stats = nullptr,
        bool score_all_candidates = true)
    {
        std::vector<std::vector<double>> seed_candidates;
        seed_candidates.reserve(4);

        if (previous_previous_positions &&
            previous_previous_positions->size() == previous_positions.size())
        {
            std::vector<double> predictive_seed =
                buildPredictiveSeed(*previous_previous_positions, previous_positions, max_step_rad);
            seed_candidates.push_back(predictive_seed);

            std::vector<double> half_predictive_seed = previous_positions;
            std::vector<double> extended_predictive_seed = previous_positions;
            for (std::size_t j = 0; j < previous_positions.size(); ++j)
            {
                const double velocity = previous_positions[j] - (*previous_previous_positions)[j];
                half_predictive_seed[j] = previous_positions[j] + 0.5 * velocity;
                extended_predictive_seed[j] =
                    previous_positions[j] + std::clamp(1.5 * velocity, -max_step_rad, max_step_rad);
            }
            seed_candidates.push_back(half_predictive_seed);
            seed_candidates.push_back(extended_predictive_seed);
        }
        seed_candidates.push_back(previous_positions);

        bool found = false;
        std::vector<double> best_positions;
        double best_score = std::numeric_limits<double>::infinity();

        for (const auto& seed_positions : seed_candidates)
        {
            std::vector<double> candidate_positions;
            const double candidate_eval_started_s = steadySeconds();
            if (!solveLocalIKCandidate(
                    state,
                    joint_model_group,
                    pose,
                    link_name,
                    joint_names,
                    previous_positions,
                    seed_positions,
                    timeout_s,
                    max_step_rad,
                    candidate_positions,
                    timing_stats))
            {
                if (timing_stats)
                {
                    timing_stats->candidate_eval_s += steadySeconds() - candidate_eval_started_s;
                }
                continue;
            }

            const double step = maxAbsDelta(candidate_positions, previous_positions);
            double curvature = 0.0;
            if (previous_previous_positions &&
                previous_previous_positions->size() == previous_positions.size())
            {
                curvature = maxAbsCurvature(
                    *previous_previous_positions,
                    previous_positions,
                    candidate_positions);
            }

            const double seed_error = maxAbsDelta(candidate_positions, seed_positions);
            const double score = 10.0 * curvature + 0.25 * step + 0.05 * seed_error;
            if (!score_all_candidates)
            {
                if (timing_stats)
                {
                    timing_stats->candidate_eval_s += steadySeconds() - candidate_eval_started_s;
                }
                state.setVariablePositions(joint_names, candidate_positions);
                state.update();
                return true;
            }
            if (score < best_score)
            {
                found = true;
                best_score = score;
                best_positions = candidate_positions;
            }
            if (timing_stats)
            {
                timing_stats->candidate_eval_s += steadySeconds() - candidate_eval_started_s;
            }
        }

        if (!found)
        {
            return false;
        }

        state.setVariablePositions(joint_names, best_positions);
        state.update();
        return true;
    }

    void computeContourIK(
        const std::shared_ptr<ComputeContourIK::Request> request,
        std::shared_ptr<ComputeContourIK::Response> response)
    {
        const auto started_at = this->now();
        response->success = false;
        response->error_code = ERROR_INVALID_REQUEST;
        response->failed_index = std::numeric_limits<uint32_t>::max();

        if (!model_)
        {
            fail(response, ERROR_INVALID_REQUEST, "robot model is not loaded");
            return;
        }
        if (request->poses.empty())
        {
            fail(response, ERROR_INVALID_REQUEST, "request contains no poses");
            return;
        }
        if (request->seed_state.name.empty() ||
            request->seed_state.name.size() != request->seed_state.position.size())
        {
            fail(response, ERROR_INVALID_REQUEST, "seed_state is incomplete");
            return;
        }

        const std::string group_name = request->group_name.empty()
            ? model_->getJointModelGroupNames().front()
            : request->group_name;
        const auto* joint_model_group = model_->getJointModelGroup(group_name);
        if (!joint_model_group)
        {
            fail(response, ERROR_INVALID_REQUEST, "unknown planning group: " + group_name);
            return;
        }

        const std::string link_name = request->link_name.empty()
            ? joint_model_group->getLinkModelNames().back()
            : request->link_name;
        if (!model_->hasLinkModel(link_name))
        {
            fail(response, ERROR_INVALID_REQUEST, "unknown IK/FK link: " + link_name);
            return;
        }

        const double timeout_s = durationFromRequest(request->timeout_s, 0.003);
        const double retry_timeout_s = durationFromRequest(request->retry_timeout_s, 0.02);
        const double fk_pos_tol_mm = durationFromRequest(request->fk_position_tolerance_mm, 0.15);
        const double fk_ori_tol_deg = durationFromRequest(request->fk_orientation_tolerance_deg, 0.25);
        const double max_step_rad = durationFromRequest(request->max_joint_step_rad, 0.025);
        const double max_span_rad = durationFromRequest(request->max_joint_span_rad, M_PI);
        const double max_endpoint_delta_rad = durationFromRequest(request->max_endpoint_delta_rad, M_PI);
        const double full_turn_span_rad =
            durationFromRequest(request->full_turn_max_joint_span_rad, 2.0 * M_PI + 0.25);
        const double full_turn_endpoint_delta_rad =
            durationFromRequest(request->full_turn_max_endpoint_delta_rad, 2.0 * M_PI + 0.25);
        const std::unordered_set<std::string> full_turn_joints(
            request->full_turn_joint_names.begin(),
            request->full_turn_joint_names.end());

        moveit::core::RobotState state(model_);
        state.setToDefaultValues();
        state.setVariablePositions(request->seed_state.name, request->seed_state.position);
        state.update();

        const std::vector<std::string> output_joint_names = request->seed_state.name;
        std::vector<double> previous_positions = request->seed_state.position;
        std::vector<double> previous_previous_positions;
        std::vector<std::vector<double>> solved_points;
        solved_points.reserve(request->poses.size());

        response->trajectory.joint_trajectory.joint_names = output_joint_names;
        response->fast_failures = 0;
        response->retries = 0;
        TimingStats timing_stats;

        auto solve_pose = [&](
            std::size_t pose_index,
            bool score_all_candidates,
            double solve_timeout_s,
            bool count_retry) -> bool
        {
            const std::vector<double>* previous_previous =
                previous_previous_positions.empty() ? nullptr : &previous_previous_positions;
            bool ok = solveLocalIK(
                state,
                joint_model_group,
                request->poses[pose_index],
                link_name,
                output_joint_names,
                previous_positions,
                previous_previous,
                solve_timeout_s,
                max_step_rad,
                &timing_stats,
                score_all_candidates);

            if (!ok && count_retry && retry_timeout_s > solve_timeout_s)
            {
                response->fast_failures += 1;
                response->retries += 1;
                ok = solveLocalIK(
                    state,
                    joint_model_group,
                    request->poses[pose_index],
                    link_name,
                    output_joint_names,
                    previous_positions,
                    previous_previous,
                    retry_timeout_s,
                    max_step_rad,
                    &timing_stats,
                    score_all_candidates);
            }
            return ok;
        };

        auto accept_current_state = [&](std::size_t pose_index) -> bool
        {
            state.update();
            std::vector<double> positions;
            positions.reserve(output_joint_names.size());
            for (std::size_t j = 0; j < output_joint_names.size(); ++j)
            {
                double value = state.getVariablePosition(output_joint_names[j]);
                value = nearestEquivalentAngle(previous_positions[j], value);
                positions.push_back(value);
            }

            state.setVariablePositions(output_joint_names, positions);
            state.update();

            double pos_error_mm = 0.0;
            double ori_error_deg = 0.0;
            const double per_point_fk_started_s = steadySeconds();
            if (!fkWithinTolerance(
                    state,
                    output_joint_names,
                    positions,
                    link_name,
                    request->poses[pose_index],
                    fk_pos_tol_mm,
                    fk_ori_tol_deg,
                    &pos_error_mm,
                    &ori_error_deg))
            {
                timing_stats.per_point_fk_s += steadySeconds() - per_point_fk_started_s;
                std::ostringstream msg;
                msg << "FK validation failed at index " << pose_index
                    << " pos_error_mm=" << pos_error_mm
                    << " ori_error_deg=" << ori_error_deg;
                fail(response, ERROR_FK_ERROR, msg.str(), static_cast<uint32_t>(pose_index));
                return false;
            }
            timing_stats.per_point_fk_s += steadySeconds() - per_point_fk_started_s;

            double max_step = 0.0;
            for (std::size_t j = 0; j < positions.size(); ++j)
            {
                max_step = std::max(max_step, std::abs(positions[j] - previous_positions[j]));
            }
            response->max_joint_step_rad =
                std::max(response->max_joint_step_rad, max_step);
            if (max_step > max_step_rad)
            {
                std::ostringstream msg;
                msg << "joint step exceeded at index " << pose_index
                    << " max_step_rad=" << max_step
                    << " limit_rad=" << max_step_rad;
                fail(response, ERROR_JOINT_STEP, msg.str(), static_cast<uint32_t>(pose_index));
                return false;
            }

            solved_points.push_back(positions);
            previous_previous_positions = previous_positions;
            previous_positions = positions;
            return true;
        };

        auto restore_solver_state = [&]()
        {
            if (solved_points.empty())
            {
                previous_positions = request->seed_state.position;
                previous_previous_positions.clear();
            }
            else
            {
                previous_positions = solved_points.back();
                if (solved_points.size() >= 2)
                {
                    previous_previous_positions = solved_points[solved_points.size() - 2];
                }
                else
                {
                    previous_previous_positions = request->seed_state.position;
                }
            }
            state.setVariablePositions(output_joint_names, previous_positions);
            state.update();
        };

        std::size_t cautious_until = 3;
        for (std::size_t i = 0; i < request->poses.size(); ++i)
        {
            const bool use_full_scoring = i < cautious_until;
            if (use_full_scoring)
            {
                timing_stats.full_score_points += 1;
            }
            else
            {
                timing_stats.fast_points += 1;
            }

            bool ok = solve_pose(i, use_full_scoring, timeout_s, true);
            if (!ok && !use_full_scoring && !solved_points.empty())
            {
                timing_stats.rollback_replays += 1;
                const std::size_t rollback_window = 8;
                const std::size_t rollback_start =
                    solved_points.size() > rollback_window
                        ? solved_points.size() - rollback_window
                        : 0;
                solved_points.resize(rollback_start);
                restore_solver_state();
                ok = true;
                for (std::size_t replay_index = rollback_start; replay_index <= i; ++replay_index)
                {
                    timing_stats.full_score_points += 1;
                    if (!solve_pose(replay_index, true, timeout_s, true) ||
                        !accept_current_state(replay_index))
                    {
                        ok = false;
                        break;
                    }
                }
                if (ok)
                {
                    cautious_until = std::max(cautious_until, i + 20);
                    continue;
                }
            }

            if (!ok)
            {
                std::ostringstream msg;
                msg << "IK failed at index " << i;
                fail(response, ERROR_IK_FAILED, msg.str(), static_cast<uint32_t>(i));
                response->points_solved = static_cast<uint32_t>(solved_points.size());
                response->solve_time_s = (this->now() - started_at).seconds();
                return;
            }

            if (!accept_current_state(i))
            {
                response->points_solved = static_cast<uint32_t>(solved_points.size());
                response->solve_time_s = (this->now() - started_at).seconds();
                return;
            }
        }

        const auto smoothing_started_at = this->now();
        const double max_curvature_before_smoothing = trajectoryMaxCurvature(solved_points);
        const std::size_t smoothing_updates = smoothSolvedPoints(
            request,
            output_joint_names,
            link_name,
            max_step_rad,
            state,
            solved_points,
            &timing_stats);
        const double smoothing_time_s = (this->now() - smoothing_started_at).seconds();
        const double max_curvature_after_smoothing = trajectoryMaxCurvature(solved_points);

        const double final_validate_started_s = steadySeconds();
        if (!validateFinalTrajectory(
                request,
                response,
                output_joint_names,
                link_name,
                full_turn_joints,
                fk_pos_tol_mm,
                fk_ori_tol_deg,
                max_step_rad,
                max_span_rad,
                max_endpoint_delta_rad,
                full_turn_span_rad,
                full_turn_endpoint_delta_rad,
                state,
                solved_points))
        {
            timing_stats.final_validate_s += steadySeconds() - final_validate_started_s;
            response->points_solved = static_cast<uint32_t>(solved_points.size());
            response->solve_time_s = (this->now() - started_at).seconds();
            return;
        }
        timing_stats.final_validate_s += steadySeconds() - final_validate_started_s;

        const double response_pack_started_s = steadySeconds();
        response->trajectory.joint_trajectory.points.reserve(solved_points.size());
        for (const auto& positions : solved_points)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = positions;
            response->trajectory.joint_trajectory.points.push_back(point);
        }
        timing_stats.response_pack_s += steadySeconds() - response_pack_started_s;

        response->success = true;
        response->error_code = ERROR_NONE;
        response->message = "ok";
        response->points_solved = static_cast<uint32_t>(solved_points.size());
        response->solve_time_s = (this->now() - started_at).seconds();
        RCLCPP_INFO(
            this->get_logger(),
            "Contour IK solved %zu poses in %.3fs: fk_max=%.4fmm/%.4fdeg max_step=%.4f span=%.4f endpoint=%.4f curvature=%.5f->%.5f smoothing_updates=%zu smoothing_s=%.3f timings{ik=%.3f candidate_eval=%.3f per_point_fk=%.3f smoothing_candidate=%.3f smoothing_fk=%.3f final_validate=%.3f response_pack=%.3f candidates=%zu/%zu smoothing_checks=%zu/%zu fast_points=%zu full_points=%zu rollbacks=%zu}",
            solved_points.size(),
            response->solve_time_s,
            response->max_fk_position_error_mm,
            response->max_fk_orientation_error_deg,
            response->max_joint_step_rad,
            response->max_joint_span_rad,
            response->max_endpoint_delta_rad,
            max_curvature_before_smoothing,
            max_curvature_after_smoothing,
            smoothing_updates,
            smoothing_time_s,
            timing_stats.ik_s,
            timing_stats.candidate_eval_s,
            timing_stats.per_point_fk_s,
            timing_stats.smoothing_candidate_s,
            timing_stats.smoothing_fk_s,
            timing_stats.final_validate_s,
            timing_stats.response_pack_s,
            timing_stats.candidate_successes,
            timing_stats.candidate_attempts,
            timing_stats.smoothing_fk_checks,
            timing_stats.smoothing_candidates,
            timing_stats.fast_points,
            timing_stats.full_score_points,
            timing_stats.rollback_replays);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContourIKHelperNode>();
    try
    {
        node->initialize();
    }
    catch (const std::exception& exc)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize contour IK helper: %s", exc.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
