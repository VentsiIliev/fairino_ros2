#include <memory>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdio>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "erob_moveit_runtime/srv/apply_ipp.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Reuse the same service definition as IPP (ApplyIPP)
using ApplyRuckig = erob_moveit_runtime::srv::ApplyIPP;

moveit_msgs::msg::RobotTrajectory sanitizeTrajectoryRequest(
    const trajectory_msgs::msg::JointTrajectory& input,
    rclcpp::Logger logger,
    double duplicate_joint_epsilon_rad = 1e-4)
{
    moveit_msgs::msg::RobotTrajectory sanitized;
    sanitized.joint_trajectory.joint_names = input.joint_names;

    if (input.points.empty())
    {
        return sanitized;
    }

    sanitized.joint_trajectory.points.reserve(input.points.size());
    sanitized.joint_trajectory.points.push_back(input.points.front());

    std::size_t removed_duplicates = 0;
    for (std::size_t i = 1; i < input.points.size(); ++i)
    {
        const auto& prev = sanitized.joint_trajectory.points.back();
        const auto& curr = input.points[i];

        if (prev.positions.size() != curr.positions.size())
        {
            sanitized.joint_trajectory.points.push_back(curr);
            continue;
        }

        double max_delta = 0.0;
        for (std::size_t j = 0; j < curr.positions.size(); ++j)
        {
            max_delta = std::max(max_delta, std::abs(curr.positions[j] - prev.positions[j]));
        }

        if (max_delta <= duplicate_joint_epsilon_rad)
        {
            ++removed_duplicates;
            continue;
        }

        sanitized.joint_trajectory.points.push_back(curr);
    }

    if (sanitized.joint_trajectory.points.size() == 1 && input.points.size() > 1)
    {
        sanitized.joint_trajectory.points.push_back(input.points.back());
    }

    if (removed_duplicates > 0)
    {
        RCLCPP_INFO(
            logger,
            "🧹 Removed %zu duplicate/near-duplicate joint waypoints before timing",
            removed_duplicates
        );
    }

    return sanitized;
}

/**
 * Log full trajectory details for debugging.
 */
void logTrajectory(
    const moveit_msgs::msg::RobotTrajectory& rt_msg,
    rclcpp::Logger logger,
    bool verbose = false)
{
    const auto& jt = rt_msg.joint_trajectory;

    RCLCPP_INFO(logger, "═══════════════════ TRAJECTORY DUMP ═══════════════════");
    RCLCPP_INFO(logger, "Joints: %zu, Points: %zu", jt.joint_names.size(), jt.points.size());

    std::string joint_names_str = "Joints: ";
    for (const auto& name : jt.joint_names)
    {
        joint_names_str += name + " ";
    }
    RCLCPP_INFO(logger, "%s", joint_names_str.c_str());

    for (size_t i = 0; i < jt.points.size(); ++i)
    {
        const auto& p = jt.points[i];
        double t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;

        if (!verbose && jt.points.size() > 10 && i > 2 && i < jt.points.size() - 3)
        {
            if (i == 3)
            {
                RCLCPP_INFO(logger, "  ... (%zu points omitted, use verbose for full log) ...",
                            jt.points.size() - 6);
            }
            continue;
        }

        std::string pos_str;
        for (size_t j = 0; j < p.positions.size(); ++j)
        {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%7.4f", p.positions[j]);
            pos_str += buf;
            if (j < p.positions.size() - 1) pos_str += ", ";
        }

        std::string vel_str;
        for (size_t j = 0; j < p.velocities.size(); ++j)
        {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%7.4f", p.velocities[j]);
            vel_str += buf;
            if (j < p.velocities.size() - 1) vel_str += ", ";
        }

        std::string acc_str;
        for (size_t j = 0; j < p.accelerations.size(); ++j)
        {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%7.4f", p.accelerations[j]);
            acc_str += buf;
            if (j < p.accelerations.size() - 1) acc_str += ", ";
        }

        RCLCPP_INFO(logger, "Point[%2zu] t=%6.3fs", i, t);
        RCLCPP_INFO(logger, "  pos: [%s]", pos_str.c_str());
        RCLCPP_INFO(logger, "  vel: [%s]", vel_str.c_str());
        RCLCPP_INFO(logger, "  acc: [%s]", acc_str.c_str());
    }

    if (!jt.points.empty())
    {
        double total_time = jt.points.back().time_from_start.sec +
                            jt.points.back().time_from_start.nanosec * 1e-9;

        double max_vel = 0.0;
        double max_acc = 0.0;
        for (const auto& p : jt.points)
        {
            for (double v : p.velocities) max_vel = std::max(max_vel, std::abs(v));
            for (double a : p.accelerations) max_acc = std::max(max_acc, std::abs(a));
        }

        RCLCPP_INFO(logger, "───────────────────────────────────────────────────────");
        RCLCPP_INFO(logger, "Summary: duration=%.3fs, max_vel=%.3f rad/s, max_acc=%.3f rad/s²",
                    total_time, max_vel, max_acc);
    }

    RCLCPP_INFO(logger, "═══════════════════════════════════════════════════════");
}

struct TrajectoryTimingStats
{
    double total_duration {0.0};
    double median_dt {0.0};
    double max_dt {0.0};
};

struct SegmentTimingInfo
{
    std::size_t start_index {0};
    std::size_t end_index {0};
    double start_time {0.0};
    double end_time {0.0};
    double dt {0.0};
};

std::string formatJointDeltaSummary(
    const moveit_msgs::msg::RobotTrajectory& rt_msg,
    std::size_t start_index,
    std::size_t end_index);

TrajectoryTimingStats computeTimingStats(const moveit_msgs::msg::RobotTrajectory& rt_msg)
{
    const auto& jt = rt_msg.joint_trajectory;
    TrajectoryTimingStats stats;

    if (jt.points.empty())
    {
        return stats;
    }

    stats.total_duration = jt.points.back().time_from_start.sec +
                           jt.points.back().time_from_start.nanosec * 1e-9;

    if (jt.points.size() < 2)
    {
        return stats;
    }

    std::vector<double> dts;
    dts.reserve(jt.points.size() - 1);

    double previous_time = jt.points.front().time_from_start.sec +
                           jt.points.front().time_from_start.nanosec * 1e-9;

    for (size_t i = 1; i < jt.points.size(); ++i)
    {
        const auto& p = jt.points[i];
        const double t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
        const double dt = t - previous_time;

        if (dt > 1e-6)
        {
            dts.push_back(dt);
            stats.max_dt = std::max(stats.max_dt, dt);
        }

        previous_time = t;
    }

    if (!dts.empty())
    {
        std::sort(dts.begin(), dts.end());
        const size_t mid = dts.size() / 2;
        stats.median_dt = (dts.size() % 2 == 0)
            ? 0.5 * (dts[mid - 1] + dts[mid])
            : dts[mid];
    }

    return stats;
}

SegmentTimingInfo findLargestGapSegment(const moveit_msgs::msg::RobotTrajectory& rt_msg)
{
    const auto& jt = rt_msg.joint_trajectory;
    SegmentTimingInfo info;

    if (jt.points.size() < 2)
    {
        return info;
    }

    double previous_time = jt.points.front().time_from_start.sec +
                           jt.points.front().time_from_start.nanosec * 1e-9;

    for (std::size_t i = 1; i < jt.points.size(); ++i)
    {
        const auto& p = jt.points[i];
        const double t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
        const double dt = t - previous_time;

        if (dt > info.dt)
        {
            info.start_index = i - 1;
            info.end_index = i;
            info.start_time = previous_time;
            info.end_time = t;
            info.dt = dt;
        }

        previous_time = t;
    }

    return info;
}

std::string formatJointDeltaSummary(
    const moveit_msgs::msg::RobotTrajectory& rt_msg,
    std::size_t start_index,
    std::size_t end_index)
{
    const auto& jt = rt_msg.joint_trajectory;
    if (jt.points.empty() || start_index >= jt.points.size() || end_index >= jt.points.size())
    {
        return "n/a";
    }

    const auto& start = jt.points[start_index];
    const auto& end = jt.points[end_index];
    const std::size_t count = std::min(start.positions.size(), end.positions.size());

    std::string summary;
    for (std::size_t j = 0; j < count; ++j)
    {
        const std::string& joint_name =
            (j < jt.joint_names.size()) ? jt.joint_names[j] : ("joint_" + std::to_string(j));
        const double delta = end.positions[j] - start.positions[j];
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s=%+.4f ", joint_name.c_str(), delta);
        summary += buf;
    }

    return summary.empty() ? "n/a" : summary;
}

bool validateRuckigAgainstSeed(
    const moveit_msgs::msg::RobotTrajectory& seeded_msg,
    const moveit_msgs::msg::RobotTrajectory& ruckig_msg,
    rclcpp::Logger logger)
{
    const TrajectoryTimingStats seeded = computeTimingStats(seeded_msg);
    const TrajectoryTimingStats output = computeTimingStats(ruckig_msg);

    if (seeded.total_duration <= 0.0 || output.total_duration <= 0.0)
    {
        return true;
    }

    const double allowed_duration_ratio = 1.75;
    const double allowed_gap = std::max(0.5, std::max(seeded.median_dt, output.median_dt) * 6.0);

    const bool excessive_total_stretch =
        output.total_duration > seeded.total_duration * allowed_duration_ratio;

    const bool excessive_local_gap =
        output.max_dt > allowed_gap;

    if (excessive_local_gap)
    {
        const SegmentTimingInfo output_gap = findLargestGapSegment(ruckig_msg);
        const SegmentTimingInfo seeded_gap = findLargestGapSegment(seeded_msg);
        RCLCPP_ERROR(
            logger,
            "❌ Ruckig output is implausible vs seeded trajectory: excessive local time gap detected "
            "(seeded_duration=%.3fs, output_duration=%.3fs, max_output_dt=%.3fs, allowed_gap=%.3fs)",
            seeded.total_duration,
            output.total_duration,
            output.max_dt,
            allowed_gap
        );
        RCLCPP_ERROR(
            logger,
            "   Ruckig silently stretched one segment far beyond the seeded trajectory cadence"
        );
        RCLCPP_ERROR(
            logger,
            "   Worst output gap: segment [%zu -> %zu], t=[%.3f -> %.3f], dt=%.3fs",
            output_gap.start_index,
            output_gap.end_index,
            output_gap.start_time,
            output_gap.end_time,
            output_gap.dt
        );
        RCLCPP_ERROR(
            logger,
            "   Output joint deltas across worst gap: %s",
            formatJointDeltaSummary(
                ruckig_msg,
                output_gap.start_index,
                output_gap.end_index
            ).c_str()
        );
        RCLCPP_ERROR(
            logger,
            "   Largest seeded gap: segment [%zu -> %zu], t=[%.3f -> %.3f], dt=%.3fs",
            seeded_gap.start_index,
            seeded_gap.end_index,
            seeded_gap.start_time,
            seeded_gap.end_time,
            seeded_gap.dt
        );
        RCLCPP_ERROR(
            logger,
            "   Seeded joint deltas across largest gap: %s",
            formatJointDeltaSummary(
                seeded_msg,
                seeded_gap.start_index,
                seeded_gap.end_index
            ).c_str()
        );
        return false;
    }

    if (excessive_total_stretch)
    {
        RCLCPP_ERROR(
            logger,
            "❌ Ruckig output is implausibly slow vs seeded trajectory: seeded_duration=%.3fs, "
            "output_duration=%.3fs, ratio=%.3f (limit=%.3f)",
            seeded.total_duration,
            output.total_duration,
            output.total_duration / seeded.total_duration,
            allowed_duration_ratio
        );
        return false;
    }

    return true;
}

/**
 * Validate a trajectory for execution safety and timing sanity.
 * Returns false if trajectory is invalid and should NOT be executed.
 */
bool validateTrajectory(
    const moveit_msgs::msg::RobotTrajectory& rt_msg,
    const moveit::core::RobotModelPtr& model,
    rclcpp::Logger logger,
    double max_duration = 30.0)
{
    const auto& jt = rt_msg.joint_trajectory;

    if (jt.points.size() < 2)
    {
        RCLCPP_ERROR(logger, "❌ Trajectory has <2 points");
        return false;
    }

    const size_t num_joints = jt.joint_names.size();
    if (num_joints == 0)
    {
        RCLCPP_ERROR(logger, "❌ No joint names in trajectory");
        return false;
    }

    std::vector<double> segment_dts;
    segment_dts.reserve(jt.points.size() - 1);

    double previous_time = jt.points.front().time_from_start.sec +
                           jt.points.front().time_from_start.nanosec * 1e-9;

    for (size_t i = 1; i < jt.points.size(); ++i)
    {
        const auto& p = jt.points[i];
        const double t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
        const double dt = t - previous_time;
        if (dt > 1e-6)
        {
            segment_dts.push_back(dt);
        }
        previous_time = t;
    }

    double median_dt = 0.0;
    if (!segment_dts.empty())
    {
        std::sort(segment_dts.begin(), segment_dts.end());
        const size_t mid = segment_dts.size() / 2;
        median_dt = (segment_dts.size() % 2 == 0)
            ? 0.5 * (segment_dts[mid - 1] + segment_dts[mid])
            : segment_dts[mid];
    }

    const double total_duration =
        jt.points.back().time_from_start.sec +
        jt.points.back().time_from_start.nanosec * 1e-9;

    // Stricter than before: contour/path trajectories should not contain giant time holes.
    const double dynamic_gap_limit = std::max(0.35, std::min(1.0, median_dt * 6.0));

    double last_time = -1.0;

    std::unordered_map<std::string, moveit::core::VariableBounds> limits;
    for (const auto* jm : model->getJointModels())
    {
        if (jm->getVariableCount() == 1)
        {
            limits[jm->getName()] = jm->getVariableBounds()[0];
        }
    }

    for (size_t i = 0; i < jt.points.size(); ++i)
    {
        const auto& p = jt.points[i];

        if (p.positions.size() != num_joints)
        {
            RCLCPP_ERROR(logger, "❌ Point %zu has wrong position size", i);
            return false;
        }

        const double t = p.time_from_start.sec +
                         p.time_from_start.nanosec * 1e-9;

        if (i > 0 && t <= last_time)
        {
            RCLCPP_ERROR(logger,
                         "❌ Non-monotonic timing at point %zu (%.6f <= %.6f)",
                         i, t, last_time);
            return false;
        }

        if (i > 0 && (t - last_time) < 1e-6)
        {
            RCLCPP_ERROR(logger,
                         "❌ Zero-duration segment at point %zu (dt=%.9f)",
                         i, t - last_time);
            return false;
        }

        if (i > 0 && (t - last_time) > dynamic_gap_limit)
        {
            RCLCPP_ERROR(
                logger,
                "❌ Excessive adjacent time gap at point %zu: %.3fs "
                "(median dt: %.3fs, limit: %.3fs, total duration: %.3fs)",
                i, t - last_time, median_dt, dynamic_gap_limit, total_duration
            );
            RCLCPP_ERROR(
                logger,
                "   Large timing hole detected after Ruckig smoothing; rejecting trajectory"
            );
            return false;
        }

        last_time = t;

        if (p.velocities.size() != num_joints)
        {
            RCLCPP_ERROR(logger,
                         "❌ Missing velocities at point %zu (got %zu, expected %zu)",
                         i, p.velocities.size(), num_joints);
            return false;
        }

        if (p.accelerations.size() != num_joints)
        {
            RCLCPP_ERROR(logger,
                         "❌ Missing accelerations at point %zu (got %zu, expected %zu)",
                         i, p.accelerations.size(), num_joints);
            return false;
        }

        for (size_t j = 0; j < num_joints; ++j)
        {
            if (!std::isfinite(p.positions[j]) ||
                !std::isfinite(p.velocities[j]) ||
                !std::isfinite(p.accelerations[j]))
            {
                RCLCPP_ERROR(logger,
                             "❌ NaN/Inf detected at point %zu joint %zu",
                             i, j);
                return false;
            }

            const auto& name = jt.joint_names[j];
            if (limits.count(name))
            {
                const auto& b = limits[name];
                if (p.positions[j] < b.min_position_ - 1e-3 ||
                    p.positions[j] > b.max_position_ + 1e-3)
                {
                    RCLCPP_ERROR(
                        logger,
                        "❌ Joint %s position %.4f out of limits [%.4f, %.4f] at point %zu",
                        name.c_str(), p.positions[j], b.min_position_, b.max_position_, i
                    );
                    return false;
                }
            }
        }

        if (i > 0)
        {
            const auto& prev = jt.points[i - 1];
            for (size_t j = 0; j < num_joints; ++j)
            {
                double jump = std::abs(p.positions[j] - prev.positions[j]);
                if (jump > 0.5)
                {
                    RCLCPP_ERROR(
                        logger,
                        "❌ Teleport detected: joint %zu jumped %.3f rad at point %zu",
                        j, jump, i
                    );
                    return false;
                }
            }
        }
    }

    if (last_time > max_duration)
    {
        RCLCPP_ERROR(
            logger,
            "❌ Trajectory duration %.2fs exceeds limit %.2fs (Ruckig likely failed)",
            last_time, max_duration
        );
        return false;
    }

    return true;
}

class RuckigHelperNode : public rclcpp::Node
{
public:
    RuckigHelperNode() : Node("ruckig_helper")
    {
        RCLCPP_INFO(this->get_logger(), "📡 Ruckig Helper Node starting...");

        service_ = this->create_service<ApplyRuckig>(
            "/apply_ruckig",
            std::bind(&RuckigHelperNode::applyRuckig, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "✅ Service '/apply_ruckig' created");
    }

    void initialize()
    {
        auto node_ptr = shared_from_this();

        RCLCPP_INFO(this->get_logger(), "⏳ Loading robot model...");
        loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_ptr);
        kinematic_model_ = loader_->getModel();

        if (!kinematic_model_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model!");
            throw std::runtime_error("Robot model load failed");
        }

        robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        robot_state_->setToDefaultValues();

        auto group_names = kinematic_model_->getJointModelGroupNames();
        if (group_names.empty())
        {
            throw std::runtime_error("Robot model has no planning groups!");
        }

        planning_group_ = group_names[0];
        RCLCPP_INFO(this->get_logger(), "📌 Using planning group: '%s'", planning_group_.c_str());

        RCLCPP_INFO(this->get_logger(), "✅ Robot model and state cached successfully");
        RCLCPP_INFO(this->get_logger(), "✅ Ruckig service '/apply_ruckig' is now ready");
        RCLCPP_INFO(this->get_logger(), "   Using jerk-limited trajectory smoothing (3rd order)");
    }

private:
    rclcpp::Service<ApplyRuckig>::SharedPtr service_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::string planning_group_;

    void applyRuckig(const std::shared_ptr<ApplyRuckig::Request> request,
                     std::shared_ptr<ApplyRuckig::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "📥 Received Ruckig request with %zu points",
                    request->trajectory.points.size());

        if (request->trajectory.points.size() >= 2)
        {
            const auto& p0 = request->trajectory.points[0];
            const auto& p1 = request->trajectory.points[1];

            double max_diff = 0.0;
            for (size_t j = 0; j < std::min(p0.positions.size(), p1.positions.size()); ++j)
            {
                double diff = std::abs(p0.positions[j] - p1.positions[j]);
                max_diff = std::max(max_diff, diff);
            }

            if (max_diff < 0.0001)
            {
                RCLCPP_WARN(this->get_logger(),
                            "⚠️  First two waypoints nearly identical (max_diff=%.6f rad) - may cause oscillation",
                            max_diff);
            }
        }

        auto sanitized_request = sanitizeTrajectoryRequest(request->trajectory, this->get_logger());
        if (sanitized_request.joint_trajectory.points.size() < 2)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Sanitized Ruckig input has <2 unique points");
            response->trajectory = moveit_msgs::msg::RobotTrajectory();
            return;
        }

        robot_trajectory::RobotTrajectory rt(kinematic_model_, planning_group_);
        rt.setRobotTrajectoryMsg(*robot_state_, sanitized_request);

        const double max_vel_scaling = std::max(0.0, std::min(1.0, request->max_velocity_scaling));
        const double max_acc_scaling = std::max(0.0, std::min(1.0, request->max_acceleration_scaling));

        RCLCPP_INFO(this->get_logger(), "⚙️  Applying Ruckig smoothing with vel_scale=%.2f, acc_scale=%.2f",
                    max_vel_scaling, max_acc_scaling);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg_seed;
        const bool seed_ok = totg_seed.computeTimeStamps(rt, max_vel_scaling, max_acc_scaling);
        if (!seed_ok)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Ruckig seed timing (TOTG) failed");
            response->trajectory = moveit_msgs::msg::RobotTrajectory();
            return;
        }

        moveit_msgs::msg::RobotTrajectory seeded_msg;
        rt.getRobotTrajectoryMsg(seeded_msg);
        rt.setRobotTrajectoryMsg(*robot_state_, seeded_msg);

//         RCLCPP_INFO(this->get_logger(), "📊 SEEDED trajectory BEFORE Ruckig:");
//         logTrajectory(seeded_msg, this->get_logger(), true);

//         RCLCPP_INFO(this->get_logger(), "📊 BEFORE Ruckig:");
//         moveit_msgs::msg::RobotTrajectory before_msg;
//         rt.getRobotTrajectoryMsg(before_msg);
//         logTrajectory(before_msg, this->get_logger(), true);

        const bool success = trajectory_processing::RuckigSmoothing::applySmoothing(
            rt,
            max_vel_scaling,
            max_acc_scaling,
            true,
            0.001
        );

//         RCLCPP_INFO(this->get_logger(), "📊 AFTER Ruckig (success=%s):", success ? "true" : "false");
//         moveit_msgs::msg::RobotTrajectory after_msg;
//         rt.getRobotTrajectoryMsg(after_msg);
//         logTrajectory(after_msg, this->get_logger(), true);

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Ruckig Smoothing FAILED - falling back to seeded TOTG trajectory");
            response->trajectory = seeded_msg;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ Ruckig Smoothing returned success");
        rt.getRobotTrajectoryMsg(response->trajectory);

        if (!validateRuckigAgainstSeed(seeded_msg, response->trajectory, this->get_logger()))
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Ruckig output deemed implausible - falling back to seeded TOTG trajectory");
            response->trajectory = seeded_msg;
            return;
        }

        const size_t num_points = response->trajectory.joint_trajectory.points.size();
        if (num_points > 0)
        {
            const auto& last_point = response->trajectory.joint_trajectory.points.back();
            const double total_time = last_point.time_from_start.sec +
                                      last_point.time_from_start.nanosec * 1e-9;
            RCLCPP_INFO(this->get_logger(), "🔍 Trajectory: %zu points, duration: %.2fs",
                        num_points, total_time);
        }

        double max_duration = 10.0 / std::max(0.1, max_vel_scaling);
        max_duration = std::min(max_duration, 60.0);

        if (!validateTrajectory(response->trajectory, kinematic_model_, this->get_logger(), max_duration))
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Trajectory validation FAILED after Ruckig");
            RCLCPP_ERROR(this->get_logger(), "   Falling back to seeded TOTG trajectory instead.");

            RCLCPP_WARN(this->get_logger(), "Dumping INVALID trajectory for debugging:");
//             logTrajectory(response->trajectory, this->get_logger(), true);

            response->trajectory = seeded_msg;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ Trajectory validated successfully");
//         logTrajectory(response->trajectory, this->get_logger(), false);
        RCLCPP_INFO(this->get_logger(), "📤 Returning %zu jerk-limited points", num_points);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RuckigHelperNode>();

    try
    {
        node->initialize();
        RCLCPP_INFO(node->get_logger(), "✅ Ruckig Helper fully initialized and ready");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "❌ Failed to initialize Ruckig Helper: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
