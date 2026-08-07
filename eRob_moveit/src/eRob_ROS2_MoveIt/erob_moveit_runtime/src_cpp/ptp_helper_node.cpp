#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include "erob_moveit_runtime/srv/compute_ptp.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


using ComputePtp = erob_moveit_runtime::srv::ComputePtp;


namespace
{

constexpr int32_t ERROR_NONE = 1;
constexpr int32_t ERROR_INVALID_REQUEST = -1;
constexpr int32_t ERROR_IK_FAILED = -2;
constexpr int32_t ERROR_COLLISION = -10;
constexpr int32_t ERROR_UNSAFE_ORIENTATION = -11;


double steadySeconds()
{
    using Clock = std::chrono::steady_clock;

    return std::chrono::duration<double>(
        Clock::now().time_since_epoch()
    ).count();
}


double nearestEquivalentAngle(
    double reference,
    double value)
{
    constexpr double two_pi = 2.0 * M_PI;

    double adjusted = value;

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


double quaternionAngleDeg(
    const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b)
{
    Eigen::Quaterniond qa = a;
    Eigen::Quaterniond qb = b;

    qa.normalize();
    qb.normalize();

    const double dot = std::clamp(
        std::abs(qa.dot(qb)),
        0.0,
        1.0
    );

    return 2.0 * std::acos(dot) * 180.0 / M_PI;
}


Eigen::Quaterniond poseQuaternion(
    const geometry_msgs::msg::Pose& pose)
{
    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    );

    q.normalize();

    return q;
}


double maxAbsDelta(
    const std::vector<double>& a,
    const std::vector<double>& b)
{
    double result = 0.0;

    const std::size_t count =
        std::min(a.size(), b.size());

    for (std::size_t i = 0; i < count; ++i)
    {
        result = std::max(
            result,
            std::abs(a[i] - b[i])
        );
    }

    return result;
}


bool approximatelySame(
    const std::vector<double>& a,
    const std::vector<double>& b,
    double tolerance = 1e-4)
{
    if (a.size() != b.size())
    {
        return false;
    }

    return maxAbsDelta(a, b) <= tolerance;
}


bool ptpSampleLoggingEnabled()
{
    const char* value =
        std::getenv("PTP_LOG_SAMPLES");

    if (value == nullptr)
    {
        return false;
    }

    return std::string(value) != "0";
}


std::string formatContacts(
    const collision_detection::
        CollisionResult::ContactMap&
            contacts)
{
    std::ostringstream out;

    out << "[";

    bool first = true;

    for (const auto& contact_pair :
         contacts)
    {
        if (!first)
        {
            out << ", ";
        }

        first = false;

        out
            << contact_pair.first.first
            << "<->"
            << contact_pair.first.second;
    }

    out << "]";

    return out.str();
}


std::string formatJointState(
    const std::vector<std::string>& names,
    const std::vector<double>& values)
{
    std::ostringstream out;

    out << std::fixed
        << std::setprecision(4)
        << "q=[";

    const std::size_t count =
        std::min(
            names.size(),
            values.size()
        );

    for (std::size_t i = 0;
         i < count;
         ++i)
    {
        if (i)
        {
            out << ", ";
        }

        out << names[i] << "=" << values[i];
    }

    out << "]";

    return out.str();
}


std::string formatPoseTransform(
    const Eigen::Isometry3d& transform)
{
    std::ostringstream out;

    out << std::fixed
        << std::setprecision(2);

    const Eigen::Vector3d position =
        transform.translation();

    const Eigen::Matrix3d rotation =
        transform.rotation();

    const double rx =
        std::atan2(
            rotation(2, 1),
            rotation(2, 2)
        );

    const double ry =
        std::atan2(
            -rotation(2, 0),
            std::sqrt(
                rotation(2, 1) * rotation(2, 1) +
                rotation(2, 2) * rotation(2, 2)
            )
        );

    const double rz =
        std::atan2(
            rotation(1, 0),
            rotation(0, 0)
        );

    out << "xyz=["
        << position.x() * 1000.0 << ", "
        << position.y() * 1000.0 << ", "
        << position.z() * 1000.0 << "]"
        << " rpy=["
        << rx * 180.0 / M_PI << ", "
        << ry * 180.0 / M_PI << ", "
        << rz * 180.0 / M_PI << "]";

    return out.str();
}

}  // namespace


class PtpHelperNode : public rclcpp::Node
{
public:

    PtpHelperNode()
        : Node("ptp_helper"),
          log_samples_(ptpSampleLoggingEnabled())
    {
        RCLCPP_INFO(
            this->get_logger(),
            "PTP helper starting..."
        );

        service_ = this->create_service<ComputePtp>(
            "/compute_ptp",
            std::bind(
                &PtpHelperNode::computePtp,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
    }


    void initialize()
    {
        auto node_ptr = shared_from_this();

        loader_ =
            std::make_shared<
                robot_model_loader::RobotModelLoader
            >(node_ptr);

        model_ = loader_->getModel();

        if (!model_)
        {
            throw std::runtime_error(
                "PTP helper failed to load robot model"
            );
        }

        /*
         * Keep a local copy of MoveIt's PlanningScene.
         *
         * Collision checking below therefore becomes an in-process
         * C++ call instead of /check_state_validity for every sample.
         */
        planning_scene_monitor_ =
            std::make_shared<
                planning_scene_monitor::PlanningSceneMonitor
            >(
                node_ptr,
                loader_,
                "ptp_planning_scene_monitor"
            );

        if (!planning_scene_monitor_->getPlanningScene())
        {
            throw std::runtime_error(
                "PTP helper failed to create PlanningSceneMonitor"
            );
        }

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        /*
         * Important because this helper starts after MoveIt and some
         * collision objects may already have been published.
         */
        if (!planning_scene_monitor_->requestPlanningSceneState(
                "/get_planning_scene"))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Could not request initial planning scene; "
                "continuing with monitored scene updates"
            );
        }

        RCLCPP_INFO(
            this->get_logger(),
            "PTP helper ready"
        );
    }


private:

    struct Candidate
    {
        std::vector<double> positions;
        double cost = 0.0;
    };


    bool log_samples_;

    rclcpp::Service<ComputePtp>::SharedPtr service_;

    std::shared_ptr<
        robot_model_loader::RobotModelLoader
    > loader_;

    moveit::core::RobotModelPtr model_;

    std::shared_ptr<
        planning_scene_monitor::PlanningSceneMonitor
    > planning_scene_monitor_;


    void fail(
        const std::shared_ptr<ComputePtp::Response>& response,
        int32_t code,
        const std::string& message)
    {
        response->success = false;
        response->noop = false;
        response->error_code = code;
        response->message = message;

        RCLCPP_WARN(
            this->get_logger(),
            "PTP rejected: %s",
            message.c_str()
        );
    }


    bool loadStartState(
        const ComputePtp::Request& request,
        moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* group,
        std::vector<double>& current_positions)
    {
        if (!group)
        {
            return false;
        }

        std::unordered_map<std::string, double> incoming;

        for (std::size_t i = 0;
             i < request.start_state.name.size() &&
             i < request.start_state.position.size();
             ++i)
        {
            incoming[
                request.start_state.name[i]
            ] = request.start_state.position[i];
        }

        /*
         * Start from defaults so non-manipulator joints still have
         * defined values.
         */
        state.setToDefaultValues();

        const auto& model_variable_names =
            model_->getVariableNames();

        for (const auto& item : incoming)
        {
            if (std::find(
                    model_variable_names.begin(),
                    model_variable_names.end(),
                    item.first
                ) != model_variable_names.end())
            {
                state.setVariablePosition(
                    item.first,
                    item.second
                );
            }
        }

        state.update();

        const auto& names =
            group->getVariableNames();

        current_positions.resize(names.size());

        for (std::size_t i = 0;
             i < names.size();
             ++i)
        {
            auto found = incoming.find(names[i]);

            if (found == incoming.end())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Start state missing joint '%s'",
                    names[i].c_str()
                );

                return false;
            }

            current_positions[i] =
                found->second;
        }

        state.setJointGroupPositions(
            group,
            current_positions
        );

        state.update();

        return true;
    }


    void normalizeCandidateNearCurrent(
        moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* group,
        const std::vector<double>& current,
        std::vector<double>& candidate)
    {
        /*
         * First try the nearest ±2π representation.
         */
        std::vector<double> normalized =
            candidate;

        for (std::size_t i = 0;
             i < normalized.size();
             ++i)
        {
            normalized[i] =
                nearestEquivalentAngle(
                    current[i],
                    normalized[i]
                );
        }

        state.setJointGroupPositions(
            group,
            normalized
        );

        state.update();

        /*
         * Only keep it if it is actually inside the robot's
         * configured limits.
         */
        if (state.satisfiesBounds(group))
        {
            candidate = normalized;
            return;
        }

        /*
         * Otherwise retain the original IK representation.
         */
        state.setJointGroupPositions(
            group,
            candidate
        );

        state.update();
    }


    double candidateCost(
        const std::vector<double>& current,
        const std::vector<double>& candidate,
        const std::vector<double>& requested_weights)
    {
        /*
         * Defaults:
         *
         * J3 receives extra cost because changing elbow family is
         * undesirable.
         *
         * J5 receives strong cost because wrist-family changes are
         * undesirable.
         *
         * J6 is deliberately cheap.
         */
        static const std::vector<double>
            defaults = {
                1.0,   // J1
                1.2,   // J2
                2.5,   // J3
                1.5,   // J4
                4.0,   // J5
                0.10   // J6
            };

        double cost = 0.0;

        for (std::size_t i = 0;
             i < candidate.size();
             ++i)
        {
            double weight = 1.0;

            if (i < requested_weights.size())
            {
                weight =
                    requested_weights[i];
            }
            else if (i < defaults.size())
            {
                weight =
                    defaults[i];
            }

            const double dq =
                candidate[i] - current[i];

            cost += weight * dq * dq;
        }

        /*
         * These are preferences, not hard safety rules.
         *
         * Collision checking below is what determines whether
         * an elbow configuration is actually dangerous.
         */
        if (candidate.size() >= 3)
        {
            const double elbow_delta =
                std::abs(
                    candidate[2] -
                    current[2]
                );

            if (elbow_delta >
                0.5 * M_PI)
            {
                const double overflow =
                    elbow_delta -
                    0.5 * M_PI;

                cost +=
                    20.0 *
                    overflow *
                    overflow;
            }
        }

        if (candidate.size() >= 5)
        {
            const double wrist_delta =
                std::abs(
                    candidate[4] -
                    current[4]
                );

            if (wrist_delta >
                0.5 * M_PI)
            {
                const double overflow =
                    wrist_delta -
                    0.5 * M_PI;

                cost +=
                    40.0 *
                    overflow *
                    overflow;
            }
        }

        return cost;
    }


    std::vector<std::vector<double>>
    buildSeeds(
        const std::vector<double>& current,
        uint32_t max_attempts)
    {
        std::vector<
            std::vector<double>
        > seeds;

        seeds.push_back(current);

        /*
         * Small perturbations help numerical IK discover nearby
         * alternative branches without using random seeds.
         *
         * We deliberately don't perturb J6 because J6 is allowed
         * to rotate and is not what defines the dangerous elbow
         * configuration.
         */
        const double small =
            15.0 * M_PI / 180.0;

        const double larger =
            35.0 * M_PI / 180.0;

        const std::vector<std::size_t>
            interesting_joints = {
                1,  // J2
                2,  // J3
                3,  // J4
                4   // J5
            };

        for (const std::size_t joint :
             interesting_joints)
        {
            if (joint >= current.size())
            {
                continue;
            }

            for (double amount :
                 {small, -small,
                  larger, -larger})
            {
                auto seed = current;
                seed[joint] += amount;
                seeds.push_back(seed);

                if (seeds.size() >= max_attempts)
                {
                    return seeds;
                }
            }
        }

        return seeds;
    }


    void collectCandidates(
        moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* group,
        const ComputePtp::Request& request,
        const std::vector<double>& current,
        std::vector<Candidate>& candidates,
        ComputePtp::Response& response)
    {
        const uint32_t requested_attempts =
            std::clamp<uint32_t>(
                request.ik_attempts,
                1,
                20
            );

        const auto seeds =
            buildSeeds(
                current,
                requested_attempts
            );

        const double timeout =
            std::clamp(
                request.ik_timeout_s,
                0.001,
                0.2
            );

        for (const auto& seed : seeds)
        {
            ++response.ik_attempts_made;

            state.setJointGroupPositions(
                group,
                seed
            );

            state.update();

            const bool ok =
                state.setFromIK(
                    group,
                    request.target_pose,
                    request.link_name,
                    timeout
                );

            if (!ok)
            {
                continue;
            }

            std::vector<double> candidate;

            state.copyJointGroupPositions(
                group,
                candidate
            );

            normalizeCandidateNearCurrent(
                state,
                group,
                current,
                candidate
            );

            /*
             * Verify the final candidate is still in bounds.
             */
            state.setJointGroupPositions(
                group,
                candidate
            );

            state.update();

            if (!state.satisfiesBounds(group))
            {
                continue;
            }

            bool duplicate = false;

            for (const auto& existing :
                 candidates)
            {
                if (approximatelySame(
                        existing.positions,
                        candidate))
                {
                    duplicate = true;
                    break;
                }
            }

            if (duplicate)
            {
                continue;
            }

            Candidate result;
            result.positions = candidate;

            result.cost =
                candidateCost(
                    current,
                    candidate,
                    request.joint_weights
                );

            candidates.push_back(
                std::move(result)
            );

            ++response.ik_solutions_found;
        }

        std::sort(
            candidates.begin(),
            candidates.end(),
            [](
                const Candidate& a,
                const Candidate& b)
            {
                return a.cost < b.cost;
            }
        );
    }


    std::size_t segmentCount(
        const ComputePtp::Request& request,
        const std::vector<double>& current,
        const std::vector<double>& target)
    {
        const double max_delta =
            maxAbsDelta(
                current,
                target
            );

        const double step =
            std::max(
                request.interpolation_step_rad,
                0.005
            );

        const std::size_t minimum =
            std::max<std::size_t>(
                request.min_interpolation_segments,
                2
            );

        const std::size_t maximum =
            std::max<std::size_t>(
                request.max_interpolation_segments,
                minimum
            );

        const std::size_t calculated =
            static_cast<std::size_t>(
                std::ceil(
                    max_delta / step
                )
            );

        return std::clamp(
            calculated,
            minimum,
            maximum
        );
    }


    bool validateCandidate(
        moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* group,
        const ComputePtp::Request& request,
        const std::vector<double>& current,
        const std::vector<double>& candidate,
        const Eigen::Quaterniond& start_orientation,
        bool orientation_locked,
        std::string& rejection_reason)
    {
        const std::size_t segments =
            segmentCount(
                request,
                current,
                candidate
            );

        const Eigen::Quaterniond
            target_orientation =
                poseQuaternion(
                    request.target_pose
                );

        const double max_orientation_error =
            orientation_locked
                ? request.locked_path_max_deviation_deg
                : request.oriented_path_max_deviation_deg;

        planning_scene_monitor::
            LockedPlanningSceneRO scene(
                planning_scene_monitor_
            );

        std::vector<double> q(
            current.size()
        );

        std::vector<std::size_t>
            failed_samples;

        std::string first_reason;
        bool first_reason_set = false;

        for (std::size_t index = 0;
             index <= segments;
             ++index)
        {
            const double t =
                static_cast<double>(index) /
                static_cast<double>(segments);

            for (std::size_t joint = 0;
                 joint < q.size();
                 ++joint)
            {
                q[joint] =
                    current[joint] +
                    t *
                    (
                        candidate[joint] -
                        current[joint]
                    );
            }

            state.setJointGroupPositions(
                group,
                q
            );

            state.update();

            const bool in_bounds =
                state.satisfiesBounds(group);

            bool colliding = false;

            collision_detection::
                CollisionResult::ContactMap
                    contacts;

            if (request.avoid_collisions)
            {
                collision_detection::
                    CollisionRequest
                        collision_request;

                collision_request.group_name =
                    request.group_name;

                collision_request.contacts = true;
                collision_request.max_contacts = 20;
                collision_request.
                    max_contacts_per_pair = 5;

                collision_detection::
                    CollisionResult
                        collision_result;

                scene->checkCollision(
                    collision_request,
                    collision_result,
                    state
                );

                colliding =
                    collision_result.collision;

                contacts =
                    collision_result.contacts;
            }

            const auto& tcp_transform =
                state.getGlobalLinkTransform(
                    request.link_name
                );

            Eigen::Quaterniond
                actual_orientation(
                    tcp_transform.rotation()
                );

            actual_orientation.normalize();

            Eigen::Quaterniond
                reference_orientation;

            if (orientation_locked)
            {
                reference_orientation =
                    start_orientation;
            }
            else
            {
                reference_orientation =
                    start_orientation.slerp(
                        t,
                        target_orientation
                    );
            }

            reference_orientation.normalize();

            const double deviation =
                quaternionAngleDeg(
                    actual_orientation,
                    reference_orientation
                );

            const bool orientation_ok =
                deviation <= max_orientation_error;

            const bool sample_ok =
                in_bounds &&
                !colliding &&
                orientation_ok;

            if (log_samples_)
            {
                std::ostringstream line;

                line
                    << "PTP sample "
                    << index
                    << "/"
                    << segments
                    << " t="
                    << std::fixed
                    << std::setprecision(3)
                    << t
                    << " bounds="
                    << (in_bounds ? "OK" : "FAIL")
                    << " collision="
                    << (colliding ? "FAIL" : "OK");

                if (colliding)
                {
                    line
                        << " contacts="
                        << formatContacts(
                            contacts
                        );
                }

                line
                    << " orientation="
                    << (orientation_ok ? "OK" : "FAIL")
                    << " dev="
                    << deviation
                    << " limit="
                    << max_orientation_error;

                RCLCPP_INFO(
                    this->get_logger(),
                    "%s %s %s",
                    line.str().c_str(),
                    formatPoseTransform(
                        tcp_transform
                    ).c_str(),
                    formatJointState(
                        group->getVariableNames(),
                        q
                    ).c_str()
                );
            }

            if (!sample_ok)
            {
                failed_samples.push_back(
                    index
                );

                if (!first_reason_set)
                {
                    first_reason_set = true;

                    if (!in_bounds)
                    {
                        first_reason =
                            "joint bounds violated at sample "
                            + std::to_string(index);
                    }
                    else if (colliding)
                    {
                        std::ostringstream message;

                        message
                            << "collision at PTP sample "
                            << index
                            << "/"
                            << segments;

                        if (!contacts.empty())
                        {
                            message
                                << " contacts="
                                << formatContacts(
                                    contacts
                                );
                        }

                        first_reason =
                            message.str();
                    }
                    else
                    {
                        std::ostringstream message;

                        message
                            << "unsafe TCP orientation at sample "
                            << index
                            << "/"
                            << segments
                            << ": deviation="
                            << deviation
                            << " deg, limit="
                            << max_orientation_error
                            << " deg";

                        first_reason =
                            message.str();
                    }
                }

                /*
                 * When detailed logging is disabled keep the
                 * original fast early-exit on first failure.
                 */
                if (!log_samples_)
                {
                    rejection_reason =
                        first_reason;

                    return false;
                }
            }
        }

        if (!failed_samples.empty())
        {
            std::ostringstream message;

            message << first_reason;

            if (
                log_samples_ &&
                failed_samples.size() > 1
            )
            {
                message
                    << "; additional failing samples: ";

                bool first = true;

                for (const std::size_t sample :
                     failed_samples)
                {
                    if (sample ==
                        failed_samples.front())
                    {
                        continue;
                    }

                    if (!first)
                    {
                        message << ", ";
                    }

                    first = false;

                    message << sample;
                }
            }

            rejection_reason =
                message.str();

            RCLCPP_WARN(
                this->get_logger(),
                "%s",
                rejection_reason.c_str()
            );

            return false;
        }

        return true;
    }


    trajectory_msgs::msg::JointTrajectory
    buildTrajectory(
        const moveit::core::JointModelGroup* group,
        const ComputePtp::Request& request,
        const std::vector<double>& current,
        const std::vector<double>& target)
    {
        trajectory_msgs::msg::JointTrajectory trajectory;

        trajectory.joint_names =
            group->getVariableNames();

        const std::size_t segments =
            segmentCount(
                request,
                current,
                target
            );

        trajectory.points.reserve(
            segments + 1
        );

        for (std::size_t index = 0;
             index <= segments;
             ++index)
        {
            const double t =
                static_cast<double>(index) /
                static_cast<double>(segments);

            trajectory_msgs::msg::
                JointTrajectoryPoint point;

            point.positions.resize(
                current.size()
            );

            point.velocities.assign(
                current.size(),
                0.0
            );

            point.accelerations.assign(
                current.size(),
                0.0
            );

            for (std::size_t joint = 0;
                 joint < current.size();
                 ++joint)
            {
                point.positions[joint] =
                    current[joint] +
                    t *
                    (
                        target[joint] -
                        current[joint]
                    );
            }

            trajectory.points.push_back(
                std::move(point)
            );
        }

        return trajectory;
    }


    void computePtp(
        const std::shared_ptr<
            ComputePtp::Request> request,
        std::shared_ptr<
            ComputePtp::Response> response)
    {
        const double total_started =
            steadySeconds();

        response->success = false;
        response->noop = false;
        response->error_code =
            ERROR_INVALID_REQUEST;

        response->ik_attempts_made = 0;
        response->ik_solutions_found = 0;
        response->candidates_validated = 0;
        response->ik_time_ms = 0.0;
        response->validation_time_ms = 0.0;
        response->total_time_ms = 0.0;

        if (!model_)
        {
            fail(
                response,
                ERROR_INVALID_REQUEST,
                "robot model unavailable"
            );

            return;
        }

        const auto* group =
            model_->getJointModelGroup(
                request->group_name
            );

        if (!group)
        {
            fail(
                response,
                ERROR_INVALID_REQUEST,
                "unknown planning group: " +
                request->group_name
            );

            return;
        }

        if (!model_->hasLinkModel(
                request->link_name))
        {
            fail(
                response,
                ERROR_INVALID_REQUEST,
                "unknown link: " +
                request->link_name
            );

            return;
        }

        moveit::core::RobotState state(
            model_
        );

        std::vector<double>
            current_positions;

        if (!loadStartState(
                *request,
                state,
                group,
                current_positions))
        {
            fail(
                response,
                ERROR_INVALID_REQUEST,
                "invalid or incomplete start state"
            );

            return;
        }

        /*
         * Get the actual current EE orientation from FK.
         */
        const auto& start_tf =
            state.getGlobalLinkTransform(
                request->link_name
            );

        Eigen::Quaterniond
            start_orientation(
                start_tf.rotation()
            );

        start_orientation.normalize();

        const Eigen::Quaterniond
            target_orientation =
                poseQuaternion(
                    request->target_pose
                );

        const double endpoint_orientation_delta =
            quaternionAngleDeg(
                start_orientation,
                target_orientation
            );

        const bool orientation_locked =
            endpoint_orientation_delta <=
            request->orientation_lock_tolerance_deg;

        std::vector<Candidate> candidates;

        const double ik_started =
            steadySeconds();

        collectCandidates(
            state,
            group,
            *request,
            current_positions,
            candidates,
            *response
        );

        response->ik_time_ms =
            (
                steadySeconds() -
                ik_started
            ) * 1000.0;

        if (candidates.empty())
        {
            fail(
                response,
                ERROR_IK_FAILED,
                "no IK candidates found"
            );

            response->total_time_ms =
                (
                    steadySeconds() -
                    total_started
                ) * 1000.0;

            return;
        }

        /*
         * Cheapest/nearest configuration first.
         *
         * As soon as one is safe, stop. This is important for
         * planning performance.
         */
        const double validation_started =
            steadySeconds();

        const Candidate*
            selected = nullptr;

        std::string
            last_rejection;

        for (const auto& candidate :
             candidates)
        {
            ++response->candidates_validated;

            state.setJointGroupPositions(
                group,
                current_positions
            );

            state.update();

            std::string reason;

            if (validateCandidate(
                    state,
                    group,
                    *request,
                    current_positions,
                    candidate.positions,
                    start_orientation,
                    orientation_locked,
                    reason))
            {
                selected =
                    &candidate;

                break;
            }

            last_rejection =
                reason;

            RCLCPP_DEBUG(
                this->get_logger(),
                "PTP candidate rejected: cost=%.6f reason=%s",
                candidate.cost,
                reason.c_str()
            );
        }

        response->validation_time_ms =
            (
                steadySeconds() -
                validation_started
            ) * 1000.0;

        if (!selected)
        {
            std::ostringstream message;

            message
                << "no safe PTP IK candidate";

            if (!last_rejection.empty())
            {
                message
                    << ": "
                    << last_rejection;
            }

            fail(
                response,
                ERROR_COLLISION,
                message.str()
            );

            response->total_time_ms =
                (
                    steadySeconds() -
                    total_started
                ) * 1000.0;

            return;
        }

        const double max_delta =
            maxAbsDelta(
                current_positions,
                selected->positions
            );

        if (max_delta <= 0.001)
        {
            response->success = true;
            response->noop = true;
            response->error_code =
                ERROR_NONE;

            response->message =
                "target already reached";

            response->total_time_ms =
                (
                    steadySeconds() -
                    total_started
                ) * 1000.0;

            return;
        }

        response->trajectory =
            buildTrajectory(
                group,
                *request,
                current_positions,
                selected->positions
            );

        response->success = true;
        response->noop = false;
        response->error_code =
            ERROR_NONE;

        std::ostringstream message;

        message
            << "PTP selected safe IK branch"
            << " candidates="
            << candidates.size()
            << " validated="
            << response->candidates_validated
            << " orientation_locked="
            << (
                orientation_locked
                    ? "true"
                    : "false"
            )
            << " endpoint_orientation_delta_deg="
            << endpoint_orientation_delta
            << " cost="
            << selected->cost;

        response->message =
            message.str();

        response->total_time_ms =
            (
                steadySeconds() -
                total_started
            ) * 1000.0;

        RCLCPP_INFO(
            this->get_logger(),
            "PTP success: %s total=%.2fms IK=%.2fms validation=%.2fms",
            response->message.c_str(),
            response->total_time_ms,
            response->ik_time_ms,
            response->validation_time_ms
        );
    }
};


int main(
    int argc,
    char** argv)
{
    rclcpp::init(
        argc,
        argv
    );

    auto node =
        std::make_shared<
            PtpHelperNode
        >();

    try
    {
        node->initialize();
    }
    catch (const std::exception& exc)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "PTP helper initialization failed: %s",
            exc.what()
        );

        rclcpp::shutdown();

        return 1;
    }

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}