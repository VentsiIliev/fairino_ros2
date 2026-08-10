#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include "erob_moveit_runtime/srv/compute_linked_lin.hpp"
#include "erob_moveit_runtime/trajectory_validation.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>

using ComputeLinkedLin = erob_moveit_runtime::srv::ComputeLinkedLin;

namespace
{

constexpr int32_t ERROR_NONE = 1;
constexpr int32_t ERROR_INVALID_REQUEST = -1;
constexpr int32_t ERROR_IK_FAILED = -2;
constexpr int32_t ERROR_FK_ERROR = -3;
constexpr int32_t ERROR_JOINT_STEP = -4;
constexpr int32_t ERROR_JOINT_SPAN = -5;
constexpr int32_t ERROR_TRAJECTORY_VALIDATION = -6;
constexpr int32_t ERROR_COLLISION = -10;

double steadySeconds()
{
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(
        Clock::now().time_since_epoch()).count();
}

double nearestEquivalentAngle(double reference, double value)
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

Eigen::Isometry3d transformMsgToEigen(
    const geometry_msgs::msg::Transform& transform)
{
    Eigen::Quaterniond rotation(
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z);
    if (rotation.norm() <= 1e-12)
    {
        rotation = Eigen::Quaterniond::Identity();
    }
    rotation.normalize();

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = rotation.toRotationMatrix();
    result.translation() = Eigen::Vector3d(
        transform.translation.x,
        transform.translation.y,
        transform.translation.z);
    return result;
}

Eigen::Isometry3d poseMsgToEigen(
    const geometry_msgs::msg::Pose& pose)
{
    Eigen::Quaterniond rotation(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    if (rotation.norm() <= 1e-12)
    {
        rotation = Eigen::Quaterniond::Identity();
    }
    rotation.normalize();

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = rotation.toRotationMatrix();
    result.translation() = Eigen::Vector3d(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    return result;
}

geometry_msgs::msg::Pose eigenToPoseMsg(
    const Eigen::Isometry3d& transform)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();

    Eigen::Quaterniond rotation(transform.rotation());
    rotation.normalize();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
}

double positionErrorMm(
    const geometry_msgs::msg::Pose& expected,
    const Eigen::Isometry3d& actual)
{
    const Eigen::Vector3d expected_position(
        expected.position.x,
        expected.position.y,
        expected.position.z);
    return (expected_position - actual.translation()).norm() * 1000.0;
}

double orientationErrorDeg(
    const geometry_msgs::msg::Pose& expected,
    const Eigen::Isometry3d& actual)
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
    const double dot = std::clamp(std::abs(expected_q.dot(actual_q)), 0.0, 1.0);
    return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

double maxAbsDelta(
    const std::vector<double>& a,
    const std::vector<double>& b)
{
    double result = 0.0;
    const std::size_t count = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        result = std::max(result, std::abs(a[i] - b[i]));
    }
    return result;
}

double computeJointSpan(
    const std::vector<double>& start,
    const std::vector<double>& current)
{
    double result = 0.0;
    const std::size_t count = std::min(start.size(), current.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        result = std::max(result, std::abs(current[i] - start[i]));
    }
    return result;
}

}  // namespace

class LinkedLinHelperNode : public rclcpp::Node
{
public:
    LinkedLinHelperNode()
        : Node("linked_lin_helper")
    {
        RCLCPP_INFO(this->get_logger(), "Linked LIN helper starting...");
    }

    void initialize()
    {
        auto node_ptr = shared_from_this();

        loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            node_ptr);
        model_ = loader_->getModel();
        if (!model_)
        {
            throw std::runtime_error(
                "Linked LIN helper failed to load robot model");
        }

        planning_scene_monitor_ =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node_ptr,
                loader_,
                "linked_lin_planning_scene_monitor");
        if (!planning_scene_monitor_->getPlanningScene())
        {
            throw std::runtime_error(
                "Linked LIN helper failed to create PlanningSceneMonitor");
        }

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        if (!planning_scene_monitor_->requestPlanningSceneState(
                "/get_planning_scene"))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Could not request initial planning scene; "
                "continuing with monitored scene updates");
        }

        advertiseService();
        RCLCPP_INFO(this->get_logger(), "Linked LIN helper ready");
    }

private:
    rclcpp::Service<ComputeLinkedLin>::SharedPtr service_;
    robot_model_loader::RobotModelLoaderPtr loader_;
    moveit::core::RobotModelPtr model_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    void advertiseService()
    {
        service_ = this->create_service<ComputeLinkedLin>(
            "/compute_linked_lin",
            [this](
                const std::shared_ptr<ComputeLinkedLin::Request> request,
                std::shared_ptr<ComputeLinkedLin::Response> response)
            {
                handleRequest(request, response);
            });

        RCLCPP_INFO(
            this->get_logger(),
            "Service '/compute_linked_lin' created");
    }

    void fail(
        const std::shared_ptr<ComputeLinkedLin::Response>& response,
        int32_t error_code,
        const std::string& message,
        uint32_t failed_index = std::numeric_limits<uint32_t>::max())
    {
        response->success = false;
        response->error_code = error_code;
        response->message = message;
        response->failed_index = failed_index;
        RCLCPP_WARN(
            this->get_logger(),
            "Linked LIN rejected: %s",
            message.c_str());
    }

    std::vector<double> seedGroupPositions(
        const moveit::core::JointModelGroup* group,
        const sensor_msgs::msg::JointState& seed_state,
        const std::shared_ptr<ComputeLinkedLin::Response>& response)
    {
        std::vector<double> result;
        result.reserve(group->getVariableNames().size());

        for (const auto& name : group->getVariableNames())
        {
            auto it = std::find(seed_state.name.begin(), seed_state.name.end(), name);
            if (it == seed_state.name.end())
            {
                fail(response, ERROR_INVALID_REQUEST, "seed_state missing joint " + name);
                return {};
            }
            const auto index = static_cast<std::size_t>(
                std::distance(seed_state.name.begin(), it));
            if (index >= seed_state.position.size())
            {
                fail(response, ERROR_INVALID_REQUEST, "seed_state position missing for joint " + name);
                return {};
            }
            result.push_back(seed_state.position[index]);
        }

        return result;
    }

    geometry_msgs::msg::Pose targetLinkPose(
        const ComputeLinkedLin::Request& request,
        const geometry_msgs::msg::Pose& pose) const
    {
        Eigen::Isometry3d target = poseMsgToEigen(pose);
        if (request.use_workobject_transform)
        {
            target = transformMsgToEigen(request.workobject_transform) * target;
        }

        /*
         * Request poses describe the active tool TCP. MoveIt IK expects the
         * requested link pose, so convert TCP target to link target.
         */
        target = target * transformMsgToEigen(request.tool_transform).inverse();
        return eigenToPoseMsg(target);
    }

    bool validateSolvedPoint(
        const ComputeLinkedLin::Request& request,
        const geometry_msgs::msg::Pose& expected_pose,
        moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* group,
        const std::vector<double>& previous,
        const std::vector<double>& start,
        const std::vector<double>& current,
        std::size_t index,
        const std::shared_ptr<ComputeLinkedLin::Response>& response)
    {
        const double step = maxAbsDelta(previous, current);
        response->max_joint_step_rad =
            std::max(response->max_joint_step_rad, step);
        if (request.max_joint_step_rad > 0.0
            && step > request.max_joint_step_rad)
        {
            std::ostringstream out;
            out
                << "joint step "
                << step
                << " rad exceeds limit "
                << request.max_joint_step_rad
                << " at linked LIN pose "
                << index;
            fail(response, ERROR_JOINT_STEP, out.str(), static_cast<uint32_t>(index));
            return false;
        }

        double max_span = 0.0;
        double max_endpoint = 0.0;
        for (std::size_t joint = 0; joint < current.size(); ++joint)
        {
            max_span = std::max(max_span, std::abs(current[joint] - start[joint]));
            max_endpoint = std::max(max_endpoint, std::abs(current[joint] - start[joint]));
        }
        response->max_joint_span_rad =
            std::max(response->max_joint_span_rad, max_span);
        response->max_endpoint_delta_rad =
            std::max(response->max_endpoint_delta_rad, max_endpoint);
        if (request.max_joint_span_rad > 0.0
            && max_span > request.max_joint_span_rad)
        {
            fail(response, ERROR_JOINT_SPAN, "linked LIN joint span limit exceeded", static_cast<uint32_t>(index));
            return false;
        }
        if (request.max_endpoint_delta_rad > 0.0
            && max_endpoint > request.max_endpoint_delta_rad)
        {
            fail(response, ERROR_JOINT_SPAN, "linked LIN endpoint delta limit exceeded", static_cast<uint32_t>(index));
            return false;
        }

        const auto& actual_link_pose =
            state.getGlobalLinkTransform(request.link_name);
        Eigen::Isometry3d actual_tcp =
            actual_link_pose * transformMsgToEigen(request.tool_transform);
        const double position_error =
            positionErrorMm(expected_pose, actual_tcp);
        const double orientation_error =
            orientationErrorDeg(expected_pose, actual_tcp);
        response->max_fk_position_error_mm =
            std::max(response->max_fk_position_error_mm, position_error);
        response->max_fk_orientation_error_deg =
            std::max(response->max_fk_orientation_error_deg, orientation_error);
        if (request.fk_position_tolerance_mm > 0.0
            && position_error > request.fk_position_tolerance_mm)
        {
            fail(response, ERROR_FK_ERROR, "linked LIN FK position tolerance exceeded", static_cast<uint32_t>(index));
            return false;
        }
        if (request.fk_orientation_tolerance_deg > 0.0
            && orientation_error > request.fk_orientation_tolerance_deg)
        {
            fail(response, ERROR_FK_ERROR, "linked LIN FK orientation tolerance exceeded", static_cast<uint32_t>(index));
            return false;
        }

        if (request.avoid_collisions)
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(
                planning_scene_monitor_);
            if (!scene)
            {
                fail(response, ERROR_INVALID_REQUEST, "PlanningScene is unavailable", static_cast<uint32_t>(index));
                return false;
            }
            if (scene->isStateColliding(state, request.group_name, false))
            {
                fail(response, ERROR_COLLISION, "linked LIN collision detected", static_cast<uint32_t>(index));
                return false;
            }
        }

        (void)group;
        return true;
    }

    void handleRequest(
        const std::shared_ptr<ComputeLinkedLin::Request> request,
        std::shared_ptr<ComputeLinkedLin::Response> response)
    {
        const double total_started_s = steadySeconds();

        response->success = false;
        response->error_code = ERROR_INVALID_REQUEST;
        response->message = "";
        response->requested_pose_count =
            static_cast<uint32_t>(request->poses.size());
        response->solved_pose_count = 0;
        response->failed_index = std::numeric_limits<uint32_t>::max();
        response->max_fk_position_error_mm = 0.0;
        response->max_fk_orientation_error_deg = 0.0;
        response->max_joint_step_rad = 0.0;
        response->max_joint_span_rad = 0.0;
        response->max_endpoint_delta_rad = 0.0;
        response->planning_time_s = 0.0;
        response->validation_time_s = 0.0;
        response->total_time_s = 0.0;

        if (request->poses.empty())
        {
            fail(response, ERROR_INVALID_REQUEST, "linked LIN request has no poses");
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }
        if (request->group_name.empty() || request->link_name.empty())
        {
            fail(response, ERROR_INVALID_REQUEST, "linked LIN request missing group/link name");
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }

        const auto* group =
            model_->getJointModelGroup(request->group_name);
        if (group == nullptr)
        {
            fail(response, ERROR_INVALID_REQUEST, "unknown MoveIt group " + request->group_name);
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }
        if (!model_->hasLinkModel(request->link_name))
        {
            fail(response, ERROR_INVALID_REQUEST, "unknown MoveIt link " + request->link_name);
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }
        const moveit::core::LinkModel* link_model =
            model_->getLinkModel(request->link_name);

        moveit::core::RobotState state(model_);
        state.setToDefaultValues();
        const std::vector<double> start =
            seedGroupPositions(group, request->seed_state, response);
        if (start.empty())
        {
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }
        state.setJointGroupPositions(group, start);
        state.update();
        if (!state.satisfiesBounds(group))
        {
            fail(response, ERROR_INVALID_REQUEST, "seed_state violates joint bounds");
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }

        response->trajectory.joint_trajectory.joint_names =
            group->getVariableNames();

        EigenSTL::vector_Isometry3d link_waypoints;
        link_waypoints.reserve(request->poses.size());
        for (const auto& pose : request->poses)
        {
            Eigen::Isometry3d target = poseMsgToEigen(pose);
            if (request->use_workobject_transform)
            {
                target = transformMsgToEigen(request->workobject_transform) * target;
            }
            target = target * transformMsgToEigen(request->tool_transform).inverse();
            link_waypoints.push_back(target);
        }

        const double planning_started_s = steadySeconds();
        std::vector<std::shared_ptr<moveit::core::RobotState>> path;
        path.reserve(request->poses.size());

        moveit::core::GroupStateValidityCallbackFn validity_cb =
            [&](moveit::core::RobotState* candidate_state,
                const moveit::core::JointModelGroup*,
                const double*) -> bool
            {
                if (request->avoid_collisions)
                {
                    planning_scene_monitor::LockedPlanningSceneRO scene(
                        planning_scene_monitor_);
                    if (!scene)
                    {
                        return false;
                    }
                    if (scene->isStateColliding(
                            *candidate_state,
                            request->group_name,
                            false))
                    {
                        return false;
                    }
                }
                return true;
            };

        const double eef_step = std::max(0.001, request->cartesian_step_m);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        const double fraction =
            moveit::core::CartesianInterpolator::computeCartesianPath(
                &state,
                group,
                path,
                link_model,
                link_waypoints,
                true,
                moveit::core::MaxEEFStep(eef_step),
                moveit::core::JumpThreshold::disabled(),
                validity_cb);
#pragma GCC diagnostic pop
        response->planning_time_s = steadySeconds() - planning_started_s;

        if (fraction < 0.999 || path.empty())
        {
            std::ostringstream out;
            out
                << "linked LIN Cartesian path incomplete: fraction="
                << fraction
                << " points="
                << path.size();
            fail(response, ERROR_IK_FAILED, out.str());
            response->solved_pose_count = static_cast<uint32_t>(
                std::floor(fraction * static_cast<double>(request->poses.size())));
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }

        response->trajectory.joint_trajectory.points.reserve(path.size());
        std::vector<double> previous = start;
        const double validation_started_s = steadySeconds();
        for (std::size_t index = 0; index < path.size(); ++index)
        {
            auto& path_state = *path[index];
            std::vector<double> current;
            path_state.copyJointGroupPositions(group, current);
            if (current.size() != previous.size())
            {
                fail(response, ERROR_INVALID_REQUEST, "linked LIN Cartesian path joint-size mismatch", static_cast<uint32_t>(index));
                response->validation_time_s += steadySeconds() - validation_started_s;
                response->total_time_s = steadySeconds() - total_started_s;
                return;
            }

            const double step = maxAbsDelta(previous, current);
            response->max_joint_step_rad =
                std::max(response->max_joint_step_rad, step);
            if (request->max_joint_step_rad > 0.0
                && step > request->max_joint_step_rad)
            {
                std::ostringstream out;
                out
                    << "joint step "
                    << step
                    << " rad exceeds limit "
                    << request->max_joint_step_rad
                    << " at linked LIN path point "
                    << index;
                fail(response, ERROR_JOINT_STEP, out.str(), static_cast<uint32_t>(index));
                response->validation_time_s += steadySeconds() - validation_started_s;
                response->total_time_s = steadySeconds() - total_started_s;
                return;
            }

            const double span = computeJointSpan(start, current);
            response->max_joint_span_rad =
                std::max(response->max_joint_span_rad, span);
            response->max_endpoint_delta_rad =
                std::max(response->max_endpoint_delta_rad, span);
            if (request->max_joint_span_rad > 0.0
                && span > request->max_joint_span_rad)
            {
                fail(response, ERROR_JOINT_SPAN, "linked LIN joint span limit exceeded", static_cast<uint32_t>(index));
                response->validation_time_s += steadySeconds() - validation_started_s;
                response->total_time_s = steadySeconds() - total_started_s;
                return;
            }
            if (request->max_endpoint_delta_rad > 0.0
                && span > request->max_endpoint_delta_rad)
            {
                fail(response, ERROR_JOINT_SPAN, "linked LIN endpoint delta limit exceeded", static_cast<uint32_t>(index));
                response->validation_time_s += steadySeconds() - validation_started_s;
                response->total_time_s = steadySeconds() - total_started_s;
                return;
            }

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = current;
            const double dt = std::max(0.05, request->cartesian_step_m > 0.0 ? request->cartesian_step_m : 0.1);
            const double time_s = dt * static_cast<double>(index);
            point.time_from_start.sec = static_cast<int32_t>(std::floor(time_s));
            point.time_from_start.nanosec =
                static_cast<uint32_t>((time_s - std::floor(time_s)) * 1e9);
            response->trajectory.joint_trajectory.points.push_back(point);

            previous = current;
        }
        response->validation_time_s += steadySeconds() - validation_started_s;
        response->solved_pose_count =
            static_cast<uint32_t>(request->poses.size());

        const auto validation =
            erob_moveit_runtime::validateJointTrajectory(
                response->trajectory.joint_trajectory);
        if (!validation.ok)
        {
            fail(
                response,
                ERROR_TRAJECTORY_VALIDATION,
                "linked LIN trajectory validation failed: " + validation.reason);
            response->total_time_s = steadySeconds() - total_started_s;
            return;
        }

        response->success = true;
        response->error_code = ERROR_NONE;
        response->message = "ok";
        response->total_time_s = steadySeconds() - total_started_s;

        RCLCPP_INFO(
            this->get_logger(),
            "Linked LIN success: poses=%u points=%zu total=%.3fs planning=%.3fs validation=%.3fs",
            response->requested_pose_count,
            response->trajectory.joint_trajectory.points.size(),
            response->total_time_s,
            response->planning_time_s,
            response->validation_time_s);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinkedLinHelperNode>();

    try
    {
        node->initialize();
        rclcpp::spin(node);
    }
    catch (const std::exception& exc)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "Linked LIN helper initialization failed: %s",
            exc.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
