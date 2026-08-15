#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "erob_moveit_runtime/srv/apply_ipp.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using ApplyIPP = erob_moveit_runtime::srv::ApplyIPP;

namespace
{

std::vector<std::string> sortedNames(const std::vector<std::string>& names)
{
    auto sorted = names;
    std::sort(sorted.begin(), sorted.end());
    return sorted;
}

}  // namespace

class IPPHelperNode : public rclcpp::Node
{
public:
    IPPHelperNode() : Node("ipp_helper")
    {
        RCLCPP_INFO(this->get_logger(), "IPP Helper Node starting...");
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

        const auto group_names = kinematic_model_->getJointModelGroupNames();
        if (group_names.empty())
        {
            throw std::runtime_error("Robot model has no planning groups!");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Robot model cached with %zu planning groups; group is resolved per request",
            group_names.size());

        advertiseService();
        RCLCPP_INFO(this->get_logger(), "IPP service '/apply_ipp' is now ready");
    }

private:
    rclcpp::Service<ApplyIPP>::SharedPtr service_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;

    std::string resolvePlanningGroup(
        const trajectory_msgs::msg::JointTrajectory& trajectory) const
    {
        const auto requested_names = sortedNames(trajectory.joint_names);
        if (requested_names.empty())
        {
            throw std::invalid_argument("trajectory has no joint names");
        }

        for (const auto& group_name : kinematic_model_->getJointModelGroupNames())
        {
            const auto* group = kinematic_model_->getJointModelGroup(group_name);
            if (!group)
            {
                continue;
            }

            if (sortedNames(group->getVariableNames()) == requested_names)
            {
                return group_name;
            }
        }

        std::string joined;
        for (const auto& name : trajectory.joint_names)
        {
            if (!joined.empty())
            {
                joined += ", ";
            }
            joined += name;
        }
        throw std::invalid_argument(
            "no MoveIt planning group exactly matches trajectory joints [" + joined + "]");
    }

    void advertiseService()
    {
        service_ = this->create_service<ApplyIPP>(
            "/apply_ipp",
            std::bind(&IPPHelperNode::applyIPP, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Service '/apply_ipp' created");
    }

    void applyIPP(const std::shared_ptr<ApplyIPP::Request> request,
                  std::shared_ptr<ApplyIPP::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "📥 Received TOTG request with %zu points",
                    request->trajectory.points.size());

        std::string planning_group;
        try
        {
            planning_group = resolvePlanningGroup(request->trajectory);
        }
        catch (const std::exception& exc)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ TOTG request rejected: %s", exc.what());
            response->trajectory = moveit_msgs::msg::RobotTrajectory();
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Using planning group '%s' for %zu trajectory joints",
            planning_group.c_str(),
            request->trajectory.joint_names.size());

        robot_trajectory::RobotTrajectory rt(kinematic_model_, planning_group);
        rt.setRobotTrajectoryMsg(*robot_state_, request->trajectory);

        const double max_vel_scaling =
            std::max(0.0, std::min(1.0, request->max_velocity_scaling));
        const double max_acc_scaling =
            std::max(0.0, std::min(1.0, request->max_acceleration_scaling));

        RCLCPP_INFO(this->get_logger(), "⚙️  Applying TOTG with vel_scale=%.2f, acc_scale=%.2f",
                    max_vel_scaling, max_acc_scaling);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        const bool success =
            totg.computeTimeStamps(rt, max_vel_scaling, max_acc_scaling);

        if (!success)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "❌ TOTG Time Parameterization FAILED - returning empty trajectory");
            response->trajectory = moveit_msgs::msg::RobotTrajectory();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ TOTG Time Parameterization succeeded");
        rt.getRobotTrajectoryMsg(response->trajectory);

        const size_t num_points = response->trajectory.joint_trajectory.points.size();
        RCLCPP_INFO(this->get_logger(), "🔍 response->trajectory has %zu points", num_points);

        if (num_points == 0)
        {
            return;
        }

        const auto& last_point = response->trajectory.joint_trajectory.points.back();
        const double total_time = last_point.time_from_start.sec +
                                  last_point.time_from_start.nanosec * 1e-9;

        if (total_time > 0.0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "📤 Returning %zu points, total time: %.2fs",
                        num_points, total_time);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "⚠️  TOTG succeeded but timestamps are zero - might not work correctly");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IPPHelperNode>();

    try
    {
        node->initialize();
        RCLCPP_INFO(node->get_logger(), "✅ IPP Helper fully initialized and ready");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "❌ Failed to initialize IPP Helper: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
