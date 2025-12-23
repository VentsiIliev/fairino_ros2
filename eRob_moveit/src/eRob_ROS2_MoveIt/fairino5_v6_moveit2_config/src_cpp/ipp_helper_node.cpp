#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "fairino5_v6_moveit2_config/srv/apply_ipp.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

using ApplyIPP = fairino5_v6_moveit2_config::srv::ApplyIPP;

class IPPHelperNode : public rclcpp::Node
{
public:
    IPPHelperNode() : Node("ipp_helper_node")
    {
        RCLCPP_INFO(this->get_logger(), "IPP Helper Node started");

        service_ = this->create_service<ApplyIPP>(
            "/apply_ipp",
            std::bind(&IPPHelperNode::applyIPP, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<ApplyIPP>::SharedPtr service_;

    void applyIPP(const std::shared_ptr<ApplyIPP::Request> request,
                  std::shared_ptr<ApplyIPP::Response> response)
    {
        auto node_ptr = shared_from_this();
        robot_model_loader::RobotModelLoader loader(node_ptr);
        auto kinematic_model = loader.getModel();

        moveit::core::RobotState robot_state(kinematic_model);
        robot_state.setToDefaultValues();

        robot_trajectory::RobotTrajectory rt(kinematic_model, "fairino5_v6_group");
        rt.setRobotTrajectoryMsg(robot_state, request->trajectory);

        // TOTG time parameterization with velocity and acceleration scaling
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;

        double max_vel_scaling = request->max_velocity_scaling;
        double max_acc_scaling = request->max_acceleration_scaling;

        // Clamp to valid range [0.0, 1.0]
        max_vel_scaling = std::max(0.0, std::min(1.0, max_vel_scaling));
        max_acc_scaling = std::max(0.0, std::min(1.0, max_acc_scaling));

        RCLCPP_INFO(this->get_logger(), "Applying TOTG with vel_scale=%.2f, acc_scale=%.2f",
                    max_vel_scaling, max_acc_scaling);

        bool success = totg.computeTimeStamps(rt, max_vel_scaling, max_acc_scaling);

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "TOTG Time Parameterization failed!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "TOTG Time Parameterization succeeded");
            moveit_msgs::msg::RobotTrajectory robot_traj_msg;
            rt.getRobotTrajectoryMsg(robot_traj_msg);
            response->trajectory = robot_traj_msg.joint_trajectory;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IPPHelperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
