#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "fairino5_v6_moveit2_config/srv/apply_ipp.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using ApplyIPP = fairino5_v6_moveit2_config::srv::ApplyIPP;

class IPPHelperNode : public rclcpp::Node
{
public:
    IPPHelperNode() : Node("ipp_helper")
    {
        RCLCPP_INFO(this->get_logger(), "📡 IPP Helper Node starting...");

        service_ = this->create_service<ApplyIPP>(
            "/apply_ipp",
            std::bind(&IPPHelperNode::applyIPP, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "✅ Service '/apply_ipp' created");
    }

    void initialize()
    {
        // Load robot model ONCE at startup (must be called after shared_ptr is created)
        auto node_ptr = shared_from_this();

        RCLCPP_INFO(this->get_logger(), "⏳ Loading robot model...");
        loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_ptr);
        kinematic_model_ = loader_->getModel();

        if (!kinematic_model_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model!");
            throw std::runtime_error("Robot model load failed");
        }

        // Create and cache robot state to avoid re-initialization
        robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        robot_state_->setToDefaultValues();

        RCLCPP_INFO(this->get_logger(), "✅ Robot model and state cached successfully");
        RCLCPP_INFO(this->get_logger(), "✅ IPP service '/apply_ipp' is now ready");
    }

private:
    rclcpp::Service<ApplyIPP>::SharedPtr service_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;  // Keep alive!
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;  // Cached state

    void applyIPP(const std::shared_ptr<ApplyIPP::Request> request,
                  std::shared_ptr<ApplyIPP::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "📥 Received TOTG request with %zu points",
                    request->trajectory.points.size());

        // Use cached robot state (no re-initialization!)
        robot_trajectory::RobotTrajectory rt(kinematic_model_, "fairino5_v6_group");

        // Set trajectory from request - this populates joint positions
        rt.setRobotTrajectoryMsg(*robot_state_, request->trajectory);

        // Clamp scaling factors to valid range [0.0, 1.0]
        double max_vel_scaling = std::max(0.0, std::min(1.0, request->max_velocity_scaling));
        double max_acc_scaling = std::max(0.0, std::min(1.0, request->max_acceleration_scaling));

        RCLCPP_INFO(this->get_logger(), "⚙️  Applying TOTG with vel_scale=%.2f, acc_scale=%.2f",
                    max_vel_scaling, max_acc_scaling);

        // TOTG time parameterization
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool success = totg.computeTimeStamps(rt, max_vel_scaling, max_acc_scaling);

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ TOTG Time Parameterization FAILED - returning empty trajectory");
            // Return empty trajectory on failure
            response->trajectory = moveit_msgs::msg::RobotTrajectory();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "✅ TOTG Time Parameterization succeeded");

            // Convert back to RobotTrajectory message (deep-copy safe!)
            rt.getRobotTrajectoryMsg(response->trajectory);

            // Verify result
            size_t num_points = response->trajectory.joint_trajectory.points.size();
            RCLCPP_INFO(this->get_logger(), "🔍 response->trajectory has %zu points", num_points);

            if (num_points > 0)
            {
                auto last_point = response->trajectory.joint_trajectory.points.back();
                double total_time = last_point.time_from_start.sec +
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
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IPPHelperNode>();

    try {
        node->initialize();  // Initialize after shared_ptr is created
        RCLCPP_INFO(node->get_logger(), "✅ IPP Helper fully initialized and ready");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ Failed to initialize IPP Helper: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
