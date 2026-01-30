#include <memory>
#include <unordered_map>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "fairino5_v6_moveit2_config/srv/apply_ipp.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Reuse the same service definition as IPP (ApplyIPP)
using ApplyRuckig = fairino5_v6_moveit2_config::srv::ApplyIPP;

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

    // Log joint names
    std::string joint_names_str = "Joints: ";
    for (const auto& name : jt.joint_names)
    {
        joint_names_str += name + " ";
    }
    RCLCPP_INFO(logger, "%s", joint_names_str.c_str());

    // Log each point
    for (size_t i = 0; i < jt.points.size(); ++i)
    {
        const auto& p = jt.points[i];
        double t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;

        // Always log time and position summary
        if (!verbose && jt.points.size() > 10 && i > 2 && i < jt.points.size() - 3)
        {
            if (i == 3) RCLCPP_INFO(logger, "  ... (%zu points omitted, use verbose for full log) ...",
                                     jt.points.size() - 6);
            continue;
        }

        // Position string
        std::string pos_str = "";
        for (size_t j = 0; j < p.positions.size(); ++j)
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "%7.4f", p.positions[j]);
            pos_str += buf;
            if (j < p.positions.size() - 1) pos_str += ", ";
        }

        // Velocity string
        std::string vel_str = "";
        for (size_t j = 0; j < p.velocities.size(); ++j)
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "%7.4f", p.velocities[j]);
            vel_str += buf;
            if (j < p.velocities.size() - 1) vel_str += ", ";
        }

        // Acceleration string
        std::string acc_str = "";
        for (size_t j = 0; j < p.accelerations.size(); ++j)
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "%7.4f", p.accelerations[j]);
            acc_str += buf;
            if (j < p.accelerations.size() - 1) acc_str += ", ";
        }

        RCLCPP_INFO(logger, "Point[%2zu] t=%6.3fs", i, t);
        RCLCPP_INFO(logger, "  pos: [%s]", pos_str.c_str());
        RCLCPP_INFO(logger, "  vel: [%s]", vel_str.c_str());
        RCLCPP_INFO(logger, "  acc: [%s]", acc_str.c_str());
    }

    // Summary stats
    if (jt.points.size() > 0)
    {
        double total_time = jt.points.back().time_from_start.sec +
                           jt.points.back().time_from_start.nanosec * 1e-9;

        // Find max velocity and acceleration
        double max_vel = 0.0, max_acc = 0.0;
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

/**
 * Validate a trajectory for execution safety.
 * Returns false if trajectory is invalid (and should NOT be executed).
 */
bool validateTrajectory(
    const moveit_msgs::msg::RobotTrajectory& rt_msg,
    const moveit::core::RobotModelPtr& model,
    rclcpp::Logger logger,
    double max_duration = 30.0)  // Configurable max duration
{
    const auto& jt = rt_msg.joint_trajectory;

    // 1️⃣ Minimum points
    if (jt.points.size() < 2)
    {
        RCLCPP_ERROR(logger, "❌ Trajectory has <2 points");
        return false;
    }

    // 2️⃣ Joint count consistency
    const size_t num_joints = jt.joint_names.size();
    if (num_joints == 0)
    {
        RCLCPP_ERROR(logger, "❌ No joint names in trajectory");
        return false;
    }

    double last_time = -1.0;

    // Joint limits from model
    std::unordered_map<std::string, moveit::core::VariableBounds> limits;
    for (const auto* jm : model->getJointModels())
    {
        if (jm->getVariableCount() == 1)
            limits[jm->getName()] = jm->getVariableBounds()[0];
    }

    for (size_t i = 0; i < jt.points.size(); ++i)
    {
        const auto& p = jt.points[i];

        // 3️⃣ Size checks
        if (p.positions.size() != num_joints)
        {
            RCLCPP_ERROR(logger, "❌ Point %zu has wrong position size", i);
            return false;
        }

        // 4️⃣ Time monotonicity
        double t = p.time_from_start.sec +
                   p.time_from_start.nanosec * 1e-9;

        if (i > 0 && t <= last_time)
        {
            RCLCPP_ERROR(logger,
                "❌ Non-monotonic timing at point %zu (%.6f <= %.6f)",
                i, t, last_time);
            return false;
        }

        // 5️⃣ Zero-time segment protection (check BEFORE updating last_time)
        if (i > 0 && (t - last_time) < 1e-6)
        {
            RCLCPP_ERROR(logger,
                "❌ Zero-duration segment at point %zu (dt=%.9f)", i, t - last_time);
            return false;
        }

        // 5️⃣b️ Maximum time gap check - detect Ruckig's silent failure mode
        // When Ruckig fails, it often creates huge time gaps (e.g., 5+ seconds between points)
        // A valid trajectory should have reasonably spaced points (max ~1 second gap)
        constexpr double MAX_TIME_GAP = 1.0;  // seconds
        if (i > 0 && (t - last_time) > MAX_TIME_GAP)
        {
            RCLCPP_ERROR(logger,
                "❌ Excessive time gap at point %zu: %.3fs (max allowed: %.1fs)",
                i, t - last_time, MAX_TIME_GAP);
            RCLCPP_ERROR(logger,
                "   This indicates Ruckig silent failure - couldn't find valid trajectory");
            return false;
        }

        last_time = t;  // Update AFTER the checks

        // 6️⃣ Velocities required
        if (p.velocities.size() != num_joints)
        {
            RCLCPP_ERROR(logger,
                "❌ Missing velocities at point %zu (got %zu, expected %zu)",
                i, p.velocities.size(), num_joints);
            return false;
        }

        // 7️⃣ Accelerations required (jerk-limited safety)
        if (p.accelerations.size() != num_joints)
        {
            RCLCPP_ERROR(logger,
                "❌ Missing accelerations at point %zu (got %zu, expected %zu)",
                i, p.accelerations.size(), num_joints);
            return false;
        }

        // 8️⃣ NaN / inf check
        for (size_t j = 0; j < num_joints; ++j)
        {
            if (!std::isfinite(p.positions[j]) ||
                !std::isfinite(p.velocities[j]) ||
                !std::isfinite(p.accelerations[j]))
            {
                RCLCPP_ERROR(logger,
                    "❌ NaN/Inf detected at point %zu joint %zu", i, j);
                return false;
            }

            // 9️⃣ Joint position limits
            const auto& name = jt.joint_names[j];
            if (limits.count(name))
            {
                const auto& b = limits[name];
                if (p.positions[j] < b.min_position_ - 1e-3 ||
                    p.positions[j] > b.max_position_ + 1e-3)
                {
                    RCLCPP_ERROR(logger,
                        "❌ Joint %s position %.4f out of limits [%.4f, %.4f] at point %zu",
                        name.c_str(), p.positions[j], b.min_position_, b.max_position_, i);
                    return false;
                }
            }
        }

        // 🔟 Teleport detection (position jump > 0.5 rad between consecutive points)
        if (i > 0)
        {
            const auto& prev = jt.points[i - 1];
            for (size_t j = 0; j < num_joints; ++j)
            {
                double jump = std::abs(p.positions[j] - prev.positions[j]);
                if (jump > 0.5)
                {
                    RCLCPP_ERROR(logger,
                        "❌ Teleport detected: joint %zu jumped %.3f rad at point %zu",
                        j, jump, i);
                    return false;
                }
            }
        }
    }

    // 1️⃣1️⃣ Duration sanity check
    if (last_time > max_duration)
    {
        RCLCPP_ERROR(logger,
            "❌ Trajectory duration %.2fs exceeds limit %.2fs (Ruckig likely failed)",
            last_time, max_duration);
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
        RCLCPP_INFO(this->get_logger(), "✅ Ruckig service '/apply_ruckig' is now ready");
        RCLCPP_INFO(this->get_logger(), "   Using jerk-limited trajectory smoothing (3rd order)");
    }

private:
    rclcpp::Service<ApplyRuckig>::SharedPtr service_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;  // Keep alive!
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;  // Cached state

    void applyRuckig(const std::shared_ptr<ApplyRuckig::Request> request,
                     std::shared_ptr<ApplyRuckig::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "📥 Received Ruckig request with %zu points -> ",
                    request->trajectory.points.size(),
                    request->trajectory.points.size());

//        RCLCPP_ERROR(this->get_logger(), "❌ RETURNING EARLY BECAUSE RUCKIG IS NOT CURRENTLY SUPPORTED");
//        response->trajectory = moveit_msgs::msg::RobotTrajectory();
//        return;

        // Debug: Check for near-duplicate points at start (can cause oscillation)
        if (request->trajectory.points.size() >= 2)
        {
            const auto& p0 = request->trajectory.points[0];
            const auto& p1 = request->trajectory.points[1];

            double max_diff = 0.0;
            for (size_t j = 0; j < std::min(p0.positions.size(), p1.positions.size()); ++j)
            {
                double diff = std::abs(p0.positions[j] - p1.positions[j]);
                if (diff > max_diff) max_diff = diff;
            }

            if (max_diff < 0.0001)  // Less than 0.006 degrees - essentially same position
            {
                RCLCPP_WARN(this->get_logger(),
                    "⚠️  First two waypoints nearly identical (max_diff=%.6f rad) - may cause oscillation",
                    max_diff);
            }
        }

        // Use cached robot state (no re-initialization!)
        robot_trajectory::RobotTrajectory rt(kinematic_model_, "fairino5_v6_group");

        // Set trajectory from request - this populates joint positions
        rt.setRobotTrajectoryMsg(*robot_state_, request->trajectory);

        // Clamp scaling factors to valid range [0.0, 1.0]
        double max_vel_scaling = std::max(0.0, std::min(1.0, request->max_velocity_scaling));
        double max_acc_scaling = std::max(0.0, std::min(1.0, request->max_acceleration_scaling));

        RCLCPP_INFO(this->get_logger(), "⚙️  Applying Ruckig smoothing with vel_scale=%.2f, acc_scale=%.2f",
                    max_vel_scaling, max_acc_scaling);

        // Log trajectory BEFORE Ruckig
        RCLCPP_INFO(this->get_logger(), "📊 BEFORE Ruckig:");
        moveit_msgs::msg::RobotTrajectory before_msg;
        rt.getRobotTrajectoryMsg(before_msg);
        logTrajectory(before_msg, this->get_logger(), true);  // verbose

        // Ruckig jerk-limited trajectory smoothing
        // Uses joint limits from URDF/SRDF automatically
        // mitigate_overshoot=true helps prevent position overshoot
        //
        // IMPORTANT: overshoot_threshold must be very tight (0.001 rad = ~0.06 deg)
        // to prevent back-and-forth motion at low velocities (e.g., vel=0.1)
        bool success = trajectory_processing::RuckigSmoothing::applySmoothing(
            rt,
            max_vel_scaling,
            max_acc_scaling,
            true,   // mitigate_overshoot
            0.001   // overshoot_threshold (radians)
        );

        // Log trajectory AFTER Ruckig
        RCLCPP_INFO(this->get_logger(), "📊 AFTER Ruckig (success=%s):", success ? "true" : "false");
        moveit_msgs::msg::RobotTrajectory after_msg;
        rt.getRobotTrajectoryMsg(after_msg);
        logTrajectory(after_msg, this->get_logger(), true);  // verbose

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Ruckig Smoothing FAILED - returning empty trajectory");
            response->trajectory = moveit_msgs::msg::RobotTrajectory();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ Ruckig Smoothing returned success");

        // Convert back to RobotTrajectory message
        rt.getRobotTrajectoryMsg(response->trajectory);

        // Log trajectory info before validation
        size_t num_points = response->trajectory.joint_trajectory.points.size();
        if (num_points > 0)
        {
            auto last_point = response->trajectory.joint_trajectory.points.back();
            double total_time = last_point.time_from_start.sec +
                               last_point.time_from_start.nanosec * 1e-9;
            RCLCPP_INFO(this->get_logger(), "🔍 Trajectory: %zu points, duration: %.2fs",
                        num_points, total_time);
        }

        // CRITICAL: Validate trajectory to catch silent Ruckig failures
        // (e.g., "extended to maximum and still did not find a solution")
        // Use a reasonable max duration based on scaling (slower = longer allowed)
        double max_duration = 10.0 / std::max(0.1, max_vel_scaling);  // e.g., 100s at 10% vel
        max_duration = std::min(max_duration, 60.0);  // Cap at 60s

        if (!validateTrajectory(response->trajectory, kinematic_model_, this->get_logger(), max_duration))
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Trajectory validation FAILED - returning empty trajectory");
            RCLCPP_ERROR(this->get_logger(), "   This usually means Ruckig couldn't find a valid solution.");
            RCLCPP_ERROR(this->get_logger(), "   Try: increase acc_scale, or decrease vel_scale");

            // Log the invalid trajectory for debugging (verbose mode)
            RCLCPP_WARN(this->get_logger(), "Dumping INVALID trajectory for debugging:");
            logTrajectory(response->trajectory, this->get_logger(), true);  // verbose=true

            response->trajectory = moveit_msgs::msg::RobotTrajectory();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ Trajectory validated successfully");

        // Log successful trajectory (non-verbose - just summary and first/last points)
        logTrajectory(response->trajectory, this->get_logger(), false);

        RCLCPP_INFO(this->get_logger(), "📤 Returning %zu jerk-limited points", num_points);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RuckigHelperNode>();

    try {
        node->initialize();  // Initialize after shared_ptr is created
        RCLCPP_INFO(node->get_logger(), "✅ Ruckig Helper fully initialized and ready");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ Failed to initialize Ruckig Helper: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}