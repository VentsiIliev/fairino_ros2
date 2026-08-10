#pragma once

#include <cstddef>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace erob_moveit_runtime
{

struct TrajectoryValidationOptions
{
    bool allow_empty = false;
    bool require_positions = true;
    bool require_strictly_increasing_time = true;
    bool require_finite_values = true;
};

struct TrajectoryValidationResult
{
    bool ok = false;
    std::string reason;
    std::size_t point_count = 0;
    std::size_t joint_count = 0;
    double duration_s = 0.0;
};

double durationToSeconds(
    const builtin_interfaces::msg::Duration& duration);

TrajectoryValidationResult validateJointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const TrajectoryValidationOptions& options = TrajectoryValidationOptions{});

}  // namespace erob_moveit_runtime
