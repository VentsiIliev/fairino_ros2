#include "erob_moveit_runtime/trajectory_validation.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace erob_moveit_runtime
{
namespace
{

TrajectoryValidationResult invalidResult(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::string reason)
{
    TrajectoryValidationResult result;
    result.ok = false;
    result.reason = std::move(reason);
    result.point_count = trajectory.points.size();
    result.joint_count = trajectory.joint_names.size();
    if (!trajectory.points.empty())
    {
        result.duration_s = durationToSeconds(
            trajectory.points.back().time_from_start);
    }
    return result;
}

bool valuesAreFinite(const std::vector<double>& values)
{
    return std::all_of(
        values.begin(),
        values.end(),
        [](double value) { return std::isfinite(value); });
}

std::string pointFieldSizeReason(
    std::size_t point_index,
    const char* field_name,
    std::size_t actual,
    std::size_t expected)
{
    std::ostringstream out;
    out
        << "point "
        << point_index
        << " "
        << field_name
        << " size "
        << actual
        << " does not match joint count "
        << expected;
    return out.str();
}

}  // namespace

double durationToSeconds(
    const builtin_interfaces::msg::Duration& duration)
{
    return static_cast<double>(duration.sec)
        + static_cast<double>(duration.nanosec) * 1e-9;
}

TrajectoryValidationResult validateJointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const TrajectoryValidationOptions& options)
{
    const std::size_t joint_count = trajectory.joint_names.size();
    const std::size_t point_count = trajectory.points.size();

    if (joint_count == 0)
    {
        return invalidResult(trajectory, "trajectory has no joint names");
    }

    if (point_count == 0 && !options.allow_empty)
    {
        return invalidResult(trajectory, "trajectory has no points");
    }

    double previous_time_s = -1.0;

    for (std::size_t point_index = 0;
         point_index < point_count;
         ++point_index)
    {
        const auto& point = trajectory.points[point_index];

        if (options.require_positions
            && point.positions.size() != joint_count)
        {
            return invalidResult(
                trajectory,
                pointFieldSizeReason(
                    point_index,
                    "positions",
                    point.positions.size(),
                    joint_count));
        }

        if (!point.velocities.empty()
            && point.velocities.size() != joint_count)
        {
            return invalidResult(
                trajectory,
                pointFieldSizeReason(
                    point_index,
                    "velocities",
                    point.velocities.size(),
                    joint_count));
        }

        if (!point.accelerations.empty()
            && point.accelerations.size() != joint_count)
        {
            return invalidResult(
                trajectory,
                pointFieldSizeReason(
                    point_index,
                    "accelerations",
                    point.accelerations.size(),
                    joint_count));
        }

        if (options.require_finite_values)
        {
            if (!valuesAreFinite(point.positions)
                || !valuesAreFinite(point.velocities)
                || !valuesAreFinite(point.accelerations)
                || !valuesAreFinite(point.effort))
            {
                std::ostringstream out;
                out << "point " << point_index << " contains non-finite values";
                return invalidResult(trajectory, out.str());
            }
        }

        const double time_s = durationToSeconds(point.time_from_start);
        if (!std::isfinite(time_s) || time_s < 0.0)
        {
            std::ostringstream out;
            out << "point " << point_index << " has invalid time_from_start";
            return invalidResult(trajectory, out.str());
        }

        if (options.require_strictly_increasing_time
            && point_index > 0
            && time_s <= previous_time_s)
        {
            std::ostringstream out;
            out
                << "point "
                << point_index
                << " time_from_start is not strictly increasing";
            return invalidResult(trajectory, out.str());
        }

        previous_time_s = time_s;
    }

    TrajectoryValidationResult result;
    result.ok = true;
    result.reason = "ok";
    result.point_count = point_count;
    result.joint_count = joint_count;
    if (!trajectory.points.empty())
    {
        result.duration_s = durationToSeconds(
            trajectory.points.back().time_from_start);
    }
    return result;
}

}  // namespace erob_moveit_runtime
