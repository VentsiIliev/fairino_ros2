#include "erob_moveit_runtime/trajectory_validation.hpp"

#include <iostream>

namespace
{

trajectory_msgs::msg::JointTrajectory makeValidTrajectory()
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {"Joint_1", "Joint_2"};

    trajectory_msgs::msg::JointTrajectoryPoint first;
    first.positions = {0.0, 1.0};
    first.time_from_start.sec = 0;
    first.time_from_start.nanosec = 100000000;

    trajectory_msgs::msg::JointTrajectoryPoint second;
    second.positions = {0.5, 1.5};
    second.time_from_start.sec = 1;
    second.time_from_start.nanosec = 0;

    trajectory.points.push_back(first);
    trajectory.points.push_back(second);
    return trajectory;
}

}  // namespace

int main()
{
    auto trajectory = makeValidTrajectory();

    const auto valid_result =
        erob_moveit_runtime::validateJointTrajectory(trajectory);

    if (!valid_result.ok
        || valid_result.point_count != 2
        || valid_result.joint_count != 2)
    {
        std::cerr
            << "valid trajectory failed: "
            << valid_result.reason
            << std::endl;
        return 1;
    }

    trajectory.points.back().time_from_start.sec = 0;
    trajectory.points.back().time_from_start.nanosec = 50000000;

    const auto invalid_result =
        erob_moveit_runtime::validateJointTrajectory(trajectory);

    if (invalid_result.ok)
    {
        std::cerr
            << "invalid trajectory unexpectedly passed"
            << std::endl;
        return 1;
    }

    std::cout
        << "trajectory_validation_smoke ok: "
        << invalid_result.reason
        << std::endl;
    return 0;
}
