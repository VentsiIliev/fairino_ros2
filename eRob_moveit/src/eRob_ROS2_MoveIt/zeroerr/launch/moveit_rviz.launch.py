import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from zeroerr_launch.cpu_policy import load_cpu_policy
from zeroerr_launch.moveit_config import build_moveit_config

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    package_path = get_package_share_directory("zeroerr")
    moveit_config = build_moveit_config("zeroerr", package_path)

    non_rt_prefix = load_cpu_policy().non_rt_prefix

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(package_path, "config", "moveit.rviz"),
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            respawn=False,
            prefix=non_rt_prefix,
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    )

    return ld
