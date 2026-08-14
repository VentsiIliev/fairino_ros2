import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from zeroerr_launch.cpu_policy import load_cpu_policy
from zeroerr_launch.moveit_config import build_moveit_config

from launch import LaunchDescription
from launch_ros.actions import Node
from zeroerr_launch.runtime_config import (
        ros2_controllers_path_from_runtime,
    )

def generate_launch_description():
    package_path = get_package_share_directory("zeroerr")
    moveit_config = build_moveit_config("zeroerr", package_path)



    control_cores = load_cpu_policy().control_cores

    ros2_controllers_path = ros2_controllers_path_from_runtime(
        package_path
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
        prefix=["chrt", "-f", "90", "taskset", "-c", control_cores],
    )

    return LaunchDescription([ros2_control_node])
