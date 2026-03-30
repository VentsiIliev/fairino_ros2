import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
        .to_moveit_configs()
    )

    package_path = get_package_share_directory("zeroerr")
    ros2_controllers_path = os.path.join(package_path, "config", "ros2_controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
        prefix=["chrt", "-f", "90", "taskset", "-c", "14,15"],
    )

    return LaunchDescription([ros2_control_node])
