from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()

    demo_ld = generate_demo_launch(moveit_config)

    velocity_monitor_node = Node(
        package='fairino5_v6_moveit2_config',
        executable='velocity_monitor.py',
        name='velocity_monitor',
        output='screen'
    )

    demo_ld.add_action(velocity_monitor_node)

    return demo_ld
