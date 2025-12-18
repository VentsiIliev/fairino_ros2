from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fairino5_v6_moveit2_config'),
                'launch',
                'moveit_rviz.launch.py'
            ])
        ])
    )

    velocity_monitor_node = Node(
        package='fairino5_v6_moveit2_config',
        executable='velocity_monitor.py',
        name='velocity_monitor',
        output='screen'
    )

    return LaunchDescription([
        rviz_launch,
        velocity_monitor_node
    ])

