from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()

    ld = LaunchDescription()

    # Spawn joint_state_broadcaster immediately
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Spawn fairino5_controller after joint_state_broadcaster succeeds
    fairino5_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fairino5_controller"],
        output="screen",
    )

    # Wait for joint_state_broadcaster to finish before spawning fairino5_controller
    spawn_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[fairino5_controller_spawner],
        )
    )

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(spawn_controller_after_joint_state)

    return ld
