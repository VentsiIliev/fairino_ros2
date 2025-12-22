from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()

    demo_ld = generate_demo_launch(moveit_config)

    # Set DISPLAY environment variable for GUI
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=os.environ.get('DISPLAY', ':0')
    )
    demo_ld.add_action(set_display)

    # Launch GUI as ROS2 node with delay to wait for controllers
    velocity_monitor_gui = Node(
        package='fairino5_v6_moveit2_config',
        executable='velocity_monitor.py',
        name='velocity_monitor',
        output='screen',
        emulate_tty=True,
        additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
    )

    # Delay GUI launch by 8 seconds to allow controllers to initialize
    delayed_gui = TimerAction(
        period=8.0,
        actions=[velocity_monitor_gui]
    )

    demo_ld.add_action(delayed_gui)

    return demo_ld
