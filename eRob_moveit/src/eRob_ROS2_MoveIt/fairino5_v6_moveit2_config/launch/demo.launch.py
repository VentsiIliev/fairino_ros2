from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # CRITICAL: Set DISPLAY first before anything else
    # Xorg runs on :1 on this Ubuntu system
    os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':1')

    # Preserve LD_LIBRARY_PATH from environment
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')

    # Build MoveIt config with only Pilz planner (disable CHOMP, OMPL, STOMP for faster startup)
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])  # Only load Pilz (fast)
        .to_moveit_configs()
    )

    demo_ld = generate_demo_launch(moveit_config)

    # Add startup info message
    startup_msg = LogInfo(
        msg="\n"
            "========================================\n"
            "  MoveIt2 System Starting Up\n"
            "========================================\n"
            "Please wait ~40 seconds for collision geometry processing...\n"
            "(This is normal - complex mounting surface mesh with magazines/tool changers)\n"
            "========================================\n"
    )
    demo_ld.add_action(startup_msg)

    # Set DISPLAY environment variable for GUI
    # Use :1 as default since Xorg typically runs on :1 on Ubuntu
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=os.environ.get('DISPLAY', ':1')
    )
    demo_ld.add_action(set_display)

    # Set LD_LIBRARY_PATH to ensure all nodes can find shared libraries
    if ld_library_path:
        set_ld_library_path = SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=ld_library_path
        )
        demo_ld.add_action(set_ld_library_path)

    # Launch IPP helper node for TOTG trajectory optimization (2nd order)
    ipp_helper_node = Node(
        package='erob_moveit_runtime',
        executable='ipp_helper',
        name='ipp_helper',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    demo_ld.add_action(ipp_helper_node)

    # Launch Ruckig helper node for jerk-limited trajectory smoothing (3rd order)
    ruckig_helper_node = Node(
        package='erob_moveit_runtime',
        executable='ruckig_helper',
        name='ruckig_helper',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    demo_ld.add_action(ruckig_helper_node)

    # Fairino-specific state publisher: TCP position from /nonrt_state_data
    # (joint vel/acc + Cartesian vel/acc from shared base class)
    fairino_state_publisher_node = Node(
        package='fairino5_v6_moveit2_config',
        executable='fairino_state_publisher.py',
        name='fairino_state_publisher',
        output='screen'
    )
    demo_ld.add_action(fairino_state_publisher_node)

    # Launch GUI as a ROS2 node (velocity monitor)
    velocity_monitor_gui = Node(
        package='fairino5_v6_moveit2_config',
        executable='fairino_runtime.py',
        name='velocity_monitor',
        output='screen',
        emulate_tty=True
    )

    plotjuggler_node = ExecuteProcess(
        cmd=[
            "/opt/ros/rolling/lib/plotjuggler/plotjuggler",
            "--disable_opengl",
        ],
        output="screen",
    )

    gui_actions = [velocity_monitor_gui, plotjuggler_node]

    # Launch GUI only after the state publisher node starts
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessStart

    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=fairino_state_publisher_node,
                on_start=gui_actions
            )
        )
    )

    return demo_ld
