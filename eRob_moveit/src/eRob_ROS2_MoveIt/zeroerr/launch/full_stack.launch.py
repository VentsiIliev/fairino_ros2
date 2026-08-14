import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from zeroerr_launch.cpu_policy import (
    load_cpu_policy,
    load_servo_policy,
)
from zeroerr_launch.moveit_config import build_moveit_config
from zeroerr_launch.runtime_config import (
    load_state_publisher_params,
    ros2_controllers_path_from_runtime,
    runtime_value,
    urdf_path_from_runtime,
)


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_fake_hardware_value = os.environ.get("ZEROERR_USE_FAKE_HARDWARE", "").strip().lower()
    default_fake_hardware = (
        "true"
        if use_fake_hardware_value in ("1", "true", "yes", "on")
        else "false"
    )
    os.environ["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    cpu_policy = load_cpu_policy()
    non_rt_prefix = cpu_policy.non_rt_prefix
    planner_prefix = cpu_policy.planner_prefix
    low_priority_non_rt_prefix = cpu_policy.low_priority_prefix
    control_prefix = cpu_policy.control_prefix

    package_path = get_package_share_directory("zeroerr")
    urdf_path = urdf_path_from_runtime(package_path)
    os.environ["ZEROERR_ROBOT_URDF"] = urdf_path

    moveit_config = build_moveit_config(
        "zeroerr",
        package_path,
        use_fake_hardware=use_fake_hardware,
    )

    demo_ld = LaunchDescription()

    demo_ld.add_action(
        LogInfo(
            msg=[
                "[ZEROERR] Expanded robot_description hardware: ",
                PythonExpression(
                    ["'GenericSystem' if '", use_fake_hardware, "' == 'true' else 'EthercatDriver'"]
                ),
            ]
        )
    )

    demo_ld.add_action(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value=default_fake_hardware,
            description="Use ros2_control mock hardware instead of ZeroErr EtherCAT hardware",
        )
    )
    demo_ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz",
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="robot_state_publisher",
        output="screen",
        respawn=True,
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 15.0},
        ],
    )
    demo_ld.add_action(robot_state_publisher)

    virtual_joint_ld = generate_static_virtual_joint_tfs_launch(moveit_config)
    for action in virtual_joint_ld.entities:
        demo_ld.add_action(action)

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    demo_ld.add_action(move_group)

    # ============================================================
    # MoveIt Servo
    # ============================================================

    servo_yaml = os.path.join(
        package_path,
        "config",
        "servo.yaml",
    )

    with open(servo_yaml) as f:
        servo_config = yaml.safe_load(f) or {}

    servo_policy = load_servo_policy(servo_config)
    if servo_policy.low_cpu:
        servo_config["update_period"] = servo_policy.period
        servo_config["publish_period"] = servo_policy.period

    servo_node = Node(
        package="zeroerr",
        executable="zeroerr_servo_node",
        # name="servo_node",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            {"moveit_servo": servo_config},
            {"update_period": servo_policy.period},
            {"planning_group_name": "manipulator"},
            {"zeroerr_servo_realtime": servo_policy.realtime},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    rviz_config = os.path.join(package_path, "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        # name="rviz",
        output="log",
        prefix=low_priority_non_rt_prefix,
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    demo_ld.add_action(rviz)

    config_dir = os.path.join(package_path, "config")

    profile_controllers_yaml = ros2_controllers_path_from_runtime(package_path)

    fake_controllers_yaml = os.path.join(
        config_dir,
        "ros2_controllers_fake.yaml",
    )

    real_controllers_yaml = os.path.join(
        config_dir,
        "ros2_controllers.yaml",
    )

    active_profile = str(
        runtime_value(package_path, "ACTIVE_PROFILE", "")
    ).strip()

    if active_profile and os.path.isfile(
            os.path.join(
                package_path,
                "config",
                active_profile,
                "ros2_controllers.yaml",
            )
    ):
        ros2_controllers_path = profile_controllers_yaml
    else:
        ros2_controllers_path = PythonExpression(
            [
                "'",
                fake_controllers_yaml,
                "' if '",
                use_fake_hardware,
                "' == 'true' else '",
                real_controllers_yaml,
                "'",
            ]
        )

    demo_ld.add_action(
        LogInfo(msg=["[ZEROERR] ros2_control config: ", ros2_controllers_path])
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="controller_manager",
        output="screen",
        prefix=control_prefix,
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )
    demo_ld.add_action(ros2_control_node)

    controllers_to_spawn = runtime_value(
        package_path,
        "CONTROLLERS_TO_SPAWN",
        ["manipulator_controller"],
    )

    controller_spawners = []

    for controller_name in controllers_to_spawn:
        controller_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name],
                output="screen",
            )
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    servo_enabled = bool(
        runtime_value(
            package_path,
            "SERVO_ENABLED",
            True,
        )
    )

    if servo_enabled:
        demo_ld.add_action(servo_node)

    for controller_spawner in controller_spawners:
        demo_ld.add_action(controller_spawner)

    demo_ld.add_action(joint_state_broadcaster_spawner)

    wait_for_slaves_op = os.path.join(package_path, "scripts", "WaitForSlavesOp.sh")
    state_publisher_params = load_state_publisher_params(package_path)

    demo_ld.add_action(
        LogInfo(
            msg=[
                "\n"
                "========================================\n"
                "  ZeroErr MoveIt2 System Starting Up\n"
                "========================================\n"
                "Hardware mode: ",
                PythonExpression(
                    ["'FAKE / GenericSystem' if '", use_fake_hardware, "' == 'true' else 'REAL / EtherCAT'"]
                ),
                "\n========================================\n",
            ]
        )
    )

    demo_ld.add_action(SetEnvironmentVariable(name="DISPLAY", value=os.environ["DISPLAY"]))
    demo_ld.add_action(SetEnvironmentVariable(name="EROB_CONFIG_PACKAGE", value="zeroerr"))
    demo_ld.add_action(SetEnvironmentVariable(name="OGRE_RTT_MODE", value="Copy"))

    if ld_library_path:
        demo_ld.add_action(
            SetEnvironmentVariable(name="LD_LIBRARY_PATH",
                                   value=ld_library_path)
        )

    wait_for_op_process = ExecuteProcess(
        cmd=[wait_for_slaves_op],
        condition=UnlessCondition(use_fake_hardware),
        output="screen",
        additional_env={
            "EXPECTED_SLAVES": "6",
            "REQUIRED_STABLE_POLLS": "2",
            "POLL_INTERVAL": "0.25",
        },
    )
    demo_ld.add_action(wait_for_op_process)

    ethercat_sdo_server = Node(
        package="ethercat_manager",
        condition=UnlessCondition(use_fake_hardware),
        executable="ethercat_sdo_srv_server",
        # name="ethercat_sdo_srv_server",
        output="screen",
        prefix=low_priority_non_rt_prefix,
    )

    zeroerr_error_monitor = Node(
        package="zeroerr",
        condition=UnlessCondition(use_fake_hardware),
        executable="zeroerr_error_monitor.py",
        # name="zeroerr_error_monitor",
        output="screen",
        emulate_tty=True,
        prefix=low_priority_non_rt_prefix,
        parameters=[{
            "master_id": 0,
            "slave_count": 6,
            "poll_period_sec": 10.0,
            "log_zero_state_once": True,
        }],
    )

    zeroerr_drive_diagnostics = Node(
        package="zeroerr",
        condition=UnlessCondition(use_fake_hardware),
        executable="zeroerr_drive_diagnostics.py",
        # name="zeroerr_drive_diagnostics",
        output="screen",
        emulate_tty=True,
        prefix=low_priority_non_rt_prefix,
        parameters=[{
            "master_id": 0,
            "slave_count": 6,
            "poll_period_sec": float(runtime_value(
                package_path,
                "ZEROERR_DRIVE_DIAGNOSTICS_POLL_PERIOD_S",
                5.0,
            )),
            "topic_name": "/zeroerr/drive_diagnostics",
        }],
    )

    drive_enable_set_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drive_enable_set_controller", "--inactive"],
        condition=UnlessCondition(use_fake_hardware),
        output="screen",
    )
    drive_disable_set_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drive_disable_set_controller", "--inactive"],
        condition=UnlessCondition(use_fake_hardware),
        output="screen",
    )
    demo_ld.add_action(TimerAction(period=3.0, actions=[drive_enable_set_spawner]))
    demo_ld.add_action(TimerAction(period=4.0, actions=[drive_disable_set_spawner]))

    ipp_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ipp_helper",
        # name="ipp_helper",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    ruckig_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ruckig_helper",
        # name="ruckig_helper",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    contour_ik_helper_node = Node(
        package="erob_moveit_runtime",
        executable="contour_ik_helper",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    ptp_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ptp_helper",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    linked_lin_helper_node = Node(
        package="erob_moveit_runtime",
        executable="linked_lin_helper",
        # name="linked_lin_helper",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    trajectory_state_validator_node = Node(
        package="erob_moveit_runtime",
        executable="trajectory_state_validator",
        # name="trajectory_state_validator",
        output="screen",
        prefix=planner_prefix,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    zeroerr_state_publisher = Node(
        package="zeroerr",
        executable="zeroerr_state_publisher.py",
        # name="zeroerr_state_publisher",
        output="screen",
        prefix=non_rt_prefix,
        parameters=[
            state_publisher_params,
            {
                "base_frame": runtime_value(
                    package_path,
                    "BASE_LINK",
                    "base_link",
                ),
                "cartesian_source_link": runtime_value(
                    package_path,
                    "CARTESIAN_SOURCE_LINK",
                    "ee_link",
                ),
                "publish_hz": float(
                    runtime_value(
                        package_path,
                        "STATE_PUBLISH_RATE_HZ",
                        50.0,
                    )
                ),
                "joint_publish_hz": float(
                    runtime_value(
                        package_path,
                        "JOINT_DERIVATIVE_PUBLISH_RATE_HZ",
                        0.0,
                    )
                ),
                "joint_input_hz": float(
                    runtime_value(
                        package_path,
                        "JOINT_STATE_INPUT_RATE_HZ",
                        0.0,
                    )
                ),
            },
        ],
    )

    zeroerr_runtime = Node(
        package="zeroerr",
        executable="zeroerr_runtime.py",
        # name="zeroerr_runtime",
        output="screen",
        emulate_tty=True,
        additional_env={
            "EROB_RUNTIME_HEADLESS": str(runtime_value(package_path, "RUNTIME_HEADLESS", "0")),
            "ZEROERR_USE_RVIZ": LaunchConfiguration("use_rviz"),
        },
        prefix=non_rt_prefix,
    )

    # Real hardware keeps the existing startup order unchanged.
    demo_ld.add_action(
        TimerAction(
            period=1.0,
            actions=[zeroerr_runtime],
            condition=UnlessCondition(use_fake_hardware),
        )
    )
    demo_ld.add_action(TimerAction(period=2.0, actions=[zeroerr_state_publisher]))

    # Fake hardware must first produce a real Cartesian state from
    # GenericSystem -> /joint_states -> TF -> /cartesian_position. Start the
    # normal runtime only after one Cartesian sample has actually arrived so
    # the production runtime code does not need fake-specific motion logic.
    fake_cartesian_ready = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "timeout 15s ros2 topic echo /cartesian_position --once >/dev/null 2>&1",
        ],
        condition=IfCondition(use_fake_hardware),
        output="screen",
    )
    demo_ld.add_action(
        TimerAction(
            period=2.1,
            actions=[fake_cartesian_ready],
            condition=IfCondition(use_fake_hardware),
        )
    )
    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=fake_cartesian_ready,
                on_exit=[
                    LogInfo(msg="[ZEROERR] Fake Cartesian state is available; starting runtime"),
                    zeroerr_runtime,
                ],
            ),
            condition=IfCondition(use_fake_hardware),
        )
    )

    demo_ld.add_action(TimerAction(period=5.0, actions=[ipp_helper_node]))
    demo_ld.add_action(TimerAction(period=6.5, actions=[ruckig_helper_node]))
    demo_ld.add_action(TimerAction(period=8.0, actions=[contour_ik_helper_node]))
    demo_ld.add_action(TimerAction(period=8.5, actions=[ptp_helper_node]))
    demo_ld.add_action(TimerAction(period=9.0, actions=[linked_lin_helper_node]))
    demo_ld.add_action(TimerAction(period=9.5, actions=[trajectory_state_validator_node]))

    delayed_zeroerr_actions = [
        TimerAction(period=10.0, actions=[ethercat_sdo_server]),
    ]
    if bool(runtime_value(package_path, "ZEROERR_DRIVE_DIAGNOSTICS_ENABLED", False)):
        delayed_zeroerr_actions.append(
            TimerAction(period=42.0, actions=[zeroerr_drive_diagnostics])
        )
    if bool(runtime_value(package_path, "ZEROERR_ERROR_MONITOR_ENABLED", False)):
        delayed_zeroerr_actions.append(
            TimerAction(period=50.0, actions=[zeroerr_error_monitor])
        )

    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_op_process,
                on_exit=delayed_zeroerr_actions,
            ),
            condition=UnlessCondition(use_fake_hardware),
        )
    )

    return demo_ld
