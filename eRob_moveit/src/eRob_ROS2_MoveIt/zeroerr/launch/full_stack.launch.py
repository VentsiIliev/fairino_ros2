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
    runtime_value,
    urdf_path_from_runtime,
)


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def _continue_on_success(event, context, *, label, actions):
    """Return follow-up actions only when the process that triggered us succeeded."""
    if event.returncode == 0:
        return actions
    if event.returncode < 0:
        return [
            LogInfo(msg=(
                f"[ZEROERR] {label} terminated by signal "
                f"{-event.returncode}; dependent startup canceled"
            ))
        ]
    return [
        LogInfo(msg=f"[ZEROERR] {label} failed (exit {event.returncode}); dependent startup stopped")
    ]


def _after_success(target, label, actions, condition=None):
    def handle_exit(event, context):
        return _continue_on_success(
            event, context, label=label, actions=actions
        )

    return RegisterEventHandler(
        OnProcessExit(
            target_action=target,
            on_exit=handle_exit,
        ),
        condition=condition,
    )


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    enable_sensorless_collision_monitor = LaunchConfiguration(
        "enable_sensorless_collision_monitor"
    )
    enable_sensorless_collision_gui = LaunchConfiguration(
        "enable_sensorless_collision_gui"
    )
    enable_collision_calibration_runner = LaunchConfiguration(
        "enable_collision_calibration_runner"
    )
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
        planning_pipelines=["pilz_industrial_motion_planner", "ompl", "stomp"],
        default_planning_pipeline="pilz_industrial_motion_planner",
    )
    collision_padding_m = float(runtime_value(
        package_path, "MOVEIT_COLLISION_PADDING_M", 0.010))
    collision_scale = float(runtime_value(
        package_path, "MOVEIT_COLLISION_SCALE", 1.0))
    # PlanningSceneMonitor constructs these names by appending `_planning` to
    # the robot-description parameter name.  Keep them flattened so launch_ros
    # cannot treat robot_description_planning as a nested parameter value.
    collision_geometry_parameters = {
        "robot_description_planning.default_robot_padding": collision_padding_m,
        "robot_description_planning.default_robot_scale": collision_scale,
    }

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
    demo_ld.add_action(
        DeclareLaunchArgument(
            "enable_sensorless_collision_monitor",
            default_value=str(
                bool(runtime_value(
                    package_path,
                    "ENABLE_SENSORLESS_COLLISION_MONITOR",
                    False,
                ))
            ).lower(),
            description="Start the passive sensorless joint-torque collision monitor",
        )
    )
    demo_ld.add_action(
        DeclareLaunchArgument(
            "enable_sensorless_collision_gui",
            default_value=str(bool(runtime_value(
                package_path,
                "SENSORLESS_COLLISION_GUI_ENABLED",
                False,
            ))).lower(),
            description="Show the read-only sensorless collision status GUI",
        )
    )
    demo_ld.add_action(
        DeclareLaunchArgument(
            "enable_collision_calibration_runner",
            default_value=str(bool(runtime_value(
                package_path,
                "COLLISION_CALIBRATION_RUNNER_ENABLED",
                False,
            ))).lower(),
            description="Start the idle-by-default MoveIt-validated calibration runner",
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
            collision_geometry_parameters,
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
            collision_geometry_parameters,
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
    config_dir = os.path.join(package_path, "config")
    fake_controllers_yaml = os.path.join(config_dir, "ros2_controllers_fake.yaml")
    real_controllers_yaml = os.path.join(config_dir, "ros2_controllers.yaml")
    ros2_controllers_path = PythonExpression(
        [
            "'", fake_controllers_yaml, "' if '",
            use_fake_hardware,
            "' == 'true' else '", real_controllers_yaml, "'",
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

    manipulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller"],
        output="screen",
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    demo_ld.add_action(joint_state_broadcaster_spawner)

    wait_for_slaves_op = os.path.join(package_path, "scripts", "WaitForSlavesOp.sh")
    wait_for_ros = os.path.join(package_path, "scripts", "wait_for_ros.py")
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

    sensorless_collision_monitor = Node(
        package="zeroerr",
        executable="zeroerr_collision_monitor.py",
        output="screen",
        emulate_tty=True,
        prefix=low_priority_non_rt_prefix,
        condition=IfCondition(PythonExpression([
            "'", enable_sensorless_collision_monitor,
            "' == 'true' and '", use_fake_hardware, "' != 'true'",
        ])),
        parameters=[{
            "slave_count": 6,
            "poll_period_sec": float(runtime_value(
                package_path,
                "SENSORLESS_COLLISION_MONITOR_PERIOD_SEC",
                0.01,
            )),
            "input_sample_period_sec": 0.0,
            "print_table": False,
            "include_gravity": True,
            "observer_gain": float(runtime_value(
                package_path, "SENSORLESS_COLLISION_OBSERVER_GAIN", 10.0)),
            "warmup_sec": float(runtime_value(
                package_path, "SENSORLESS_COLLISION_WARMUP_SEC", 8.0)),
            "drive_stable_sec": float(runtime_value(
                package_path, "SENSORLESS_COLLISION_DRIVE_STABLE_SEC", 2.0)),
            "urdf_path": urdf_path,
            "base_link": "base_link",
            "tip_link": runtime_value(package_path, "COLLISION_TIP_LINK", "tool0"),
            "num_joints": int(runtime_value(package_path, "NUM_JOINTS", 6)),
            "collision_config_path": os.path.join(
                package_path, "config", "collision_monitor_config.json"
            ),
            "torque_log_enabled": bool(runtime_value(
                package_path,
                "SENSORLESS_COLLISION_LOG_ENABLED",
                False,
            )),
            "torque_log_path": str(runtime_value(
                package_path,
                "SENSORLESS_COLLISION_LOG_PATH",
                "/home/ilv/ros2_ws/eRob_moveit/zeroerr_data/collision_detection/collision_training.csv",
            )),
            "torque_log_period_sec": float(runtime_value(
                package_path,
                "SENSORLESS_COLLISION_LOG_PERIOD_SEC",
                0.05,
            )),
        }],
    )

    sensorless_collision_gui = Node(
        package="zeroerr",
        executable="zeroerr_collision_status_gui.py",
        output="screen",
        prefix=low_priority_non_rt_prefix,
        condition=IfCondition(PythonExpression([
            "'", enable_sensorless_collision_monitor,
            "' == 'true' and '", enable_sensorless_collision_gui,
            "' == 'true' and '", use_fake_hardware, "' != 'true'",
        ])),
    )

    collision_calibration_runner = Node(
        package="zeroerr",
        executable="zeroerr_collision_calibration_runner.py",
        output="screen",
        emulate_tty=True,
        prefix=low_priority_non_rt_prefix,
        condition=IfCondition(enable_collision_calibration_runner),
        parameters=[{
            "joint_names": runtime_value(package_path, "JOINT_NAMES", [
                "Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6",
            ]),
            "planning_group": str(runtime_value(package_path, "PLANNING_GROUP", "manipulator")),
            "controller_action": str(runtime_value(
                package_path,
                "ACTION_FOLLOW_TRAJECTORY",
                "/manipulator_controller/follow_joint_trajectory",
            )),
            "dry_run": bool(runtime_value(
                package_path, "COLLISION_CALIBRATION_DRY_RUN", True)),
            "joint_sweep_amplitudes_rad": runtime_value(
                package_path,
                "COLLISION_CALIBRATION_JOINT_SWEEP_AMPLITUDES_RAD",
                [0.15] * 6,
            ),
            "sweep_levels": int(runtime_value(
                package_path, "COLLISION_CALIBRATION_SWEEP_LEVELS", 2)),
            "coupled_sample_count": int(runtime_value(
                package_path, "COLLISION_CALIBRATION_COUPLED_SAMPLE_COUNT", 48)),
            "coupled_amplitude_scale": float(runtime_value(
                package_path, "COLLISION_CALIBRATION_COUPLED_AMPLITUDE_SCALE", 0.35)),
            "coupled_return_interval": int(runtime_value(
                package_path, "COLLISION_CALIBRATION_COUPLED_RETURN_INTERVAL", 4)),
            "base_rotation_sample_count": int(runtime_value(
                package_path, "COLLISION_CALIBRATION_BASE_ROTATION_SAMPLE_COUNT", 12)),
            "base_rotation_min_rad": float(runtime_value(
                package_path, "COLLISION_CALIBRATION_BASE_ROTATION_MIN_RAD", -3.0)),
            "base_rotation_max_rad": float(runtime_value(
                package_path, "COLLISION_CALIBRATION_BASE_ROTATION_MAX_RAD", 3.0)),
            "max_interpolation_step_rad": float(runtime_value(
                package_path, "COLLISION_CALIBRATION_MAX_INTERPOLATION_STEP_RAD", 0.015)),
            "velocity_scalings": runtime_value(
                package_path, "COLLISION_CALIBRATION_VELOCITY_SCALINGS",
                [0.04, 0.10, 0.20, 0.30, 0.40, 0.55, 0.70, 0.85, 1.00]),
            "acceleration_scalings": runtime_value(
                package_path, "COLLISION_CALIBRATION_ACCELERATION_SCALINGS",
                [0.04, 0.12, 0.30, 0.40, 0.50, 0.65, 0.80, 0.90, 1.00]),
            "joint_sampling_min_rad": runtime_value(
                package_path, "COLLISION_CALIBRATION_JOINT_SAMPLING_MIN_RAD",
                [-3.0, -1.5, -1.8, -2.5, -2.5, -3.0]),
            "joint_sampling_max_rad": runtime_value(
                package_path, "COLLISION_CALIBRATION_JOINT_SAMPLING_MAX_RAD",
                [3.0, 1.5, 1.8, 2.0, 2.5, 3.0]),
            "settle_time_sec": float(runtime_value(
                package_path, "COLLISION_CALIBRATION_SETTLE_TIME_SEC", 0.35)),
            "coverage_log_period_sec": float(runtime_value(
                package_path, "COLLISION_CALIBRATION_COVERAGE_LOG_PERIOD_SEC", 0.05)),
            "coverage_voxel_size_m": float(runtime_value(
                package_path, "COLLISION_CALIBRATION_COVERAGE_VOXEL_SIZE_M", 0.05)),
            "output_directory": str(runtime_value(
                package_path,
                "COLLISION_CALIBRATION_OUTPUT_DIRECTORY",
                "/home/ilv/ros2_ws/eRob_moveit/zeroerr_data/collision_detection/calibration_runs",
            )),
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
            collision_geometry_parameters,
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
            collision_geometry_parameters,
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
            collision_geometry_parameters,
            # Explicit fallback used by the validator itself.  This makes the
            # calibration safety margin independent of MoveIt parameter-loading
            # differences between ROS distributions.
            {"collision_padding_m": collision_padding_m},
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
                "cartesian_source_link": runtime_value(package_path, "CARTESIAN_SOURCE_LINK", "ee_link"),
                "publish_hz": float(runtime_value(package_path, "STATE_PUBLISH_RATE_HZ", 50.0)),
                "joint_publish_hz": float(runtime_value(package_path, "JOINT_DERIVATIVE_PUBLISH_RATE_HZ", 0.0)),
                "joint_input_hz": float(runtime_value(package_path, "JOINT_STATE_INPUT_RATE_HZ", 0.0)),
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

    # Controller spawners already wait for controller_manager. Chain them and
    # only start state consumers after each preceding spawner exits cleanly.
    demo_ld.add_action(_after_success(
        joint_state_broadcaster_spawner,
        "joint_state_broadcaster spawner",
        [manipulator_controller_spawner, rviz],
    ))
    demo_ld.add_action(_after_success(
        manipulator_controller_spawner,
        "manipulator_controller spawner",
        [
            zeroerr_state_publisher,
            servo_node,
            sensorless_collision_monitor,
            sensorless_collision_gui,
        ],
    ))

    demo_ld.add_action(_after_success(
        manipulator_controller_spawner,
        "manipulator_controller spawner",
        [drive_enable_set_spawner],
        condition=UnlessCondition(use_fake_hardware),
    ))
    demo_ld.add_action(_after_success(
        drive_enable_set_spawner,
        "drive_enable_set_controller spawner",
        [drive_disable_set_spawner],
        condition=UnlessCondition(use_fake_hardware),
    ))

    moveit_ready = ExecuteProcess(
        cmd=[wait_for_ros, "--service", "/get_planning_scene"],
        output="screen",
    )
    demo_ld.add_action(moveit_ready)
    helper_nodes = [
        ipp_helper_node,
        ruckig_helper_node,
        contour_ik_helper_node,
        ptp_helper_node,
        trajectory_state_validator_node,
    ]
    demo_ld.add_action(_after_success(moveit_ready, "MoveIt readiness check", helper_nodes))

    runtime_ready = ExecuteProcess(
        cmd=[
            wait_for_ros,
            "--topic", "/joint_states",
            "--topic", "/cartesian_position",
            "--service", "/apply_ipp",
            "--service", "/apply_ruckig",
            "--service", "/compute_contour_ik",
            "--service", "/compute_ptp",
            "--service", "/validate_trajectory_states",
        ],
        output="screen",
    )
    demo_ld.add_action(runtime_ready)
    demo_ld.add_action(_after_success(
        runtime_ready,
        "runtime readiness check",
        [zeroerr_runtime, collision_calibration_runner],
    ))

    # EtherCAT auxiliary services start after the bus is demonstrably in OP.
    demo_ld.add_action(_after_success(
        wait_for_op_process,
        "EtherCAT OP readiness check",
        [ethercat_sdo_server],
        condition=UnlessCondition(use_fake_hardware),
    ))

    sdo_ready = ExecuteProcess(
        cmd=[wait_for_ros, "--service", "/ethercat_manager/get_sdo"],
        condition=UnlessCondition(use_fake_hardware),
        output="screen",
    )
    demo_ld.add_action(_after_success(
        wait_for_op_process,
        "EtherCAT OP readiness check",
        [sdo_ready],
        condition=UnlessCondition(use_fake_hardware),
    ))

    sdo_dependents = []
    if bool(runtime_value(package_path, "ZEROERR_DRIVE_DIAGNOSTICS_ENABLED", False)):
        sdo_dependents.append(zeroerr_drive_diagnostics)
    if bool(runtime_value(package_path, "ZEROERR_ERROR_MONITOR_ENABLED", False)):
        sdo_dependents.append(zeroerr_error_monitor)
    if sdo_dependents:
        demo_ld.add_action(_after_success(
            sdo_ready,
            "EtherCAT SDO readiness check",
            sdo_dependents,
            condition=UnlessCondition(use_fake_hardware),
        ))

    return demo_ld
