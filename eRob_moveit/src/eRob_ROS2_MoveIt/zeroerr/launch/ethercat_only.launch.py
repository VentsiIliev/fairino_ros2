from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import shutil
import yaml


def _urdf_path_from_runtime(package_path: str) -> str:
    rt_yaml = os.path.join(package_path, "config", "runtime.yaml")
    try:
        with open(rt_yaml) as f:
            rt = yaml.safe_load(f) or {}
        path = rt.get("URDF_PATH", "")
        if path and os.path.isfile(path):
            return path
    except Exception:
        pass
    return os.path.join(package_path, "config", "erob_arm_family_motor_masses.urdf")


def generate_launch_description():
    torque_log_enabled = LaunchConfiguration("torque_log_enabled")
    torque_log_path = LaunchConfiguration("torque_log_path")
    terminal_prefix = None
    if os.environ.get("ZEROERR_COLLISION_MONITOR_TERMINAL", "0") == "1":
        terminal = shutil.which("x-terminal-emulator") or shutil.which("xterm")
        if terminal:
            terminal_prefix = f"{terminal} -e"
    launch_collision_gui = os.environ.get("ZEROERR_COLLISION_MONITOR_GUI", "0") == "1"

    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
        .robot_description(
            file_path="config/eRobo3.urdf.xacro",
            mappings={"robot_urdf": _urdf_path_from_runtime(get_package_share_directory("zeroerr"))},
        )
        .to_moveit_configs()
    )

    package_path = get_package_share_directory("zeroerr")
    ros2_controllers_path = os.path.join(package_path, "config", "ros2_controllers.yaml")
    wait_for_slaves_op = os.path.join(package_path, "scripts", "WaitForSlavesOp.sh")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "world", "--child-frame-id", "base_link",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"],
        output="screen",
    )

    load_manipulator_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "manipulator_controller"],
        output="screen",
    )
    load_drag_effort_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_effort_controller"],
        output="screen",
    )
    load_drag_torque_offset_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_torque_offset_controller"],
        output="screen",
    )
    load_drag_mode_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_mode_controller"],
        output="screen",
    )
    load_drag_enable_set_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_enable_set_controller"],
        output="screen",
    )
    load_drag_disable_set_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_disable_set_controller"],
        output="screen",
    )

    wait_for_op_process = ExecuteProcess(
        cmd=[wait_for_slaves_op],
        output="screen",
        additional_env={
            "EXPECTED_SLAVES": "6",
            "REQUIRED_STABLE_POLLS": "3",
            "POLL_INTERVAL": "1",
        },
    )

    ethercat_sdo_server = Node(
        package="ethercat_manager",
        executable="ethercat_sdo_srv_server",
        name="ethercat_sdo_srv_server",
        output="screen",
    )

    collision_monitor_kwargs = {
        "package": "zeroerr",
        "executable": "zeroerr_collision_monitor.py",
        "name": "zeroerr_collision_monitor",
        "output": "screen",
        "additional_env": {
            "EROB_CONFIG_PACKAGE": "zeroerr",
        },
        "parameters": [{
            "slave_count": 6,
            "poll_period_sec": 0.005,
            "confirm_cycles": 3,
            "print_table": False,
            "use_inverse_dynamics": True,
            "dynamics_estimator_mode": "momentum_observer",
            "friction_coulomb_nm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "friction_viscous_nm_per_rad_s": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "urdf_path": _urdf_path_from_runtime(package_path),
            "base_link": "base_link",
            "tip_link": "disk_link",
            "num_joints": 6,
            "external_torque_thresholds": [12.0, 12.0, 10.0, 8.0, 6.0, 5.0],
            "filter_alpha": 0.7,
            "include_gravity": True,
            "static_torque_bias_nm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "torque_log_enabled": torque_log_enabled,
            "torque_log_path": torque_log_path,
        }],
    }
    if terminal_prefix:
        collision_monitor_kwargs["prefix"] = terminal_prefix
        collision_monitor_kwargs["parameters"][0]["print_table"] = True

    zeroerr_collision_monitor = Node(**collision_monitor_kwargs)
    launch_actions = [zeroerr_collision_monitor]
    if launch_collision_gui:
        launch_actions.append(
            Node(
                package="zeroerr",
                executable="zeroerr_collision_monitor_gui.py",
                name="zeroerr_collision_monitor_gui",
                output="screen",
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument("torque_log_enabled", default_value="false"),
        DeclareLaunchArgument(
            "torque_log_path",
            default_value=os.path.join(package_path, "data", "torque_sensor_log.csv"),
        ),
        SetEnvironmentVariable("EROB_CONFIG_PACKAGE", "zeroerr"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("MESA_GL_VERSION_OVERRIDE", "3.3"),
        SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1"),
        SetEnvironmentVariable("OGRE_RTT_MODE", "Copy"),
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        load_joint_state_broadcaster,
        load_manipulator_controller,
        load_drag_effort_controller,
        load_drag_torque_offset_controller,
        load_drag_mode_controller,
        load_drag_enable_set_controller,
        load_drag_disable_set_controller,
        ethercat_sdo_server,
        wait_for_op_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_op_process,
                on_exit=[TimerAction(period=12.0, actions=launch_actions)],
            )
        ),
    ])
