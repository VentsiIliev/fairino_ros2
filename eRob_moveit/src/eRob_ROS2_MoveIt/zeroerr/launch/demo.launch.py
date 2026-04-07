import os
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory


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
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    os.environ["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    terminal_prefix = None
    if os.environ.get("ZEROERR_COLLISION_MONITOR_TERMINAL", "0") == "1":
        terminal = shutil.which("x-terminal-emulator") or shutil.which("xterm")
        if terminal:
            terminal_prefix = f"{terminal} -e"
    launch_collision_gui = os.environ.get("ZEROERR_COLLISION_MONITOR_GUI", "0") == "1"
    launch_plotjuggler = os.environ.get("ZEROERR_PLOTJUGGLER", "0") == "1"

    package_path = get_package_share_directory("zeroerr")
    urdf_path = _urdf_path_from_runtime(package_path)

    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
        .robot_description(
            file_path="config/eRobo3.urdf.xacro",
            mappings={"robot_urdf": urdf_path},
        )
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    demo_ld = generate_demo_launch(moveit_config)

    wait_for_slaves_op = os.path.join(package_path, "scripts", "WaitForSlavesOp.sh")

    demo_ld.add_action(
        LogInfo(
            msg="\n"
                "========================================\n"
                "  ZeroErr MoveIt2 System Starting Up\n"
                "========================================\n"
                "Using Fairino-style demo launch with ZeroErr EtherCAT wait\n"
                "========================================\n"
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
        output="screen",
        additional_env={
            "EXPECTED_SLAVES": "6",
            "REQUIRED_STABLE_POLLS": "3",
            "POLL_INTERVAL": "1",
        },
    )
    demo_ld.add_action(wait_for_op_process)

    ethercat_sdo_server = Node(
        package="ethercat_manager",
        executable="ethercat_sdo_srv_server",
        name="ethercat_sdo_srv_server",
        output="screen",
    )
    demo_ld.add_action(ethercat_sdo_server)

    drag_effort_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_effort_controller"],
        output="screen",
    )
    drag_torque_offset_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_torque_offset_controller"],
        output="screen",
    )
    drag_mode_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_mode_controller"],
        output="screen",
    )
    drag_enable_set_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_enable_set_controller"],
        output="screen",
    )
    drag_disable_set_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drag_disable_set_controller"],
        output="screen",
    )
    demo_ld.add_action(drag_effort_spawner)
    demo_ld.add_action(drag_torque_offset_spawner)
    demo_ld.add_action(drag_mode_spawner)
    demo_ld.add_action(drag_enable_set_spawner)
    demo_ld.add_action(drag_disable_set_spawner)

    ipp_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ipp_helper",
        name="ipp_helper",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    demo_ld.add_action(ipp_helper_node)

    ruckig_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ruckig_helper",
        name="ruckig_helper",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    demo_ld.add_action(ruckig_helper_node)

    # ZeroErr-specific state publisher: TCP position via TF2 lookup (URDF-consistent)
    # (joint vel/acc + Cartesian vel/acc from shared base class)
    zeroerr_state_publisher = Node(
        package="zeroerr",
        executable="zeroerr_state_publisher.py",
        name="zeroerr_state_publisher",
        output="screen",
    )
    demo_ld.add_action(zeroerr_state_publisher)

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
        "measured_torque_source": "drive_torque",
        "static_torque_bias_nm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint_models": [
                "eRob80H100T",
                "eRob80H100T",
                "eRob80H100T",
                "eRob70H100T",
                "eRob70H100T",
                "eRob70H100T",
            ],
            "model_names": ["eRob70H100T", "eRob80H100T"],
            "model_rated_current_ma": [3500.0, 5500.0],
            "model_output_torque_constant_nm_per_a": [4.76, 8.475],
            "friction_coulomb_nm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "friction_viscous_nm_per_rad_s": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "urdf_path": urdf_path,
            "base_link": "base_link",
            "tip_link": "Link_6",
            "num_joints": 6,
        "external_torque_thresholds": [12.0, 12.0, 10.0, 8.0, 6.0, 5.0],
        "filter_alpha": 0.7,
        "include_gravity": False,
        }],
    }
    if terminal_prefix:
        collision_monitor_kwargs["prefix"] = terminal_prefix
        collision_monitor_kwargs["parameters"][0]["print_table"] = True

    zeroerr_collision_monitor = Node(**collision_monitor_kwargs)
    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_op_process,
                on_exit=[TimerAction(period=12.0, actions=[zeroerr_collision_monitor])],
            )
        )
    )

    if launch_collision_gui:
        zeroerr_collision_monitor_gui = Node(
            package="zeroerr",
            executable="zeroerr_collision_monitor_gui.py",
            name="zeroerr_collision_monitor_gui",
            output="screen",
        )
        demo_ld.add_action(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_op_process,
                    on_exit=[TimerAction(period=14.0, actions=[zeroerr_collision_monitor_gui])],
                )
            )
        )

    if launch_plotjuggler:
        plotjuggler_node = ExecuteProcess(
            cmd=["/opt/ros/rolling/lib/plotjuggler/plotjuggler"],
            output="screen",
        )
        demo_ld.add_action(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_op_process,
                    on_exit=[TimerAction(period=16.0, actions=[plotjuggler_node])],
                )
            )
        )

    velocity_monitor_gui = Node(
        package="zeroerr",
        executable="zeroerr_runtime.py",
        name="velocity_monitor",
        output="screen",
        emulate_tty=True,
    )
    # Delay the runtime GUI until the RT/control path and MoveIt startup have settled.
    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=zeroerr_state_publisher,
                on_start=[TimerAction(period=45.0, actions=[velocity_monitor_gui])],
            )
        )
    )

    return demo_ld
