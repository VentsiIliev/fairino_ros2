import os
import shutil

from ament_index_python.packages import get_package_share_directory
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

    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    demo_ld = generate_demo_launch(moveit_config)

    package_path = get_package_share_directory("zeroerr")
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
        "parameters": [{
            "slave_count": 6,
            "poll_period_sec": 0.1,
            "confirm_cycles": 3,
            "print_table": False,
            "use_inverse_dynamics": True,
            "dynamics_estimator_mode": "momentum_observer",
            "measured_torque_source": "current_based_torque",
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
            "urdf_path": os.path.join(package_path, "config", "erob_arm.urdf"),
            "base_link": "base_link",
            "tip_link": "tool0",
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

    velocity_monitor_gui = Node(
        package="erob_moveit_runtime",
        executable="main.py",
        additional_env={"EROB_CONFIG_PACKAGE": "zeroerr"},
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
