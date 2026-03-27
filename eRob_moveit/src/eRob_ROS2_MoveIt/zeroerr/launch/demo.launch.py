import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    os.environ["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")

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
    demo_ld.add_action(SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"))
    demo_ld.add_action(SetEnvironmentVariable(name="MESA_GL_VERSION_OVERRIDE", value="3.3"))
    demo_ld.add_action(SetEnvironmentVariable(name="LIBGL_DRI3_DISABLE", value="1"))
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

    velocity_monitor_gui = Node(
        package="erob_moveit_runtime",
        executable="main.py",
        additional_env={"EROB_CONFIG_PACKAGE": "zeroerr"},
        name="velocity_monitor",
        output="screen",
        emulate_tty=True,
    )
    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=zeroerr_state_publisher,
                on_start=[TimerAction(period=12.0, actions=[velocity_monitor_gui])],
            )
        )
    )

    return demo_ld
