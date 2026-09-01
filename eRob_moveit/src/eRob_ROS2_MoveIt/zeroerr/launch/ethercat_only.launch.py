import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from zeroerr_launch.moveit_config import build_moveit_config
from zeroerr_launch.runtime_config import urdf_path_from_runtime

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("zeroerr")
    os.environ["ZEROERR_ROBOT_URDF"] = urdf_path_from_runtime(package_path)

    moveit_config = build_moveit_config("zeroerr", package_path)

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
    load_drive_enable_set_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drive_enable_set_controller", "--inactive"],
        output="screen",
    )
    load_drive_disable_set_controller = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drive_disable_set_controller", "--inactive"],
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

    helper_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    ipp_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ipp_helper",
        name="ipp_helper",
        output="screen",
        parameters=helper_parameters,
    )

    ruckig_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ruckig_helper",
        name="ruckig_helper",
        output="screen",
        parameters=helper_parameters,
    )

    contour_ik_helper_node = Node(
        package="erob_moveit_runtime",
        executable="contour_ik_helper",
        name="contour_ik_helper",
        output="screen",
        parameters=helper_parameters,
    )

    trajectory_state_validator_node = Node(
        package="erob_moveit_runtime",
        executable="trajectory_state_validator",
        name="trajectory_state_validator",
        output="screen",
        parameters=helper_parameters,
    )

    return LaunchDescription([
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
        load_drive_enable_set_controller,
        load_drive_disable_set_controller,
        ethercat_sdo_server,
        ipp_helper_node,
        ruckig_helper_node,
        contour_ik_helper_node,
        trajectory_state_validator_node,
        wait_for_op_process,
    ])
