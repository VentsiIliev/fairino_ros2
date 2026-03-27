from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
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

    wait_for_op_process = ExecuteProcess(
        cmd=[wait_for_slaves_op],
        output="screen",
        additional_env={
            "EXPECTED_SLAVES": "6",
            "REQUIRED_STABLE_POLLS": "3",
            "POLL_INTERVAL": "1",
        },
    )

    return LaunchDescription([
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("MESA_GL_VERSION_OVERRIDE", "3.3"),
        SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1"),
        SetEnvironmentVariable("OGRE_RTT_MODE", "Copy"),
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        load_joint_state_broadcaster,
        load_manipulator_controller,
        wait_for_op_process,
    ])
