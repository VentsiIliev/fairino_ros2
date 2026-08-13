import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def _resolve_config_path(config_yaml: str, value: str) -> str:
    path = str(value or "").strip()
    if not path:
        return ""
    if path.startswith("package://"):
        package_and_rel = path[len("package://"):]
        package_name, _, rel_path = package_and_rel.partition("/")
        if package_name and rel_path:
            return os.path.join(get_package_share_directory(package_name), rel_path)
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(os.path.dirname(config_yaml), path))


def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}

def _load_runtime_config(package_path: str) -> dict:
    rt_yaml = os.path.join(package_path, "config", "runtime.yaml")
    with open(rt_yaml) as f:
        rt = yaml.safe_load(f) or {}
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()
    if profile:
        profile_yaml = os.path.join(package_path, "config", profile, "runtime.yaml")
        with open(profile_yaml) as f:
            rt.update(yaml.safe_load(f) or {})
        rt["_ACTIVE_RUNTIME_CONFIG_PATH"] = profile_yaml
    else:
        rt["_ACTIVE_RUNTIME_CONFIG_PATH"] = rt_yaml
    return rt


def _runtime_urdf_path(package_path: str) -> str:
    rt = _load_runtime_config(package_path)
    return _resolve_config_path(rt["_ACTIVE_RUNTIME_CONFIG_PATH"], rt.get("URDF_PATH", ""))


def _runtime_srdf_path(package_path: str) -> str | None:
    rt = _load_runtime_config(package_path)
    path = _resolve_config_path(rt["_ACTIVE_RUNTIME_CONFIG_PATH"], rt.get("SRDF_PATH", ""))
    return path if path and os.path.isfile(path) else None


def generate_launch_description():
    package_path = get_package_share_directory("zeroerr")
    urdf_path = _runtime_urdf_path(package_path)
    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
        .robot_description(
            file_path="config/urdfs/eRobo3.urdf.xacro",
            mappings={"robot_urdf": urdf_path},
        )
        .robot_description_semantic(file_path=_runtime_srdf_path(package_path))
        .planning_pipelines(
            default_planning_pipeline="pilz_industrial_motion_planner",
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )



    non_rt_cores = os.environ.get("ZEROERR_NON_RT_CORES", "0-13")
    planner_cores = os.environ.get("ZEROERR_PLANNER_CORES", non_rt_cores)
    planner_prefix = f"taskset -c {planner_cores}"

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    ld.add_action(
        Node(
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
    )



    return ld
