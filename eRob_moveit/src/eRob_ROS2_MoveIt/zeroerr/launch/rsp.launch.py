import os
import yaml

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


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


def generate_launch_description():
    package_path = get_package_share_directory("zeroerr")
    urdf_path = _runtime_urdf_path(package_path)
    moveit_config = MoveItConfigsBuilder(
        "eRobo3", package_name="zeroerr"
    ).robot_description(
        file_path="config/urdfs/eRobo3.urdf.xacro",
        mappings={"robot_urdf": urdf_path},
    ).to_moveit_configs()
    return generate_rsp_launch(moveit_config)
