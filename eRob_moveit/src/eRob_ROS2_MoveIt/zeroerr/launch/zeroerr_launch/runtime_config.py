"""Profile-aware runtime config helpers shared by ZeroErr launch files.

Centralizes the runtime.yaml / contour_ik_config.yaml / ptp_config.yaml and
erob_state_publisher_config.yaml parsing so individual launch files do not
duplicate the profile merging logic.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory


def _runtime_yaml_path(package_path: str) -> str:
    return os.path.join(package_path, "config", "runtime.yaml")


def _profile_runtime_yaml_path(package_path: str, profile: str) -> str:
    return os.path.join(package_path, "config", profile, "runtime.yaml")


def _contour_ik_yaml_path(package_path: str) -> str:
    return os.path.join(package_path, "config", "contour_ik_config.yaml")


def _profile_contour_ik_yaml_path(package_path: str, profile: str) -> str:
    return os.path.join(package_path, "config", profile, "contour_ik_config.yaml")


def _extra_runtime_yaml_paths(package_path: str) -> list[str]:
    return [
        os.path.join(package_path, "config", "contour_ik_config.yaml"),
        os.path.join(package_path, "config", "ptp_config.yaml"),
    ]


def _profile_extra_runtime_yaml_paths(package_path: str, profile: str) -> list[str]:
    return [
        os.path.join(package_path, "config", profile, "contour_ik_config.yaml"),
        os.path.join(package_path, "config", profile, "ptp_config.yaml"),
    ]


def _merge_config(base: dict, override: dict) -> dict:
    merged = dict(base)
    merged.update(override)
    return merged


def load_runtime_config(package_path: str) -> dict:
    rt_yaml = _runtime_yaml_path(package_path)
    try:
        with open(rt_yaml) as f:
            rt = yaml.safe_load(f) or {}
    except Exception:
        raise RuntimeError(f"Failed to read runtime config: {rt_yaml}")

    for config_yaml in _extra_runtime_yaml_paths(package_path):
        if not os.path.isfile(config_yaml):
            continue
        with open(config_yaml) as f:
            rt = _merge_config(rt, yaml.safe_load(f) or {})

    profile = str(
        os.environ.get("ZEROERR_ACTIVE_PROFILE", rt.get("ACTIVE_PROFILE", ""))
    ).strip()
    if profile:
        rt["ACTIVE_PROFILE"] = profile
    if profile:
        profile_yaml = _profile_runtime_yaml_path(package_path, profile)
        if not os.path.isfile(profile_yaml):
            raise RuntimeError(
                f"runtime.yaml requested ACTIVE_PROFILE '{profile}', but profile config was not found: {profile_yaml}"
            )
        with open(profile_yaml) as f:
            profile_rt = yaml.safe_load(f) or {}
        merged = _merge_config(rt, profile_rt)
        for profile_config_yaml in _profile_extra_runtime_yaml_paths(package_path, profile):
            if not os.path.isfile(profile_config_yaml):
                continue
            with open(profile_config_yaml) as f:
                merged = _merge_config(merged, yaml.safe_load(f) or {})
        merged["_ACTIVE_RUNTIME_CONFIG_PATH"] = profile_yaml
        return merged
    rt["_ACTIVE_RUNTIME_CONFIG_PATH"] = rt_yaml
    return rt


def resolve_config_path(config_yaml: str, value: str) -> str:
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


def urdf_path_from_runtime(package_path: str) -> str:
    rt = load_runtime_config(package_path)
    path = resolve_config_path(rt["_ACTIVE_RUNTIME_CONFIG_PATH"], rt.get("URDF_PATH", ""))
    if path and os.path.isfile(path):
        return path
    raise RuntimeError(
        f"runtime config must define a valid URDF_PATH. Resolved value: {path}"
    )


def srdf_path_from_runtime(package_path: str) -> str | None:
    rt = load_runtime_config(package_path)
    path = resolve_config_path(rt["_ACTIVE_RUNTIME_CONFIG_PATH"], rt.get("SRDF_PATH", ""))
    if path and os.path.isfile(path):
        return path
    return None


def runtime_value(package_path: str, key: str, default):
    try:
        rt = load_runtime_config(package_path)
        return rt.get(key, default)
    except Exception:
        return default


def _deep_merge(base: dict, override: dict) -> dict:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _state_publisher_yaml_path(package_path: str) -> str:
    return os.path.join(package_path, "config", "erob_state_publisher_config.yaml")


def _profile_state_publisher_yaml_path(package_path: str, profile: str) -> str:
    return os.path.join(package_path, "config", profile, "erob_state_publisher_config.yaml")


def _extract_node_params(config: dict) -> dict:
    return dict(config.get("zeroerr_state_publisher", {}).get("ros__parameters", {}) or {})


def load_state_publisher_params(package_path: str) -> dict:
    base_yaml = _state_publisher_yaml_path(package_path)
    try:
        with open(base_yaml) as f:
            params = _extract_node_params(yaml.safe_load(f) or {})
    except Exception as exc:
        raise RuntimeError(f"Failed to read state publisher config: {base_yaml}: {exc}")

    rt = load_runtime_config(package_path)
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()
    if not profile:
        return params

    profile_yaml = _profile_state_publisher_yaml_path(package_path, profile)
    if not os.path.isfile(profile_yaml):
        return params

    try:
        with open(profile_yaml) as f:
            profile_params = _extract_node_params(yaml.safe_load(f) or {})
    except Exception as exc:
        raise RuntimeError(f"Failed to read profile state publisher config: {profile_yaml}: {exc}")
    return _deep_merge(params, profile_params)


def ros2_controllers_path_from_runtime(package_path: str) -> str:
    rt = load_runtime_config(package_path)
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()

    if profile:
        candidate = os.path.join(
            package_path,
            "config",
            profile,
            "ros2_controllers.yaml",
        )
        if os.path.isfile(candidate):
            return candidate

    return os.path.join(
        package_path,
        "config",
        "ros2_controllers.yaml",
    )


def moveit_controllers_path_from_runtime(package_path: str) -> str:
    rt = load_runtime_config(package_path)
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()

    if profile:
        candidate = os.path.join(
            package_path,
            "config",
            profile,
            "moveit_controllers.yaml",
        )
        if os.path.isfile(candidate):
            return candidate

    return os.path.join(
        package_path,
        "config",
        "moveit_controllers.yaml",
    )


def kinematics_path_from_runtime(package_path: str) -> str:
    rt = load_runtime_config(package_path)
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()

    if profile:
        candidate = os.path.join(
            package_path,
            "config",
            profile,
            "kinematics.yaml",
        )
        if os.path.isfile(candidate):
            return candidate

    return os.path.join(
        package_path,
        "config",
        "kinematics.yaml",
    )


def joint_limits_path_from_runtime(package_path: str) -> str:
    rt = load_runtime_config(package_path)
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()

    if profile:
        candidate = os.path.join(
            package_path,
            "config",
            profile,
            "joint_limits.yaml",
        )
        if os.path.isfile(candidate):
            return candidate

    return os.path.join(
        package_path,
        "config",
        "joint_limits.yaml",
    )


def initial_positions_path_from_runtime(package_path: str) -> str:
    rt = load_runtime_config(package_path)
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()

    if profile:
        candidate = os.path.join(
            package_path,
            "config",
            profile,
            "initial_positions.yaml",
        )
        if os.path.isfile(candidate):
            return candidate

    return os.path.join(
        package_path,
        "config",
        "initial_positions.yaml",
    )


def ros2_control_xacro_path_from_runtime(package_path: str) -> str:
    rt = load_runtime_config(package_path)
    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()

    if profile:
        candidate = os.path.join(
            package_path,
            "config",
            profile,
            "eRobo3.ros2_control.xacro",
        )
        if os.path.isfile(candidate):
            return candidate

    return os.path.join(
        package_path,
        "config",
        "urdfs",
        "eRobo3.ros2_control.xacro",
    )
