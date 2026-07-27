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

    profile = str(rt.get("ACTIVE_PROFILE", "")).strip()
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


def _urdf_path_from_runtime(package_path: str) -> str:
    rt = _load_runtime_config(package_path)
    path = _resolve_config_path(rt["_ACTIVE_RUNTIME_CONFIG_PATH"], rt.get("URDF_PATH", ""))
    if path and os.path.isfile(path):
        return path
    raise RuntimeError(
        f"runtime config must define a valid URDF_PATH. Resolved value: {path}"
    )


def _srdf_path_from_runtime(package_path: str) -> str | None:
    rt = _load_runtime_config(package_path)
    path = _resolve_config_path(rt["_ACTIVE_RUNTIME_CONFIG_PATH"], rt.get("SRDF_PATH", ""))
    if path and os.path.isfile(path):
        return path
    return None


def _runtime_value(package_path: str, key: str, default):
    try:
        rt = _load_runtime_config(package_path)
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


def _load_state_publisher_params(package_path: str) -> dict:
    base_yaml = _state_publisher_yaml_path(package_path)
    try:
        with open(base_yaml) as f:
            params = _extract_node_params(yaml.safe_load(f) or {})
    except Exception as exc:
        raise RuntimeError(f"Failed to read state publisher config: {base_yaml}: {exc}")

    rt = _load_runtime_config(package_path)
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
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    os.environ["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    non_rt_cores = os.environ.get("ZEROERR_NON_RT_CORES", "0-13")
    non_rt_prefix = f"taskset -c {non_rt_cores}"
    low_priority_non_rt_prefix = f"taskset -c {non_rt_cores} nice -n 19"

    package_path = get_package_share_directory("zeroerr")
    urdf_path = _urdf_path_from_runtime(package_path)
    os.environ["ZEROERR_ROBOT_URDF"] = urdf_path

    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
        .robot_description(
            file_path="config/urdfs/eRobo3.urdf.xacro",
            mappings={"robot_urdf": urdf_path},
        )
        .robot_description_semantic(file_path=_srdf_path_from_runtime(package_path))
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner","ompl","stomp"])
        .to_moveit_configs()
    )

    demo_ld = generate_demo_launch(moveit_config)

    wait_for_slaves_op = os.path.join(package_path, "scripts", "WaitForSlavesOp.sh")
    state_publisher_params = _load_state_publisher_params(package_path)

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
        prefix=low_priority_non_rt_prefix,
    )

    """USED FOR DEBUG TO READ MOTOR ERROR CODES"""
    zeroerr_error_monitor = Node(
        package="zeroerr",
        executable="zeroerr_error_monitor.py",
        name="zeroerr_error_monitor",
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

    drive_enable_set_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drive_enable_set_controller", "--inactive"],
        output="screen",
    )
    drive_disable_set_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "drive_disable_set_controller", "--inactive"],
        output="screen",
    )
    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_op_process,
                on_exit=[
                    TimerAction(period=0.5, actions=[drive_enable_set_spawner]),
                    TimerAction(period=2.0, actions=[drive_disable_set_spawner]),
                ],
            )
        )
    )

    ipp_helper_node = Node(
        package="erob_moveit_runtime",
        executable="ipp_helper",
        name="ipp_helper",
        output="screen",
        prefix=non_rt_prefix,
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
        name="ruckig_helper",
        output="screen",
        prefix=non_rt_prefix,
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
        name="contour_ik_helper",
        output="screen",
        prefix=non_rt_prefix,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ZeroErr-specific state publisher: TCP position via TF2 lookup (URDF-consistent)
    # (joint vel/acc + Cartesian vel/acc from shared base class)
    zeroerr_state_publisher = Node(
        package="zeroerr",
        executable="zeroerr_state_publisher.py",
        name="zeroerr_state_publisher",
        output="screen",
        prefix=non_rt_prefix,
        parameters=[
            state_publisher_params,
            {
                "cartesian_source_link": _runtime_value(package_path, "CARTESIAN_SOURCE_LINK", "ee_link"),
                "publish_hz": float(_runtime_value(package_path, "STATE_PUBLISH_RATE_HZ", 50.0)),
                "joint_publish_hz": float(_runtime_value(package_path, "JOINT_DERIVATIVE_PUBLISH_RATE_HZ", 0.0)),
                "joint_input_hz": float(_runtime_value(package_path, "JOINT_STATE_INPUT_RATE_HZ", 0.0)),
            },
        ],
    )

    """PLOT JUGGLER IS ONLY USED FOR DEBUG"""
    # plotjuggler_node = ExecuteProcess(
    #     cmd=[
    #         "plotjuggler",
    #         "--disable_opengl",
    #     ],
    #     output="screen",
    # )
    # demo_ld.add_action(
    #     RegisterEventHandler(
    #         OnProcessExit(
    #             target_action=wait_for_op_process,
    #             on_exit=[TimerAction(period=16.0, actions=[plotjuggler_node])],
    #         )
    #     )
    # )

    zeroerr_runtime = Node(
        package="zeroerr",
        executable="zeroerr_runtime.py",
        name="zeroerr_runtime",
        output="screen",
        emulate_tty=True,
        additional_env={
            "EROB_RUNTIME_HEADLESS": str(_runtime_value(package_path, "RUNTIME_HEADLESS", "0")),
        },
        prefix=non_rt_prefix,
    )
    delayed_zeroerr_actions = [
        TimerAction(period=4.0, actions=[zeroerr_state_publisher]),
        TimerAction(period=7.0, actions=[ipp_helper_node]),
        TimerAction(period=10.0, actions=[ruckig_helper_node]),
        TimerAction(period=13.0, actions=[contour_ik_helper_node]),
        TimerAction(period=16.0, actions=[zeroerr_runtime]),
        TimerAction(period=35.0, actions=[ethercat_sdo_server]),
    ]
    if bool(_runtime_value(package_path, "ZEROERR_ERROR_MONITOR_ENABLED", False)):
        delayed_zeroerr_actions.append(
            TimerAction(period=50.0, actions=[zeroerr_error_monitor])
        )

    # Delay ZeroErr-specific processes until EtherCAT OP is stable, then bring them
    # up in a fixed order. This avoids stacking controller spawners, MoveIt helper
    # model loads, runtime initialization, and SDO diagnostics in the same timing
    # window as the 1 ms EtherCAT/control loops.
    demo_ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_op_process,
                on_exit=delayed_zeroerr_actions,
            )
        )
    )

    return demo_ld
