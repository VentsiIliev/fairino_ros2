#!/usr/bin/env python3
"""Runtime configuration loader for eRob MoveIt packages.

Robot-specific values live in <robot_config_package>/config/runtime.yaml.
This module preserves the old cfg.CONSTANT access pattern while moving the
actual robot description values out of the shared runtime package.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

DEFAULTS = {
    'TOPIC_ROBOT_STATUS': '/robot_status',
    'TOPIC_CARTESIAN_POSITION': '/cartesian_position',
    'TOPIC_CARTESIAN_VELOCITY': '/cartesian_velocity',
    'TOPIC_CARTESIAN_ACCELERATION': '/cartesian_acceleration',
    'TOPIC_JOINT_VELOCITY': '/joint_velocity',
    'TOPIC_JOINT_ACCELERATION': '/joint_acceleration',
    'TOPIC_PLANNING_SCENE': '/planning_scene',
    'TOPIC_ATTACHED_COLLISION_OBJECT': '/attached_collision_object',
    'TOPIC_SAFETY_WALLS': '/safety_walls',
    'TOPIC_ACTIVE_TOOL_MARKERS': '/active_tool_markers',
    'ROBOT_STATUS_PUBLISH_ENABLED': True,
    'ACTIVE_TOOL_COLLISION_ENABLED': False,
    'ACTIVE_TOOL_COLLISION_ID': 'active_tool_collision',
    'ACTIVE_TOOL_COLLISION_LINK': 'ee_link',
    'ACTIVE_TOOL_COLLISION_RADIUS_M': 0.012,
    'ACTIVE_TOOL_COLLISION_LENGTH_M': 0.17,
    'ACTIVE_TOOL_COLLISION_ORIGIN': [0, 0, 0, 0, 0, 0],
    'ACTIVE_TOOL_COLLISION_USE_ACTIVE_TOOL_TRANSFORM': True,
    'ACTIVE_TOOL_COLLISION_TOUCH_LINKS': ['ee_link', 'tool0', 'tcp', 'Link_6'],
    'MOUNTING_SURFACE_COLLISION_OBJECT_ENABLED': False,
    'MOUNTING_SURFACE_COLLISION_OBJECT_ID': 'mounting_surface_collision',
    'MOUNTING_SURFACE_COLLISION_OBJECT_FRAME': 'mounting_surface',
    'MOUNTING_SURFACE_COLLISION_BOX_M': [0.115, 0.190, 0.1475],
    'MOUNTING_SURFACE_COLLISION_ORIGIN': [100.0, 576.45, 158.0, 0.0, 0.0, 0.0],
    'MOTION_PASSAGES': [],
    'CARTESIAN_SERVO_SERVICE_TIMEOUT_S': 5.0,
    'CARTESIAN_SERVO_TF_TIMEOUT_S': 0.25,
    'CARTESIAN_SERVO_STOP_ZERO_DWELL_S': 0.05,
    'CARTESIAN_SERVO_PUBLISH_RATE_HZ': 100.0,
    'SERVICE_CARTESIAN_PATH': '/compute_cartesian_path',
    'SERVICE_MOTION_SEQUENCE': '/plan_sequence_path',
    'SERVICE_IK': '/compute_ik',
    'SERVICE_FK': '/compute_fk',
    'SERVICE_CONTOUR_IK': '/compute_contour_ik',
    'SERVICE_LINKED_LIN': '/compute_linked_lin',
    'SERVICE_TRAJECTORY_STATE_VALIDATION': '/validate_trajectory_states',
    'SERVICE_APPLY_IPP': '/apply_ipp',
    'SERVICE_APPLY_RUCKIG': '/apply_ruckig',
    'SERVICE_STATE_VALIDITY': '/check_state_validity',
    'SAFETY_WALLS_ENABLED': True,
    'SAFETY_MARGIN_M': 0.01,
    'WALL_XY_OFFSET_M': 0.13,
    'WALL_THICKNESS_M': 0.01,
    'WALL_BYPASS_LINKS': [],
    'SAFETY_WALL_NAMES': ['wall_x_min', 'wall_x_max', 'wall_y_min', 'wall_y_max', 'wall_z_min', 'wall_z_max'],
    'TOOL_REGISTRY': {'TOOL_0': [0, 0, 0, 0, 0, 0]},
    'TOOL_ID_MAP': {0: 'TOOL_0'},
    'WORKOBJECT_REGISTRY': {},
    'WORKOBJECT_ID_MAP': {},
    'DEFAULT_WORKOBJECT_ID': 0,
    'CARTESIAN_SOURCE_LINK': 'ee_link',
    'DEFAULT_VEL_PERCENT': 30,
    'DEFAULT_ACC_PERCENT': 30,
    'DEFAULT_VEL_SCALING': 0.6,
    'DEFAULT_ACC_SCALING': 0.4,
    'DEFAULT_JERK_SCALING': 0.5,
    'DEFAULT_ORIENTATION': [180.0, 0.0, 0.0],
    'PTP_LOCK_ORIENTATION_TOL_DEG': 2.0,
    'PTP_LOCKED_PATH_MAX_DRIFT_DEG': 2.0,
    'PTP_ORIENTED_PATH_MAX_DEVIATION_DEG': 10.0,
    'PTP_WRIST_PENALTY_START_DEG': 45.0,
    'PTP_MAX_WRIST_DELTA_DEG': 160.0,
    'PTP_LOCKED_MAX_WRIST_DELTA_DEG': 120.0,
    'PTP_NOOP_JOINT_DELTA_RAD': 0.001,
    'PTP_JOINT_INTERPOLATION_STEP_RAD': 0.08,
    'PTP_MIN_INTERPOLATION_SEGMENTS': 8,
    'PTP_MAX_INTERPOLATION_SEGMENTS': 80,
    'JOG_AVOID_COLLISIONS': True,
    'JOG_BLOCKING_TIMEOUT_S': 5.0,
    'JOG_MAX_ORIENTATION_STEP_DEG': 5.0,
    'SERVO_JOG_LINEAR_SPEED_AT_100_PERCENT_MM_S': 100.0,
    'SERVO_JOG_ANGULAR_SPEED_AT_100_PERCENT_DEG_S': 30.0,
    'SERVO_JOG_DEFAULT_LINEAR_MM_S': 10.0,
    'SERVO_JOG_DEFAULT_ANGULAR_DEG_S': 3.0,
    'SERVO_JOG_MAX_DURATION_S': 10.0,
    'ENABLE_COLLISION_CHECKING': True,  # Global toggle for all collision avoidance
    'MOVE_LIN_STRATEGY': 'cartesian_path',
    'CARTESIAN_MIN_FRACTION': 1,
    'CARTESIAN_STATE_VALIDITY_ENABLED': True,
    'CARTESIAN_STATE_VALIDITY_STRIDE': 1,
    'CARTESIAN_STATE_VALIDITY_TIMEOUT_S': 5.0,
    'ORDERED_BLEND_BATCH_VALIDATION_ENABLED': True,
    'ORDERED_BLEND_BATCH_VALIDATION_WORKERS': 4,
    'ORDERED_BLEND_BATCH_VALIDATION_TIMEOUT_S': 2.0,
    'ORDERED_BLEND_BATCH_CHECK_COLLISIONS': True,
    'ORDERED_BLEND_JOINT_RATE_GUARD_ENABLED': True,
    'ORDERED_BLEND_JOINT_RATE_LIMITS_RAD_S': {},
    'FOLLOW_PATH_CARTESIAN_FALLBACK_ENABLED': True,
    'FOLLOW_PATH_CARTESIAN_TIMEOUT_S': 10.0,
    'FOLLOW_PATH_DENSIFY_MAX_TRANSLATION_MM': 0.0,
    'FOLLOW_PATH_DENSIFY_MAX_ORIENTATION_DEG': 0.0,
    'CARTESIAN_ALLOW_START_COLLISION_ESCAPE': True,
    'CARTESIAN_START_COLLISION_ESCAPE_DEPTH_TOL_M': 0.001,
    'CARTESIAN_FAILURE_DIAGNOSTICS_ENABLED': False,
    'JACOBIAN_FALLBACK_MM': 0.1,
    'JACOBIAN_FALLBACK_MIN_FRACTION': 1,
    'JACOBIAN_FALLBACK_MIN_DELTA_MM': 0.1,
    'SHORT_CARTESIAN_JACOBIAN_FALLBACK_MAX_DELTA_MM': 2.0,
    'JACOBIAN_MAX_JOINT_STEP': 0.05,
    'JACOBIAN_MIN_DURATION_S': 0.05,
    'JACOBIAN_SHORT_MOVE_MIN_DURATION_S': 0.20,
    'JACOBIAN_DAMPING': 1e-5,
    'JACOBIAN_NUM_DIFF_EPS': 1e-7,
    'MOTION_QUEUE_MAX_SIZE': 10,
    'EXECUTOR_GOAL_POS_TOL_RAD': 0.02,
    'EXECUTOR_PATH_POS_TOL_RAD': 0.35,
    'EXECUTOR_ACTIVE_DRIVE_MONITOR_ENABLED': True,
    'EXECUTOR_ACTIVE_DRIVE_MONITOR_PERIOD_S': 0.05,
    'EXECUTOR_ACTIVE_DRIVE_MONITOR_GRACE_S': 0.25,
    'EXECUTOR_ACTIVE_DRIVE_MONITOR_BAD_SAMPLES': 3,
    'EXECUTOR_DRIVE_ENABLE_BEFORE_TRAJECTORY': True,
    'EXECUTOR_DRIVE_ENABLE_WAIT_TIMEOUT_S': 2.0,
    'EXECUTOR_DRIVE_ENABLE_STABLE_BEFORE_TRAJECTORY_S': 0.15,
    'STARTUP_AUTO_ENABLE_DRIVES': False,
    'STARTUP_AUTO_ENABLE_DRIVES_TIMEOUT_S': 30.0,
    'STARTUP_AUTO_ENABLE_DRIVES_RETRY_PERIOD_S': 1.0,
    'STARTUP_AUTO_ENABLE_DRIVES_VERIFY_TIMEOUT_S': 5.0,
    'EXECUTOR_CANCEL_ON_DRIVE_DISABLE': True,
    'EXECUTOR_TIME_MULTIPLIER': 2.0,
    'EXECUTOR_TIME_MIN_S': 8.0,
    'EXECUTOR_START_HOLD_S': 0.06,
    'EXECUTOR_START_RAMP_POINTS': 3,
    'EXECUTOR_PATH_STOP_ENABLED': True,
    'EXECUTOR_PATH_STOP_DURATION_S': 0.30,
    'EXECUTOR_PATH_STOP_SAMPLE_PERIOD_S': 0.04,
    'EXECUTOR_PATH_STOP_TRACKING_TOL_RAD': 0.20,
    'EXECUTOR_PATH_STOP_HOLD_JOINT_NAMES': ['Joint_6', 'j6'],
    'EXECUTOR_PATH_STOP_HELD_GOAL_TOL_RAD': 0.35,
    'EXECUTOR_POST_UNWIND_ENABLED': False,
    'EXECUTOR_POST_UNWIND_JOINT_NAME': 'Joint_6',
    'EXECUTOR_POST_UNWIND_TARGET_RANGE_RAD': 3.141592653589793,
    'EXECUTOR_POST_UNWIND_MIN_DELTA_RAD': 0.5,
    'EXECUTOR_POST_UNWIND_SPEED_RAD_S': 1.8,
    'EXECUTOR_POST_UNWIND_ACCEL_RAD_S2': 2.5,
    'EXECUTOR_POST_UNWIND_ROTATION_AXIS_INDEX': 5,
    'EXECUTOR_POST_UNWIND_ROTATION_AXIS_SIGN': 1.0,
    'EXECUTOR_POST_UNWIND_ROTATIONAL_SEGMENT_DEG': 180.0,
    'EXECUTOR_POST_UNWIND_USE_DIRECT_IK': False,
    'EXECUTOR_POST_UNWIND_DIRECT_IK_FALLBACK_CARTESIAN': True,
    'EXECUTOR_POST_UNWIND_DIRECT_IK_STEP_DEG': 4.0,
    'EXECUTOR_POST_UNWIND_DIRECT_IK_OPTIMIZER': '',
    'EXECUTOR_POST_UNWIND_MIN_DURATION_S': 1,
    'EXECUTOR_POST_UNWIND_MAX_SEGMENT_RAD': 1.2,
    'EXECUTOR_POST_UNWIND_SAMPLE_PERIOD_S': 0.05,
    'EXECUTOR_POST_UNWIND_VERIFY_TOL_RAD': 0.12,
    'EXECUTOR_POST_UNWIND_VERIFY_TIMEOUT_S': 0.5,
    'EXECUTOR_POST_UNWIND_SKIP_NOOP_DURATION_S': 1.0,
    'EXECUTOR_POST_UNWIND_SKIP_NOOP_MAX_JOINT_DELTA_RAD': 0.02,
    'EXECUTOR_UNWIND_DIAGNOSTICS_ENABLED': True,
    'EXECUTOR_UNWIND_DIAGNOSTIC_DELAYS_S': [0.2, 0.5, 1.0],
    'EXECUTOR_UNWIND_CANCEL_ON_DRIVE_DISABLE': True,
    'EXECUTOR_SUPPRESS_UNWIND_AFTER_ORDERED_FAILURE_S': 10.0,
    'EXECUTOR_ORDERED_FINAL_UNWIND_LIVE_EXECUTION': True,
    'EXECUTOR_ORDERED_START_MATCH_ENABLED': True,
    'EXECUTOR_ORDERED_START_MATCH_TOL_RAD': 0.02,
    'EXECUTOR_ORDERED_START_MATCH_TIMEOUT_S': 0.35,
    'TRAJ_METRICS_ENABLED': False,
    'TRAJ_METRICS_FK_SAMPLE_LIMIT': 80,
    'TRAJ_METRICS_FK_TIMEOUT_S': 0.25,
    'SINGLE_TARGET_JOINT_RATE_LIMITS_RAD_S': {'Joint_6': 1.2, 'j6': 1.2},
    'OPTIMIZER_START_ALIGN_TOL_RAD': 0.002,
    'OPTIMIZER_START_MERGE_TOL_RAD': 0.002,
    'PATH_APPROACH_THRESHOLD_MM': 100.0,
    'PATH_EEF_STEP_SCALE': 1.35,
    'PATH_EEF_STEP_MIN_M': 0.005,
    'PATH_EEF_STEP_MAX_M': 0.015,
    'PATH_WAYPOINT_SIMPLIFY_ENABLED': False,
    'PATH_WAYPOINT_SIMPLIFY_POSITION_TOL_MM': 0.35,
    'PATH_WAYPOINT_SIMPLIFY_ORIENTATION_TOL_DEG': 0.35,
    'PATH_WAYPOINT_SIMPLIFY_MAX_TRANSLATION_MM': 12.0,
    'PATH_WAYPOINT_SIMPLIFY_MAX_ORIENTATION_DEG': 4.0,
    'CONTOUR_DIRECT_IK_ENABLED': False,
    'CONTOUR_BATCH_IK_ENABLED': True,
    'CONTOUR_BATCH_IK_SERVICE_TIMEOUT_S': 10.0,
    'CONTOUR_DIRECT_IK_MIN_POINTS': 50,
    'CONTOUR_DIRECT_IK_MIN_TOTAL_LENGTH_MM': 20.0,
    'CONTOUR_IK_TIMEOUT_S': 0.003,
    'CONTOUR_IK_ATTEMPTS': 1,
    'CONTOUR_IK_RETRY_TIMEOUT_S': 0.02,
    'CONTOUR_IK_RETRY_ATTEMPTS': 3,
    'CONTOUR_IK_FK_POSITION_TOL_MM': 0.15,
    'CONTOUR_IK_FK_ORIENTATION_TOL_DEG': 0.25,
    'CONTOUR_IK_MAX_JOINT_STEP_RAD': 0.08,
    'CONTOUR_IK_MAX_JOINT_SPAN_RAD': 3.141592653589793,
    'CONTOUR_IK_MAX_ENDPOINT_DELTA_RAD': 3.141592653589793,
    'CONTOUR_IK_FULL_TURN_JOINT_NAMES': ['Joint_6', 'j6'],
    'CONTOUR_IK_FULL_TURN_MAX_JOINT_SPAN_RAD': 6.6,
    'CONTOUR_IK_FULL_TURN_MAX_ENDPOINT_DELTA_RAD': 6.6,
    'CONTOUR_IK_SMOOTHING_ENABLED': True,
    'CONTOUR_IK_SMOOTHING_ITERATIONS': 2,
    'CONTOUR_IK_SMOOTHING_ALPHA': 0.35,
    'CONTOUR_IK_SMOOTHING_FK_POSITION_TOL_MM': 0.05,
    'CONTOUR_IK_SMOOTHING_FK_ORIENTATION_TOL_DEG': 0.10,
    'LINKED_LIN_HELPER_ENABLED': False,
    'LINKED_LIN_SERVICE_TIMEOUT_S': 10.0,
    'LINKED_LIN_CARTESIAN_STEP_M': 0.01,
    'LINKED_LIN_JUMP_THRESHOLD': 0.0,
    'LINKED_LIN_DENSIFY_MAX_TRANSLATION_MM': 8.0,
    'LINKED_LIN_DENSIFY_MAX_ORIENTATION_DEG': 2.0,
    'LINKED_LIN_FK_POSITION_TOL_MM': 0.15,
    'LINKED_LIN_FK_ORIENTATION_TOL_DEG': 0.25,
    'LINKED_LIN_MAX_JOINT_STEP_RAD': 0.08,
    'LINKED_LIN_MAX_JOINT_SPAN_RAD': 3.141592653589793,
    'LINKED_LIN_MAX_ENDPOINT_DELTA_RAD': 3.141592653589793,
    'LINKED_LIN_FULL_TURN_JOINT_NAMES': ['Joint_6', 'j6'],
    'LINKED_LIN_FULL_TURN_MAX_JOINT_SPAN_RAD': 6.6,
    'LINKED_LIN_FULL_TURN_MAX_ENDPOINT_DELTA_RAD': 6.6,
    'CONTOUR_VALIDATE_REDUCED_MOVEIT_ENABLED': False,
    'CONTOUR_VALIDATE_REDUCED_POSITION_TOL_MM': 0.35,
    'CONTOUR_VALIDATE_REDUCED_ORIENTATION_TOL_DEG': 0.35,
    'CONTOUR_VALIDATE_REDUCED_MAX_TRANSLATION_MM': 10.0,
    'CONTOUR_STATE_VALIDITY_ENABLED': False,
    'CONTOUR_STATE_VALIDITY_STRIDE': 10,
    'TRAJECTORY_OPTIMIZER': 'TOTG',
    'TOTG_PATH_DIAG_ASYNC': True,
    'PATH_TRAJECTORY_OPTIMIZER': 'RUCKIG',
    # Ordered LIN/PTP blending
    'ORDERED_BLEND_SAMPLES': 12,
    'ORDERED_BLEND_JUNCTION_TOL_RAD': 0.02,
    'ORDERED_BLEND_MIN_RADIUS_MM': 0.5,
    'RUCKIG_SAMPLE_DT_S': 0.008,
    'RUCKIG_FALLBACK_REDUCE_SCALING': True,
    'RUCKIG_FALLBACK_VEL_MULTIPLIER': 0.5,
    'RUCKIG_FALLBACK_ACC_MULTIPLIER': 0.5,
    'RUCKIG_FALLBACK_MIN_VEL_SCALING': 0.1,
    'RUCKIG_FALLBACK_MIN_ACC_SCALING': 0.1,
    'OPT_SERVICE_TIMEOUT_S': 5.0,
    'BLOCKING_POS_THRESHOLD_MM': 0.2,
    'BLOCKING_MOVE_TIMEOUT_S': 60.0,
    'BLOCKING_CHECK_INTERVAL_S': 0.01,
    'STATUS_PUBLISH_RATE_HZ': 10.0,
    'MONITOR_UPDATE_RATE_HZ': 50.0,
    'RUNTIME_JOINT_STATE_INPUT_RATE_HZ': 0.0,
    'RUNTIME_DYNAMIC_STATE_INPUT_RATE_HZ': 0.0,
    'MONITOR_VELOCITY_WINDOW': 5,
    'MONITOR_ACCELERATION_WINDOW': 5,
    'MARKER_PUBLISH_INTERVAL_S': 2.0,
    'ACTIVE_TOOL_MARKER_PUBLISH_HZ': 1.0,
    'COLLISION_RATE_THRESHOLDS': [15.0, 15.0, 12.0, 8.0, 6.0, 4.0],
    'COLLISION_SUSTAINED_THRESHOLDS': [12.0, 12.0, 10.0, 8.0, 6.0, 5.0],
    'COLLISION_CONFIRMATION_SAMPLES': 1,
    'COLLISION_RECOVERY_TIME_S': 1.0,
    'COLLISION_FILTER_ALPHA': 0.7,
    'COLLISION_HISTORY_BUFFER': 100,
    'ETHERCAT_WATCHDOG_ENABLED': False,
    'ETHERCAT_EXPECTED_SLAVES': 6,
    'ETHERCAT_WATCHDOG_POLL_S': 0.5,
    'ETHERCAT_WATCHDOG_CMD_TIMEOUT_S': 1.0,
    'ETHERCAT_RECOVERY_ENABLED': True,
    'ETHERCAT_RECOVERY_MIN_INTERVAL_S': 2.0,
    'ETHERCAT_RECOVERY_CMD_TIMEOUT_S': 2.0,
    'MOTION_ERROR_HARDWARE_NOT_READY': -12,
    'MOTION_ERROR_DRIVE_NOT_ENABLED': -13,
    'MOTION_ERROR_CONTROLLER_EXECUTION_FAILED': -14,
    'DRIVE_ENABLE_SET_COMMAND_TOPIC': '/drive_enable_set_controller/commands',
    'DRIVE_DISABLE_SET_COMMAND_TOPIC': '/drive_disable_set_controller/commands',
    'DEFAULT_WORKOBJECT': [0, 0, 0, 0, 0, 0],
    'REST_HOST': '0.0.0.0',
    'REST_PORT': 5000,
    'REST_WS_STATE_ENABLED': True,
    'REST_WS_STATE_HOST': '0.0.0.0',
    'REST_WS_STATE_PORT': 5001,
    'REST_WS_STATE_RATE_HZ': 50.0,
    'REST_WS_EXECUTION_ENABLED': True,
    'REST_WS_EXECUTION_HOST': '0.0.0.0',
    'REST_WS_EXECUTION_PORT': 5002,
    'REST_WS_EXECUTION_RATE_HZ': 10.0,
    'WS_EXTRACT_MAX_RETRIES': 60,
    'WS_EXTRACT_RETRY_DELAY': 2.0,
    'MONITOR_WAIT_TIMEOUT_S': 10.0,

}

REQUIRED_KEYS = frozenset({
    'ROBOT_BACKEND',
    'NUM_JOINTS',
    'JOINT_NAMES',
    'PLANNING_GROUP',
    'BASE_LINK',
    'EE_LINK',
    'WRIST_LINK',
    'CARTESIAN_SOURCE_LINK',
    'COLLISION_TIP_LINK',
    'URDF_PATH',
    'ACTION_FOLLOW_TRAJECTORY',
    'REST_LOG',
})


def _config_package() -> str:
    package_name = os.environ.get('EROB_CONFIG_PACKAGE', '').strip()
    if not package_name:
        raise RuntimeError(
            'EROB_CONFIG_PACKAGE is not set. '
            'Launch the runtime through a robot-specific launch file or set '
            'EROB_CONFIG_PACKAGE explicitly.'
        )
    return package_name


def _runtime_yaml_path() -> Path | None:
    package_name = _config_package()
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package_name)) / 'config' / 'runtime.yaml'
    except Exception:
        source_root = Path(__file__).resolve().parents[2]
        candidate = source_root / package_name / 'config' / 'runtime.yaml'
        return candidate if candidate.exists() else None


def _profile_runtime_yaml_path(profile: str) -> Path | None:
    package_name = _config_package()
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory(package_name)) / 'config' / profile / 'runtime.yaml'
        return candidate if candidate.exists() else None
    except Exception:
        source_root = Path(__file__).resolve().parents[2]
        candidate = source_root / package_name / 'config' / profile / 'runtime.yaml'
        return candidate if candidate.exists() else None


def _config_yaml_path(filename: str) -> Path | None:
    package_name = _config_package()
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory(package_name)) / 'config' / filename
        return candidate if candidate.exists() else None
    except Exception:
        source_root = Path(__file__).resolve().parents[2]
        candidate = source_root / package_name / 'config' / filename
        return candidate if candidate.exists() else None


def _profile_config_yaml_path(profile: str, filename: str) -> Path | None:
    package_name = _config_package()
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory(package_name)) / 'config' / profile / filename
        return candidate if candidate.exists() else None
    except Exception:
        source_root = Path(__file__).resolve().parents[2]
        candidate = source_root / package_name / 'config' / profile / filename
        return candidate if candidate.exists() else None


def _load_yaml_file(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            merged = dict(result[key])
            merged.update(value)
            result[key] = merged
        else:
            result[key] = value
    return result


def _resolve_config_path(config_yaml: Path, value: Any) -> str:
    path = str(value or '').strip()
    if not path:
        return ''
    if path.startswith('package://'):
        package_and_rel = path[len('package://'):]
        package_name, _, rel_path = package_and_rel.partition('/')
        if package_name and rel_path:
            from ament_index_python.packages import get_package_share_directory
            return str(Path(get_package_share_directory(package_name)) / rel_path)
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return str(candidate)
    return str((config_yaml.parent / candidate).resolve())


PATH_KEYS = frozenset({
    'URDF_PATH',
    'SRDF_PATH',
})


def _load_runtime_config() -> dict[str, Any]:
    path = _runtime_yaml_path()
    if not path or not path.exists():
        return dict(DEFAULTS)
    active_path = path
    loaded = _load_yaml_file(path)
    config = _merge(DEFAULTS, loaded)

    for filename in ('contour_ik_config.yaml', 'ptp_config.yaml', 'jacobian_config.yaml'):
        config_path = _config_yaml_path(filename)
        if config_path and config_path.exists():
            config = _merge(config, _load_yaml_file(config_path))

    profile = str(config.get('ACTIVE_PROFILE', '')).strip()
    if profile:
        profile_path = _profile_runtime_yaml_path(profile)
        if not profile_path or not profile_path.exists():
            raise RuntimeError(
                f"Runtime config {path} requested ACTIVE_PROFILE '{profile}', "
                "but no profile runtime.yaml was found"
            )
        config = _merge(config, _load_yaml_file(profile_path))
        active_path = profile_path
        for filename in ('contour_ik_config.yaml', 'ptp_config.yaml', 'jacobian_config.yaml'):
            profile_config_path = _profile_config_yaml_path(profile, filename)
            if profile_config_path and profile_config_path.exists():
                config = _merge(config, _load_yaml_file(profile_config_path))

    for key in PATH_KEYS:
        if key in config and config[key]:
            config[key] = _resolve_config_path(active_path, config[key])

    missing = sorted(key for key in REQUIRED_KEYS if key not in config or config[key] in (None, ''))
    if missing:
        raise RuntimeError(
            f"Runtime config {path} is missing required keys: {', '.join(missing)}"
        )
    config['_RUNTIME_CONFIG_PATH'] = str(path)
    config['_ACTIVE_RUNTIME_CONFIG_PATH'] = str(active_path)
    config['WALL_BYPASS_LINKS'] = frozenset(config.get('WALL_BYPASS_LINKS', []))
    config['SAFETY_WALL_NAMES'] = frozenset(config.get('SAFETY_WALL_NAMES', []))
    config['TOOL_ID_MAP'] = {int(k): v for k, v in dict(config.get('TOOL_ID_MAP', {})).items()}
    config['WORKOBJECT_ID_MAP'] = {int(k): v for k, v in dict(config.get('WORKOBJECT_ID_MAP', {})).items()}
    default_workobject = list(config.get('DEFAULT_WORKOBJECT', [0, 0, 0, 0, 0, 0]) or [0, 0, 0, 0, 0, 0])
    if not config.get('WORKOBJECT_REGISTRY'):
        default_user_id = int(config.get('DEFAULT_WORKOBJECT_ID', 0) or 0)
        default_name = f'WOBJ_{default_user_id}'
        config['WORKOBJECT_REGISTRY'] = {default_name: default_workobject}
        config['WORKOBJECT_ID_MAP'] = {default_user_id: default_name}
    if 0 not in config['WORKOBJECT_ID_MAP']:
        config['WORKOBJECT_ID_MAP'][0] = 'WOBJ_0'
    if 'WOBJ_0' not in config['WORKOBJECT_REGISTRY']:
        config['WORKOBJECT_REGISTRY']['WOBJ_0'] = [0, 0, 0, 0, 0, 0]
    return config


_CONFIG = _load_runtime_config()
globals().update(_CONFIG)


def resolve_avoid_collisions(requested_value):
    """Resolve avoid_collisions flag based on global ENABLE_COLLISION_CHECKING.

    If ENABLE_COLLISION_CHECKING is False, always return False (disable all collision checks).
    Otherwise, use the requested_value (which may be None for default behavior).
    """
    if not _CONFIG.get('ENABLE_COLLISION_CHECKING', True):
        return False
    if requested_value is None:
        return _CONFIG.get('JOG_AVOID_COLLISIONS', True)
    return requested_value


def get_tool_registry_snapshot() -> dict[str, Any]:
    return {
        'tool_registry': {
            str(name): [float(v) for v in values]
            for name, values in dict(TOOL_REGISTRY).items()
        },
        'tool_id_map': {
            int(tool_id): str(name)
            for tool_id, name in dict(TOOL_ID_MAP).items()
        },
        'active_runtime_config_path': _CONFIG.get('_ACTIVE_RUNTIME_CONFIG_PATH'),
    }


def resolve_tool_name(tool_id: int | str) -> str:
    try:
        resolved_tool_id = int(tool_id)
    except (TypeError, ValueError):
        raise ValueError('tool_id must be an integer') from None
    if resolved_tool_id < 0:
        raise ValueError('tool_id must be non-negative')
    tool_name = TOOL_ID_MAP.get(resolved_tool_id, f'TOOL_{resolved_tool_id}')
    if tool_name not in TOOL_REGISTRY:
        raise ValueError(f'tool_id {resolved_tool_id} maps to unknown tool {tool_name!r}')
    return tool_name


def _validate_tool_transform(transform) -> list[float]:
    if not isinstance(transform, (list, tuple)) or len(transform) != 6:
        raise ValueError('transform must contain exactly 6 values [x, y, z, rx, ry, rz]')
    values = []
    for value in transform:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            raise ValueError('transform values must be numeric') from None
        if not (parsed == parsed and parsed not in (float('inf'), float('-inf'))):
            raise ValueError('transform values must be finite')
        values.append(parsed)
    return values


def _validate_tool_name(name: str) -> str:
    cleaned = str(name or '').strip()
    if not cleaned:
        raise ValueError('tool name must not be empty')
    if not all(ch.isalnum() or ch == '_' for ch in cleaned):
        raise ValueError('tool name may contain only letters, numbers, and underscores')
    return cleaned


def update_tool_registry(tool_id: int, name: str | None, transform, persist: bool = False) -> dict[str, Any]:
    try:
        resolved_tool_id = int(tool_id)
    except (TypeError, ValueError):
        raise ValueError('tool_id must be an integer') from None
    if resolved_tool_id < 0:
        raise ValueError('tool_id must be non-negative')

    current_name = TOOL_ID_MAP.get(resolved_tool_id, f'TOOL_{resolved_tool_id}')
    tool_name = _validate_tool_name(name or current_name)
    values = _validate_tool_transform(transform)

    TOOL_REGISTRY[tool_name] = values
    TOOL_ID_MAP[resolved_tool_id] = tool_name
    _CONFIG['TOOL_REGISTRY'] = TOOL_REGISTRY
    _CONFIG['TOOL_ID_MAP'] = TOOL_ID_MAP

    if persist:
        _persist_tool_registry()

    return get_tool_registry_snapshot()


def get_workobject_registry_snapshot() -> dict[str, Any]:
    return {
        'workobject_registry': {
            str(name): [float(v) for v in values]
            for name, values in dict(WORKOBJECT_REGISTRY).items()
        },
        'workobject_id_map': {
            int(user_id): str(name)
            for user_id, name in dict(WORKOBJECT_ID_MAP).items()
        },
        'default_workobject_id': int(_CONFIG.get('DEFAULT_WORKOBJECT_ID', 0) or 0),
        'active_runtime_config_path': _CONFIG.get('_ACTIVE_RUNTIME_CONFIG_PATH'),
    }


def resolve_workobject_name(user_id: int | str) -> str:
    try:
        resolved_user_id = int(user_id)
    except (TypeError, ValueError):
        raise ValueError('user_id must be an integer') from None
    if resolved_user_id < 0:
        raise ValueError('user_id must be non-negative')
    workobject_name = WORKOBJECT_ID_MAP.get(resolved_user_id, f'WOBJ_{resolved_user_id}')
    if workobject_name not in WORKOBJECT_REGISTRY:
        raise ValueError(f'user_id {resolved_user_id} maps to unknown workobject {workobject_name!r}')
    return workobject_name


def _validate_workobject_transform(transform) -> list[float]:
    return _validate_tool_transform(transform)


def _validate_workobject_name(name: str) -> str:
    cleaned = str(name or '').strip()
    if not cleaned:
        raise ValueError('workobject name must not be empty')
    if not all(ch.isalnum() or ch == '_' for ch in cleaned):
        raise ValueError('workobject name may contain only letters, numbers, and underscores')
    return cleaned


def update_workobject_registry(user_id: int, name: str | None, transform, persist: bool = False) -> dict[str, Any]:
    try:
        resolved_user_id = int(user_id)
    except (TypeError, ValueError):
        raise ValueError('user_id must be an integer') from None
    if resolved_user_id < 0:
        raise ValueError('user_id must be non-negative')

    current_name = WORKOBJECT_ID_MAP.get(resolved_user_id, f'WOBJ_{resolved_user_id}')
    workobject_name = _validate_workobject_name(name or current_name)
    values = _validate_workobject_transform(transform)

    WORKOBJECT_REGISTRY[workobject_name] = values
    WORKOBJECT_ID_MAP[resolved_user_id] = workobject_name
    _CONFIG['WORKOBJECT_REGISTRY'] = WORKOBJECT_REGISTRY
    _CONFIG['WORKOBJECT_ID_MAP'] = WORKOBJECT_ID_MAP

    if persist:
        _persist_workobject_registry()

    return get_workobject_registry_snapshot()


def _persist_tool_registry() -> None:
    path_value = _CONFIG.get('_ACTIVE_RUNTIME_CONFIG_PATH')
    if not path_value:
        raise RuntimeError('active runtime config path is unavailable')
    path = Path(path_value)
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    text = _replace_top_level_yaml_block(text, 'TOOL_REGISTRY', _format_tool_registry_block())
    text = _replace_top_level_yaml_block(text, 'TOOL_ID_MAP', _format_tool_id_map_block())
    path.write_text(text, encoding='utf-8')


def _persist_workobject_registry() -> None:
    path_value = _CONFIG.get('_ACTIVE_RUNTIME_CONFIG_PATH')
    if not path_value:
        raise RuntimeError('active runtime config path is unavailable')
    path = Path(path_value)
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    text = _replace_top_level_yaml_block(text, 'WORKOBJECT_REGISTRY', _format_workobject_registry_block())
    text = _replace_top_level_yaml_block(text, 'WORKOBJECT_ID_MAP', _format_workobject_id_map_block())
    path.write_text(text, encoding='utf-8')


def _format_tool_registry_block() -> list[str]:
    lines = ['TOOL_REGISTRY:']
    for name, values in dict(TOOL_REGISTRY).items():
        lines.append(f'  {name}:')
        for value in values:
            lines.append(f'  - {_format_yaml_number(float(value))}')
    return lines


def _format_tool_id_map_block() -> list[str]:
    lines = ['TOOL_ID_MAP:']
    for tool_id, name in dict(TOOL_ID_MAP).items():
        lines.append(f'  {int(tool_id)}: {name}')
    return lines


def _format_workobject_registry_block() -> list[str]:
    lines = ['WORKOBJECT_REGISTRY:']
    for name, values in dict(WORKOBJECT_REGISTRY).items():
        lines.append(f'  {name}:')
        for value in values:
            lines.append(f'  - {_format_yaml_number(float(value))}')
    return lines


def _format_workobject_id_map_block() -> list[str]:
    lines = ['WORKOBJECT_ID_MAP:']
    for user_id, name in dict(WORKOBJECT_ID_MAP).items():
        lines.append(f'  {int(user_id)}: {name}')
    return lines


def _format_yaml_number(value: float) -> str:
    if value.is_integer():
        return str(int(value))
    return f'{value:.12g}'


def _replace_top_level_yaml_block(text: str, key: str, replacement_lines: list[str]) -> str:
    lines = text.splitlines()
    key_prefix = f'{key}:'
    start = None
    for index, line in enumerate(lines):
        if line.startswith(key_prefix):
            start = index
            break

    if start is None:
        if lines and lines[-1].strip():
            lines.append('')
        lines.extend(replacement_lines)
        return '\n'.join(lines) + '\n'

    end = start + 1
    while end < len(lines):
        line = lines[end]
        if line and not line[0].isspace():
            break
        end += 1

    return '\n'.join(lines[:start] + replacement_lines + lines[end:]) + '\n'
