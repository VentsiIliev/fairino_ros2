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
    'TOPIC_SAFETY_WALLS': '/safety_walls',
    'SERVICE_CARTESIAN_PATH': '/compute_cartesian_path',
    'SERVICE_IK': '/compute_ik',
    'SERVICE_FK': '/compute_fk',
    'SERVICE_APPLY_IPP': '/apply_ipp',
    'SERVICE_STATE_VALIDITY': '/check_state_validity',
    'SAFETY_WALLS_ENABLED': True,
    'SAFETY_MARGIN_M': 0.01,
    'WALL_XY_OFFSET_M': 0.13,
    'WALL_THICKNESS_M': 0.01,
    'WALL_BYPASS_LINKS': [],
    'SAFETY_WALL_NAMES': ['wall_x_min', 'wall_x_max', 'wall_y_min', 'wall_y_max', 'wall_z_min', 'wall_z_max'],
    'TOOL_REGISTRY': {'TOOL_0': [0, 0, 0, 0, 0, 0]},
    'TOOL_ID_MAP': {0: 'TOOL_0'},
    'DEFAULT_VEL_PERCENT': 30,
    'DEFAULT_ACC_PERCENT': 30,
    'DEFAULT_VEL_SCALING': 0.6,
    'DEFAULT_ACC_SCALING': 0.4,
    'DEFAULT_JERK_SCALING': 0.5,
    'DEFAULT_ORIENTATION': [180.0, 0.0, 0.0],
    'PTP_LOCK_ORIENTATION_TOL_DEG': 2.0,
    'PTP_LOCKED_PATH_MAX_DRIFT_DEG': 2.0,
    'PTP_ORIENTED_PATH_MAX_DEVIATION_DEG': 5.0,
    'PTP_WRIST_PENALTY_START_DEG': 45.0,
    'PTP_MAX_WRIST_DELTA_DEG': 160.0,
    'PTP_LOCKED_MAX_WRIST_DELTA_DEG': 120.0,
    'PTP_NOOP_JOINT_DELTA_RAD': 0.001,
    'PTP_JOINT_INTERPOLATION_STEP_RAD': 0.08,
    'PTP_MIN_INTERPOLATION_SEGMENTS': 8,
    'PTP_MAX_INTERPOLATION_SEGMENTS': 80,
    'JOG_AVOID_COLLISIONS': True,
    'JOG_BLOCKING_TIMEOUT_S': 5.0,
    'CARTESIAN_MIN_FRACTION': 1,
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
    'EXECUTOR_TIME_MULTIPLIER': 2.0,
    'EXECUTOR_TIME_MIN_S': 8.0,
    'EXECUTOR_START_HOLD_S': 0.06,
    'EXECUTOR_START_RAMP_POINTS': 3,
    'EXECUTOR_POST_UNWIND_ENABLED': False,
    'EXECUTOR_POST_UNWIND_JOINT_NAME': 'Joint_6',
    'EXECUTOR_POST_UNWIND_TARGET_RANGE_RAD': 3.141592653589793,
    'EXECUTOR_POST_UNWIND_MIN_DELTA_RAD': 0.5,
    'EXECUTOR_POST_UNWIND_SPEED_RAD_S': 1.2,
    'EXECUTOR_POST_UNWIND_ACCEL_RAD_S2': 1.5,
    'EXECUTOR_POST_UNWIND_MIN_DURATION_S': 1.5,
    'EXECUTOR_POST_UNWIND_SKIP_NOOP_DURATION_S': 1.0,
    'EXECUTOR_POST_UNWIND_SKIP_NOOP_MAX_JOINT_DELTA_RAD': 0.02,
    'OPTIMIZER_START_ALIGN_TOL_RAD': 0.002,
    'OPTIMIZER_START_MERGE_TOL_RAD': 0.002,
    'PATH_APPROACH_THRESHOLD_MM': 100.0,
    'TRAJECTORY_OPTIMIZER': 'TOTG',
    'PATH_TRAJECTORY_OPTIMIZER': 'RUCKIG',
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
    'MONITOR_VELOCITY_WINDOW': 5,
    'MONITOR_ACCELERATION_WINDOW': 5,
    'MARKER_PUBLISH_INTERVAL_S': 2.0,
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
    'MOTION_ERROR_HARDWARE_NOT_READY': -12,
    'DRAG_MODE_ENABLED_DEFAULT': False,
    'DRAG_MODE_UPDATE_RATE_HZ': 50.0,
    'DRAG_MODE_MODE_COMMAND_TOPIC': '/drag_mode_controller/commands',
    'DRAG_MODE_EFFORT_COMMAND_TOPIC': '/drag_effort_controller/commands',
    'DRAG_MODE_TORQUE_OFFSET_COMMAND_TOPIC': '/drag_torque_offset_controller/commands',
    'DRAG_MODE_ENABLE_SET_COMMAND_TOPIC': '/drag_enable_set_controller/commands',
    'DRAG_MODE_DISABLE_SET_COMMAND_TOPIC': '/drag_disable_set_controller/commands',
    'DRAG_MODE_CSP_VALUE': 8.0,
    'DRAG_MODE_CST_VALUE': 10.0,
    'DRAG_MODE_COMPENSATION_SCALE': 1.0,
    'DRAG_MODE_JOINT_COMPENSATION_SCALE': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    'DRAG_MODE_DAMPING_NM_PER_RAD_S': [1.5, 1.5, 1.2, 0.8, 0.5, 0.3],
    'DRAG_MODE_MAX_EFFORT_NM': [10.0, 10.0, 8.0, 5.0, 3.0, 2.0],
    'DRAG_MODE_MAX_TORQUE_OFFSET_NM': [60.0, 60.0, 45.0, 25.0, 12.0, 8.0],
    'DRAG_MODE_MODE_SETTLE_TIMEOUT_S': 2.0,
    'DRAG_MODE_DISABLE_PULSE_S': 0.10,
    'DRAG_MODE_ENABLE_PULSE_S': 0.10,
    'DRAG_MODE_CONFIG_PATH': '/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/drag_mode_config.json',
    'DRAG_MODE_JOINT_MODELS': ['eRob80H100T', 'eRob80H100T', 'eRob80H100T', 'eRob70H100T', 'eRob70H100T', 'eRob70H100T'],
    'DRAG_MODE_MODEL_NAMES': ['eRob70H100T', 'eRob80H100T'],
    'DRAG_MODE_MODEL_RATED_CURRENT_MA': [3500.0, 5500.0],
    'DRAG_MODE_MODEL_OUTPUT_TORQUE_CONSTANT_NM_PER_A': [4.76, 8.475],
    'DEFAULT_WORKOBJECT': [0, 0, 0, 0, 0, 0],
    'REST_HOST': '0.0.0.0',
    'REST_PORT': 5000,
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


def _load_runtime_config() -> dict[str, Any]:
    path = _runtime_yaml_path()
    if not path or not path.exists():
        return dict(DEFAULTS)
    with path.open('r', encoding='utf-8') as handle:
        loaded = yaml.safe_load(handle) or {}
    config = _merge(DEFAULTS, loaded)
    missing = sorted(key for key in REQUIRED_KEYS if key not in config or config[key] in (None, ''))
    if missing:
        raise RuntimeError(
            f"Runtime config {path} is missing required keys: {', '.join(missing)}"
        )
    config['WALL_BYPASS_LINKS'] = frozenset(config.get('WALL_BYPASS_LINKS', []))
    config['SAFETY_WALL_NAMES'] = frozenset(config.get('SAFETY_WALL_NAMES', []))
    config['TOOL_ID_MAP'] = {int(k): v for k, v in dict(config.get('TOOL_ID_MAP', {})).items()}
    return config


_CONFIG = _load_runtime_config()
globals().update(_CONFIG)
