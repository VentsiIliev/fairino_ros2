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
    'NUM_JOINTS': 6,
    'JOINT_NAMES': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
    'PLANNING_GROUP': 'fairino5_v6_group',
    'BASE_LINK': 'base_link',
    'EE_LINK': 'ee_link',
    'WRIST_LINK': 'wrist3_link',
    'COLLISION_TIP_LINK': 'wrist3_link',
    'URDF_PATH': '/home/ilv/ros2_ws/src/fairino_description/urdf/fairino5_v6.urdf',
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
    'ACTION_FOLLOW_TRAJECTORY': '/fairino5_controller/follow_joint_trajectory',
    'SAFETY_WALLS_ENABLED': True,
    'SAFETY_MARGIN_M': 0.01,
    'WALL_XY_OFFSET_M': 0.13,
    'WALL_THICKNESS_M': 0.01,
    'WALL_BYPASS_LINKS': [],
    'SAFETY_WALL_NAMES': ['wall_x_min', 'wall_x_max', 'wall_y_min', 'wall_y_max', 'wall_z_min', 'wall_z_max'],
    'TOOL_REGISTRY': {'TOOL_0': [0, 0, 0, 0, 0, 0], 'TOOL_1': [0.081, -7.250, 0, 0, 0, 0]},
    'TOOL_ID_MAP': {0: 'TOOL_0', 1: 'TOOL_1'},
    'DEFAULT_VEL_PERCENT': 30,
    'DEFAULT_ACC_PERCENT': 30,
    'DEFAULT_VEL_SCALING': 0.6,
    'DEFAULT_ACC_SCALING': 0.4,
    'DEFAULT_JERK_SCALING': 0.5,
    'DEFAULT_ORIENTATION': [180.0, 0.0, 0.0],
    'CARTESIAN_MIN_FRACTION': 1,
    'JACOBIAN_FALLBACK_MM': 0.1,
    'JACOBIAN_FALLBACK_MIN_FRACTION': 1,
    'JACOBIAN_FALLBACK_MIN_DELTA_MM': 0.1,
    'SHORT_CARTESIAN_JACOBIAN_FALLBACK_MAX_DELTA_MM': 2.0,
    'JACOBIAN_MAX_JOINT_STEP': 0.05,
    'JACOBIAN_MIN_DURATION_S': 0.05,
    'JACOBIAN_DAMPING': 1e-5,
    'JACOBIAN_NUM_DIFF_EPS': 1e-7,
    'MOTION_QUEUE_MAX_SIZE': 10,
    'EXECUTOR_GOAL_POS_TOL_RAD': 0.01,
    'EXECUTOR_TIME_MULTIPLIER': 2.0,
    'EXECUTOR_TIME_MIN_S': 5.0,
    'PATH_APPROACH_THRESHOLD_MM': 100.0,
    'TRAJECTORY_OPTIMIZER': 'TOTG',
    'RUCKIG_SAMPLE_DT_S': 0.008,
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
    'DH_D1': 0.152,
    'DH_A2': -0.425,
    'DH_A3': -0.39501,
    'DH_D4': 0.1021,
    'DH_D5': 0.102,
    'DEFAULT_WORKOBJECT': [0, 0, 0, 0, 0, 0],
    'REST_HOST': '0.0.0.0',
    'REST_PORT': 5000,
    'REST_LOG': '/tmp/fairino_rest_server.log',
    'WS_EXTRACT_MAX_RETRIES': 60,
    'WS_EXTRACT_RETRY_DELAY': 2.0,
    'MONITOR_WAIT_TIMEOUT_S': 10.0,
}


def _config_package() -> str:
    return os.environ.get('EROB_CONFIG_PACKAGE', 'fairino5_v6_moveit2_config')


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
    config['WALL_BYPASS_LINKS'] = frozenset(config.get('WALL_BYPASS_LINKS', []))
    config['SAFETY_WALL_NAMES'] = frozenset(config.get('SAFETY_WALL_NAMES', []))
    config['TOOL_ID_MAP'] = {int(k): v for k, v in dict(config.get('TOOL_ID_MAP', {})).items()}
    return config


_CONFIG = _load_runtime_config()
globals().update(_CONFIG)
