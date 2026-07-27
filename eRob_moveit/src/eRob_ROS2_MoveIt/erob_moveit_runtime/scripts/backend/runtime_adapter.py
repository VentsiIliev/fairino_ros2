from __future__ import annotations

import config
import numpy as np
import time
from threading import Lock

from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray


ZEROERR_DRIVE_ERROR_CODES = {
    0x0000: "no active drive error",
    0x2214: "motor current is over current",
    0x2250: "sum of motor three phase current exceeds the limit",
    0x2341: "U phase over current",
    0x2342: "V phase over current",
    0x2343: "W phase over current",
    0x3210: "bus voltage is overvoltage",
    0x3220: "bus voltage is undervoltage",
    0x4110: "power component temperature is too high",
    0x7121: "blocked motor rotation",
    0x7305: "load-side single-turn encoder read data is incorrect",
    0x7306: "motor-side single-turn encoder read data is incorrect",
    0x730D: "battery warning error",
    0x730F: "battery low voltage",
    0x7311: "sampled motor-end position error exceeds the limit",
    0x7314: "battery reconnection detected; reset load-side encoder to clear alarm",
    0x7315: "sampled load-side position error exceeds the limit",
    0x7350: "motor-side encoder type is not supported",
    0x7374: "multi-turn position error",
    0x7377: "reset pin error detected",
    0x737A: "load-side single-turn encoder startup error",
    0x737E: "motor-side single-turn encoder startup error",
    0x8130: "CAN heartbeat error",
    0x8400: "velocity error exceeds the limit value",
    0x8401: "motor velocity exceeds the limit value",
    0x8500: "position error exceeds the limit value",
    0xA000: "master station offline / EtherCAT communication abnormal",
    0xF004: "EtherCAT initialization error",
    0xF005: "STO function is activated",
    0xF006: "multi-turn circle count error",
    0xF008: "bus voltage below minimum allowable supply voltage (19V)",
}


class RuntimeBackendAdapter:
    """Backend-specific runtime policy hooks for the shared MoveIt runtime."""

    backend_name = "generic"
    supports_drive_enable = False

    def get_monitor_tcp_transform(self, robot_controller):
        """Return the transform RobotMonitor should apply on /cartesian_position."""
        return robot_controller.T_monitor_tool

    def get_planning_tool_transform(self, robot_controller, registry_tool_transform):
        """Return the ee_link -> TCP transform used by MoveIt planning."""
        return registry_tool_transform

    def initialize_runtime_controller(self, robot_controller):
        """Initialize backend-specific runtime state after RobotController setup."""
        robot_controller.get_logger().info("[DriveEnable] Disabled for this robot backend")

    def on_joint_state(self, robot_controller, msg):
        """Observe /joint_states for backend-specific drive diagnostics."""
        return

    def set_drive_operation_enabled(self, robot_controller, enabled: bool) -> dict:
        return {
            "success": False,
            "enabled": False,
            "requested_enabled": bool(enabled),
            "state": "UNSUPPORTED",
            "controller_switch_ok": False,
        }

    def is_drive_operation_enabled_for_motion(self, robot_controller) -> bool:
        return True

    def get_drive_enable_fault_reason(self, robot_controller) -> str:
        return "drive enable is not required for this robot backend"

    def get_drive_operation_status(self, robot_controller) -> dict:
        return {
            "success": True,
            "supported": False,
            "requested_enabled": False,
            "actual_enabled": True,
            "motion_allowed_by_drive_enable": True,
            "state": "NOT_REQUIRED",
        }

    def reset_drive_operation_request(self, robot_controller) -> None:
        return

    def format_drive_state_snapshot(self, robot_controller, label: str):
        return None

    def log_drive_state_snapshot(self, robot_controller, label: str) -> None:
        return

    def log_drive_state_before_first_motion(self, robot_controller) -> None:
        return

    def get_unwind_drive_state(self, robot_controller, unwind_check):
        return None

    def get_all_drive_states(self, robot_controller):
        return []


class FairinoRuntimeAdapter(RuntimeBackendAdapter):
    backend_name = "fairino"

    def get_monitor_tcp_transform(self, robot_controller):
        # Fairino /cartesian_position is sourced from the mechanical flange.
        # The tool registry stores flange -> TCP transforms for calibrated tools.
        return robot_controller.T_monitor_tool

    def get_planning_tool_transform(self, robot_controller, registry_tool_transform):
        # MoveIt targets ee_link, so convert the flange -> TCP registry transform
        # into the equivalent ee_link -> TCP transform before removing TCP offset.
        if robot_controller.T_ee_link is None:
            return registry_tool_transform
        return np.linalg.inv(robot_controller.T_ee_link) @ registry_tool_transform


class ZeroErrRuntimeAdapter(RuntimeBackendAdapter):
    backend_name = "zeroerr"
    supports_drive_enable = True

    _DRIVE_SET_CONTROLLER_NAMES = (
        'drive_enable_set_controller',
        'drive_disable_set_controller',
    )

    def initialize_runtime_controller(self, robot_controller):
        self._drive_state_lock = Lock()
        self._drive_enable_lock = Lock()
        self._drive_operation_enabled_requested = False
        self._drive_joint_order = list(config.JOINT_NAMES)
        self._drive_mode_display = np.zeros(config.NUM_JOINTS, dtype=float)
        self._drive_statusword = np.zeros(config.NUM_JOINTS, dtype=float)
        self._drive_error_code = np.zeros(config.NUM_JOINTS, dtype=float)
        self._drive_effort_actual = np.zeros(config.NUM_JOINTS, dtype=float)
        self._drive_startup_snapshot_logged = False
        self._drive_pre_motion_snapshot_logged = False
        self._drive_enable_set_pub = robot_controller.create_publisher(
            Float64MultiArray,
            config.DRIVE_ENABLE_SET_COMMAND_TOPIC,
            10,
        )
        self._drive_disable_set_pub = robot_controller.create_publisher(
            Float64MultiArray,
            config.DRIVE_DISABLE_SET_COMMAND_TOPIC,
            10,
        )
        self._drive_state_sub = robot_controller.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            lambda msg: self._drive_state_callback(robot_controller, msg),
            10,
        )

    def on_joint_state(self, robot_controller, msg):
        if len(msg.effort) >= config.NUM_JOINTS:
            with self._drive_state_lock:
                self._drive_effort_actual = np.array(msg.effort[:config.NUM_JOINTS], dtype=float)

    def set_drive_operation_enabled(self, robot_controller, enabled: bool) -> dict:
        if enabled and not robot_controller.is_hardware_ready_for_motion():
            with self._drive_enable_lock:
                self._drive_operation_enabled_requested = False
            reason = robot_controller.get_hardware_fault_reason()
            robot_controller.get_logger().error(f'[DriveEnable] Enable rejected: {reason}')
            return {
                "success": False,
                "enabled": False,
                "requested_enabled": False,
                "state": "HARDWARE_NOT_READY",
                "controller_switch_ok": False,
                "error": reason,
            }
        switch_ok = self._ensure_drive_set_controllers_active(robot_controller)
        if not switch_ok:
            return {
                "enabled": False,
                "requested_enabled": bool(enabled),
                "state": "ERROR",
                "controller_switch_ok": False,
                "error": "failed to activate enable_set/disable_set controllers",
            }

        if enabled:
            robot_controller._send_hold_position_trajectory(
                reason='drive enable',
                suppress_drive_disable_cancel=True,
            )
            time.sleep(0.25)

        pulse = np.ones(config.NUM_JOINTS, dtype=float)
        zeros = np.zeros(config.NUM_JOINTS, dtype=float)
        enable_msg = Float64MultiArray()
        disable_msg = Float64MultiArray()
        if enabled:
            enable_msg.data = pulse.tolist()
            disable_msg.data = zeros.tolist()
        else:
            enable_msg.data = zeros.tolist()
            disable_msg.data = pulse.tolist()
        self._drive_enable_set_pub.publish(enable_msg)
        self._drive_disable_set_pub.publish(disable_msg)
        time.sleep(0.05)
        enable_msg.data = zeros.tolist()
        disable_msg.data = zeros.tolist()
        self._drive_enable_set_pub.publish(enable_msg)
        self._drive_disable_set_pub.publish(disable_msg)
        # Give the zero reset one controller cycle window before releasing set interfaces.
        time.sleep(0.05)
        deactivate_ok = self._deactivate_drive_set_controllers(robot_controller)
        with self._drive_enable_lock:
            self._drive_operation_enabled_requested = bool(enabled)
        robot_controller.get_logger().info(
            f"[DriveEnable] {'Enable' if enabled else 'Disable'} operation requested via enable_set/disable_set"
        )
        return {
            "success": True,
            "requested_enabled": bool(enabled),
            "state": "ENABLE_REQUESTED" if enabled else "DISABLE_REQUESTED",
            "controller_switch_ok": True,
            "controller_deactivate_ok": deactivate_ok,
            "mode": "csp",
        }

    def is_drive_operation_enabled_for_motion(self, robot_controller) -> bool:
        with self._drive_enable_lock:
            requested_enabled = bool(self._drive_operation_enabled_requested)
        if not requested_enabled:
            return False
        with self._drive_state_lock:
            statusword = [int(round(value)) for value in self._drive_statusword.tolist()]
        if len(statusword) < config.NUM_JOINTS:
            return False
        return all(
            self._decode_statusword_state(value) == 'operation_enabled'
            for value in statusword[:config.NUM_JOINTS]
        )

    def get_drive_enable_fault_reason(self, robot_controller) -> str:
        with self._drive_enable_lock:
            requested_enabled = bool(self._drive_operation_enabled_requested)
        if not requested_enabled:
            return "drive operation is not enabled; call POST /drive/enable before motion"
        with self._drive_state_lock:
            statusword = [int(round(value)) for value in self._drive_statusword.tolist()]
        statusword_state = [
            self._decode_statusword_state(value)
            for value in statusword[:config.NUM_JOINTS]
        ]
        return (
            "drive enable was requested, but not all drives report operation_enabled "
            f"(status_state={statusword_state})"
        )

    def get_drive_operation_status(self, robot_controller) -> dict:
        with self._drive_enable_lock:
            requested_enabled = bool(self._drive_operation_enabled_requested)
        with self._drive_state_lock:
            statusword = [int(round(value)) for value in self._drive_statusword.tolist()]
        statusword_state = [
            self._decode_statusword_state(value)
            for value in statusword[:config.NUM_JOINTS]
        ]
        actual_enabled = (
            len(statusword_state) == config.NUM_JOINTS
            and all(state == 'operation_enabled' for state in statusword_state)
        )
        return {
            "success": True,
            "requested_enabled": requested_enabled,
            "actual_enabled": actual_enabled,
            "motion_allowed_by_drive_enable": requested_enabled and actual_enabled,
            "state": "OPERATION_ENABLED" if actual_enabled else (
                "ENABLE_REQUESTED" if requested_enabled else "DISABLED"
            ),
            "statusword": statusword,
            "status_state": statusword_state,
        }

    def reset_drive_operation_request(self, robot_controller) -> None:
        with self._drive_enable_lock:
            self._drive_operation_enabled_requested = False

    def _ensure_drive_set_controllers_active(self, robot_controller) -> bool:
        return self._switch_drive_set_controllers(robot_controller, activate=True)

    def _deactivate_drive_set_controllers(self, robot_controller) -> bool:
        return self._switch_drive_set_controllers(robot_controller, activate=False)

    def _switch_drive_set_controllers(self, robot_controller, *, activate: bool) -> bool:
        controller_states = robot_controller._get_controller_states()
        if controller_states is None:
            return False
        if not robot_controller.switch_controller_client.wait_for_service(timeout_sec=2.0):
            robot_controller.get_logger().error('[DriveEnable] /controller_manager/switch_controller not available')
            return False

        from builtin_interfaces.msg import Duration
        from controller_manager_msgs.srv import SwitchController

        request = SwitchController.Request()
        request.strictness = 2
        request.activate_asap = True
        request.timeout = Duration(sec=2, nanosec=0)
        if activate:
            request.activate_controllers = [
                name for name in self._DRIVE_SET_CONTROLLER_NAMES
                if controller_states.get(name) != 'active'
            ]
            request.deactivate_controllers = []
            action = 'Activating'
            failure_action = 'activation'
        else:
            request.activate_controllers = []
            request.deactivate_controllers = [
                name for name in self._DRIVE_SET_CONTROLLER_NAMES
                if controller_states.get(name) == 'active'
            ]
            action = 'Deactivating'
            failure_action = 'deactivation'

        target_controllers = request.activate_controllers or request.deactivate_controllers
        if not target_controllers:
            return True

        robot_controller.get_logger().info(
            f"[DriveEnable] {action} set controllers: {target_controllers}"
        )
        future = robot_controller.switch_controller_client.call_async(request)
        response = robot_controller._wait_for_service_future(future, timeout_s=3.0)
        if response is None:
            robot_controller.get_logger().error(f'[DriveEnable] Timed out during set controller {failure_action}')
            return False
        if not response.ok:
            robot_controller.get_logger().error(
                f'[DriveEnable] Controller manager rejected set controller {failure_action}'
            )
            return False
        return True

    def _drive_state_callback(self, robot_controller, msg: DynamicJointState):
        if robot_controller._runtime_dynamic_input_period > 0.0:
            now = time.monotonic()
            if now - robot_controller._last_runtime_dynamic_input_ts < robot_controller._runtime_dynamic_input_period:
                return
            robot_controller._last_runtime_dynamic_input_ts = now

        joint_index = {name: idx for idx, name in enumerate(self._drive_joint_order)}
        statusword = np.zeros(config.NUM_JOINTS, dtype=float)
        error_code = np.zeros(config.NUM_JOINTS, dtype=float)
        mode_display = self._drive_mode_display.copy()
        for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
            index = joint_index.get(joint_name)
            if index is None:
                continue
            for name, value in zip(interface_value.interface_names, interface_value.values):
                if name == 'statusword' and np.isfinite(value):
                    statusword[index] = float(value)
                elif name == 'error_code' and np.isfinite(value):
                    error_code[index] = float(value)
                elif name == 'mode_display' and np.isfinite(value):
                    mode_display[index] = float(value)
        with self._drive_state_lock:
            self._drive_statusword = statusword
            self._drive_error_code = error_code
            self._drive_mode_display = mode_display

    def format_drive_state_snapshot(self, robot_controller, label: str):
        with self._drive_state_lock:
            statusword = [int(round(value)) for value in self._drive_statusword.tolist()]
            error_code = [int(round(value)) for value in self._drive_error_code.tolist()]
            mode_display = [int(round(value)) for value in self._drive_mode_display.tolist()]
            effort = [round(value, 3) for value in self._drive_effort_actual.tolist()]
        statusword_bits = [self._decode_statusword_bits(value) for value in statusword]
        statusword_state = [self._decode_statusword_state(value) for value in statusword]
        error_text = [self._decode_drive_error_code(value) for value in error_code]
        return (
            f'[DriveState] {label}: '
            f'statusword={statusword} '
            f'status_state={statusword_state} '
            f'status_bits={statusword_bits} '
            f'error_code={[f"0x{value:04X}" for value in error_code]} '
            f'error_text={error_text} '
            f'mode_display={mode_display} '
            f'effort={effort}'
        )

    def _maybe_log_drive_state_snapshot(self, robot_controller, label: str):
        with self._drive_state_lock:
            if label == 'startup':
                if self._drive_startup_snapshot_logged:
                    return
                self._drive_startup_snapshot_logged = True
            elif label == 'before_first_motion':
                if self._drive_pre_motion_snapshot_logged:
                    return
                self._drive_pre_motion_snapshot_logged = True
            else:
                return
        robot_controller.get_logger().info(self.format_drive_state_snapshot(robot_controller, label))

    def log_drive_state_before_first_motion(self, robot_controller) -> None:
        self._maybe_log_drive_state_snapshot(robot_controller, 'before_first_motion')

    def log_drive_state_snapshot(self, robot_controller, label: str) -> None:
        self._maybe_log_drive_state_snapshot(robot_controller, label)

    def get_unwind_drive_state(self, robot_controller, unwind_check):
        joint_name = str(unwind_check.get('joint_name') or '')
        joint_index = unwind_check.get('joint_index')
        drive_order = list(self._drive_joint_order or [])
        if joint_name in drive_order:
            joint_index = drive_order.index(joint_name)

        try:
            joint_index = int(joint_index)
        except (TypeError, ValueError):
            return None

        with self._drive_state_lock:
            status_values = [int(round(value)) for value in self._drive_statusword.tolist()]
        if joint_index < 0 or joint_index >= len(status_values):
            return None

        statusword = status_values[joint_index]
        return {
            'joint_name': joint_name,
            'joint_index': joint_index,
            'statusword': statusword,
            'state': self._decode_statusword_state(statusword),
            'bits': self._decode_statusword_bits(statusword),
        }

    def get_all_drive_states(self, robot_controller):
        with self._drive_state_lock:
            status_values = [int(round(value)) for value in self._drive_statusword.tolist()]
        if not status_values:
            return []

        joint_names = list(getattr(config, 'JOINT_NAMES', []) or [])
        states = []
        for index, statusword in enumerate(status_values):
            joint_name = joint_names[index] if index < len(joint_names) else f'joint_{index + 1}'
            states.append({
                'joint_name': joint_name,
                'joint_index': index,
                'statusword': statusword,
                'state': self._decode_statusword_state(statusword),
                'bits': self._decode_statusword_bits(statusword),
            })
        return states

    def _decode_statusword_bits(self, statusword: int) -> str:
        flags = (
            ("rtso", 0),
            ("so", 1),
            ("oe", 2),
            ("f", 3),
            ("ve", 4),
            ("qs", 5),
            ("sod", 6),
            ("w", 7),
            ("rm", 9),
            ("tr", 10),
            ("ila", 11),
        )
        active = [name for name, bit in flags if statusword & (1 << bit)]
        return "+".join(active) if active else "-"

    def _decode_statusword_state(self, statusword: int) -> str:
        state_code = statusword & 0x006F
        state_map = {
            0x0000: 'not_ready_to_switch_on',
            0x0040: 'switch_on_disabled',
            0x0021: 'ready_to_switch_on',
            0x0023: 'switched_on',
            0x0027: 'operation_enabled',
            0x0007: 'quick_stop_active',
            0x000F: 'fault_reaction_active',
            0x0008: 'fault',
        }
        return state_map.get(state_code, f'unknown(0x{state_code:04X})')

    def _decode_drive_error_code(self, error_code: int) -> str:
        return ZEROERR_DRIVE_ERROR_CODES.get(error_code, "unknown drive error code")

    def get_planning_tool_transform(self, robot_controller, registry_tool_transform):
        # ZeroErr can publish /cartesian_position from the wrist/flange frame
        # while MoveIt still plans on ee_link. In that mode the registry stores
        # wrist/flange -> TCP and planning needs ee_link -> TCP.
        source_link = str(getattr(config, "CARTESIAN_SOURCE_LINK", config.EE_LINK))
        if source_link == config.WRIST_LINK and robot_controller.T_ee_link is not None:
            return np.linalg.inv(robot_controller.T_ee_link) @ registry_tool_transform
        return registry_tool_transform


def create_runtime_adapter():
    backend_name = str(getattr(config, "ROBOT_BACKEND", "generic")).lower()
    if backend_name == "fairino":
        return FairinoRuntimeAdapter()
    if backend_name == "zeroerr":
        return ZeroErrRuntimeAdapter()
    return RuntimeBackendAdapter()
