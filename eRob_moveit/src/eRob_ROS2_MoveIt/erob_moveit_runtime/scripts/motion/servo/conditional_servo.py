from __future__ import annotations

import math
import threading
import time
import uuid
from copy import deepcopy


TERMINAL_STATES = {
    "condition_met",
    "boundary_reached",
    "timeout",
    "sensor_fault",
    "cancelled",
    "start_failed",
    "stop_failed",
}


class ConditionalServoSupervisor:
    """Own one event-driven Servo-until-condition operation in ROS 2."""

    def __init__(self, robot_getter, *, logger, monitor_rate_hz: float = 50.0):
        self._robot_getter = robot_getter
        self._logger = logger
        self._period_s = 1.0 / max(float(monitor_rate_hz), 1.0)
        self._lock = threading.RLock()
        self._operation = None
        self._sensor_connected = False
        self._previous_joint_positions = None
        self._previous_joint_sample_ns = None
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="ConditionalServoMonitor",
        )
        self._monitor_thread.start()

    def start(self, *, servo: dict, condition: dict, boundary: dict | None,
              timeout_s: float, sensor_stale_timeout_s: float = 0.5,
              restore_collision_checking: bool = True) -> dict:
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and > 0")
        sensor_stale_timeout_s = float(sensor_stale_timeout_s)
        if not math.isfinite(sensor_stale_timeout_s) or sensor_stale_timeout_s <= 0.0:
            raise ValueError("sensor_stale_timeout_s must be finite and > 0")
        sensor = str(condition.get("source") or "").strip()
        if not sensor:
            raise ValueError("condition.source is required")
        required_state = bool(condition.get("required_state", True))
        require_fresh_transition = bool(condition.get("require_fresh_transition", True))
        parsed_boundary = self._parse_boundary(boundary)
        if parsed_boundary is not None:
            servo_axis = str(getattr(servo.get("axis"), "name", servo.get("axis"))).strip().lower()
            servo_direction = int(getattr(servo.get("direction"), "value", servo.get("direction")))
            if servo_axis != parsed_boundary["axis"]:
                raise ValueError("boundary.axis must match the commanded Servo axis")
            expected_operator = "less_or_equal" if servo_direction < 0 else "greater_or_equal"
            if parsed_boundary["operator"] != expected_operator:
                raise ValueError("boundary.operator does not match the commanded Servo direction")
            if parsed_boundary["user"] != int(servo.get("user", 0)):
                raise ValueError("boundary.user must match servo.user")
            if parsed_boundary["tool"] != int(servo.get("tool", 0)):
                raise ValueError("boundary.tool must match servo.tool")

        with self._lock:
            if not self._sensor_connected:
                raise RuntimeError("sensor websocket is not connected")
            if self._operation is not None and self._operation["state"] not in TERMINAL_STATES:
                raise RuntimeError("a conditional Servo operation is already active")

            operation_id = str(uuid.uuid4())
            now_ns = time.monotonic_ns()
            operation = {
                "operation_id": operation_id,
                "state": "arming",
                "reason": None,
                "servo": dict(servo),
                "condition": {
                    "source": sensor,
                    "required_state": required_state,
                    "require_fresh_transition": require_fresh_transition,
                },
                "boundary": parsed_boundary,
                "restore_collision_checking": bool(restore_collision_checking),
                "armed_monotonic_ns": now_ns,
                "started_monotonic_ns": None,
                "trigger_monotonic_ns": None,
                "stop_started_monotonic_ns": None,
                "stopped_monotonic_ns": None,
                "deadline_monotonic_ns": now_ns + int(timeout_s * 1_000_000_000),
                "timeout_s": timeout_s,
                "sensor_stale_timeout_s": sensor_stale_timeout_s,
                "sensor_connected": self._sensor_connected,
                "sensor_stream_id": None,
                "sensor_baseline_sequence": None,
                "sensor_last_sequence": None,
                "sensor_ready": not require_fresh_transition,
                "sensor_last_state": None,
                "sensor_last_received_monotonic_ns": None,
                "sensor_detected_monotonic_ns": None,
                "final_pose": None,
                "stop_result": None,
            }
            self._operation = operation

        robot = self._robot_getter()
        if robot is None:
            self._finish_without_stop(operation_id, "start_failed", "robot runtime unavailable")
            return self.snapshot()
        try:
            result = robot.start_servo_jog(**servo)
        except Exception as exc:
            self._logger.exception("Conditional Servo start failed")
            self._finish_without_stop(operation_id, "start_failed", str(exc))
            return self.snapshot()

        if result != 0:
            self._finish_without_stop(operation_id, "start_failed", f"servo_start_failed:{result}")
            return self.snapshot()
        with self._lock:
            if self._matches_active(operation_id):
                self._operation["state"] = "moving"
                self._operation["started_monotonic_ns"] = time.monotonic_ns()
        self._log_transition(operation_id, "moving")
        return self.snapshot()

    def accept_sensor_event(self, event: dict) -> bool:
        if not isinstance(event, dict) or "state" not in event:
            return False
        sensor = str(event.get("sensor") or "").strip()
        stream_id = str(event.get("stream_id") or "").strip()
        try:
            sequence = int(event["sequence"])
        except (KeyError, TypeError, ValueError):
            return False
        raw_state = event.get("state")
        if isinstance(raw_state, str):
            normalized = raw_state.strip().lower()
            if normalized in {"active", "true", "on", "1"}:
                state = True
            elif normalized in {"inactive", "false", "off", "0"}:
                state = False
            else:
                self._trigger_stop("sensor_fault", f"invalid_sensor_state:{raw_state}")
                return False
        else:
            state = bool(raw_state)

        should_trigger = False
        with self._lock:
            op = self._operation
            if op is None or op["state"] != "moving" or sensor != op["condition"]["source"]:
                return False
            if stream_id:
                if op["sensor_stream_id"] not in (None, stream_id):
                    op["sensor_ready"] = False
                    op["sensor_baseline_sequence"] = None
                op["sensor_stream_id"] = stream_id
            last_sequence = op["sensor_last_sequence"]
            if last_sequence is not None and sequence <= last_sequence:
                return False
            op["sensor_last_sequence"] = sequence
            received_ns = time.monotonic_ns()
            op["sensor_last_received_monotonic_ns"] = received_ns
            op["sensor_last_state"] = state
            required = op["condition"]["required_state"]
            if op["condition"]["require_fresh_transition"] and not op["sensor_ready"]:
                if state != required:
                    op["sensor_ready"] = True
                    op["sensor_baseline_sequence"] = sequence
                return True
            should_trigger = state == required
            if should_trigger:
                detected_ns = event.get("detected_monotonic_ns")
                if isinstance(detected_ns, int) and 0 < detected_ns <= received_ns:
                    op["sensor_detected_monotonic_ns"] = detected_ns
                    op["sensor_transport_latency_ms"] = (received_ns - detected_ns) / 1_000_000.0
        if should_trigger:
            self._trigger_stop("condition_met", "external condition met")
        return True

    def set_sensor_connected(self, connected: bool) -> None:
        with self._lock:
            self._sensor_connected = bool(connected)
            if self._operation is not None:
                self._operation["sensor_connected"] = self._sensor_connected
            active = self._operation is not None and self._operation["state"] == "moving"
        if not connected and active:
            self._trigger_stop("sensor_fault", "sensor websocket disconnected")

    def observe_joint_state(self, positions, velocities=None) -> None:
        try:
            current_positions = [float(value) for value in positions[:6]]
        except (TypeError, ValueError):
            return
        if len(current_positions) < 6:
            return
        now_ns = time.monotonic_ns()
        measured_velocities = None
        try:
            raw_velocities = [] if velocities is None else list(velocities)
            candidate = [float(value) for value in raw_velocities[:6]]
            if len(candidate) == 6 and all(math.isfinite(value) for value in candidate):
                measured_velocities = candidate
        except (TypeError, ValueError):
            pass
        with self._lock:
            if measured_velocities is None and self._previous_joint_positions is not None:
                dt_s = (now_ns - self._previous_joint_sample_ns) / 1_000_000_000.0
                if dt_s > 1e-6:
                    measured_velocities = [
                        (current_positions[index] - self._previous_joint_positions[index]) / dt_s
                        for index in range(6)
                    ]
            self._previous_joint_positions = current_positions
            self._previous_joint_sample_ns = now_ns
            op = self._operation
            if op is None or op["state"] != "awaiting_stationary" or measured_velocities is None:
                return
            max_abs = max(abs(value) for value in measured_velocities)
            op["max_abs_joint_velocity_rad_s"] = max_abs
            op["stationary_samples"] = op.get("stationary_samples", 0) + 1 if max_abs <= 0.01 else 0
            if op["stationary_samples"] < 3:
                return
            op["state"] = op["trigger_state"]
            op["stopped_monotonic_ns"] = now_ns
            operation_id = op["operation_id"]
            terminal_state = op["state"]
            reason = op["reason"]
        self._log_transition(operation_id, terminal_state, reason)

    def cancel(self) -> dict:
        self._trigger_stop("cancelled", "cancel requested")
        return self.snapshot()

    def snapshot(self) -> dict:
        with self._lock:
            if self._operation is None:
                return {"active": False, "state": "idle", "sensor_connected": self._sensor_connected}
            result = deepcopy(self._operation)
        result["active"] = result["state"] not in TERMINAL_STATES
        return result

    def _monitor_loop(self) -> None:
        while True:
            time.sleep(self._period_s)
            with self._lock:
                op = deepcopy(self._operation)
            if op is None:
                continue
            if op["state"] == "awaiting_stationary":
                if time.monotonic_ns() - op["stop_started_monotonic_ns"] >= 2_000_000_000:
                    with self._lock:
                        if self._matches_active(op["operation_id"]) and self._operation["state"] == "awaiting_stationary":
                            self._operation["state"] = "stop_failed"
                            self._operation["reason"] = "stationary confirmation timeout"
                    self._log_transition(op["operation_id"], "stop_failed", "stationary confirmation timeout")
                continue
            if op["state"] != "moving":
                continue
            if time.monotonic_ns() >= op["deadline_monotonic_ns"]:
                self._trigger_stop("timeout", "operation timeout")
                continue
            last_sensor_ns = op.get("sensor_last_received_monotonic_ns") or op["armed_monotonic_ns"]
            if time.monotonic_ns() - last_sensor_ns >= int(op["sensor_stale_timeout_s"] * 1_000_000_000):
                self._trigger_stop("sensor_fault", "sensor stream stale")
                continue
            boundary = op.get("boundary")
            if boundary is None:
                continue
            robot = self._robot_getter()
            try:
                pose = robot.get_current_position(user_id=boundary["user"]) if robot is not None else None
            except Exception:
                self._logger.exception("Conditional Servo boundary pose read failed")
                self._trigger_stop("sensor_fault", "boundary_pose_unavailable")
                continue
            if pose is None or len(pose) < 3:
                continue
            value = float(pose[boundary["axis_index"]])
            reached = value <= boundary["value_mm"] if boundary["operator"] == "less_or_equal" else value >= boundary["value_mm"]
            if reached:
                self._trigger_stop("boundary_reached", f"{boundary['axis']}={value:.3f}", final_pose=pose)

    def _trigger_stop(self, state: str, reason: str, final_pose=None) -> None:
        with self._lock:
            op = self._operation
            if op is None or op["state"] != "moving":
                return
            operation_id = op["operation_id"]
            op["state"] = "stopping"
            op["reason"] = reason
            op["trigger_state"] = state
            op["trigger_monotonic_ns"] = time.monotonic_ns()
            op["stop_started_monotonic_ns"] = time.monotonic_ns()
            if final_pose is not None:
                op["final_pose"] = [float(value) for value in final_pose[:6]]
        self._log_transition(operation_id, "stopping", reason)
        robot = self._robot_getter()
        try:
            result = robot.stop_servo_jog(
                restore_collision_checking=bool(op["restore_collision_checking"])
            ) if robot is not None else -1
        except Exception as exc:
            self._logger.exception("Conditional Servo local stop failed")
            result = -1
            reason = f"{reason}; stop_exception:{exc}"
        stopped_ns = time.monotonic_ns()
        with self._lock:
            if not self._matches_active(operation_id):
                return
            self._operation["stop_result"] = result
            self._operation["reason"] = reason
            self._operation["state"] = "awaiting_stationary" if result == 0 else "stop_failed"
            self._operation["stop_command_completed_monotonic_ns"] = stopped_ns
            self._operation["stationary_samples"] = 0
        self._log_transition(operation_id, "awaiting_stationary" if result == 0 else "stop_failed", reason)

    def _finish_without_stop(self, operation_id: str, state: str, reason: str) -> None:
        with self._lock:
            if self._matches_active(operation_id):
                self._operation["state"] = state
                self._operation["reason"] = reason
                self._operation["stopped_monotonic_ns"] = time.monotonic_ns()
        self._log_transition(operation_id, state, reason)

    def _matches_active(self, operation_id: str) -> bool:
        return self._operation is not None and self._operation["operation_id"] == operation_id

    def _log_transition(self, operation_id: str, state: str, reason: str | None = None) -> None:
        self._logger.info(
            f"[CONDITIONAL_SERVO] operation_id={operation_id} state={state} "
            f"reason={reason} monotonic_ns={time.monotonic_ns()}"
        )

    @staticmethod
    def _parse_boundary(boundary: dict | None) -> dict | None:
        if boundary is None:
            return None
        axis = str(boundary.get("axis") or "").strip().lower()
        if axis not in {"x", "y", "z"}:
            raise ValueError("boundary.axis must be x, y, or z")
        operator = str(boundary.get("operator") or "").strip().lower()
        if operator not in {"less_or_equal", "greater_or_equal"}:
            raise ValueError("boundary.operator must be less_or_equal or greater_or_equal")
        value_mm = float(boundary["value_mm"])
        if not math.isfinite(value_mm):
            raise ValueError("boundary.value_mm must be finite")
        return {
            "frame": "user",
            "user": int(boundary.get("user", 0)),
            "tool": int(boundary.get("tool", 0)),
            "axis": axis,
            "axis_index": {"x": 0, "y": 1, "z": 2}[axis],
            "operator": operator,
            "value_mm": value_mm,
        }
