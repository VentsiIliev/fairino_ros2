from __future__ import annotations

import math
import threading
import time

import config
from enums import Direction, RobotAxis
from motion.servo.cartesian_servo.i_cartesian_servo import (
    CartesianServoFrame,
    CartesianServoResult,
)

from .i_jog_capability import JogCapability


class ServoJogCapability(JogCapability):
    """Finite jog implemented by MoveIt Servo velocity commands."""

    def __init__(self, backend) -> None:
        self._backend = backend
        self._lock = threading.Lock()
        self._continuous_active = False
        self._continuous_collision_override_active = False

    def start_jog(
        self,
        axis: RobotAxis,
        direction: Direction,
        step: float,
        vel: float,
        acc: float,
        *,
        frame: CartesianServoFrame = CartesianServoFrame.USER,
        tool: int = 0,
        user: int = 0,
    ) -> int:
        prepared = self._prepare_jog(axis, direction, step=step, vel=vel)
        if prepared is None:
            return -1
        axis_val, dir_val, linear_mm_s, angular_deg_s, duration_s = prepared
        if duration_s is None or duration_s <= 0.0:
            return -1

        backend = self._backend
        node = backend.node
        if node is None:
            return -1

        ready = self._check_ready("SERVO_JOG_FIXED")
        if ready != 0:
            return ready

        node.get_logger().info(
            "[SERVO_JOG] Starting fixed-distance servo jog: "
            f"frame={frame.value} user={user} tool={tool} axis={axis.name} "
            f"direction={direction.name} step={step:.3f} vel={vel:.3f} "
            f"duration_s={duration_s:.3f}"
        )

        if not self._start_servo_command(
            frame=frame,
            tool=tool,
            user=user,
            linear_mm_s=linear_mm_s,
            angular_deg_s=angular_deg_s,
        ):
            return -1

        try:
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                time.sleep(min(0.02, max(0.001, deadline - time.monotonic())))
            return 0
        finally:
            self._stop_servo()

    def start_continuous_jog(
        self,
        axis: RobotAxis,
        direction: Direction,
        vel: float | None = None,
        acc: float | None = None,
        *,
        frame: CartesianServoFrame = CartesianServoFrame.USER,
        tool: int = 0,
        user: int = 0,
        linear_mm_s: float | None = None,
        angular_deg_s: float | None = None,
        disable_collision_checking: bool = False,
    ) -> int:
        with self._lock:
            if self._continuous_active:
                return -1

            prepared = self._prepare_jog(
                axis,
                direction,
                step=None,
                vel=vel,
                linear_mm_s=linear_mm_s,
                angular_deg_s=angular_deg_s,
            )
            if prepared is None:
                return -1
            axis_val, dir_val, linear_mm_s, angular_deg_s, _ = prepared

            ready = self._check_ready("SERVO_JOG_CONTINUOUS")
            if ready != 0:
                return ready

            node = self._backend.node
            node.get_logger().info(
                "[SERVO_JOG] Starting continuous servo jog: "
                f"frame={frame.value} user={user} tool={tool} axis={axis.name} "
                f"direction={direction.name} linear_mm_s={linear_mm_s} angular_deg_s={angular_deg_s}"
            )

            if not self._start_servo_command(
                frame=frame,
                tool=tool,
                user=user,
                linear_mm_s=linear_mm_s,
                angular_deg_s=angular_deg_s,
                disable_collision_checking=disable_collision_checking,
            ):
                return -1

            self._continuous_active = True
            self._continuous_collision_override_active = bool(disable_collision_checking)
            return 0

    def stop_continuous_jog(self, *, restore_collision_checking: bool = True) -> int:
        with self._lock:
            if not self._continuous_active:
                result = True
            else:
                result = self._stop_servo()
                self._continuous_active = False
            restore_ok = True
            if self._continuous_collision_override_active and restore_collision_checking:
                setter = getattr(self._backend.cartesian_servo, "set_collision_checking", None)
                restore_ok = bool(callable(setter) and setter(True))
                if not restore_ok:
                    self._backend.node.get_logger().error(
                        "[SERVO_JOG] Collision checking restore was not confirmed after stop"
                    )
                self._continuous_collision_override_active = False
            return 0 if result and restore_ok else -1

    def move_z_to_target(
        self,
        *,
        target_z_mm: float,
        fast_linear_mm_s: float,
        final_linear_mm_s: float,
        slowdown_distance_mm: float,
        tolerance_mm: float,
        maximum_distance_mm: float,
        timeout_s: float,
        poll_interval_s: float,
        frame: CartesianServoFrame = CartesianServoFrame.USER,
        tool: int = 0,
        user: int = 0,
        disable_collision_checking: bool = False,
    ) -> dict:
        """Retract +Z to a hard target using an entirely local feedback loop."""
        values = (
            target_z_mm,
            fast_linear_mm_s,
            final_linear_mm_s,
            slowdown_distance_mm,
            tolerance_mm,
            maximum_distance_mm,
            timeout_s,
            poll_interval_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return {"result": -1, "success": False, "error": "invalid_bounded_servo_config"}
        tolerance = max(0.0, float(tolerance_mm))
        slowdown_distance = float(slowdown_distance_mm)
        if (
            float(fast_linear_mm_s) <= 0.0
            or float(final_linear_mm_s) <= 0.0
            or slowdown_distance <= tolerance
            or float(maximum_distance_mm) <= 0.0
            or float(timeout_s) <= 0.0
            or float(poll_interval_s) <= 0.0
        ):
            return {"result": -1, "success": False, "error": "invalid_bounded_servo_config"}

        with self._lock:
            ready = self._check_ready("SERVO_JOG_TARGET_Z")
            if ready != 0:
                return {"result": ready, "success": False, "error": "motion_not_ready"}
            start_pose = self._backend.get_current_position(user_id=user)
            if not start_pose or len(start_pose) < 3:
                return {"result": -1, "success": False, "error": "position_unreadable"}
            start_z = float(start_pose[2])
            requested_distance = float(target_z_mm) - start_z
            if requested_distance <= tolerance:
                return {"result": -1, "success": False, "error": "target_not_above_start"}
            if requested_distance > float(maximum_distance_mm):
                return {"result": -1, "success": False, "error": "maximum_distance_exceeded"}

            if not self._start_servo_command(
                frame=frame,
                tool=tool,
                user=user,
                linear_mm_s=(0.0, 0.0, float(fast_linear_mm_s)),
                angular_deg_s=(0.0, 0.0, 0.0),
                disable_collision_checking=disable_collision_checking,
            ):
                return {"result": -1, "success": False, "error": "servo_start_failed"}

            final_phase = False
            error = ""
            deadline = time.monotonic() + float(timeout_s)
            try:
                while not error:
                    pose = self._backend.get_current_position(user_id=user)
                    if not pose or len(pose) < 3:
                        error = "position_unreadable"
                        break
                    live_z = float(pose[2])
                    travelled = live_z - start_z
                    remaining = float(target_z_mm) - live_z
                    if travelled > float(maximum_distance_mm) + tolerance:
                        error = "maximum_distance_exceeded"
                        break
                    if remaining <= tolerance:
                        break
                    if not final_phase and remaining <= slowdown_distance:
                        update = self._backend.cartesian_servo.update(
                            linear_mm_s=(0.0, 0.0, float(final_linear_mm_s)),
                            angular_deg_s=(0.0, 0.0, 0.0),
                        )
                        if update != CartesianServoResult.OK:
                            error = "slowdown_update_failed"
                            break
                        final_phase = True
                        self._backend.node.get_logger().info(
                            "[SERVO_JOG_TARGET_Z] Switched to final speed: "
                            f"live_z={live_z:.3f} target_z={target_z_mm:.3f} "
                            f"linear_mm_s={final_linear_mm_s:.3f}"
                        )
                    if time.monotonic() >= deadline:
                        error = "timeout"
                        break
                    time.sleep(max(0.002, min(0.02, float(poll_interval_s))))
            finally:
                stopped = self._stop_servo()
                restore_ok = True
                if disable_collision_checking:
                    setter = getattr(self._backend.cartesian_servo, "set_collision_checking", None)
                    restore_ok = bool(callable(setter) and setter(True))
                if not stopped and not error:
                    error = "servo_stop_failed"
                if not restore_ok and not error:
                    error = "collision_restore_failed"

            final_pose = self._backend.get_current_position(user_id=user)
            final_z = None if not final_pose or len(final_pose) < 3 else float(final_pose[2])
            if not error and final_z is None:
                error = "final_position_unreadable"
            if not error and abs(final_z - float(target_z_mm)) > tolerance:
                error = "final_position_mismatch"
            return {
                "result": 0 if not error else -1,
                "success": not error,
                "error": error or None,
                "start_z": start_z,
                "target_z": float(target_z_mm),
                "final_z": final_z,
                "final_phase": final_phase,
            }

    def _prepare_jog(
        self,
        axis: RobotAxis,
        direction: Direction,
        *,
        step: float | None,
        vel: float | None,
        linear_mm_s: float | None = None,
        angular_deg_s: float | None = None,
    ) -> tuple[int, int, tuple[float, float, float], tuple[float, float, float], float | None] | None:
        axis_val = axis.value if hasattr(axis, "value") else int(axis)
        dir_val = direction.value if hasattr(direction, "value") else int(direction)
        if axis_val not in [1, 2, 3, 4, 5, 6] or dir_val not in [1, -1]:
            return None

        linear_mm_s, angular_deg_s, duration_s = self._command_for_jog(
            axis_val=axis_val,
            direction=dir_val,
            step=step,
            vel=vel,
            linear_mm_s=linear_mm_s,
            angular_deg_s=angular_deg_s,
        )
        return axis_val, dir_val, linear_mm_s, angular_deg_s, duration_s

    def _check_ready(self, label: str) -> int:
        backend = self._backend
        node = backend.node
        if node is None or backend.cartesian_servo is None:
            return -1

        drive_error = backend._reject_if_drive_not_enabled(label)
        if drive_error is not None:
            return drive_error
        if not node.is_hardware_ready_for_motion():
            node.get_logger().error(
                f"[{label}] Rejected: {node.get_hardware_fault_reason()}"
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        if node.is_motion_active() or node.has_pending_motion():
            node.get_logger().info(f"[{label}] Busy or queued motion pending - ignoring")
            return -1
        has_prepared_chain = getattr(backend, "has_active_prepared_ordered_motion_chain", None)
        if callable(has_prepared_chain) and has_prepared_chain():
            node.get_logger().error(
                f"[{label}] Rejected: an ordered motion chain is prepared or executing"
            )
            return -1

        return 0

    def _start_servo_command(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
        user: int,
        linear_mm_s,
        angular_deg_s,
        disable_collision_checking: bool = False,
    ) -> bool:
        node = self._backend.node
        servo = self._backend.cartesian_servo
        configure_collision = getattr(servo, "configure_next_start_collision_checking", None)
        if disable_collision_checking and not callable(configure_collision):
            node.get_logger().error(
                "[SERVO_JOG] Refusing motion: pre-start collision policy is unsupported"
            )
            return False
        if callable(configure_collision):
            configure_collision(not disable_collision_checking)
        start_result = servo.start(frame=frame, tool=tool, user=user)
        if start_result not in (CartesianServoResult.OK, CartesianServoResult.ALREADY_RUNNING):
            detail = getattr(servo, "last_start_failure", None)
            if detail:
                node.get_logger().error(
                    f"[SERVO_JOG] Servo start failed: {start_result.value} ({detail})"
                )
            else:
                node.get_logger().error(f"[SERVO_JOG] Servo start failed: {start_result.value}")
            return False

        update_result = servo.update(
            linear_mm_s=linear_mm_s,
            angular_deg_s=angular_deg_s,
        )
        if update_result != CartesianServoResult.OK:
            node.get_logger().error(f"[SERVO_JOG] Servo update failed: {update_result.value}")
            self._stop_servo()
            if disable_collision_checking:
                setter = getattr(servo, "set_collision_checking", None)
                if not callable(setter) or not setter(True):
                    node.get_logger().error(
                        "[SERVO_JOG] Collision checking restore failed after start failure"
                    )
            return False

        return True

    def _stop_servo(self) -> bool:
        node = self._backend.node
        servo = self._backend.cartesian_servo
        if node is None or servo is None:
            return True
        try:
            servo.update(
                linear_mm_s=(0.0, 0.0, 0.0),
                angular_deg_s=(0.0, 0.0, 0.0),
            )
        except Exception:
            node.get_logger().debug("[SERVO_JOG] zero update failed", exc_info=True)
        stop_result = servo.stop()
        if stop_result not in (CartesianServoResult.OK, CartesianServoResult.NOT_STARTED):
            node.get_logger().warning(f"[SERVO_JOG] Servo stop returned {stop_result.value}")
            return False
        return True

    @staticmethod
    def _command_for_jog(
        *,
        axis_val: int,
        direction: int,
        step: float | None,
        vel: float | None,
        linear_mm_s: float | None = None,
        angular_deg_s: float | None = None,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float | None]:
        sign = 1.0 if int(direction) > 0 else -1.0
        distance = None
        if step is not None:
            signed_step = float(step) * sign
            distance = abs(signed_step)
            sign = 1.0 if signed_step >= 0.0 else -1.0

        if linear_mm_s is not None:
            linear_speed_mm_s = abs(float(linear_mm_s))
        elif vel is not None:
            velocity_percent = max(1.0, min(100.0, abs(float(vel))))
            linear_speed_mm_s = (
                float(getattr(config, "SERVO_JOG_LINEAR_SPEED_AT_100_PERCENT_MM_S", 100.0))
                * velocity_percent / 100.0
            )
        else:
            linear_speed_mm_s = float(getattr(config, "SERVO_JOG_DEFAULT_LINEAR_MM_S", 10.0))

        if angular_deg_s is not None:
            angular_speed_deg_s = abs(float(angular_deg_s))
        elif vel is not None:
            velocity_percent = max(1.0, min(100.0, abs(float(vel))))
            angular_speed_deg_s = (
                float(getattr(config, "SERVO_JOG_ANGULAR_SPEED_AT_100_PERCENT_DEG_S", 30.0))
                * velocity_percent / 100.0
            )
        else:
            angular_speed_deg_s = float(getattr(config, "SERVO_JOG_DEFAULT_ANGULAR_DEG_S", 3.0))

        linear_speed_mm_s = max(0.1, linear_speed_mm_s)
        angular_speed_deg_s = max(0.1, angular_speed_deg_s)
        max_duration_s = max(0.05, float(getattr(config, "SERVO_JOG_MAX_DURATION_S", 10.0)))

        linear = [0.0, 0.0, 0.0]
        angular = [0.0, 0.0, 0.0]
        if axis_val <= 3:
            linear[axis_val - 1] = sign * linear_speed_mm_s
            duration_s = None if distance is None else distance / linear_speed_mm_s
        else:
            angular[axis_val - 4] = sign * angular_speed_deg_s
            duration_s = None if distance is None else distance / angular_speed_deg_s

        if duration_s is None:
            return tuple(linear), tuple(angular), None
        if not math.isfinite(duration_s):
            duration_s = 0.0
        duration_s = min(max(0.0, duration_s), max_duration_s)
        return tuple(linear), tuple(angular), duration_s
