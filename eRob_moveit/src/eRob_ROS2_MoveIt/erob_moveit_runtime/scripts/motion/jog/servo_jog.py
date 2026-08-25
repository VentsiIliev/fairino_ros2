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
            ):
                return -1

            self._continuous_active = True
            return 0

    def stop_continuous_jog(self) -> int:
        with self._lock:
            if not self._continuous_active:
                return 0
            result = self._stop_servo()
            self._continuous_active = False
            return 0 if result else -1

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

        return 0

    def _start_servo_command(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
        user: int,
        linear_mm_s,
        angular_deg_s,
    ) -> bool:
        node = self._backend.node
        servo = self._backend.cartesian_servo
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

        begin_manual_jog = getattr(servo, "begin_manual_jog", None)
        if callable(begin_manual_jog) and not begin_manual_jog():
            node.get_logger().error("[SERVO_JOG] Could not disable collision checking for manual jog")
            self._stop_servo()
            return False

        update_result = servo.update(
            linear_mm_s=linear_mm_s,
            angular_deg_s=angular_deg_s,
        )
        if update_result != CartesianServoResult.OK:
            node.get_logger().error(f"[SERVO_JOG] Servo update failed: {update_result.value}")
            self._stop_servo()
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
        end_manual_jog = getattr(servo, "end_manual_jog", None)
        if callable(end_manual_jog):
            end_manual_jog()
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
