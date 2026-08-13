from __future__ import annotations

from enums import Direction, RobotAxis
from motion.servo.cartesian_servo.i_cartesian_servo import CartesianServoFrame

from .i_jog_capability import JogCapability


class PlannedJogCapability(JogCapability):
    """Legacy jog behavior: plan and execute a finite Cartesian step."""

    def __init__(self, backend) -> None:
        self._backend = backend

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
        return self._backend._start_planned_jog(
            axis,
            direction,
            step,
            vel,
            acc,
            frame=frame,
            tool=tool,
            user=user,
        )

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
        if self._backend.node is not None:
            self._backend.node.get_logger().error(
                "[JOG] Continuous jog requires the ServoJog capability"
            )
        return -1

    def stop_continuous_jog(self) -> int:
        return 0
