from __future__ import annotations

from abc import ABC, abstractmethod

from enums import Direction, RobotAxis
from motion.servo.cartesian_servo.i_cartesian_servo import CartesianServoFrame


class JogCapability(ABC):
    @abstractmethod
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
        raise NotImplementedError

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
        raise NotImplementedError

    def stop_continuous_jog(self) -> int:
        raise NotImplementedError
