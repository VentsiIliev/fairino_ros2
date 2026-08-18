from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from motion.servo.cartesian_servo.i_cartesian_servo import CartesianServo


class IRobotBackend(ABC):
    @abstractmethod
    def set_workobject(self, workobject: Any, user_id: int = 0) -> None: ...

    @abstractmethod
    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True, trajectory_optimizer=None): ...

    @abstractmethod
    def move_ptp(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True, trajectory_optimizer=None): ...

    @abstractmethod
    def execute_path(
        self,
        path,
        rx=None,
        ry=None,
        rz=None,
        vel=0.6,
        acc=0.4,
        blocking=True,
        trajectory_optimizer=None,
        orientation_mode="constant",
    ): ...

    @abstractmethod
    def execute_sequence(self, segments, tool=0, user=0, blocking=True): ...

    @abstractmethod
    def execute_ordered_motion_chain(self, segments, tool=0, user=0, blocking=True, trajectory_optimizer=None): ...

    @abstractmethod
    def unwind_joint6(self, blocking=True, queue_if_busy=True, vel=None, acc=None): ...

    @abstractmethod
    def get_safety_walls_status(self): ...

    @abstractmethod
    def enable_safety_walls(self): ...

    @abstractmethod
    def disable_safety_walls(self): ...

    @abstractmethod
    def get_current_position(self): ...

    @abstractmethod
    def get_current_flange_position(self): ...

    @abstractmethod
    def get_current_joints(self): ...

    @abstractmethod
    def get_current_velocity(self): ...

    @abstractmethod
    def start_jog(self, axis, direction, step, vel, acc, *, frame=None, tool=0, user=0): ...

    @abstractmethod
    def joint_jog(self, joint, direction, step, vel, acc, blocking=True): ...

    @abstractmethod
    def start_servo_jog(
        self,
        axis,
        direction,
        vel=None,
        acc=None,
        *,
        frame=None,
        tool=0,
        user=0,
        linear_mm_s=None,
        angular_deg_s=None,
    ): ...

    @abstractmethod
    def stop_servo_jog(self): ...

    @property
    @abstractmethod
    def cartesian_servo(self) -> CartesianServo | None:
        ...

    @abstractmethod
    def stop_motion(self): ...
