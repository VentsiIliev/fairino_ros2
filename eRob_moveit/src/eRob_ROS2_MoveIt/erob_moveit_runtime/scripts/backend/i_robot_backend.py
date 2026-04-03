from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class IRobotBackend(ABC):
    @abstractmethod
    def set_workobject(self, workobject: Any, user_id: int = 0) -> None: ...

    @abstractmethod
    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True, trajectory_optimizer=None): ...

    @abstractmethod
    def execute_path(self, path, rx=None, ry=None, rz=None, vel=0.6, acc=0.4, blocking=True): ...

    @abstractmethod
    def get_safety_walls_status(self): ...

    @abstractmethod
    def enable_safety_walls(self): ...

    @abstractmethod
    def disable_safety_walls(self): ...

    @abstractmethod
    def get_current_position(self): ...

    @abstractmethod
    def get_current_velocity(self): ...

    @abstractmethod
    def start_jog(self, axis, direction, step, vel, acc): ...

    @abstractmethod
    def stop_motion(self): ...
