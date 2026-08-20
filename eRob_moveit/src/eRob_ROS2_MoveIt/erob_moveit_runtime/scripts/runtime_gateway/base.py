#!/usr/bin/env python3
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class RuntimeGateway(ABC):
    """Stable runtime contract used by API layers.

    Implementations may be local (direct Python calls) or ROS-based (services,
    actions, topic subscriptions). Gateways must not return HTTP-specific
    objects (e.g. Flask responses); they return runtime-level dictionaries,
    primitives, and runtime data objects so API layers can map them to their
    own transport.
    """

    # --- startup / readiness -----------------------------------------------

    @abstractmethod
    def startup_status(self) -> dict:
        """Runtime-level startup status (readiness, motion stack state)."""
        raise NotImplementedError

    @abstractmethod
    def runtime_state_snapshot(self) -> dict:
        """Combined runtime state snapshot (drives, interlocks, readiness)."""
        raise NotImplementedError

    @abstractmethod
    def is_runtime_initialized(self) -> bool:
        """True when the runtime (robot backend and controller node) is available."""
        raise NotImplementedError

    @abstractmethod
    def is_motion_stack_ready(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def get_motion_stack_fault_reason(self) -> str | None:
        raise NotImplementedError

    # --- status ------------------------------------------------------------

    @abstractmethod
    def status(self) -> dict:
        """Runtime status dictionary for the /status endpoint."""
        raise NotImplementedError

    @abstractmethod
    def last_submitted_task_id(self) -> Any:
        """Last submitted motion task id (may be None)."""
        raise NotImplementedError

    @abstractmethod
    def state_snapshot(self) -> dict:
        """Combined state snapshot (position, velocity, status, safety walls)."""
        raise NotImplementedError

    @abstractmethod
    def state_kinematics(self) -> dict:
        """Kinematics state (position, velocity, acceleration); may be partial."""
        raise NotImplementedError

    @abstractmethod
    def ordered_motion_chain_status(self) -> dict:
        """Ordered motion chain status; unsupported backends report supported=False."""
        raise NotImplementedError

    # --- motion ------------------------------------------------------------

    @abstractmethod
    def stop_motion(self) -> dict:
        """Stop active motion and clear queued work; returns runtime result dict."""
        raise NotImplementedError

    @abstractmethod
    def move_linear(
        self,
        position: list[float],
        tool: int = 0,
        user: int = 0,
        vel: float = 30,
        acc: float = 30,
        blocking: bool = True,
        trajectory_optimizer: str | None = None,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def move_ptp(
        self,
        position: list[float],
        tool: int = 0,
        user: int = 0,
        vel: float = 30,
        acc: float = 30,
        blocking: bool = True,
        trajectory_optimizer: str | None = None,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def execute_path(
        self,
        path: list[Any],
        rx: float | None = None,
        ry: float | None = None,
        rz: float | None = None,
        vel: float = 0.6,
        acc: float = 0.4,
        blocking: bool = True,
        trajectory_optimizer: str | None = None,
        orientation_mode: str = "constant",
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def execute_sequence(
        self,
        segments: list[Any],
        tool: int = 0,
        user: int = 0,
        blocking: bool = True,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def execute_ordered_motion_chain(
        self,
        segments: list[Any],
        tool: int = 0,
        user: int = 0,
        blocking: bool = True,
        trajectory_optimizer: str | None = None,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def unwind_joint6(
        self,
        blocking: bool = True,
        queue_if_busy: bool = True,
        vel: float | None = None,
        acc: float | None = None,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def jog(
        self,
        axis: Any,
        direction: Any,
        step: float,
        vel: float,
        acc: float,
        frame: Any | None = None,
        tool: int = 0,
        user: int = 0,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def joint_jog(
        self,
        joint: Any,
        direction: Any,
        step: float,
        vel: float,
        acc: float,
        blocking: bool = True,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def servo_jog_start(
        self,
        axis: Any,
        direction: Any,
        vel: float,
        acc: float,
        frame: Any | None = None,
        tool: int = 0,
        user: int = 0,
        linear_mm_s: float | None = None,
        angular_deg_s: float | None = None,
    ) -> int:
        raise NotImplementedError

    @abstractmethod
    def servo_jog_stop(self) -> int:
        raise NotImplementedError

    # --- position / kinematics queries ------------------------------------

    @abstractmethod
    def get_current_position(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def get_current_base_tcp_position(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def get_current_flange_position(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def get_current_velocity(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def get_current_acceleration(self) -> Any:
        raise NotImplementedError

    # --- cartesian servo ---------------------------------------------------

    @abstractmethod
    def cartesian_servo_available(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def cartesian_servo_start(self, frame: Any, tool: int, user: int) -> Any:
        """Start Cartesian servo; returns a CartesianServoResult-compatible value."""
        raise NotImplementedError

    @abstractmethod
    def cartesian_servo_update(self, linear_mm_s: list[float], angular_deg_s: list[float]) -> Any:
        raise NotImplementedError

    @abstractmethod
    def cartesian_servo_stop(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def cartesian_servo_status(self) -> Any | None:
        """Current Cartesian servo status object or None when unavailable."""
        raise NotImplementedError

    # --- drives / interlocks ----------------------------------------------

    @abstractmethod
    def set_drive_operation_enabled(self, enabled: bool) -> dict:
        raise NotImplementedError

    @abstractmethod
    def drive_operation_status(self) -> dict:
        """Drive operation status plus hardware readiness."""
        raise NotImplementedError

    @abstractmethod
    def motion_interlock_status(self) -> dict:
        raise NotImplementedError

    @abstractmethod
    def reset_motion_interlock(self) -> dict:
        raise NotImplementedError

    # --- safety walls ------------------------------------------------------

    @abstractmethod
    def get_safety_walls_status(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def enable_safety_walls(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def disable_safety_walls(self) -> Any:
        raise NotImplementedError

    # --- tools / workobject ------------------------------------------------

    @abstractmethod
    def tool_registry(self) -> dict:
        raise NotImplementedError

    @abstractmethod
    def active_tool_name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def set_active_tool(self, tool_name: str) -> str:
        raise NotImplementedError

    @abstractmethod
    def update_tool_registry(self, tool_id: int, name: str | None, transform: Any, persist: bool) -> dict:
        raise NotImplementedError

    @abstractmethod
    def workobject_registry(self) -> dict:
        raise NotImplementedError

    @abstractmethod
    def active_workobject(self) -> dict:
        raise NotImplementedError

    @abstractmethod
    def set_active_workobject(self, user_id: int) -> dict:
        raise NotImplementedError

    @abstractmethod
    def update_workobject_registry(self, user_id: int, name: str | None, transform: Any, persist: bool) -> dict:
        raise NotImplementedError

    @abstractmethod
    def set_workobject(self, origin: list[float], user_id: int) -> None:
        raise NotImplementedError

    # --- validation / IO ---------------------------------------------------

    @abstractmethod
    def validate_pose(
        self,
        start_position: list[float],
        target_position: list[float],
        tool: int = 0,
        user: int = 0,
        start_joint_state_payload: dict | None = None,
    ) -> dict:
        raise NotImplementedError

    @abstractmethod
    def set_digital_output(self, port: int, value: int) -> int:
        raise NotImplementedError
