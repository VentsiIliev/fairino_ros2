from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Sequence


class CartesianServoFrame(str, Enum):
    BASE = "base"
    TOOL = "tool"


class CartesianServoState(str, Enum):
    STOPPED = "stopped"
    RUNNING = "running"
    ERROR = "error"


class CartesianServoResult(str, Enum):
    OK = "ok"

    ALREADY_RUNNING = "already_running"
    NOT_STARTED = "not_started"

    START_FAILED = "start_failed"
    UPDATE_FAILED = "update_failed"
    STOP_FAILED = "stop_failed"

    INVALID_COMMAND = "invalid_command"


@dataclass(frozen=True)
class CartesianServoCommand:
    linear_mm_s: tuple[float, float, float]
    angular_deg_s: tuple[float, float, float]


@dataclass(frozen=True)
class CartesianServoStatus:
    state: CartesianServoState
    frame: CartesianServoFrame | None
    tool: int | None
    command: CartesianServoCommand | None
    error: str | None = None


class CartesianServo(ABC):
    """
    Base Cartesian servo state machine.

    Public methods are intentionally non-overridable by convention.

    Implementations provide only:

        _start_impl()
        _update_impl()
        _stop_impl()

    This guarantees identical lifecycle semantics for MoveIt, Dummy,
    and any future Cartesian Servo implementation.
    """

    def __init__(self) -> None:
        self._state = CartesianServoState.STOPPED
        self._frame: CartesianServoFrame | None = None
        self._tool: int | None = None
        self._command: CartesianServoCommand | None = None
        self._error: str | None = None

    # ============================================================
    # Public API / template methods
    # ============================================================

    def start(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
    ) -> CartesianServoResult:

        if self._state == CartesianServoState.RUNNING:
            return CartesianServoResult.ALREADY_RUNNING

        frame = CartesianServoFrame(frame)
        tool = int(tool)

        try:
            success = self._on_start(
                frame=frame,
                tool=tool,
            )
        except Exception as exc:
            self._state = CartesianServoState.ERROR
            self._error = str(exc)
            return CartesianServoResult.START_FAILED

        if not success:
            self._state = CartesianServoState.ERROR
            self._error = "Servo implementation failed to start"
            return CartesianServoResult.START_FAILED

        self._frame = frame
        self._tool = tool
        self._command = self._zero_command()
        self._error = None
        self._state = CartesianServoState.RUNNING

        return CartesianServoResult.OK

    def update(
        self,
        *,
        linear_mm_s: Sequence[float],
        angular_deg_s: Sequence[float],
    ) -> CartesianServoResult:

        if self._state != CartesianServoState.RUNNING:
            return CartesianServoResult.NOT_STARTED

        try:
            command = CartesianServoCommand(
                linear_mm_s=self._vector3(linear_mm_s),
                angular_deg_s=self._vector3(angular_deg_s),
            )
        except (TypeError, ValueError):
            return CartesianServoResult.INVALID_COMMAND

        try:
            success = self._on_update(command)
        except Exception as exc:
            self._state = CartesianServoState.ERROR
            self._error = str(exc)
            return CartesianServoResult.UPDATE_FAILED

        if not success:
            self._state = CartesianServoState.ERROR
            self._error = "Servo implementation failed to update"
            return CartesianServoResult.UPDATE_FAILED

        self._command = command

        return CartesianServoResult.OK

    def stop(self) -> CartesianServoResult:

        if self._state == CartesianServoState.STOPPED:
            return CartesianServoResult.NOT_STARTED

        try:
            success = self.on_stop()
        except Exception as exc:
            self._state = CartesianServoState.ERROR
            self._error = str(exc)
            return CartesianServoResult.STOP_FAILED

        if not success:
            self._state = CartesianServoState.ERROR
            self._error = "Servo implementation failed to stop"
            return CartesianServoResult.STOP_FAILED

        self._state = CartesianServoState.STOPPED
        self._frame = None
        self._tool = None
        self._command = None
        self._error = None

        return CartesianServoResult.OK

    def get_status(self) -> CartesianServoStatus:
        return CartesianServoStatus(
            state=self._state,
            frame=self._frame,
            tool=self._tool,
            command=self._command,
            error=self._error,
        )

    def is_running(self) -> bool:
        return self._state == CartesianServoState.RUNNING

    # ============================================================
    # Implementation hooks
    # ============================================================

    @abstractmethod
    def _on_start(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
    ) -> bool:
        """Backend-specific servo startup."""
        raise NotImplementedError

    @abstractmethod
    def _on_update(
        self,
        command: CartesianServoCommand,
    ) -> bool:
        """Backend-specific command replacement."""
        raise NotImplementedError

    @abstractmethod
    def __on_stop(self) -> bool:
        """Backend-specific servo shutdown."""
        raise NotImplementedError

    # ============================================================
    # Helpers
    # ============================================================

    @staticmethod
    def _zero_command() -> CartesianServoCommand:
        return CartesianServoCommand(
            linear_mm_s=(0.0, 0.0, 0.0),
            angular_deg_s=(0.0, 0.0, 0.0),
        )

    @staticmethod
    def _vector3(values: Sequence[float]) -> tuple[float, float, float]:
        if values is None or len(values) != 3:
            raise ValueError("Expected exactly 3 values")

        return (
            float(values[0]),
            float(values[1]),
            float(values[2]),
        )