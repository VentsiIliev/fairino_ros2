from __future__ import annotations

import logging
import threading
import time

from motion.servo.cartesian_servo.i_cartesian_servo import (
    CartesianServo,
    CartesianServoCommand,
    CartesianServoFrame,
)


class DummyCartesianServo(CartesianServo):
    """
    Dummy Cartesian Servo implementation.

    Simulates the behavior expected from the real MoveIt Cartesian Servo:

        start()
            Starts a Cartesian servo session.

        update()
            Replaces the current velocity command.

        stop()
            Stops Cartesian motion by changing the internally published
            command to zero.

    A background worker remains alive for the lifetime of this object and
    simulates continuously publishing the latest command at publish_rate_hz.

    This means REST/API clients only need to send update() when the desired
    command changes. They do not need to continuously stream commands.
    """

    def __init__(
        self,
        *,
        publish_rate_hz: float = 100.0,
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__()

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self._publish_rate_hz = float(publish_rate_hz)
        self._publish_period_s = 1.0 / self._publish_rate_hz

        self._logger = logger or logging.getLogger(
            "DummyCartesianServo"
        )

        self._command_lock = threading.Lock()
        self._published_command = self._zero_command()

        self._shutdown_event = threading.Event()

        self._publish_count = 0

        self._worker = threading.Thread(
            target=self._publish_loop,
            name="DummyCartesianServoPublisher",
            daemon=True,
        )
        self._worker.start()

        self._logger.info(
            "[CARTESIAN_SERVO] Dummy publisher started rate=%.1f Hz",
            self._publish_rate_hz,
        )

    # ============================================================
    # CartesianServo implementation hooks
    # ============================================================

    def _on_start(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
        user: int,
    ) -> bool:
        """
        Start a new Cartesian Servo session.

        The background publisher is already alive. Starting simply
        establishes the frame/tool session and resets the command to zero.
        """

        with self._command_lock:
            self._published_command = self._zero_command()

        self._logger.info(
            "[CARTESIAN_SERVO] START frame=%s tool=%d user=%d",
            frame.value,
            tool,
            user,
        )

        return True

    def _on_update(
        self,
        command: CartesianServoCommand,
    ) -> bool:
        """
        Replace the command continuously published by the worker.
        """

        with self._command_lock:
            self._published_command = command

        self._logger.info(
            "[CARTESIAN_SERVO] UPDATE "
            "linear_mm_s=%s angular_deg_s=%s",
            command.linear_mm_s,
            command.angular_deg_s,
        )

        return True

    def _on_stop(self) -> bool:
        """
        Stop Cartesian motion.

        The worker stays alive but publishes zero velocity.
        """

        zero = self._zero_command()

        with self._command_lock:
            self._published_command = zero

        self._logger.info(
            "[CARTESIAN_SERVO] STOP -> zero command"
        )

        return True

    # ============================================================
    # Simulated high-rate publisher
    # ============================================================

    def _publish_loop(self) -> None:
        """
        Simulate the high-rate ROS2 publishing loop.

        The real MoveIt implementation will publish TwistStamped
        messages here. The dummy only records/logs the operation.
        """

        next_publish = time.monotonic()

        while not self._shutdown_event.is_set():
            now = time.monotonic()

            if now < next_publish:
                self._shutdown_event.wait(next_publish - now)
                continue

            next_publish += self._publish_period_s

            # Prevent runaway catch-up if the process was paused.
            if now - next_publish > self._publish_period_s:
                next_publish = now + self._publish_period_s

            with self._command_lock:
                command = self._published_command

            self._simulate_publish(command)

    def _simulate_publish(
        self,
        command: CartesianServoCommand,
    ) -> None:
        """
        Simulate publishing one high-rate Cartesian Servo command.

        Deliberately DEBUG rather than INFO because this can execute
        100+ times per second.
        """

        self._publish_count += 1

        self._logger.debug(
            "[CARTESIAN_SERVO] PUBLISH #%d "
            "state=%s frame=%s tool=%s "
            "linear_mm_s=%s angular_deg_s=%s",
            self._publish_count,
            self._state.value,
            self._frame.value if self._frame else None,
            self._tool,
            command.linear_mm_s,
            command.angular_deg_s,
        )

    # ============================================================
    # Diagnostics / lifecycle
    # ============================================================

    @property
    def publish_rate_hz(self) -> float:
        return self._publish_rate_hz

    @property
    def publish_count(self) -> int:
        return self._publish_count

    @property
    def last_published_command(self) -> CartesianServoCommand:
        with self._command_lock:
            return self._published_command

    def shutdown(self) -> None:
        """
        Permanently terminate the dummy publisher.

        This is different from stop():

            stop()     -> end current servo session / zero velocity
            shutdown() -> terminate the background worker

        shutdown() should normally only be used during application exit.
        """

        self._logger.info(
            "[CARTESIAN_SERVO] Dummy publisher shutting down"
        )

        self._shutdown_event.set()

        if (
            self._worker.is_alive()
            and threading.current_thread() is not self._worker
        ):
            self._worker.join(timeout=1.0)

        self._logger.info(
            "[CARTESIAN_SERVO] Dummy publisher stopped "
            "publish_count=%d",
            self._publish_count,
        )
