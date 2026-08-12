from __future__ import annotations

import logging
import math
import threading
import time

import config
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

from .i_cartesian_servo import (
    CartesianServo,
    CartesianServoCommand,
    CartesianServoFrame,
)

logger = logging.getLogger(__name__)


class MoveItCartesianServo(CartesianServo):
    """
    MoveIt Servo implementation of CartesianServo.

    Public API is inherited from CartesianServo:

        start(frame=..., tool=...)
        update(linear_mm_s=..., angular_deg_s=...)
        stop()

    Behaviour:

    - start():
        * activates the requested tool/TCP
        * resolves BASE or TOOL command frame
        * ensures the underlying MoveIt Servo node is started
        * creates a zero velocity command
        * starts an internal high-rate publisher

    - update():
        * replaces the stored Cartesian velocity command once
        * does NOT directly need to publish continuously

    - internal ROS timer:
        * republishes the latest command continuously
        * gives every TwistStamped a fresh ROS timestamp

    - stop():
        * immediately changes command to zero
        * immediately publishes zero
        * stops the high-rate publisher
        * leaves the underlying MoveIt Servo node running

    - shutdown():
        * intended for application shutdown
        * stops the underlying MoveIt Servo node
    """

    def __init__(
        self,
        *,
        node: Node,
        base_frame: str,
        servo_command_topic: str = "/servo_node/delta_twist_cmds",
        servo_start_service: str = "/servo_node/start_servo",
        servo_stop_service: str = "/servo_node/stop_servo",
        publish_rate_hz: float = 100.0,
        service_timeout_s: float = 2.0,
    ) -> None:
        super().__init__()

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        base_frame = str(base_frame).strip()
        if not base_frame:
            raise ValueError("base_frame must not be empty")

        self._node = node

        self._base_frame = base_frame

        self._publish_rate_hz = float(publish_rate_hz)
        self._publish_period_s = 1.0 / self._publish_rate_hz
        self._service_timeout_s = float(service_timeout_s)

        self._command_lock = threading.Lock()

        self._command_frame_id: str | None = None
        self._latest_command: CartesianServoCommand | None = None

        self._publish_timer = None

        # The underlying MoveIt Servo node stays running between
        # CartesianServo start()/stop() sessions.
        self._moveit_servo_started = False
        self._moveit_servo_lock = threading.Lock()

        self._twist_pub = self._node.create_publisher(
            TwistStamped,
            servo_command_topic,
            1,
        )

        self._start_client = self._node.create_client(
            Trigger,
            servo_start_service,
        )

        self._stop_client = self._node.create_client(
            Trigger,
            servo_stop_service,
        )

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] initialized "
            "base_frame=%s command_topic=%s publish_rate_hz=%.1f",
            self._base_frame,
            servo_command_topic,
            self._publish_rate_hz,
        )

    # ============================================================
    # CartesianServo hooks
    # ============================================================

    def _on_start(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
    ) -> bool:
        """
        Start one Cartesian servo session.

        Generic lifecycle validation has already been performed by
        CartesianServo.start().
        """

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] START "
            "frame=%s tool=%d",
            frame.value,
            tool,
        )

        # --------------------------------------------------------
        # Activate selected tool/TCP
        # --------------------------------------------------------

        try:
            tool_name = config.resolve_tool_name(tool)
        except ValueError as exc:
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "invalid tool=%d: %s",
                tool,
                exc,
            )
            return False

        try:
            self._node.set_tool(tool_name)
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "exception while activating tool=%d (%s)",
                tool,
                tool_name,
            )
            return False

        # --------------------------------------------------------
        # Resolve the command frame
        # --------------------------------------------------------

        try:
            command_frame_id = self._resolve_command_frame(
                frame=frame,
                tool=tool,
            )
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "failed to resolve frame=%s tool=%d",
                frame.value,
                tool,
            )
            return False

        # --------------------------------------------------------
        # Start underlying MoveIt Servo if needed
        # --------------------------------------------------------

        if not self._ensure_moveit_servo_started():
            return False

        # --------------------------------------------------------
        # Initialize session state with zero velocity
        # --------------------------------------------------------

        with self._command_lock:
            self._command_frame_id = command_frame_id
            self._latest_command = self._zero_command()

        # --------------------------------------------------------
        # Start continuous publication
        # --------------------------------------------------------

        try:
            self._start_publish_timer()
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "failed to start command publishing"
            )

            with self._command_lock:
                self._command_frame_id = None
                self._latest_command = None

            return False

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] RUNNING "
            "frame=%s ros_frame=%s tool=%d rate_hz=%.1f",
            frame.value,
            command_frame_id,
            tool,
            self._publish_rate_hz,
        )

        return True

    def _on_update(
        self,
        command: CartesianServoCommand,
    ) -> bool:
        """
        Replace the currently commanded Cartesian velocity.

        The ROS timer continues publishing this command until another
        update() or stop().
        """

        with self._command_lock:
            if not self._command_frame_id:
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] UPDATE rejected: "
                    "command frame is not configured"
                )
                return False

            self._latest_command = command

            frame_id = self._command_frame_id

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] UPDATE "
            "frame=%s linear_mm_s=%s angular_deg_s=%s",
            frame_id,
            command.linear_mm_s,
            command.angular_deg_s,
        )

        return True

    def _on_stop(self) -> bool:
        """
        Stop the active Cartesian servo session.

        The underlying MoveIt Servo node remains started.
        """

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] STOP"
        )

        zero = self._zero_command()

        with self._command_lock:
            frame_id = self._command_frame_id

            if frame_id:
                self._latest_command = zero

        # Do not wait for the next 100 Hz timer iteration.
        # Send zero immediately.
        if frame_id:
            try:
                self._publish_command(
                    command=zero,
                    frame_id=frame_id,
                )
            except Exception:
                logger.exception(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "failed to publish stop command"
                )
                return False

        self._stop_publish_timer()

        with self._command_lock:
            self._latest_command = None
            self._command_frame_id = None

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] STOPPED"
        )

        return True

    # ============================================================
    # High-rate command publisher
    # ============================================================

    def _start_publish_timer(self) -> None:
        if self._publish_timer is not None:
            self._stop_publish_timer()

        self._publish_timer = self._node.create_timer(
            self._publish_period_s,
            self._publish_timer_callback,
        )

        logger.debug(
            "[MOVEIT_CARTESIAN_SERVO] publisher started "
            "period_ms=%.3f",
            self._publish_period_s * 1000.0,
        )

    def _stop_publish_timer(self) -> None:
        timer = self._publish_timer
        self._publish_timer = None

        if timer is None:
            return

        try:
            timer.cancel()
        except Exception:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] timer cancel failed",
                exc_info=True,
            )

        try:
            self._node.destroy_timer(timer)
        except Exception:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] timer destruction failed",
                exc_info=True,
            )

    def _publish_timer_callback(self) -> None:
        """
        Republish the latest stored command.

        update() is therefore event-based from the platform, while
        MoveIt Servo still receives a fresh command at publish_rate_hz.
        """

        with self._command_lock:
            command = self._latest_command
            frame_id = self._command_frame_id

        if command is None or not frame_id:
            return

        try:
            self._publish_command(
                command=command,
                frame_id=frame_id,
            )
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "command publish failed"
            )

    def _publish_command(
        self,
        *,
        command: CartesianServoCommand,
        frame_id: str,
    ) -> None:
        msg = TwistStamped()

        # This must be fresh on every internal publication.
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        # mm/s -> m/s
        msg.twist.linear.x = float(command.linear_mm_s[0]) / 1000.0
        msg.twist.linear.y = float(command.linear_mm_s[1]) / 1000.0
        msg.twist.linear.z = float(command.linear_mm_s[2]) / 1000.0

        # deg/s -> rad/s
        msg.twist.angular.x = math.radians(
            float(command.angular_deg_s[0])
        )
        msg.twist.angular.y = math.radians(
            float(command.angular_deg_s[1])
        )
        msg.twist.angular.z = math.radians(
            float(command.angular_deg_s[2])
        )

        self._twist_pub.publish(msg)

    # ============================================================
    # Frame resolution
    # ============================================================

    def _resolve_command_frame(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
    ) -> str:
        if frame == CartesianServoFrame.BASE:
            return self._base_frame

        if frame == CartesianServoFrame.TOOL:
            # Tool TCP offsets are applied in software via node.set_tool()
            # (planner_context.T_tool), so TOOL-frame twist commands are
            # expressed relative to the end-effector link, which is the TF
            # frame that carries the tool.
            tool_frame = getattr(config, "EE_LINK", "")

            if not tool_frame:
                raise RuntimeError(
                    "No end-effector link configured for TOOL frame"
                )

            return str(tool_frame)

        # Should already have been prevented by the base class.
        raise ValueError(
            f"Unsupported CartesianServoFrame: {frame!r}"
        )

    # ============================================================
    # Underlying MoveIt Servo lifecycle
    # ============================================================

    def _ensure_moveit_servo_started(self) -> bool:
        """
        Start MoveIt Servo once.

        CartesianServo.stop() does not stop it because Cartesian Servo
        sessions are expected to occur frequently.
        """

        with self._moveit_servo_lock:
            if self._moveit_servo_started:
                return True

            logger.info(
                "[MOVEIT_CARTESIAN_SERVO] "
                "starting underlying MoveIt Servo"
            )

            if not self._start_client.wait_for_service(
                timeout_sec=self._service_timeout_s,
            ):
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "start_servo service unavailable"
                )
                return False

            future = self._start_client.call_async(
                Trigger.Request()
            )

            if not self._wait_future(
                future,
                timeout_s=self._service_timeout_s,
            ):
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "start_servo timed out"
                )
                return False

            try:
                response = future.result()
            except Exception:
                logger.exception(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "start_servo service call failed"
                )
                return False

            if response is None:
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "start_servo returned no response"
                )
                return False

            if not response.success:
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "start_servo rejected: %s",
                    response.message,
                )
                return False

            self._moveit_servo_started = True

            logger.info(
                "[MOVEIT_CARTESIAN_SERVO] "
                "underlying MoveIt Servo started"
            )

            return True

    def shutdown(self) -> bool:
        """
        Stop the adapter and underlying MoveIt Servo.

        This is for runtime shutdown, NOT normal servo.stop().
        """

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] shutdown"
        )

        # Stop high-rate publication first.
        self._stop_publish_timer()

        with self._command_lock:
            frame_id = self._command_frame_id

        # Best-effort zero before shutting Servo down.
        if frame_id:
            try:
                self._publish_command(
                    command=self._zero_command(),
                    frame_id=frame_id,
                )
            except Exception:
                logger.warning(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "failed to publish zero during shutdown",
                    exc_info=True,
                )

        with self._command_lock:
            self._latest_command = None
            self._command_frame_id = None

        with self._moveit_servo_lock:
            if not self._moveit_servo_started:
                return True

            if not self._stop_client.wait_for_service(
                timeout_sec=self._service_timeout_s,
            ):
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "stop_servo service unavailable"
                )
                return False

            future = self._stop_client.call_async(
                Trigger.Request()
            )

            if not self._wait_future(
                future,
                timeout_s=self._service_timeout_s,
            ):
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "stop_servo timed out"
                )
                return False

            try:
                response = future.result()
            except Exception:
                logger.exception(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "stop_servo service call failed"
                )
                return False

            if response is None or not response.success:
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "stop_servo rejected: %s",
                    response.message if response else "no response",
                )
                return False

            self._moveit_servo_started = False

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] shutdown complete"
        )

        return True

    # ============================================================
    # ROS service helper
    # ============================================================

    @staticmethod
    def _wait_future(
        future,
        *,
        timeout_s: float,
    ) -> bool:
        """
        Wait for a ROS future while another executor thread spins the node.

        Do not use this if the same thread is responsible for spinning
        the executor.
        """

        deadline = time.monotonic() + float(timeout_s)

        while not future.done():
            if time.monotonic() >= deadline:
                return False

            time.sleep(0.005)

        return True