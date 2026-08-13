from __future__ import annotations

import logging
import math
import threading
import time

import config
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import SetBool

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

    Design
    ------

    The external MoveIt Servo node is launched together with the robot
    runtime and remains alive continuously.

    CartesianServo.start():
        - activates the requested software tool/TCP
        - resolves BASE or TOOL command frame
        - resumes MoveIt Servo using /servo_node/pause_servo
        - stores a zero velocity command
        - starts an internal high-rate publisher

    CartesianServo.update():
        - replaces the stored Cartesian velocity command
        - does not itself need to publish continuously

    Internal publisher:
        - republishes the latest command at publish_rate_hz
        - generates a fresh ROS timestamp on every publication

    CartesianServo.stop():
        - replaces the active command with zero
        - immediately publishes zero
        - stops the internal high-rate publisher
        - pauses MoveIt Servo

    shutdown():
        - safely publishes zero
        - stops the publisher
        - pauses MoveIt Servo

    Public units:
        linear  -> mm/s
        angular -> deg/s

    MoveIt Servo units:
        linear  -> m/s
        angular -> rad/s
    """

    def __init__(
        self,
        *,
        node: Node,
        base_frame: str,
        servo_command_topic: str = "/servo_node/delta_twist_cmds",
        servo_pause_service: str = "/servo_node/pause_servo",
        publish_rate_hz: float = 100.0,
        service_timeout_s: float = 2.0,
    ) -> None:
        super().__init__()

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        if service_timeout_s <= 0.0:
            raise ValueError("service_timeout_s must be > 0")

        base_frame = str(base_frame).strip()
        if not base_frame:
            raise ValueError("base_frame must not be empty")

        self._node = node
        self._base_frame = base_frame

        self._publish_rate_hz = float(publish_rate_hz)
        self._publish_period_s = 1.0 / self._publish_rate_hz
        self._service_timeout_s = float(service_timeout_s)

        # Protects:
        #   _command_frame_id
        #   _latest_command
        self._command_lock = threading.Lock()

        self._command_frame_id: str | None = None
        self._latest_command: CartesianServoCommand | None = None

        self._publish_timer = None

        # --------------------------------------------------------
        # MoveIt Servo command publisher
        # --------------------------------------------------------

        self._twist_pub = self._node.create_publisher(
            TwistStamped,
            servo_command_topic,
            1,
        )

        # --------------------------------------------------------
        # MoveIt Servo lifecycle control
        #
        # This MoveIt Servo version does NOT expose:
        #
        #   /servo_node/start_servo
        #   /servo_node/stop_servo
        #
        # The servo node itself stays alive. We control whether it
        # processes commands through:
        #
        #   /servo_node/pause_servo
        #
        # std_srvs/srv/SetBool:
        #
        #   True  -> paused
        #   False -> running
        # --------------------------------------------------------

        self._pause_client = self._node.create_client(
            SetBool,
            servo_pause_service,
        )

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] initialized "
            "base_frame=%s command_topic=%s pause_service=%s "
            "publish_rate_hz=%.1f",
            self._base_frame,
            servo_command_topic,
            servo_pause_service,
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
        Start one Cartesian Servo session.

        The external MoveIt Servo node is already running as a ROS node.
        Here we configure the command frame/tool, resume Servo, and begin
        publishing fresh TwistStamped commands.
        """

        # --------------------------------------------------------
        # 1. Activate requested software tool/TCP
        # --------------------------------------------------------

        if not self._node.set_active_tool(tool):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to activate tool=%d",
                tool,
            )
            return False

        # --------------------------------------------------------
        # 2. Resolve command frame
        # --------------------------------------------------------

        try:
            command_frame_id = self._resolve_command_frame(
                frame=frame,
                tool=tool,
            )
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to resolve command frame "
                "frame=%s tool=%d",
                frame,
                tool,
            )
            return False

        # --------------------------------------------------------
        # 3. Install initial zero command
        #
        # Do this before resuming Servo so that the first command the
        # high-rate publisher can send is guaranteed to be zero.
        # --------------------------------------------------------

        zero_command = self._zero_command()

        with self._command_lock:
            self._command_frame_id = command_frame_id
            self._latest_command = zero_command

        # --------------------------------------------------------
        # 4. Resume MoveIt Servo
        # --------------------------------------------------------

        if not self._set_servo_paused(False):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to resume MoveIt Servo"
            )

            with self._command_lock:
                self._command_frame_id = None
                self._latest_command = None

            return False

        # --------------------------------------------------------
        # 5. Publish an immediate zero command
        #
        # Do not wait for the first timer tick.
        # --------------------------------------------------------

        try:
            self._publish_command(
                command=zero_command,
                frame_id=command_frame_id,
            )
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to publish initial zero command"
            )

            # Try to leave Servo in a safe paused state.
            try:
                self._set_servo_paused(True)
            except Exception:
                logger.exception(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Failed to pause Servo after start failure"
                )

            with self._command_lock:
                self._command_frame_id = None
                self._latest_command = None

            return False

        # --------------------------------------------------------
        # 6. Start high-rate republisher
        # --------------------------------------------------------

        try:
            self._start_publish_timer()
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to start command publisher"
            )

            try:
                self._publish_command(
                    command=zero_command,
                    frame_id=command_frame_id,
                )
            except Exception:
                logger.debug(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Zero publish after timer failure failed",
                    exc_info=True,
                )

            try:
                self._set_servo_paused(True)
            except Exception:
                logger.debug(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Pause after timer failure failed",
                    exc_info=True,
                )

            with self._command_lock:
                self._command_frame_id = None
                self._latest_command = None

            return False

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] STARTED "
            "frame=%s ros_frame=%s tool=%d "
            "publish_rate_hz=%.1f",
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
        Replace the current Cartesian velocity command.

        update() is intentionally event-based.

        The command is stored once here and the internal ROS timer
        republishes it continuously with fresh timestamps.
        """

        with self._command_lock:
            if not self._command_frame_id:
                logger.error(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "UPDATE rejected: command frame is not configured"
                )
                return False

            self._latest_command = command
            frame_id = self._command_frame_id

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] UPDATE "
            "frame=%s "
            "linear_mm_s=%s "
            "angular_deg_s=%s",
            frame_id,
            command.linear_mm_s,
            command.angular_deg_s,
        )

        return True

    def _on_stop(self) -> bool:
        """
        Stop the current Cartesian Servo session.

        The external MoveIt Servo ROS node remains alive.

        Sequence:
            1. replace stored command with zero
            2. immediately publish zero
            3. stop high-rate publisher
            4. pause MoveIt Servo
            5. clear session state
        """

        zero_command = self._zero_command()

        # --------------------------------------------------------
        # 1. Replace active command with zero
        # --------------------------------------------------------

        with self._command_lock:
            frame_id = self._command_frame_id
            self._latest_command = zero_command

        # --------------------------------------------------------
        # 2. Immediately publish zero
        # --------------------------------------------------------

        if frame_id:
            try:
                self._publish_command(
                    command=zero_command,
                    frame_id=frame_id,
                )
            except Exception:
                logger.exception(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Failed to publish stop zero command"
                )

                # We still continue and attempt to pause Servo.

        # --------------------------------------------------------
        # 3. Stop high-rate publisher
        # --------------------------------------------------------

        self._stop_publish_timer()

        # --------------------------------------------------------
        # 4. Pause MoveIt Servo
        # --------------------------------------------------------

        if not self._set_servo_paused(True):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to pause MoveIt Servo"
            )

            # Keep frame/command information available because the
            # base class will transition to ERROR.
            return False

        # --------------------------------------------------------
        # 5. Clear backend session state
        # --------------------------------------------------------

        with self._command_lock:
            self._latest_command = None
            self._command_frame_id = None

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] STOPPED"
        )

        return True

    # ============================================================
    # MoveIt Servo pause/resume
    # ============================================================

    def _set_servo_paused(
        self,
        paused: bool,
    ) -> bool:
        """
        Pause or resume the external MoveIt Servo node.

        SetBool:
            True  -> pause
            False -> resume
        """

        if not self._pause_client.wait_for_service(
            timeout_sec=self._service_timeout_s,
        ):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause_servo service unavailable"
            )
            return False

        request = SetBool.Request()
        request.data = bool(paused)

        logger.debug(
            "[MOVEIT_CARTESIAN_SERVO] "
            "pause_servo request paused=%s",
            paused,
        )

        future = self._pause_client.call_async(request)

        if not self._wait_future(
            future,
            timeout_s=self._service_timeout_s,
        ):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause_servo timed out paused=%s",
                paused,
            )
            return False

        try:
            response = future.result()
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause_servo service call failed "
                "paused=%s",
                paused,
            )
            return False

        if response is None:
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause_servo returned no response "
                "paused=%s",
                paused,
            )
            return False

        if not response.success:
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause_servo rejected "
                "paused=%s message=%s",
                paused,
                response.message,
            )
            return False

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] "
            "MoveIt Servo %s message=%s",
            "paused" if paused else "resumed",
            response.message,
        )

        return True

    # ============================================================
    # High-rate command publisher
    # ============================================================

    def _start_publish_timer(self) -> None:
        """
        Start the internal high-rate command republisher.
        """

        if self._publish_timer is not None:
            self._stop_publish_timer()

        self._publish_timer = self._node.create_timer(
            self._publish_period_s,
            self._publish_timer_callback,
        )

        logger.debug(
            "[MOVEIT_CARTESIAN_SERVO] "
            "publisher started period_ms=%.3f",
            self._publish_period_s * 1000.0,
        )

    def _stop_publish_timer(self) -> None:
        """
        Stop and destroy the internal command timer.
        """

        timer = self._publish_timer
        self._publish_timer = None

        if timer is None:
            return

        try:
            timer.cancel()
        except Exception:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "timer cancel failed",
                exc_info=True,
            )

        try:
            self._node.destroy_timer(timer)
        except Exception:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "timer destruction failed",
                exc_info=True,
            )

        logger.debug(
            "[MOVEIT_CARTESIAN_SERVO] publisher stopped"
        )

    def _publish_timer_callback(self) -> None:
        """
        Republish the latest stored command.

        The platform therefore only needs to call update() when the
        requested velocity changes.

        MoveIt Servo still receives commands continuously at
        publish_rate_hz with a fresh ROS timestamp.
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
        """
        Convert the public command units and publish TwistStamped.
        """

        msg = TwistStamped()

        # --------------------------------------------------------
        # Fresh timestamp for every publication.
        #
        # This is important because MoveIt Servo rejects stale
        # Cartesian commands.
        # --------------------------------------------------------

        msg.header.stamp = (
            self._node.get_clock().now().to_msg()
        )
        msg.header.frame_id = frame_id

        # --------------------------------------------------------
        # Linear velocity
        #
        # Public API:
        #     mm/s
        #
        # MoveIt Servo:
        #     m/s
        # --------------------------------------------------------

        msg.twist.linear.x = (
            float(command.linear_mm_s[0]) / 1000.0
        )
        msg.twist.linear.y = (
            float(command.linear_mm_s[1]) / 1000.0
        )
        msg.twist.linear.z = (
            float(command.linear_mm_s[2]) / 1000.0
        )

        # --------------------------------------------------------
        # Angular velocity
        #
        # Public API:
        #     deg/s
        #
        # MoveIt Servo:
        #     rad/s
        # --------------------------------------------------------

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
        """
        Resolve the public CartesianServoFrame to the TF frame used
        in TwistStamped.header.frame_id.
        """

        if frame == CartesianServoFrame.BASE:
            return self._base_frame

        if frame == CartesianServoFrame.TOOL:
            # The software TCP is selected through:
            #
            #     node.set_active_tool(tool)
            #
            # The Cartesian twist itself is expressed in the
            # end-effector TF frame.
            #
            # This preserves the behavior of the existing runtime,
            # where TCP offsets are handled separately from the
            # physical MoveIt end-effector link.

            tool_frame = str(
                getattr(config, "EE_LINK", "")
            ).strip()

            if not tool_frame:
                raise RuntimeError(
                    "No end-effector link configured "
                    "for TOOL Cartesian Servo frame"
                )

            return tool_frame

        # The base CartesianServo class should already prevent this.
        raise ValueError(
            f"Unsupported CartesianServoFrame: {frame!r}"
        )

    # ============================================================
    # Shutdown
    # ============================================================

    def shutdown(self) -> None:
        """
        Best-effort safe shutdown.

        This does not destroy the external MoveIt Servo ROS node.
        It only stops this adapter's command stream and pauses Servo.
        """

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] shutting down"
        )

        zero_command = self._zero_command()

        # Capture frame before clearing anything.
        with self._command_lock:
            frame_id = self._command_frame_id
            self._latest_command = zero_command

        # --------------------------------------------------------
        # Publish zero immediately
        # --------------------------------------------------------

        if frame_id:
            try:
                self._publish_command(
                    command=zero_command,
                    frame_id=frame_id,
                )
            except Exception:
                logger.debug(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "zero publish during shutdown failed",
                    exc_info=True,
                )

        # --------------------------------------------------------
        # Stop our high-rate publisher
        # --------------------------------------------------------

        self._stop_publish_timer()

        # --------------------------------------------------------
        # Pause MoveIt Servo
        # --------------------------------------------------------

        try:
            self._set_servo_paused(True)
        except Exception:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause during shutdown failed",
                exc_info=True,
            )

        with self._command_lock:
            self._latest_command = None
            self._command_frame_id = None

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] shutdown complete"
        )

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
        Wait for a ROS future while another executor thread spins
        the node.

        IMPORTANT:
        This assumes that some other executor thread is spinning
        the node. Do not use this from the only executor thread.
        """

        deadline = (
            time.monotonic() + float(timeout_s)
        )

        while not future.done():
            if time.monotonic() >= deadline:
                return False

            time.sleep(0.005)

        return True