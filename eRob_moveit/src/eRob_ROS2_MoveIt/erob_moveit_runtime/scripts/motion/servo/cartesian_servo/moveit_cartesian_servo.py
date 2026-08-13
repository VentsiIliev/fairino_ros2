from __future__ import annotations

import logging
import math
import threading
import time

import config
import numpy as np
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool
from utils.transformation_utils import TransformationUtils

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
        servo_collision_checking_service: str = "/servo_node/set_collision_checking",
        servo_switch_command_type_service: str = "/servo_node/switch_command_type",
        servo_status_topic: str = "/servo_node/status",
        publish_rate_hz: float = 100.0,
        service_timeout_s: float = 2.0,
        tf_timeout_s: float = 0.05,
    ) -> None:
        super().__init__()

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        if service_timeout_s <= 0.0:
            raise ValueError("service_timeout_s must be > 0")

        if tf_timeout_s <= 0.0:
            raise ValueError("tf_timeout_s must be > 0")

        base_frame = str(base_frame).strip()
        if not base_frame:
            raise ValueError("base_frame must not be empty")

        self._node = node
        self._base_frame = base_frame

        self._publish_rate_hz = float(publish_rate_hz)
        self._publish_period_s = 1.0 / self._publish_rate_hz
        self._service_timeout_s = float(service_timeout_s)
        self._tf_timeout_s = float(tf_timeout_s)

        # Protects:
        #   _command_frame_id
        #   _latest_command
        self._command_lock = threading.Lock()

        self._command_frame_id: str | None = None
        self._command_frame: CartesianServoFrame | None = None
        self._tool_transform = np.eye(4)
        self._workobject_transform = np.eye(4)
        self._latest_command: CartesianServoCommand | None = None

        self._publish_timer = None
        self._servo_paused: bool | None = None
        self._twist_command_type_selected = False
        self._latest_servo_status_code: int | None = None
        self._collision_checking_enabled: bool | None = None
        self._collision_checking_request_lock = threading.Lock()
        self._session_generation = 0
        self._collision_escape_active = False
        self._collision_escape_enabled = bool(
            getattr(config, "CARTESIAN_SERVO_COLLISION_ESCAPE_ENABLED", True)
        )
        self._collision_escape_min_linear_m_s = float(
            getattr(config, "CARTESIAN_SERVO_COLLISION_ESCAPE_MIN_LINEAR_M_S", 0.001)
        )
        self._collision_escape_axis_base = self._normalize_escape_axis(
            getattr(config, "CARTESIAN_SERVO_COLLISION_ESCAPE_AXIS_BASE", [0.0, 0.0, 1.0])
        )
        self._last_start_failure: str | None = None
        self._last_publish_diagnostic_s = 0.0

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

        self._collision_checking_client = self._node.create_client(
            SetBool,
            servo_collision_checking_service,
        )

        self._switch_command_type_client = self._node.create_client(
            ServoCommandType,
            servo_switch_command_type_service,
        )

        self._servo_status_sub = self._node.create_subscription(
            ServoStatus,
            servo_status_topic,
            self._servo_status_callback,
            10,
        )

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] initialized "
            "base_frame=%s command_topic=%s pause_service=%s "
            "collision_checking_service=%s status_topic=%s "
            "publish_rate_hz=%.1f service_timeout_s=%.2f tf_timeout_s=%.2f "
            "collision_escape_enabled=%s",
            self._base_frame,
            servo_command_topic,
            servo_pause_service,
            servo_collision_checking_service,
            servo_status_topic,
            self._publish_rate_hz,
            self._service_timeout_s,
            self._tf_timeout_s,
            self._collision_escape_enabled,
        )

    @staticmethod
    def _normalize_escape_axis(value) -> np.ndarray:
        axis = np.array(value, dtype=float)
        if axis.shape != (3,):
            logger.warning(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Invalid CARTESIAN_SERVO_COLLISION_ESCAPE_AXIS_BASE=%s; using +Z",
                value,
            )
            axis = np.array([0.0, 0.0, 1.0], dtype=float)

        norm = float(np.linalg.norm(axis))
        if norm <= 1e-9:
            logger.warning(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Zero CARTESIAN_SERVO_COLLISION_ESCAPE_AXIS_BASE; using +Z"
            )
            return np.array([0.0, 0.0, 1.0], dtype=float)

        return axis / norm

    def _servo_status_callback(self, msg: ServoStatus) -> None:
        self._latest_servo_status_code = int(msg.code)

    def _set_twist_command_type(self) -> bool:
        """
        Configure MoveIt Servo to accept Cartesian Twist commands.
        """

        if self._twist_command_type_selected:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "switch_command_type skipped; TWIST already selected"
            )
            return True

        if not self._switch_command_type_client.wait_for_service(
                timeout_sec=self._service_timeout_s,
        ):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "switch_command_type service unavailable"
            )
            return False

        request = ServoCommandType.Request()

        # MoveIt Servo command type:
        #   0 = JOINT_JOG
        #   1 = TWIST
        #   2 = POSE
        request.command_type = 1

        logger.debug(
            "[MOVEIT_CARTESIAN_SERVO] "
            "switch_command_type request command_type=TWIST"
        )

        future = self._switch_command_type_client.call_async(request)

        if not self._wait_future(
                future,
                timeout_s=self._service_timeout_s,
        ):
            logger.warning(
                "[MOVEIT_CARTESIAN_SERVO] "
                "switch_command_type timed out; continuing with TWIST command stream"
            )
            self._twist_command_type_selected = True
            return True

        try:
            response = future.result()
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "switch_command_type service call failed"
            )
            return False

        if response is None:
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "switch_command_type returned no response"
            )
            return False

        if not response.success:
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "switch_command_type rejected"
            )
            return False

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] "
            "MoveIt Servo command type set to TWIST"
        )

        self._twist_command_type_selected = True
        return True

    @property
    def last_start_failure(self) -> str | None:
        return self._last_start_failure

    def _fail_start(self, reason: str) -> bool:
        self._last_start_failure = str(reason)
        logger.error("[MOVEIT_CARTESIAN_SERVO] START failed: %s", self._last_start_failure)
        try:
            self._node.get_logger().error(
                f"[MOVEIT_CARTESIAN_SERVO] START failed: {self._last_start_failure}"
            )
        except Exception:
            pass
        return False

    # ============================================================
    # CartesianServo hooks
    # ============================================================

    def _on_start(
        self,
        *,
        frame: CartesianServoFrame,
        tool: int,
        user: int,
    ) -> bool:
        """
        Start one Cartesian Servo session.

        The external MoveIt Servo node is already running as a ROS node.
        Here we configure the command frame/tool, resume Servo, and begin
        publishing fresh TwistStamped commands.
        """
        logger.debug(f"[MOVEIT_CARTESIAN_SERVO] START frame={frame} tool={tool} user={user}")
        self._last_start_failure = None
        self._latest_servo_status_code = None
        self._collision_escape_active = False
        self._collision_checking_enabled = None
        with self._command_lock:
            self._session_generation += 1

        # --------------------------------------------------------
        # 1. Activate requested software tool/TCP
        # --------------------------------------------------------

        if not self._node.set_active_tool(tool):
            return self._fail_start(f"failed to activate tool={tool}")
        logger.debug(
            "[MOVEIT_CARTESIAN_SERVO] "
            "Successfully activated tool=%d",
            tool
        )
        # --------------------------------------------------------
        # 2. Resolve command frame
        # --------------------------------------------------------

        try:
            command_frame_id = self._resolve_command_frame(frame=frame)
            tool_transform = np.array(
                getattr(self._node, "T_tool", np.eye(4)),
                dtype=float,
                copy=True,
            )
            workobject_transform = self._get_workobject_transform(user)

            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Resolved command frame: %s tool_translation_m=%s user=%d",
                command_frame_id,
                np.array2string(tool_transform[:3, 3], precision=4),
                user,
            )
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to resolve command frame "
                "frame=%s tool=%d",
                frame,
                tool,
            )
            return self._fail_start(f"failed to resolve command frame frame={frame.value} tool={tool}")

        # --------------------------------------------------------
        # 3. Install initial zero command
        # --------------------------------------------------------

        zero_command = self._zero_command()

        with self._command_lock:
            self._command_frame_id = command_frame_id
            self._command_frame = frame
            self._tool_transform = tool_transform
            self._workobject_transform = workobject_transform
            self._latest_command = zero_command

        # --------------------------------------------------------
        # 4. Select Cartesian Twist command mode
        # --------------------------------------------------------

        if not self._set_twist_command_type():
            with self._command_lock:
                self._command_frame_id = None
                self._command_frame = None
                self._tool_transform = np.eye(4)
                self._workobject_transform = np.eye(4)
                self._latest_command = None

            return self._fail_start("failed to select TWIST command type")

        # --------------------------------------------------------
        # 5. Resume MoveIt Servo. The Servo node enables its collision monitor
        # as part of unpausing, so do not start it separately here.
        # --------------------------------------------------------

        if not self._set_servo_paused(False):
            with self._command_lock:
                self._command_frame_id = None
                self._command_frame = None
                self._tool_transform = np.eye(4)
                self._workobject_transform = np.eye(4)
                self._latest_command = None

            return self._fail_start("failed to resume MoveIt Servo")

        # --------------------------------------------------------
        # 6. Publish immediate zero
        # --------------------------------------------------------

        try:
            self._publish_command(
                command=zero_command,
                frame_id=command_frame_id,
                command_frame=frame,
                tool_transform=tool_transform,
                workobject_transform=workobject_transform,
            )
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to publish initial zero command"
            )

            try:
                self._set_servo_paused(True)
            except Exception:
                logger.exception(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Failed to pause Servo after start failure"
                )

            with self._command_lock:
                self._command_frame_id = None
                self._command_frame = None
                self._tool_transform = np.eye(4)
                self._workobject_transform = np.eye(4)
                self._latest_command = None

            return self._fail_start("failed to publish initial zero command")

        # --------------------------------------------------------
        # 7. Start high-rate republisher
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
                    command_frame=frame,
                    tool_transform=tool_transform,
                    workobject_transform=workobject_transform,
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
                self._command_frame = None
                self._tool_transform = np.eye(4)
                self._workobject_transform = np.eye(4)
                self._latest_command = None

            return self._fail_start("failed to start command publisher")

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
            command_frame = self._command_frame

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] UPDATE "
            "ros_frame=%s command_frame=%s "
            "linear_mm_s=%s "
            "angular_deg_s=%s",
            frame_id,
            command_frame.value if command_frame else None,
            command.linear_mm_s,
            command.angular_deg_s,
        )
        try:
            self._node.get_logger().info(
                "[MOVEIT_CARTESIAN_SERVO] UPDATE "
                f"ros_frame={frame_id} "
                f"command_frame={command_frame.value if command_frame else None} "
                f"linear_mm_s={command.linear_mm_s} "
                f"angular_deg_s={command.angular_deg_s}"
            )
        except Exception:
            pass

        return True

    def _on_stop(self) -> bool:
        """
        Stop the current Cartesian Servo session.

        The external MoveIt Servo ROS node remains alive.

        Sequence:
            1. replace stored command with zero
            2. immediately publish zero
            3. stop high-rate publisher
            4. clear session state
            5. pause MoveIt Servo in the background
        """

        zero_command = self._zero_command()

        # --------------------------------------------------------
        # 1. Replace active command with zero
        # --------------------------------------------------------

        with self._command_lock:
            frame_id = self._command_frame_id
            command_frame = self._command_frame
            tool_transform = np.array(self._tool_transform, dtype=float, copy=True)
            workobject_transform = np.array(self._workobject_transform, dtype=float, copy=True)
            self._latest_command = zero_command
            stop_generation = self._session_generation

        # --------------------------------------------------------
        # 2. Immediately publish zero
        # --------------------------------------------------------

        if frame_id:
            try:
                self._publish_command(
                    command=zero_command,
                    frame_id=frame_id,
                    command_frame=command_frame,
                    tool_transform=tool_transform,
                    workobject_transform=workobject_transform,
                )
            except Exception:
                logger.exception(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Failed to publish stop zero command"
                )

                # We still continue and attempt to pause Servo.
            else:
                zero_dwell_s = max(
                    0.0,
                    float(getattr(config, "CARTESIAN_SERVO_STOP_ZERO_DWELL_S", 0.05)),
                )
                if zero_dwell_s > 0.0:
                    time.sleep(zero_dwell_s)

        # --------------------------------------------------------
        # 3. Stop high-rate publisher
        # --------------------------------------------------------

        self._stop_publish_timer()

        # --------------------------------------------------------
        # 4. Clear backend session state before returning to HTTP clients.
        # --------------------------------------------------------

        with self._command_lock:
            self._latest_command = None
            self._command_frame_id = None
            self._command_frame = None
            self._tool_transform = np.eye(4)
            self._workobject_transform = np.eye(4)

        # --------------------------------------------------------
        # 5. Pause MoveIt Servo asynchronously. The Servo node stops its
        # collision monitor as part of pausing and that can take long enough to
        # trip UI HTTP timeouts, so the stop endpoint must not wait for it.
        # --------------------------------------------------------

        threading.Thread(
            target=self._pause_servo_after_stop,
            args=(stop_generation,),
            daemon=True,
        ).start()

        logger.info(
            "[MOVEIT_CARTESIAN_SERVO] STOPPED"
        )

        return True

    def _pause_servo_after_stop(self, generation: int) -> None:
        """
        Finish Servo pause after a public stop response has been returned.
        """

        with self._command_lock:
            if generation != self._session_generation:
                logger.debug(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Skipping stale background pause generation=%d current=%d",
                    generation,
                    self._session_generation,
                )
                return

        pause_ok = self._set_servo_paused(True)

        with self._command_lock:
            stale_after_pause = generation != self._session_generation
            active_after_pause = self._command_frame_id is not None

        if stale_after_pause and active_after_pause:
            logger.warning(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Background pause completed after a newer session started; "
                "resuming Servo generation=%d current=%d",
                generation,
                self._session_generation,
            )
            self._set_servo_paused(False)
            return

        if not pause_ok:
            logger.warning(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Failed to confirm MoveIt Servo pause during background stop; "
                "command publisher is stopped and zero command was sent"
            )
            self._servo_paused = True

    # ============================================================
    # MoveIt Servo pause/resume and collision checking
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

        paused = bool(paused)
        if self._servo_paused is paused:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause_servo skipped; already %s",
                "paused" if paused else "resumed",
            )
            return True

        if not self._pause_client.wait_for_service(
            timeout_sec=self._service_timeout_s,
        ):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "pause_servo service unavailable"
            )
            return False

        request = SetBool.Request()
        request.data = paused

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
            response_message = str(getattr(response, "message", ""))
            if "already active" in response_message.lower():
                logger.info(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "MoveIt Servo already %s message=%s",
                    "paused" if paused else "resumed",
                    response_message,
                )
                self._servo_paused = paused
                return True

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

        self._servo_paused = paused
        return True

    def _set_collision_checking(
        self,
        enabled: bool,
    ) -> bool:
        """
        Enable or disable the external MoveIt Servo collision monitor.

        Normal Servo operation keeps this enabled. The runtime only disables it
        for a deliberate escape command after Servo has already entered
        HALT_FOR_COLLISION.
        """

        enabled = bool(enabled)
        if self._collision_checking_enabled is enabled:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "set_collision_checking skipped; already %s",
                "enabled" if enabled else "disabled",
            )
            return True

        if not self._collision_checking_client.wait_for_service(
            timeout_sec=self._service_timeout_s,
        ):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "set_collision_checking service unavailable"
            )
            return False

        request = SetBool.Request()
        request.data = enabled

        logger.warning(
            "[MOVEIT_CARTESIAN_SERVO] "
            "set_collision_checking request enabled=%s",
            enabled,
        )

        future = self._collision_checking_client.call_async(request)

        if not self._wait_future(
            future,
            timeout_s=self._service_timeout_s,
        ):
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "set_collision_checking timed out enabled=%s",
                enabled,
            )
            return False

        try:
            response = future.result()
        except Exception:
            logger.exception(
                "[MOVEIT_CARTESIAN_SERVO] "
                "set_collision_checking service call failed enabled=%s",
                enabled,
            )
            return False

        if response is None:
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "set_collision_checking returned no response enabled=%s",
                enabled,
            )
            return False

        if not response.success:
            logger.error(
                "[MOVEIT_CARTESIAN_SERVO] "
                "set_collision_checking rejected enabled=%s message=%s",
                enabled,
                response.message,
            )
            return False

        self._collision_checking_enabled = enabled
        if enabled:
            self._collision_escape_active = False

        logger.warning(
            "[MOVEIT_CARTESIAN_SERVO] "
            "MoveIt Servo collision checking %s message=%s",
            "enabled" if enabled else "disabled",
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
            command_frame = self._command_frame
            tool_transform = np.array(self._tool_transform, dtype=float, copy=True)
            workobject_transform = np.array(self._workobject_transform, dtype=float, copy=True)

        if command is None or not frame_id:
            return

        try:
            self._publish_command(
                command=command,
                frame_id=frame_id,
                command_frame=command_frame,
                tool_transform=tool_transform,
                workobject_transform=workobject_transform,
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
        command_frame: CartesianServoFrame | None,
        tool_transform,
        workobject_transform,
    ) -> None:
        """
        Convert the public command units and publish an EE-link twist.
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

        linear_m_s = np.array(
            [
                float(command.linear_mm_s[0]) / 1000.0,
                float(command.linear_mm_s[1]) / 1000.0,
                float(command.linear_mm_s[2]) / 1000.0,
            ],
            dtype=float,
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

        angular_rad_s = np.array(
            [
                math.radians(float(command.angular_deg_s[0])),
                math.radians(float(command.angular_deg_s[1])),
                math.radians(float(command.angular_deg_s[2])),
            ],
            dtype=float,
        )

        linear_base, angular_base = self._to_moveit_ee_twist_in_base(
            command_frame=command_frame,
            tool_transform=tool_transform,
            workobject_transform=workobject_transform,
            linear=linear_m_s,
            angular=angular_rad_s,
        )

        self._update_collision_escape(linear_base)

        msg.twist.linear.x = float(linear_base[0])
        msg.twist.linear.y = float(linear_base[1])
        msg.twist.linear.z = float(linear_base[2])
        msg.twist.angular.x = float(angular_base[0])
        msg.twist.angular.y = float(angular_base[1])
        msg.twist.angular.z = float(angular_base[2])

        self._twist_pub.publish(msg)
        speed = float(np.linalg.norm(linear_base) + np.linalg.norm(angular_base))
        now_s = time.monotonic()
        if speed > 1e-9 and now_s - self._last_publish_diagnostic_s >= 1.0:
            self._last_publish_diagnostic_s = now_s
            try:
                self._node.get_logger().info(
                    "[MOVEIT_CARTESIAN_SERVO] PUBLISH "
                    f"frame_id={frame_id} "
                    f"linear_base_m_s={tuple(float(v) for v in linear_base)} "
                    f"angular_base_rad_s={tuple(float(v) for v in angular_base)}"
                )
            except Exception:
                pass

    def _to_moveit_ee_twist_in_base(
        self,
        *,
        command_frame: CartesianServoFrame | None,
        tool_transform,
        workobject_transform,
        linear: np.ndarray,
        angular: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Convert the public TCP twist to the EE-link twist expected by MoveIt Servo.

        LIN/PTP convert a user TCP pose with:
            T_ee = T_tcp @ inv(T_tool)

        The velocity equivalent is:
            v_ee = v_tcp - omega x r_ee_to_tcp
        """

        if command_frame is None:
            command_frame = CartesianServoFrame.BASE

        tool_transform = np.array(tool_transform, dtype=float, copy=False)
        T_base_ee = self._lookup_base_to_ee_transform()
        R_base_ee = T_base_ee[:3, :3]
        R_tool = tool_transform[:3, :3]
        p_tool = tool_transform[:3, 3]
        R_workobject = np.array(workobject_transform, dtype=float, copy=False)[:3, :3]

        if command_frame == CartesianServoFrame.TOOL:
            R_base_tool = R_base_ee @ R_tool
            linear_base = R_base_tool @ linear
            angular_base = R_base_tool @ angular
        elif command_frame == CartesianServoFrame.USER:
            linear_base = R_workobject @ linear
            angular_base = R_workobject @ angular
        else:
            linear_base = linear
            angular_base = angular

        ee_to_tcp_base = R_base_ee @ p_tool
        ee_linear_base = linear_base - np.cross(angular_base, ee_to_tcp_base)
        return ee_linear_base, angular_base

    def _update_collision_escape(self, linear_base: np.ndarray) -> None:
        """
        Let the operator back away from a hard Servo collision halt.

        MoveIt Servo can clamp commands while collision checking is active,
        including commands that move out of contact. For the mounting fixture,
        the escape direction is configured in the base frame and defaults to +Z.

        Collision checking service calls are intentionally dispatched in the
        background. This method runs from the Servo publish timer and must keep
        publishing fresh commands while the collision monitor changes state.
        """

        if not self._collision_escape_enabled:
            return

        status_code = self._latest_servo_status_code
        escape_speed_m_s = float(np.dot(linear_base, self._collision_escape_axis_base))
        should_escape = escape_speed_m_s > self._collision_escape_min_linear_m_s

        if should_escape:
            if not self._collision_escape_active:
                self._collision_escape_active = True
                logger.warning(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Collision escape active: status=%s escape_speed_m_s=%.4f",
                    status_code,
                    escape_speed_m_s,
                )
                self._set_collision_checking_background(
                    False,
                    reason=(
                        "collision escape "
                        f"status={status_code} "
                        f"escape_speed_m_s={escape_speed_m_s:.4f}"
                    ),
                )
            return

        if self._collision_escape_active:
            self._collision_escape_active = False
            logger.warning(
                "[MOVEIT_CARTESIAN_SERVO] "
                "Collision escape cleared: status=%s escape_speed_m_s=%.4f",
                status_code,
                escape_speed_m_s,
            )
            self._set_collision_checking_background(
                True,
                reason=(
                    "collision escape cleared "
                    f"status={status_code} "
                    f"escape_speed_m_s={escape_speed_m_s:.4f}"
                ),
            )

    def _set_collision_checking_background(
        self,
        enabled: bool,
        *,
        reason: str,
    ) -> None:
        threading.Thread(
            target=self._set_collision_checking_worker,
            args=(bool(enabled), str(reason)),
            daemon=True,
        ).start()

    def _set_collision_checking_worker(
        self,
        enabled: bool,
        reason: str,
    ) -> None:
        with self._collision_checking_request_lock:
            if not self._set_collision_checking(enabled):
                logger.warning(
                    "[MOVEIT_CARTESIAN_SERVO] "
                    "Background set_collision_checking failed enabled=%s reason=%s",
                    enabled,
                    reason,
                )

    def _lookup_base_to_ee_transform(self) -> np.ndarray:
        ee_link = str(getattr(config, "EE_LINK", "")).strip()
        if not ee_link:
            raise RuntimeError("No EE_LINK configured for Cartesian Servo")

        transform = self._node.tf_buffer.lookup_transform(
            self._base_frame,
            ee_link,
            Time(),
            timeout=Duration(seconds=self._tf_timeout_s),
        )
        return TransformationUtils.tf2_to_transform(transform)

    def _get_workobject_transform(self, user: int) -> np.ndarray:
        robot = getattr(self._node, "robot", None)
        workobject = None
        if robot is not None and hasattr(robot, "get_workobject"):
            workobject = robot.get_workobject(user)
        if workobject is None:
            return np.eye(4)
        return np.array(getattr(workobject, "transform", np.eye(4)), dtype=float, copy=True)

    # ============================================================
    # Frame resolution
    # ============================================================

    def _resolve_command_frame(
        self,
        *,
        frame: CartesianServoFrame,
    ) -> str:
        """
        Resolve the public CartesianServoFrame to the TF frame used
        in TwistStamped.header.frame_id.
        """

        if frame in (CartesianServoFrame.BASE, CartesianServoFrame.USER, CartesianServoFrame.TOOL):
            return self._base_frame

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
            command_frame = self._command_frame
            tool_transform = np.array(self._tool_transform, dtype=float, copy=True)
            workobject_transform = np.array(self._workobject_transform, dtype=float, copy=True)
            self._latest_command = zero_command

        # --------------------------------------------------------
        # Publish zero immediately
        # --------------------------------------------------------

        if frame_id:
            try:
                self._publish_command(
                    command=zero_command,
                    frame_id=frame_id,
                    command_frame=command_frame,
                    tool_transform=tool_transform,
                    workobject_transform=workobject_transform,
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
        # Restore normal collision checking
        # --------------------------------------------------------

        try:
            self._set_collision_checking(True)
        except Exception:
            logger.debug(
                "[MOVEIT_CARTESIAN_SERVO] "
                "collision checking restore during shutdown failed",
                exc_info=True,
            )

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
            self._command_frame = None
            self._tool_transform = np.eye(4)
            self._workobject_transform = np.eye(4)

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
