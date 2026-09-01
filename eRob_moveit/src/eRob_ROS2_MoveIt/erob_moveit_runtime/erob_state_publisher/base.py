"""
CartesianPublisherBase
======================
Shared base class for robot-specific Cartesian/joint state publishers.

Publishes the common topic interface used by erob_moveit_runtime:
  /cartesian_position      (geometry_msgs/PoseStamped)
  /cartesian_velocity      (geometry_msgs/TwistStamped)
  /cartesian_acceleration  (geometry_msgs/TwistStamped)
  /cartesian_jerk          (geometry_msgs/TwistStamped)
  /cartesian_position_norm_mm   (std_msgs/Float64)
  /cartesian_velocity_norm_mps  (std_msgs/Float64)
  /cartesian_velocity_norm_mmps (std_msgs/Float64)
  /cartesian_acceleration_norm_mps2 (std_msgs/Float64)
  /cartesian_acceleration_norm_mmps2 (std_msgs/Float64)
  /cartesian_jerk_norm_mps3     (std_msgs/Float64)
  /cartesian_jerk_norm_mmps3    (std_msgs/Float64)
  /joint_velocity          (std_msgs/Float64MultiArray)
  /joint_acceleration      (std_msgs/Float64MultiArray)
  /joint_jerk              (std_msgs/Float64MultiArray)

Subclasses must implement _get_cartesian_pose() -> Optional[PoseStamped].

Joint velocity/acceleration: differentiated from /joint_states (or from
  msg.velocity if available at the source).
Cartesian velocity/acceleration/jerk: differentiated from position history at 50 Hz.
"""

from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


_DEFAULT_PUBLISH_HZ = 50.0
_CART_HISTORY = 5   # samples kept for Cartesian derivative estimation
_STATIONARY_JOINT_VEL_NORM_RAD_S = 0.02
_STATIONARY_CART_SPAN_M = 0.0005


def _fa(data: np.ndarray) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = data.tolist()
    return msg


def _f64(value: float) -> Float64:
    msg = Float64()
    msg.data = float(value)
    return msg


def _estimate_first_derivative(
        samples: deque[tuple[np.ndarray, float]]) -> np.ndarray:
    """Estimate d/dt from newest-first sample history with low added lag."""
    n = len(samples)
    if n < 2:
        return np.zeros(3)

    newest_t = samples[-1][1]
    oldest_t = samples[0][1]
    total_dt = newest_t - oldest_t
    if total_dt <= 1e-6:
        return np.zeros(3)

    dt = total_dt / (n - 1)
    values = [entry[0] for entry in reversed(samples)]
    p0 = values[0]
    p1 = values[1]

    if n == 2:
        return (p0 - p1) / dt
    if n == 3:
        p2 = values[2]
        return (3.0 * p0 - 4.0 * p1 + p2) / (2.0 * dt)
    if n == 4:
        p2 = values[2]
        p3 = values[3]
        return (11.0 * p0 - 18.0 * p1 + 9.0 * p2 - 2.0 * p3) / (6.0 * dt)

    p2 = values[2]
    p3 = values[3]
    p4 = values[4]
    return (
        25.0 * p0 - 48.0 * p1 + 36.0 * p2 - 16.0 * p3 + 3.0 * p4
    ) / (12.0 * dt)


def _estimate_second_derivative(
        samples: deque[tuple[np.ndarray, float]]) -> np.ndarray:
    """Estimate d2/dt2 directly from position history."""
    n = len(samples)
    if n < 3:
        return np.zeros(3)

    newest_t = samples[-1][1]
    oldest_t = samples[0][1]
    total_dt = newest_t - oldest_t
    if total_dt <= 1e-6:
        return np.zeros(3)

    dt = total_dt / (n - 1)
    values = [entry[0] for entry in reversed(samples)]
    p0 = values[0]
    p1 = values[1]
    p2 = values[2]

    if n == 3:
        return (p0 - 2.0 * p1 + p2) / (dt * dt)
    if n == 4:
        p3 = values[3]
        return (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) / (dt * dt)

    p3 = values[3]
    p4 = values[4]
    return (
        35.0 * p0 - 104.0 * p1 + 114.0 * p2 - 56.0 * p3 + 11.0 * p4
    ) / (12.0 * dt * dt)


class CartesianPublisherBase(Node):
    """
    Abstract base: handles publishing all five state topics.

    Subclasses override _get_cartesian_pose() to inject a robot-specific
    Cartesian source (TF2 lookup, native SDK message, etc.).
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # ── Publishers ────────────────────────────────────────────────────────
        self._cart_pos_pub = self._create_topic_publisher(
            'cartesian_position', PoseStamped, '/cartesian_position')
        self._cart_vel_pub = self._create_topic_publisher(
            'cartesian_velocity', TwistStamped, '/cartesian_velocity')
        self._cart_acc_pub = self._create_topic_publisher(
            'cartesian_acceleration', TwistStamped, '/cartesian_acceleration')
        self._cart_jerk_pub = self._create_topic_publisher(
            'cartesian_jerk', TwistStamped, '/cartesian_jerk')
        self._cart_pos_norm_mm_pub = self._create_topic_publisher(
            'cartesian_position_norm_mm', Float64, '/cartesian_position_norm_mm')
        self._cart_vel_norm_mps_pub = self._create_topic_publisher(
            'cartesian_velocity_norm_mps', Float64, '/cartesian_velocity_norm_mps')
        self._cart_vel_norm_mmps_pub = self._create_topic_publisher(
            'cartesian_velocity_norm_mmps', Float64, '/cartesian_velocity_norm_mmps')
        self._cart_acc_norm_mps2_pub = self._create_topic_publisher(
            'cartesian_acceleration_norm_mps2', Float64, '/cartesian_acceleration_norm_mps2')
        self._cart_acc_norm_mmps2_pub = self._create_topic_publisher(
            'cartesian_acceleration_norm_mmps2', Float64, '/cartesian_acceleration_norm_mmps2')
        self._cart_jerk_norm_mps3_pub = self._create_topic_publisher(
            'cartesian_jerk_norm_mps3', Float64, '/cartesian_jerk_norm_mps3')
        self._cart_jerk_norm_mmps3_pub = self._create_topic_publisher(
            'cartesian_jerk_norm_mmps3', Float64, '/cartesian_jerk_norm_mmps3')
        self._joint_vel_pub = self._create_topic_publisher(
            'joint_velocity', Float64MultiArray, '/joint_velocity')
        self._joint_acc_pub = self._create_topic_publisher(
            'joint_acceleration', Float64MultiArray, '/joint_acceleration')
        self._joint_jerk_pub = self._create_topic_publisher(
            'joint_jerk', Float64MultiArray, '/joint_jerk')
        self._cartesian_velocity_enabled = (
            self._cart_vel_pub is not None
            or self._cart_vel_norm_mps_pub is not None
            or self._cart_vel_norm_mmps_pub is not None
        )
        self._cartesian_acceleration_enabled = (
            self._cart_acc_pub is not None
            or self._cart_acc_norm_mps2_pub is not None
            or self._cart_acc_norm_mmps2_pub is not None
        )
        self._cartesian_jerk_enabled = (
            self._cart_jerk_pub is not None
            or self._cart_jerk_norm_mps3_pub is not None
            or self._cart_jerk_norm_mmps3_pub is not None
        )
        self._joint_acceleration_enabled = self._joint_acc_pub is not None
        self._joint_jerk_enabled = self._joint_jerk_pub is not None
        self._joint_derivative_processing_enabled = (
            self._joint_vel_pub is not None
            or self._joint_acceleration_enabled
            or self._joint_jerk_enabled
            or self._cartesian_velocity_enabled
            or self._cartesian_acceleration_enabled
            or self._cartesian_jerk_enabled
        )

        # ── Joint-state tracking ──────────────────────────────────────────────
        self._joint_positions: Optional[np.ndarray] = None
        self._prev_positions: Optional[np.ndarray] = None
        self._prev_velocities: Optional[np.ndarray] = None
        self._prev_accelerations: Optional[np.ndarray] = None
        self._prev_joint_time: Optional[float] = None
        self._last_joint_publish_time: Optional[float] = None
        self._last_joint_process_time: Optional[float] = None
        self._latest_joint_velocity_norm: float = 0.0

        # ── Cartesian history for low-noise derivative estimation ──────────────
        # History is oldest -> newest. Velocity/acceleration are estimated from
        # actual Cartesian position samples, not by repeatedly differencing the
        # previously estimated signal.
        self._cart_history: deque[tuple[np.ndarray, float]] = deque(
            maxlen=_CART_HISTORY)
        self._cart_acc_history: deque[tuple[np.ndarray, float]] = deque(
            maxlen=_CART_HISTORY)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10,
            event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False),
        )

        self.declare_parameter('publish_hz', _DEFAULT_PUBLISH_HZ)
        publish_hz = float(self.get_parameter('publish_hz').value)
        publish_hz = max(1.0, publish_hz)
        self.declare_parameter('joint_publish_hz', 0.0)
        joint_publish_hz = float(self.get_parameter('joint_publish_hz').value)
        self._joint_publish_period = (
            1.0 / joint_publish_hz if joint_publish_hz > 0.0 else 0.0
        )
        self.declare_parameter('joint_input_hz', 0.0)
        joint_input_hz = float(self.get_parameter('joint_input_hz').value)
        self._joint_input_period = (
            1.0 / joint_input_hz if joint_input_hz > 0.0 else 0.0
        )

        # ── Cartesian publish timer ───────────────────────────────────────────
        self.create_timer(1.0 / publish_hz, self._cartesian_timer_cb)

        self.get_logger().info(
            f'[{node_name}] Started — publishing at {publish_hz:.1f} Hz')
        if self._joint_publish_period > 0.0:
            self.get_logger().info(
                f'[{node_name}] Joint derivative topics throttled to '
                f'{joint_publish_hz:.1f} Hz')
        if self._joint_input_period > 0.0:
            self.get_logger().info(
                f'[{node_name}] Joint input processing throttled to '
                f'{joint_input_hz:.1f} Hz')

    # ── Abstract: subclasses must implement ───────────────────────────────────

    def _get_cartesian_pose(self) -> Optional[PoseStamped]:
        """Return current EE pose in base_link frame, or None if unavailable."""
        raise NotImplementedError

    def _create_topic_publisher(self, key: str, msg_type, topic: str):
        parameter_name = f'state_topics.{key}'
        self.declare_parameter(parameter_name, True)
        if not bool(self.get_parameter(parameter_name).value):
            self.get_logger().info(f'[StatePublisher] Disabled topic {topic}')
            return None
        return self.create_publisher(
            msg_type,
            topic,
            10,
            event_callbacks=PublisherEventCallbacks(use_default_callbacks=False),
        )

    def _publish_array_if_subscribed(self, publisher, data: np.ndarray) -> None:
        if publisher is not None:
            publisher.publish(_fa(data))

    def _publish_float_if_subscribed(self, publisher, value: float) -> None:
        if publisher is not None:
            publisher.publish(_f64(value))

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _joint_callback(self, msg: JointState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            self._joint_input_period > 0.0
            and self._last_joint_process_time is not None
            and now - self._last_joint_process_time < self._joint_input_period
        ):
            return
        self._last_joint_process_time = now

        n = len(msg.position)
        if n < 6:
            return

        positions = np.array(msg.position[:6])
        if not self._joint_derivative_processing_enabled:
            self._joint_positions = positions
            self._prev_positions = positions
            self._prev_joint_time = now
            return

        # Prefer velocity field from message if populated
        if msg.velocity and len(msg.velocity) >= 6:
            velocities = np.array(msg.velocity[:6])
        elif (self._prev_positions is not None
              and self._prev_joint_time is not None):
            dt = now - self._prev_joint_time
            velocities = (
                (positions - self._prev_positions) / dt
                if dt > 1e-6
                else (self._prev_velocities
                      if self._prev_velocities is not None
                      else np.zeros(6))
            )
        else:
            velocities = np.zeros(6)

        # Acceleration by differentiating velocity
        if (self._joint_acceleration_enabled or self._joint_jerk_enabled) and (
                self._prev_velocities is not None
                and self._prev_joint_time is not None):
            dt = now - self._prev_joint_time
            accelerations = (
                (velocities - self._prev_velocities) / dt
                if dt > 1e-6 else np.zeros(6)
            )
        else:
            accelerations = np.zeros(6)

        if self._joint_jerk_enabled and (
                self._prev_accelerations is not None
                and self._prev_joint_time is not None):
            dt = now - self._prev_joint_time
            jerks = (
                (accelerations - self._prev_accelerations) / dt
                if dt > 1e-6 else np.zeros(6)
            )
        else:
            jerks = np.zeros(6)

        should_publish = True
        if self._joint_publish_period > 0.0:
            should_publish = (
                self._last_joint_publish_time is None
                or now - self._last_joint_publish_time >= self._joint_publish_period
            )

        if should_publish:
            self._publish_array_if_subscribed(self._joint_vel_pub, velocities)
            self._publish_array_if_subscribed(self._joint_acc_pub, accelerations)
            self._publish_array_if_subscribed(self._joint_jerk_pub, jerks)
            self._last_joint_publish_time = now
        self._latest_joint_velocity_norm = float(np.linalg.norm(velocities))

        self._joint_positions = positions
        self._prev_positions = positions
        self._prev_velocities = velocities
        self._prev_accelerations = accelerations
        self._prev_joint_time = now

    def _cartesian_timer_cb(self) -> None:
        pose = self._get_cartesian_pose()
        if pose is None:
            return

        if self._cart_pos_pub is not None:
            self._cart_pos_pub.publish(pose)

        now = self.get_clock().now().nanoseconds * 1e-9
        pos = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ])
        if self._cart_pos_norm_mm_pub is not None:
            self._cart_pos_norm_mm_pub.publish(_f64(np.linalg.norm(pos) * 1000.0))

        derivatives_enabled = (
            self._cartesian_velocity_enabled
            or self._cartesian_acceleration_enabled
            or self._cartesian_jerk_enabled
        )
        vel = np.zeros(3)
        acc = np.zeros(3)
        jerk = np.zeros(3)
        if derivatives_enabled:
            self._cart_history.append((pos, now))
            vel = _estimate_first_derivative(self._cart_history)
            if self._cartesian_acceleration_enabled or self._cartesian_jerk_enabled:
                acc = _estimate_second_derivative(self._cart_history)
            if self._cartesian_jerk_enabled:
                self._cart_acc_history.append((acc, now))
                jerk = _estimate_first_derivative(self._cart_acc_history)

        # Native Cartesian pose can jitter slightly at rest. When joint-space
        # motion is effectively zero and the recent Cartesian sample spread is
        # tiny, clamp derivatives to zero instead of publishing numerical noise.
        if derivatives_enabled and self._cart_history:
            positions = np.stack([entry[0] for entry in self._cart_history], axis=0)
            cart_span = float(np.linalg.norm(np.ptp(positions, axis=0)))
            if (self._latest_joint_velocity_norm < _STATIONARY_JOINT_VEL_NORM_RAD_S
                    and cart_span < _STATIONARY_CART_SPAN_M):
                vel = np.zeros(3)
                acc = np.zeros(3)
                jerk = np.zeros(3)
                if self._cartesian_jerk_enabled:
                    self._cart_acc_history.clear()
                    self._cart_acc_history.append((acc, now))

        stamp = pose.header.stamp
        frame = pose.header.frame_id

        if self._cart_vel_pub is not None:
            vel_msg = TwistStamped()
            vel_msg.header.stamp = stamp
            vel_msg.header.frame_id = frame
            vel_msg.twist.linear.x = float(vel[0])
            vel_msg.twist.linear.y = float(vel[1])
            vel_msg.twist.linear.z = float(vel[2])
            self._cart_vel_pub.publish(vel_msg)
        if self._cart_vel_norm_mps_pub is not None or self._cart_vel_norm_mmps_pub is not None:
            vel_norm = np.linalg.norm(vel)
            self._publish_float_if_subscribed(self._cart_vel_norm_mps_pub, vel_norm)
            self._publish_float_if_subscribed(self._cart_vel_norm_mmps_pub, vel_norm * 1000.0)

        if self._cart_acc_pub is not None:
            acc_msg = TwistStamped()
            acc_msg.header.stamp = stamp
            acc_msg.header.frame_id = frame
            acc_msg.twist.linear.x = float(acc[0])
            acc_msg.twist.linear.y = float(acc[1])
            acc_msg.twist.linear.z = float(acc[2])
            self._cart_acc_pub.publish(acc_msg)
        if self._cart_acc_norm_mps2_pub is not None or self._cart_acc_norm_mmps2_pub is not None:
            acc_norm = np.linalg.norm(acc)
            self._publish_float_if_subscribed(self._cart_acc_norm_mps2_pub, acc_norm)
            self._publish_float_if_subscribed(self._cart_acc_norm_mmps2_pub, acc_norm * 1000.0)

        if self._cart_jerk_pub is not None:
            jerk_msg = TwistStamped()
            jerk_msg.header.stamp = stamp
            jerk_msg.header.frame_id = frame
            jerk_msg.twist.linear.x = float(jerk[0])
            jerk_msg.twist.linear.y = float(jerk[1])
            jerk_msg.twist.linear.z = float(jerk[2])
            self._cart_jerk_pub.publish(jerk_msg)
        if self._cart_jerk_norm_mps3_pub is not None or self._cart_jerk_norm_mmps3_pub is not None:
            jerk_norm = np.linalg.norm(jerk)
            self._publish_float_if_subscribed(self._cart_jerk_norm_mps3_pub, jerk_norm)
            self._publish_float_if_subscribed(self._cart_jerk_norm_mmps3_pub, jerk_norm * 1000.0)
