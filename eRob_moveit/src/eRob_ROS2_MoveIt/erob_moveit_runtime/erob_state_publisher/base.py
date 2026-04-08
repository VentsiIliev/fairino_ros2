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
  /cartesian_acceleration_norm_mps2 (std_msgs/Float64)
  /cartesian_jerk_norm_mps3     (std_msgs/Float64)
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
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


_PUBLISH_HZ = 50
_CART_HISTORY = 5   # positions kept for vel/acc differentiation


def _fa(data: np.ndarray) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = data.tolist()
    return msg


def _f64(value: float) -> Float64:
    msg = Float64()
    msg.data = float(value)
    return msg


class CartesianPublisherBase(Node):
    """
    Abstract base: handles publishing all five state topics.

    Subclasses override _get_cartesian_pose() to inject a robot-specific
    Cartesian source (TF2 lookup, native SDK message, etc.).
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # ── Publishers ────────────────────────────────────────────────────────
        self._cart_pos_pub = self.create_publisher(
            PoseStamped, '/cartesian_position', 10)
        self._cart_vel_pub = self.create_publisher(
            TwistStamped, '/cartesian_velocity', 10)
        self._cart_acc_pub = self.create_publisher(
            TwistStamped, '/cartesian_acceleration', 10)
        self._cart_jerk_pub = self.create_publisher(
            TwistStamped, '/cartesian_jerk', 10)
        self._cart_pos_norm_mm_pub = self.create_publisher(
            Float64, '/cartesian_position_norm_mm', 10)
        self._cart_vel_norm_mps_pub = self.create_publisher(
            Float64, '/cartesian_velocity_norm_mps', 10)
        self._cart_acc_norm_mps2_pub = self.create_publisher(
            Float64, '/cartesian_acceleration_norm_mps2', 10)
        self._cart_jerk_norm_mps3_pub = self.create_publisher(
            Float64, '/cartesian_jerk_norm_mps3', 10)
        self._joint_vel_pub = self.create_publisher(
            Float64MultiArray, '/joint_velocity', 10)
        self._joint_acc_pub = self.create_publisher(
            Float64MultiArray, '/joint_acceleration', 10)
        self._joint_jerk_pub = self.create_publisher(
            Float64MultiArray, '/joint_jerk', 10)

        # ── Joint-state tracking ──────────────────────────────────────────────
        self._joint_positions: Optional[np.ndarray] = None
        self._prev_positions: Optional[np.ndarray] = None
        self._prev_velocities: Optional[np.ndarray] = None
        self._prev_accelerations: Optional[np.ndarray] = None
        self._prev_joint_time: Optional[float] = None

        # ── Cartesian history for vel/acc/jerk differentiation ────────────────
        # Each entry: (position_xyz_m, velocity_xyz_m_s, acceleration_xyz_m_s2, timestamp_s)
        self._cart_history: deque = deque(maxlen=_CART_HISTORY)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._joint_callback, 10)

        # ── 50 Hz Cartesian publish timer ──────────────────────────────────────
        self.create_timer(1.0 / _PUBLISH_HZ, self._cartesian_timer_cb)

        self.get_logger().info(
            f'[{node_name}] Started — publishing at {_PUBLISH_HZ} Hz')

    # ── Abstract: subclasses must implement ───────────────────────────────────

    def _get_cartesian_pose(self) -> Optional[PoseStamped]:
        """Return current EE pose in base_link frame, or None if unavailable."""
        raise NotImplementedError

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _joint_callback(self, msg: JointState) -> None:
        n = len(msg.position)
        if n < 6:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        positions = np.array(msg.position[:6])

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
        if (self._prev_velocities is not None
                and self._prev_joint_time is not None):
            dt = now - self._prev_joint_time
            accelerations = (
                (velocities - self._prev_velocities) / dt
                if dt > 1e-6 else np.zeros(6)
            )
        else:
            accelerations = np.zeros(6)

        if (self._prev_accelerations is not None
                and self._prev_joint_time is not None):
            dt = now - self._prev_joint_time
            jerks = (
                (accelerations - self._prev_accelerations) / dt
                if dt > 1e-6 else np.zeros(6)
            )
        else:
            jerks = np.zeros(6)

        self._joint_vel_pub.publish(_fa(velocities))
        self._joint_acc_pub.publish(_fa(accelerations))
        self._joint_jerk_pub.publish(_fa(jerks))

        self._joint_positions = positions
        self._prev_positions = positions
        self._prev_velocities = velocities
        self._prev_accelerations = accelerations
        self._prev_joint_time = now

    def _cartesian_timer_cb(self) -> None:
        pose = self._get_cartesian_pose()
        if pose is None:
            return

        self._cart_pos_pub.publish(pose)

        now = self.get_clock().now().nanoseconds * 1e-9
        pos = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ])
        pos_norm = np.linalg.norm(pos)
        self._cart_pos_norm_mm_pub.publish(_f64(pos_norm * 1000.0))

        vel = np.zeros(3)
        acc = np.zeros(3)
        jerk = np.zeros(3)
        if self._cart_history:
            prev_pos, prev_vel, prev_acc, prev_t = self._cart_history[-1]
            dt = now - prev_t
            if dt > 1e-6:
                vel = (pos - prev_pos) / dt
                acc = (vel - prev_vel) / dt
                jerk = (acc - prev_acc) / dt

        self._cart_history.append((pos, vel, acc, now))

        stamp = pose.header.stamp
        frame = pose.header.frame_id

        vel_msg = TwistStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = frame
        vel_msg.twist.linear.x = float(vel[0])
        vel_msg.twist.linear.y = float(vel[1])
        vel_msg.twist.linear.z = float(vel[2])
        self._cart_vel_pub.publish(vel_msg)
        vel_norm = np.linalg.norm(vel)
        self._cart_vel_norm_mps_pub.publish(_f64(vel_norm))

        acc_msg = TwistStamped()
        acc_msg.header.stamp = stamp
        acc_msg.header.frame_id = frame
        acc_msg.twist.linear.x = float(acc[0])
        acc_msg.twist.linear.y = float(acc[1])
        acc_msg.twist.linear.z = float(acc[2])
        self._cart_acc_pub.publish(acc_msg)
        acc_norm = np.linalg.norm(acc)
        self._cart_acc_norm_mps2_pub.publish(_f64(acc_norm))

        jerk_msg = TwistStamped()
        jerk_msg.header.stamp = stamp
        jerk_msg.header.frame_id = frame
        jerk_msg.twist.linear.x = float(jerk[0])
        jerk_msg.twist.linear.y = float(jerk[1])
        jerk_msg.twist.linear.z = float(jerk[2])
        self._cart_jerk_pub.publish(jerk_msg)
        jerk_norm = np.linalg.norm(jerk)
        self._cart_jerk_norm_mps3_pub.publish(_f64(jerk_norm))
