#!/usr/bin/env python3
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Float64MultiArray
from utils.transformation_utils import TransformationUtils
import time
import config


class RobotMonitor:
    """
    Monitor and compute robot kinematics from joint states and Cartesian data.

    By default the legacy global state topics are used. A runtime may provide a
    topic_prefix (for example ``/robot1``) so the same monitor implementation
    consumes only state produced for that robot.
    """

    def __init__(
        self,
        ros_node,
        velocity_window_size=config.MONITOR_VELOCITY_WINDOW,
        acceleration_window_size=config.MONITOR_ACCELERATION_WINDOW,
        tcp_transform=None,
        stable_update_rate_hz=config.MONITOR_UPDATE_RATE_HZ,
        topic_prefix="",
    ):
        """Initialize robot monitor."""
        self.node = ros_node
        self.topic_prefix = self._normalize_topic_prefix(
            topic_prefix or getattr(ros_node, "state_topic_prefix", "")
        )
        self.stable_update_rate_hz = stable_update_rate_hz
        self.stable_update_callback = None
        self.last_stable_update_time = 0.0
        self.prev_positions = None
        self.prev_velocities = None
        self.prev_time = None

        self.velocity_window = deque(maxlen=velocity_window_size)
        self.acceleration_window = deque(maxlen=acceleration_window_size)

        self.latest_data = {
            'positions': np.zeros(6),
            'velocities': np.zeros(6),
            'accelerations': np.zeros(6),
            'vel_magnitude': 0.0,
            'acc_magnitude': 0.0,
            # None means no Cartesian PoseStamped has been received yet.
            # A six-zero array is a legitimate pose and must not be used as an
            # availability sentinel because RobotController readiness checks
            # distinguish unavailable state through prev_cartesian is None.
            'cartesian': None,
            'cart_velocity': np.zeros(3),
            'cart_acceleration': np.zeros(3),
            'cart_vel_magnitude': 0.0,
            'cart_acc_magnitude': 0.0,
            'efforts': np.zeros(6),
        }

        self.stable_data = self.latest_data.copy()
        self._stable_timer = None

        self.T_tcp = tcp_transform if tcp_transform is not None else np.eye(4)
        self._last_source_transform = None

        cartesian_position_topic = self._topic(config.TOPIC_CARTESIAN_POSITION)
        cartesian_velocity_topic = self._topic(config.TOPIC_CARTESIAN_VELOCITY)
        cartesian_acceleration_topic = self._topic(config.TOPIC_CARTESIAN_ACCELERATION)
        joint_velocity_topic = self._topic(config.TOPIC_JOINT_VELOCITY)
        joint_acceleration_topic = self._topic(config.TOPIC_JOINT_ACCELERATION)

        self.cart_pos_sub = self.node.create_subscription(
            PoseStamped,
            cartesian_position_topic,
            self._cartesian_position_callback,
            10,
        )
        self.cart_vel_sub = self.node.create_subscription(
            TwistStamped,
            cartesian_velocity_topic,
            self._cartesian_velocity_callback,
            10,
        )
        self.cart_acc_sub = self.node.create_subscription(
            TwistStamped,
            cartesian_acceleration_topic,
            self._cartesian_acceleration_callback,
            10,
        )
        self.joint_vel_sub = self.node.create_subscription(
            Float64MultiArray,
            joint_velocity_topic,
            self._joint_velocity_callback,
            10,
        )
        self.joint_acc_sub = self.node.create_subscription(
            Float64MultiArray,
            joint_acceleration_topic,
            self._joint_acceleration_callback,
            10,
        )

        self.node.get_logger().info(
            f"RobotMonitor: Subscribed to {cartesian_position_topic}"
        )
        self.node.get_logger().info(
            f"RobotMonitor: Subscribed to {cartesian_velocity_topic}"
        )
        self.node.get_logger().info(
            f"RobotMonitor: Subscribed to {cartesian_acceleration_topic}"
        )
        self.node.get_logger().info(
            f"RobotMonitor: Subscribed to {joint_velocity_topic}"
        )
        self.node.get_logger().info(
            f"RobotMonitor: Subscribed to {joint_acceleration_topic}"
        )

        if self.stable_update_rate_hz > 0:
            self._start_stable_update_timer()

    @staticmethod
    def _normalize_topic_prefix(topic_prefix):
        value = str(topic_prefix or "").strip().strip("/")
        return f"/{value}" if value else ""

    def _topic(self, topic):
        name = str(topic or "").strip()
        if not self.topic_prefix:
            return name
        return f"{self.topic_prefix}/{name.lstrip('/')}"

    def _start_stable_update_timer(self):
        timer_period = 1.0 / self.stable_update_rate_hz
        self._stable_timer = self.node.create_timer(
            timer_period,
            self._stable_update_callback,
        )
        self.node.get_logger().info(
            f"RobotMonitor: Started stable update timer at "
            f"{self.stable_update_rate_hz}Hz ({timer_period*1000:.2f}ms)"
        )

    def _stable_update_callback(self):
        current_time = time.time()

        if self.last_stable_update_time > 0.0:
            dt = (current_time - self.last_stable_update_time) * 1000.0
            hz = 1000.0 / dt if dt > 0 else 0.0
            # self.node.get_logger().info(f"[STABLE_UPDATE] dt={dt:.2f}ms, Hz={hz:.1f}")

        self.last_stable_update_time = current_time
        self.stable_data = self.latest_data.copy()

        if self.stable_update_callback is not None:
            self.stable_update_callback(self.stable_data)

    def _cartesian_position_callback(self, msg: PoseStamped):
        ee_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

        q = msg.pose.orientation
        ee_rot_matrix = TransformationUtils.quaternion_to_matrix(
            np.array([q.x, q.y, q.z, q.w])
        )

        T_ee = np.eye(4)
        T_ee[:3, :3] = ee_rot_matrix
        T_ee[:3, 3] = ee_pos

        self._last_source_transform = T_ee
        self._update_cartesian_from_source()

    def set_tcp_transform(self, tcp_transform):
        self.T_tcp = tcp_transform if tcp_transform is not None else np.eye(4)
        self._update_cartesian_from_source()

    def _update_cartesian_from_source(self):
        if self._last_source_transform is None:
            return

        source_pos = self._last_source_transform[:3, 3] * 1000.0
        source_euler_deg = TransformationUtils.matrix_to_euler(
            self._last_source_transform[:3, :3]
        )
        self.latest_data['cartesian_source'] = np.concatenate([
            source_pos,
            source_euler_deg,
        ])

        T_tcp = self._last_source_transform @ self.T_tcp
        tcp_pos = T_tcp[:3, 3] * 1000.0
        tcp_euler_deg = TransformationUtils.matrix_to_euler(T_tcp[:3, :3])
        self.latest_data['cartesian'] = np.concatenate([tcp_pos, tcp_euler_deg])

    def _cartesian_velocity_callback(self, msg: TwistStamped):
        cart_vel = np.array([
            msg.twist.linear.x * 1000.0,
            msg.twist.linear.y * 1000.0,
            msg.twist.linear.z * 1000.0,
        ])

        self.latest_data['cart_velocity'] = cart_vel
        self.latest_data['cart_vel_magnitude'] = np.linalg.norm(cart_vel)

    def _cartesian_acceleration_callback(self, msg: TwistStamped):
        cart_acc = np.array([
            msg.twist.linear.x * 1000.0,
            msg.twist.linear.y * 1000.0,
            msg.twist.linear.z * 1000.0,
        ])

        self.latest_data['cart_acceleration'] = cart_acc
        self.latest_data['cart_acc_magnitude'] = np.linalg.norm(cart_acc)

    def _joint_velocity_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            joint_vel = np.array(msg.data[:6])
            self.latest_data['velocities'] = joint_vel
            self.latest_data['vel_magnitude'] = np.linalg.norm(joint_vel)

    def _joint_acceleration_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            joint_acc = np.array(msg.data[:6])
            self.latest_data['accelerations'] = joint_acc
            self.latest_data['acc_magnitude'] = np.linalg.norm(joint_acc)

    def update_joint_state(self, joint_positions, timestamp=None):
        joint_positions = np.array(joint_positions[:6])
        self.latest_data['positions'] = joint_positions
        return self.latest_data.copy()

    def _compute_delta_time(self, timestamp):
        if self.prev_time is None:
            return None

        dt = (timestamp - self.prev_time).total_seconds()
        if dt <= 0.0:
            return None

        return dt

    def _compute_joint_dynamics(self, joint_positions, dt):
        if self.prev_positions is not None and dt is not None:
            velocities = (joint_positions - self.prev_positions) / dt
        else:
            velocities = np.zeros(6)

        self.velocity_window.append(velocities)
        avg_velocities = np.mean(self.velocity_window, axis=0)

        if self.prev_velocities is not None and dt is not None:
            accelerations = (velocities - self.prev_velocities) / dt
        else:
            accelerations = np.zeros(6)

        self.acceleration_window.append(accelerations)
        avg_accelerations = np.mean(self.acceleration_window, axis=0)

        vel_mag = np.linalg.norm(avg_velocities)
        acc_mag = np.linalg.norm(avg_accelerations)

        return avg_velocities, avg_accelerations, vel_mag, acc_mag

    def compute_cartesian_position(self, joint_positions):
        return self.compute_fk(joint_positions, tcp_transform=self.T_tcp)

    def _update_previous_states(self, joint_positions, velocities, timestamp):
        self.prev_positions = joint_positions
        self.prev_velocities = velocities
        self.prev_time = timestamp

    @staticmethod
    def compute_fk(q, tcp_transform=None):
        T = np.eye(4)
        T = T @ TransformationUtils.rot_z(q[0])
        T = T @ TransformationUtils.trans(0, 0, config.DH_D1) @ TransformationUtils.rot_x(np.pi / 2) @ TransformationUtils.rot_z(q[1])
        T = T @ TransformationUtils.trans(config.DH_A2, 0, 0) @ TransformationUtils.rot_z(q[2])
        T = T @ TransformationUtils.trans(config.DH_A3, 0, 0) @ TransformationUtils.rot_z(q[3])
        T = T @ TransformationUtils.trans(0, 0, config.DH_D4) @ TransformationUtils.rot_x(np.pi / 2) @ TransformationUtils.rot_z(q[4])
        T = T @ TransformationUtils.trans(0, 0, config.DH_D5) @ TransformationUtils.rot_x(-np.pi / 2) @ TransformationUtils.rot_z(q[5])

        if tcp_transform is not None:
            T = T @ tcp_transform

        pos = T[:3, 3]
        euler_deg = TransformationUtils.matrix_to_euler(T[:3, :3])
        return np.concatenate([pos, euler_deg])

    def get_latest_data(self):
        return self.latest_data.copy()

    def get_stable_data(self):
        return self.stable_data.copy()

    def set_stable_update_callback(self, callback):
        self.stable_update_callback = callback

    def get_cartesian_position(self):
        return self.latest_data.get('cartesian')

    def get_cartesian_source_position(self):
        return self.latest_data.get('cartesian_source')

    def get_cartesian_velocity(self):
        return self.latest_data.get('cart_velocity', np.zeros(3))

    def get_cartesian_acceleration(self):
        return self.latest_data.get('cart_acceleration', np.zeros(3))

    def get_joint_velocities(self):
        return self.latest_data.get('velocities', np.zeros(6))

    def get_joint_accelerations(self):
        return self.latest_data.get('accelerations', np.zeros(6))
