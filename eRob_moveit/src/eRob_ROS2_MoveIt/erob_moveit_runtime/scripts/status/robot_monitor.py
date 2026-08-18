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

    Subscribes to:
    - /cartesian_position (PoseStamped): Cartesian position and orientation from C++ node
    - /cartesian_velocity (TwistStamped): Cartesian velocity from C++ node
    - /cartesian_acceleration (TwistStamped): Cartesian acceleration from C++ node
    - /joint_velocity (Float64MultiArray): Joint velocities from C++ node
    - /joint_acceleration (Float64MultiArray): Joint accelerations from C++ node
    """

    def __init__(self, ros_node, velocity_window_size=config.MONITOR_VELOCITY_WINDOW, acceleration_window_size=config.MONITOR_ACCELERATION_WINDOW, tcp_transform=None, stable_update_rate_hz=config.MONITOR_UPDATE_RATE_HZ):
        """
        Initialize robot monitor.

        Args:
            ros_node: ROS 2 node instance for creating subscriptions
            velocity_window_size: Window size for velocity smoothing
            acceleration_window_size: Window size for acceleration smoothing
            tcp_transform: 4x4 homogeneous transform from wrist3 to TCP (unused, kept for compatibility)
            stable_update_rate_hz: Fixed rate for stable data sampling (default: 50Hz = 20ms)
        """
        self.node = ros_node
        self.stable_update_rate_hz = stable_update_rate_hz
        self.stable_update_callback = None
        self.last_stable_update_time = 0.0
        self.prev_positions = None
        self.prev_velocities = None
        self.prev_time = None

        self.velocity_window = deque(maxlen=velocity_window_size)
        self.acceleration_window = deque(maxlen=acceleration_window_size)

        self.latest_data = {
            'positions': np.zeros(6),              # Joint positions [j1..j6] in radians
            'velocities': np.zeros(6),             # Joint velocities [j1..j6] in rad/s
            'accelerations': np.zeros(6),          # Joint accelerations [j1..j6] in rad/s²
            'vel_magnitude': 0.0,                  # Magnitude of joint velocity vector in rad/s
            'acc_magnitude': 0.0,                  # Magnitude of joint acceleration vector in rad/s²
            'cartesian': np.zeros(6),              # TCP pose [x, y, z, rx, ry, rz]: position in mm, orientation in degrees
            'cart_velocity': np.zeros(3),          # TCP linear velocity [vx, vy, vz] in mm/s
            'cart_acceleration': np.zeros(3),      # TCP linear acceleration [ax, ay, az] in mm/s²
            'cart_vel_magnitude': 0.0,             # Magnitude of TCP linear velocity in mm/s
            'cart_acc_magnitude': 0.0,             # Magnitude of TCP linear acceleration in mm/s²
            'efforts': np.zeros(6)                 # Joint efforts/torques [j1..j6] in N·m (currently unused)
        }

        self.stable_data = self.latest_data.copy()
        self._stable_timer = None

        self.T_tcp = tcp_transform if tcp_transform is not None else np.eye(4)
        self._last_source_transform = None

        self.cart_pos_sub = self.node.create_subscription(
            PoseStamped,
            config.TOPIC_CARTESIAN_POSITION,
            self._cartesian_position_callback,
            10
        )

        # Subscribe to Cartesian velocity from C++ node
        self.cart_vel_sub = self.node.create_subscription(
            TwistStamped,
            config.TOPIC_CARTESIAN_VELOCITY,
            self._cartesian_velocity_callback,
            10
        )

        # Subscribe to Cartesian acceleration from C++ node
        self.cart_acc_sub = self.node.create_subscription(
            TwistStamped,
            config.TOPIC_CARTESIAN_ACCELERATION,
            self._cartesian_acceleration_callback,
            10
        )

        # Subscribe to joint velocity from C++ node
        self.joint_vel_sub = self.node.create_subscription(
            Float64MultiArray,
            config.TOPIC_JOINT_VELOCITY,
            self._joint_velocity_callback,
            10
        )

        # Subscribe to joint acceleration from C++ node
        self.joint_acc_sub = self.node.create_subscription(
            Float64MultiArray,
            config.TOPIC_JOINT_ACCELERATION,
            self._joint_acceleration_callback,
            10
        )

        self.node.get_logger().info(f"RobotMonitor: Subscribed to {config.TOPIC_CARTESIAN_POSITION}")
        self.node.get_logger().info(f"RobotMonitor: Subscribed to {config.TOPIC_CARTESIAN_VELOCITY}")
        self.node.get_logger().info(f"RobotMonitor: Subscribed to {config.TOPIC_CARTESIAN_ACCELERATION}")
        self.node.get_logger().info(f"RobotMonitor: Subscribed to {config.TOPIC_JOINT_VELOCITY}")
        self.node.get_logger().info(f"RobotMonitor: Subscribed to {config.TOPIC_JOINT_ACCELERATION}")

        if self.stable_update_rate_hz > 0:
            self._start_stable_update_timer()

    def _start_stable_update_timer(self):
        timer_period = 1.0 / self.stable_update_rate_hz
        self._stable_timer = self.node.create_timer(timer_period, self._stable_update_callback)
        self.node.get_logger().info(f"RobotMonitor: Started stable update timer at {self.stable_update_rate_hz}Hz ({timer_period*1000:.2f}ms)")

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
            msg.pose.position.z
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
        source_euler_deg = TransformationUtils.matrix_to_euler(self._last_source_transform[:3, :3])
        self.latest_data['cartesian_source'] = np.concatenate([source_pos, source_euler_deg])

        T_tcp = self._last_source_transform @ self.T_tcp
        tcp_pos = T_tcp[:3, 3] * 1000.0
        tcp_euler_deg = TransformationUtils.matrix_to_euler(T_tcp[:3, :3])
        self.latest_data['cartesian'] = np.concatenate([tcp_pos, tcp_euler_deg])

    def _cartesian_velocity_callback(self, msg: TwistStamped):

        cart_vel = np.array([
            msg.twist.linear.x * 1000.0,
            msg.twist.linear.y * 1000.0,
            msg.twist.linear.z * 1000.0
        ])

        self.latest_data['cart_velocity'] = cart_vel
        self.latest_data['cart_vel_magnitude'] = np.linalg.norm(cart_vel)

    def _cartesian_acceleration_callback(self, msg: TwistStamped):

        cart_acc = np.array([
            msg.twist.linear.x * 1000.0,
            msg.twist.linear.y * 1000.0,
            msg.twist.linear.z * 1000.0
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
        """
        Update monitor with new joint positions.
        Velocities and accelerations come from C++ node subscriptions (/joint_velocity, /joint_acceleration).
        Cartesian position/velocity/acceleration come from C++ node subscriptions.

        Args:
            joint_positions: Array of 6 joint positions (radians)
            timestamp: datetime timestamp of the measurement (optional, kept for compatibility)

        Returns:
            dict: Latest computed data including positions, velocities, accelerations
        """
        joint_positions = np.array(joint_positions[:6])

        # Update joint positions only - velocities/accelerations updated via callbacks
        self.latest_data['positions'] = joint_positions

        return self.latest_data.copy()

    def _compute_delta_time(self, timestamp):
        """
        Compute time delta between current and previous measurement.

        Args:
            timestamp: Current timestamp

        Returns:
            float or None: Time delta in seconds, or None if invalid
        """
        if self.prev_time is None:
            return None

        dt = (timestamp - self.prev_time).total_seconds()
        if dt <= 0.0:
            return None

        return dt

    def _compute_joint_dynamics(self, joint_positions, dt):
        """
        Compute joint velocities and accelerations.

        Args:
            joint_positions: Current joint positions (radians)
            dt: Time delta (seconds)

        Returns:
            tuple: (velocities, accelerations, vel_magnitude, acc_magnitude)
        """
        # Compute joint velocities
        if self.prev_positions is not None and dt is not None:
            velocities = (joint_positions - self.prev_positions) / dt
        else:
            velocities = np.zeros(6)

        self.velocity_window.append(velocities)
        avg_velocities = np.mean(self.velocity_window, axis=0)

        # Compute joint accelerations
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
        """
        Compute Cartesian position from joint positions using forward kinematics.

        NOTE: This is kept for backward compatibility. The actual Cartesian position
        is received from /cartesian_position subscription (computed in C++ for performance).

        Args:
            joint_positions: Array of 6 joint positions (radians)

        Returns:
            np.array: [x, y, z, rx, ry, rz] (position in meters, orientation in degrees)
        """
        return self.compute_fk(joint_positions, tcp_transform=self.T_tcp)


    def _update_previous_states(self, joint_positions, velocities, timestamp):
        """
        Update previous state variables for next iteration.

        Args:
            joint_positions: Current joint positions
            velocities: Current joint velocities
            timestamp: Current timestamp
        """
        self.prev_positions = joint_positions
        self.prev_velocities = velocities
        self.prev_time = timestamp

    @staticmethod
    def compute_fk(q, tcp_transform=None):
        """
        Forward Kinematics for 6-DOF robot with optional TCP offset.

        Args:
            q (array-like): 6 joint positions in radians [q1, q2, ..., q6]
            tcp_transform (np.ndarray or None): 4x4 homogeneous matrix from wrist3 to TCP

        Returns:
            np.ndarray: [x, y, z, rx, ry, rz] (position in meters, orientation in degrees)
        """
        T = np.eye(4)

        if getattr(config, 'ROBOT_BACKEND', 'fairino') == 'zeroerr':
            # Keep fallback FK consistent with the ZeroErr URDF chain
            # base_link -> ee_link. Normal runtime pose reporting still comes
            # from /cartesian_position, which is generated from TF/URDF.
            T = T @ TransformationUtils.trans(0, 0, 0) @ TransformationUtils.rot_z(q[0])
            T = T @ TransformationUtils.trans(0, 0.053, 0.1405) @ TransformationUtils.rot_y(-q[1])
            T = T @ TransformationUtils.trans(0, 0.0005, 0.3635) @ TransformationUtils.rot_y(-q[2])
            T = T @ TransformationUtils.trans(0, -0.014, 0.311) @ TransformationUtils.rot_y(-q[3])
            T = T @ TransformationUtils.trans(0, 0.047, 0.039) @ TransformationUtils.rot_z(q[4])
            T = T @ TransformationUtils.trans(0, 0.0608, 0.047) @ TransformationUtils.rot_y(q[5])
            T = T @ TransformationUtils.rot_x(-np.pi / 2)
        else:
            # Fairino5 v6 DH FK.
            T = T @ TransformationUtils.rot_z(q[0])
            T = T @ TransformationUtils.trans(0, 0, config.DH_D1) @ TransformationUtils.rot_x(np.pi / 2) @ TransformationUtils.rot_z(q[1])
            T = T @ TransformationUtils.trans(config.DH_A2, 0, 0) @ TransformationUtils.rot_z(q[2])
            T = T @ TransformationUtils.trans(config.DH_A3, 0, 0) @ TransformationUtils.rot_z(q[3])
            T = T @ TransformationUtils.trans(0, 0, config.DH_D4) @ TransformationUtils.rot_x(np.pi / 2) @ TransformationUtils.rot_z(q[4])
            T = T @ TransformationUtils.trans(0, 0, config.DH_D5) @ TransformationUtils.rot_x(-np.pi / 2) @ TransformationUtils.rot_z(q[5])

        # Apply TCP offset if provided
        if tcp_transform is not None:
            T = T @ tcp_transform

        # Extract position and orientation
        pos = T[:3, 3]
        euler_deg = TransformationUtils.matrix_to_euler(T[:3, :3])

        return np.concatenate([pos, euler_deg])

    def get_latest_data(self):
        """
        Get the latest computed data.

        Returns:
            dict: Copy of latest data
        """
        return self.latest_data.copy()

    def get_stable_data(self):
        """
        Get stable data sampled at fixed rate.

        Returns:
            dict: Copy of stable data (updated at stable_update_rate_hz)
        """
        return self.stable_data.copy()

    def set_stable_update_callback(self, callback):
        """
        Register callback to be called at stable update rate.

        Args:
            callback: Function to call with stable_data as argument
        """
        self.stable_update_callback = callback

    def get_cartesian_position(self):
        """
        Get current Cartesian position.

        Returns:
            np.array: [x, y, z, rx, ry, rz] or None
        """
        return self.latest_data.get('cartesian')

    def get_cartesian_source_position(self):
        """
        Get current unmodified Cartesian source position.

        For Fairino this is the flange pose reported by the native controller.
        For generic/ZeroErr backends this is the pose published on /cartesian_position.

        Returns:
            np.array: [x, y, z, rx, ry, rz] or None
        """
        return self.latest_data.get('cartesian_source')

    def get_cartesian_velocity(self):
        """
        Get current Cartesian velocity (linear only).

        Returns:
            np.array: [vx, vy, vz] in mm/s
        """
        return self.latest_data.get('cart_velocity', np.zeros(3))

    def get_cartesian_acceleration(self):
        """
        Get current Cartesian acceleration (linear only).

        Returns:
            np.array: [ax, ay, az] in mm/s²
        """
        return self.latest_data.get('cart_acceleration', np.zeros(3))

    def get_joint_velocities(self):
        """
        Get current joint velocities.

        Returns:
            np.array: Joint velocities in rad/s
        """
        return self.latest_data.get('velocities', np.zeros(6))

    def get_joint_accelerations(self):
        """
        Get current joint accelerations.

        Returns:
            np.array: Joint accelerations in rad/s²
        """
        return self.latest_data.get('accelerations', np.zeros(6))
