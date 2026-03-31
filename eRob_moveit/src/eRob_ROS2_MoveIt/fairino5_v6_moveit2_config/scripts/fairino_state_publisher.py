#!/usr/bin/env python3
"""
Fairino State Publisher
=======================
Publishes Cartesian position by reading the Fairino robot controller's native
TCP data from /nonrt_state_data (fairino_msgs/RobotNonrtState).

Falls back to Fairino5 v6 DH-parameter FK computed from /joint_states when the
native message has not arrived within 0.5 s (e.g. during initialisation or if
the hardware interface is down).

All other topics (/joint_velocity, /joint_acceleration, /cartesian_velocity,
/cartesian_acceleration) are handled by the shared CartesianPublisherBase.
"""

import time
from typing import Optional

import numpy as np
import rclpy
from erob_state_publisher.base import CartesianPublisherBase
from fairino_msgs.msg import RobotNonrtState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation


# ── Fairino5 v6 DH parameters ────────────────────────────────────────────────
_D1   =  0.152
_A2   = -0.425
_A3   = -0.39501
_D4   =  0.1021
_D5   =  0.102


def _Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[0, 0] = c;  T[0, 1] = -s
    T[1, 0] = s;  T[1, 1] =  c
    return T


def _Rx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[1, 1] = c;  T[1, 2] = -s
    T[2, 1] = s;  T[2, 2] =  c
    return T


def _Tz(d: float) -> np.ndarray:
    T = np.eye(4); T[2, 3] = d; return T


def _Tx(d: float) -> np.ndarray:
    T = np.eye(4); T[0, 3] = d; return T


def _fairino_dh_fk(q: np.ndarray) -> np.ndarray:
    """Fairino5 v6 forward kinematics → 4×4 homogeneous transform (metres)."""
    T = np.eye(4)
    T = T @ _Rz(q[0]) @ _Tz(_D1) @ _Rx(np.pi / 2) @ _Rz(q[1])
    T = T @ _Tx(_A2) @ _Rz(q[2])
    T = T @ _Tx(_A3) @ _Rz(q[3]) @ _Tz(_D4) @ _Rx(np.pi / 2) @ _Rz(q[4])
    T = T @ _Tz(_D5) @ _Rx(-np.pi / 2) @ _Rz(q[5])
    return T


def _transform_to_pose_stamped(T: np.ndarray, stamp, frame_id: str) -> PoseStamped:
    quat = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(T[0, 3])
    pose.pose.position.y = float(T[1, 3])
    pose.pose.position.z = float(T[2, 3])
    pose.pose.orientation.x = float(quat[0])
    pose.pose.orientation.y = float(quat[1])
    pose.pose.orientation.z = float(quat[2])
    pose.pose.orientation.w = float(quat[3])
    return pose


class FairinoStatePublisher(CartesianPublisherBase):
    """Cartesian source: native Fairino TCP via /nonrt_state_data (DH FK fallback)."""

    _NATIVE_TIMEOUT_S = 0.5

    def __init__(self) -> None:
        super().__init__('fairino_state_publisher')

        self._native_pose: Optional[PoseStamped] = None
        self._native_last_recv: float = 0.0
        self._last_tool_num: Optional[int] = None
        self._last_work_num: Optional[int] = None

        self.create_subscription(
            RobotNonrtState,
            '/nonrt_state_data',
            self._nonrt_callback,
            10,
        )
        self.get_logger().info(
            '[FairinoStatePublisher] Cartesian source: /nonrt_state_data '
            '(DH FK fallback after %.1fs silence)' % self._NATIVE_TIMEOUT_S)

    def _nonrt_callback(self, msg: RobotNonrtState) -> None:
        stamp = self.get_clock().now().to_msg()

        # Publish flange pose, not the controller's already-tool-shifted TCP pose.
        # The runtime then reconstructs ee_link/TCP from the URDF and runtime tool
        # registry, keeping reported position consistent with planning.
        rx_rad = msg.flange_a_cur_pos * np.pi / 180.0
        ry_rad = msg.flange_b_cur_pos * np.pi / 180.0
        rz_rad = msg.flange_c_cur_pos * np.pi / 180.0
        quat = (Rotation.from_euler('xyz', [rx_rad, ry_rad, rz_rad])
                .as_quat())

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = msg.flange_x_cur_pos / 1000.0
        pose.pose.position.y = msg.flange_y_cur_pos / 1000.0
        pose.pose.position.z = msg.flange_z_cur_pos / 1000.0
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        self._native_pose = pose
        self._native_last_recv = time.monotonic()

        tool_num = int(getattr(msg, 'tool_num', -1))
        work_num = int(getattr(msg, 'work_num', -1))
        if tool_num != self._last_tool_num or work_num != self._last_work_num:
            self.get_logger().info(
                '[FairinoStatePublisher] Active controller frames: tool=%d workobject=%d',
                tool_num,
                work_num,
            )
            self._last_tool_num = tool_num
            self._last_work_num = work_num

    def _get_cartesian_pose(self) -> Optional[PoseStamped]:
        elapsed = time.monotonic() - self._native_last_recv
        if elapsed <= self._NATIVE_TIMEOUT_S and self._native_pose is not None:
            return self._native_pose

        # DH FK fallback
        if self._joint_positions is None:
            return None

        T = _fairino_dh_fk(self._joint_positions)
        return _transform_to_pose_stamped(
            T, self.get_clock().now().to_msg(), 'base_link')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FairinoStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
