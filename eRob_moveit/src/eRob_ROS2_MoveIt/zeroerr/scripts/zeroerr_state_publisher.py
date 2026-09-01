#!/usr/bin/env python3
"""
ZeroErr State Publisher
=======================
Publishes Cartesian position by looking up the configured source frame in 'base_link'
via TF2, which is populated by the standard robot_state_publisher node from
/joint_states + URDF.

This ensures the reported Cartesian position is ALWAYS consistent with the
URDF model that MoveIt uses for planning — avoiding the FK mismatch that
would occur if a different DH parameterisation were used.

All other topics (/joint_velocity, /joint_acceleration, /cartesian_velocity,
/cartesian_acceleration) are handled by the shared CartesianPublisherBase.
"""
from setproctitle import setproctitle

setproctitle("zeroerr_state_publisher")
from typing import Optional

import rclpy
from erob_state_publisher.base import CartesianPublisherBase
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


_BASE_FRAME = 'base_link'
_DEFAULT_SOURCE_FRAME = 'ee_link'


class _QuietTransformListener(tf2_ros.TransformListener):
    """TF listener without unused default middleware event callbacks."""

    def __init__(self, buffer: tf2_ros.Buffer, node: Node) -> None:
        self.buffer = buffer
        self.node = node
        self.group = ReentrantCallbackGroup()
        event_callbacks = SubscriptionEventCallbacks(use_default_callbacks=False)
        self.tf_sub = node.create_subscription(
            TFMessage,
            '/tf',
            self.callback,
            QoSProfile(
                depth=100,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
            callback_group=self.group,
            event_callbacks=event_callbacks,
        )
        self.tf_static_sub = node.create_subscription(
            TFMessage,
            '/tf_static',
            self.static_callback,
            QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
            callback_group=self.group,
            event_callbacks=event_callbacks,
        )


class ZeroErrStatePublisher(CartesianPublisherBase):
    """Cartesian source: TF2 lookup of the configured source frame in 'base_link'."""

    def __init__(self) -> None:
        super().__init__('zeroerr_state_publisher')
        self.declare_parameter('cartesian_source_link', _DEFAULT_SOURCE_FRAME)
        self._source_frame = str(
            self.get_parameter('cartesian_source_link').value
            or _DEFAULT_SOURCE_FRAME
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = _QuietTransformListener(self._tf_buffer, self)
        self._tf_warn_logged = False

        self.get_logger().info(
            f'[ZeroErrStatePublisher] Cartesian source: TF2 '
            f'{_BASE_FRAME} ← {self._source_frame}')

    def _get_cartesian_pose(self) -> Optional[PoseStamped]:
        try:
            t = self._tf_buffer.lookup_transform(
                _BASE_FRAME,
                self._source_frame,
                rclpy.time.Time(),          # latest available
            )
            self._tf_warn_logged = False

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = _BASE_FRAME
            tr = t.transform.translation
            ro = t.transform.rotation
            pose.pose.position.x = tr.x
            pose.pose.position.y = tr.y
            pose.pose.position.z = tr.z
            pose.pose.orientation.x = ro.x
            pose.pose.orientation.y = ro.y
            pose.pose.orientation.z = ro.z
            pose.pose.orientation.w = ro.w
            return pose

        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            if not self._tf_warn_logged:
                self.get_logger().warning(
                    f'[ZeroErrStatePublisher] TF2 lookup failed '
                    f'({_BASE_FRAME}←{self._source_frame}): {exc}')
                self._tf_warn_logged = True
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZeroErrStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
