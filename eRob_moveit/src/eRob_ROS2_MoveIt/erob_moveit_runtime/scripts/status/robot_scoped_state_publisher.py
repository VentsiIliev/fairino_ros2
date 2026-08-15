#!/usr/bin/env python3
"""Per-robot runtime state publisher for combined multi-robot ROS graphs.

This node owns exactly one RobotRuntimeContext. It may subscribe to a combined
/joint_states topic, but it publishes only the selected robot's state under a
robot-specific topic prefix. Downstream RobotMonitor instances therefore never
need to know that other robots exist.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf2_ros

import config


class RobotScopedStatePublisher(Node):
    def __init__(
        self,
        robot_context,
        *,
        topic_prefix: str | None = None,
        publish_hz: float | None = None,
        node_name: str | None = None,
    ):
        self.robot_context = robot_context
        self.topic_prefix = self._normalize_prefix(
            topic_prefix if topic_prefix is not None else robot_context.name
        )
        resolved_node_name = str(
            node_name or f"{robot_context.name}_state_publisher"
        )
        super().__init__(resolved_node_name)

        self._joint_names = list(robot_context.joint_names)
        self._joint_positions = None
        self._joint_velocities = None
        self._joint_accelerations = None
        self._prev_joint_positions = None
        self._prev_joint_velocities = None
        self._prev_joint_time = None

        self._prev_cartesian_position = None
        self._prev_cartesian_velocity = None
        self._prev_cartesian_time = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cartesian_position_pub = self.create_publisher(
            PoseStamped,
            self._topic(config.TOPIC_CARTESIAN_POSITION),
            10,
        )
        self._cartesian_velocity_pub = self.create_publisher(
            TwistStamped,
            self._topic(config.TOPIC_CARTESIAN_VELOCITY),
            10,
        )
        self._cartesian_acceleration_pub = self.create_publisher(
            TwistStamped,
            self._topic(config.TOPIC_CARTESIAN_ACCELERATION),
            10,
        )
        self._joint_velocity_pub = self.create_publisher(
            Float64MultiArray,
            self._topic(config.TOPIC_JOINT_VELOCITY),
            10,
        )
        self._joint_acceleration_pub = self.create_publisher(
            Float64MultiArray,
            self._topic(config.TOPIC_JOINT_ACCELERATION),
            10,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        hz = float(
            publish_hz
            if publish_hz is not None
            else getattr(config, "STATE_PUBLISH_RATE_HZ", 50.0)
        )
        hz = max(1.0, hz)
        self.create_timer(1.0 / hz, self._publish_cartesian_state)

        self.get_logger().info(
            f"[RobotScopedState] robot={robot_context.name} "
            f"joints={self._joint_names} prefix={self.topic_prefix} "
            f"tf={robot_context.base_link}<-{robot_context.cartesian_source_link} "
            f"rate={hz:.1f}Hz"
        )

    @staticmethod
    def _normalize_prefix(prefix) -> str:
        value = str(prefix or "").strip().strip("/")
        return f"/{value}" if value else ""

    def _topic(self, configured_topic) -> str:
        name = str(configured_topic or "").strip().lstrip("/")
        return f"{self.topic_prefix}/{name}" if self.topic_prefix else f"/{name}"

    @staticmethod
    def _array_message(values) -> Float64MultiArray:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        return msg

    def _joint_state_callback(self, msg: JointState) -> None:
        names = list(getattr(msg, "name", []) or [])
        positions = list(getattr(msg, "position", []) or [])
        if not names or len(names) != len(positions):
            return

        index_by_name = {name: index for index, name in enumerate(names)}
        if any(name not in index_by_name for name in self._joint_names):
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        scoped_positions = np.array(
            [positions[index_by_name[name]] for name in self._joint_names],
            dtype=float,
        )

        incoming_velocities = list(getattr(msg, "velocity", []) or [])
        if len(incoming_velocities) == len(names):
            scoped_velocities = np.array(
                [incoming_velocities[index_by_name[name]] for name in self._joint_names],
                dtype=float,
            )
        elif self._prev_joint_positions is not None and self._prev_joint_time is not None:
            dt = now - self._prev_joint_time
            scoped_velocities = (
                (scoped_positions - self._prev_joint_positions) / dt
                if dt > 1e-6
                else np.zeros(len(self._joint_names))
            )
        else:
            scoped_velocities = np.zeros(len(self._joint_names))

        if self._prev_joint_velocities is not None and self._prev_joint_time is not None:
            dt = now - self._prev_joint_time
            scoped_accelerations = (
                (scoped_velocities - self._prev_joint_velocities) / dt
                if dt > 1e-6
                else np.zeros(len(self._joint_names))
            )
        else:
            scoped_accelerations = np.zeros(len(self._joint_names))

        self._joint_positions = scoped_positions
        self._joint_velocities = scoped_velocities
        self._joint_accelerations = scoped_accelerations
        self._prev_joint_positions = scoped_positions.copy()
        self._prev_joint_velocities = scoped_velocities.copy()
        self._prev_joint_time = now

        self._joint_velocity_pub.publish(
            self._array_message(scoped_velocities)
        )
        self._joint_acceleration_pub.publish(
            self._array_message(scoped_accelerations)
        )

    def _publish_cartesian_state(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.robot_context.base_link,
                self.robot_context.cartesian_source_link,
                rclpy.time.Time(),
            )
        except Exception:
            return

        now_msg = self.get_clock().now().to_msg()
        now = self.get_clock().now().nanoseconds * 1e-9

        pose = PoseStamped()
        pose.header.stamp = now_msg
        pose.header.frame_id = self.robot_context.base_link
        pose.pose.position.x = float(transform.transform.translation.x)
        pose.pose.position.y = float(transform.transform.translation.y)
        pose.pose.position.z = float(transform.transform.translation.z)
        pose.pose.orientation = transform.transform.rotation
        self._cartesian_position_pub.publish(pose)

        current_position = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ])

        if self._prev_cartesian_position is not None and self._prev_cartesian_time is not None:
            dt = now - self._prev_cartesian_time
            velocity = (
                (current_position - self._prev_cartesian_position) / dt
                if dt > 1e-6
                else np.zeros(3)
            )
        else:
            velocity = np.zeros(3)

        if self._prev_cartesian_velocity is not None and self._prev_cartesian_time is not None:
            dt = now - self._prev_cartesian_time
            acceleration = (
                (velocity - self._prev_cartesian_velocity) / dt
                if dt > 1e-6
                else np.zeros(3)
            )
        else:
            acceleration = np.zeros(3)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = now_msg
        velocity_msg.header.frame_id = self.robot_context.base_link
        velocity_msg.twist.linear.x = float(velocity[0])
        velocity_msg.twist.linear.y = float(velocity[1])
        velocity_msg.twist.linear.z = float(velocity[2])
        self._cartesian_velocity_pub.publish(velocity_msg)

        acceleration_msg = TwistStamped()
        acceleration_msg.header.stamp = now_msg
        acceleration_msg.header.frame_id = self.robot_context.base_link
        acceleration_msg.twist.linear.x = float(acceleration[0])
        acceleration_msg.twist.linear.y = float(acceleration[1])
        acceleration_msg.twist.linear.z = float(acceleration[2])
        self._cartesian_acceleration_pub.publish(acceleration_msg)

        self._prev_cartesian_position = current_position
        self._prev_cartesian_velocity = velocity
        self._prev_cartesian_time = now
