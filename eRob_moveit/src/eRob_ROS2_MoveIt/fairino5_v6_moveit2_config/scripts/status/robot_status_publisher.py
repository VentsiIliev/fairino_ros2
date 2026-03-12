#!/usr/bin/env python3
"""Robot status publisher - broadcasts execution state and queue size."""

import json
from std_msgs.msg import String
import config


class RobotStatusPublisher:
    """Publishes robot execution status and queue information to ROS2 topic."""

    def __init__(self, node, motion_queue, topic_name=config.TOPIC_ROBOT_STATUS, publish_rate=config.STATUS_PUBLISH_RATE_HZ):
        """
        Initialize status publisher.

        Args:
            node: ROS2 node instance
            motion_queue: MotionQueue instance to monitor
            topic_name: Topic name for status messages
            publish_rate: Publishing frequency in Hz
        """
        self.node = node
        self.motion_queue = motion_queue

        # Create publisher
        self.publisher = node.create_publisher(String, topic_name, 10)

        # Create timer for periodic publishing
        timer_period = 1.0 / publish_rate
        self.timer = node.create_timer(timer_period, self._publish_status)

        node.get_logger().info(f'[StatusPublisher] Publishing to {topic_name} at {publish_rate} Hz')

    def _publish_status(self):
        """Publish current robot status."""
        queue_status = self.motion_queue.get_status()

        with self.node.lock:
            is_executing = self.node.is_executing

        # Get collision status if detector exists
        collision_status = {}
        external_torque = [0.0] * 6
        expected_torque = [0.0] * 6
        measured_torque = [0.0] * 6

        if hasattr(self.node, 'collision_detector'):
            detector = self.node.collision_detector
            collision_status = detector.get_status()
            external_torque = collision_status.get('external_torque', [0.0] * 6)

            # Get expected torque history if available
            if hasattr(detector, 'expected_torque_history'):
                hist = detector.expected_torque_history
                if len(hist) > 0:
                    expected_torque = list(hist[-1])

        # Get measured torque from current joint state
        if hasattr(self.node, 'current_joint_state') and self.node.current_joint_state is not None:
            if len(self.node.current_joint_state.effort) >= 6:
                measured_torque = list(self.node.current_joint_state.effort[:6])

        status_data = {
            'is_executing': is_executing,
            'is_available': not is_executing,
            'queue_size': queue_status['queue_size'],
            'current_task_id': queue_status['current_task_id'],
            'timestamp': self.node.get_clock().now().to_msg().sec,
            'collision_detected': collision_status.get('state', 'CLEAR') in ['DETECTED', 'RECOVERING'],
            'collision_state': collision_status.get('state', 'CLEAR'),
            'collision_armed': collision_status.get('armed', False),
            'external_torque': external_torque,
            'expected_torque': expected_torque,
            'measured_torque': measured_torque,
            'rate_thresholds': collision_status.get('rate_thresholds', [0.0] * 6),
            'effective_rate_thresholds': collision_status.get('effective_rate_thresholds', [0.0] * 6),
            'current_rate': collision_status.get('current_rate', [0.0] * 6),
        }

        msg = String()
        msg.data = json.dumps(status_data)
        self.publisher.publish(msg)

    def get_status_dict(self):
        """Get current status as dictionary (for REST API)."""
        queue_status = self.motion_queue.get_status()

        with self.node.lock:
            is_executing = self.node.is_executing

        return {
            'is_executing': is_executing,
            'is_available': not is_executing,
            'queue_size': queue_status['queue_size'],
            'current_task_id': queue_status['current_task_id'],
            'max_queue_size': queue_status['max_size']
        }

    def destroy(self):
        """Clean up timer and publisher."""
        if self.timer:
            self.timer.cancel()
            self.node.destroy_timer(self.timer)
