#!/usr/bin/env python3
"""Robot status publisher - broadcasts execution state and queue size."""

import json
from std_msgs.msg import String


class RobotStatusPublisher:
    """Publishes robot execution status and queue information to ROS2 topic."""

    def __init__(self, node, motion_queue, topic_name='/robot_status', publish_rate=10.0):
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

        status_data = {
            'is_executing': is_executing,
            'is_available': not is_executing,
            'queue_size': queue_status['queue_size'],
            'current_task_id': queue_status['current_task_id'],
            'timestamp': self.node.get_clock().now().to_msg().sec
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
