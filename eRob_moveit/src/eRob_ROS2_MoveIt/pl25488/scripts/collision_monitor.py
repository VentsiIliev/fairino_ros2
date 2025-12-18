#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from std_msgs.msg import Bool
import numpy as np
from collections import deque


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')

        self.declare_parameter('effort_threshold', [0.2, 0.2, 0.2, 0.15, 0.15, 0.1])
        self.declare_parameter('detection_window', 0.02)
        self.declare_parameter('retract_distance', 0.05)
        self.declare_parameter('recovery_enabled', False)
        self.declare_parameter('startup_delay', 30.0)

        self.effort_threshold = self.get_parameter('effort_threshold').value
        self.detection_window = self.get_parameter('detection_window').value
        self.retract_distance = self.get_parameter('retract_distance').value
        self.recovery_enabled = self.get_parameter('recovery_enabled').value
        startup_delay = self.get_parameter('startup_delay').value

        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

        self.current_position = None
        self.current_effort = None
        self.last_safe_position = None
        self.collision_detected = False
        self.initialized = False

        self.stop_timer = None
        self.reset_timer = None

        self.effort_history = {joint: deque(maxlen=5) for joint in self.joint_names}

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.collision_pub = self.create_publisher(Bool, '/collision_detected', 10)

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_controller/follow_joint_trajectory'
        )

        self.goal_handle = None

        self.get_logger().info('Collision monitor starting...')
        self.get_logger().info(f'Waiting {startup_delay}s for joints to initialize')
        self.get_logger().info(f'Effort thresholds: {self.effort_threshold}')

        self.init_timer = self.create_timer(startup_delay, self.enable_monitoring)

    def enable_monitoring(self):
        self.initialized = True
        self.init_timer.cancel()
        self.get_logger().info('Collision monitoring ENABLED - HIGH SENSITIVITY MODE')

    def joint_state_callback(self, msg):
        try:
            positions = {}
            efforts = {}

            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    positions[name] = msg.position[i] if i < len(msg.position) else 0.0
                    efforts[name] = abs(msg.effort[i]) if i < len(msg.effort) else 0.0

                    self.effort_history[name].append(efforts[name])

            if len(positions) == len(self.joint_names):
                self.current_position = [positions[j] for j in self.joint_names]
                self.current_effort = [efforts[j] for j in self.joint_names]

                if not self.collision_detected:
                    self.last_safe_position = self.current_position.copy()

                if self.initialized:
                    self.check_collision()

        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {e}')

    def check_collision(self):
        if self.current_effort is None:
            return

        for i, (effort, threshold) in enumerate(zip(self.current_effort, self.effort_threshold)):
            joint_name = self.joint_names[i]
            avg_effort = np.mean(list(self.effort_history[joint_name])) if self.effort_history[joint_name] else 0.0

            if avg_effort > threshold:
                self.get_logger().warning(
                    f'COLLISION on {joint_name}! '
                    f'Effort: {avg_effort:.3f} Nm > {threshold:.3f} Nm - STOPPING'
                )
                self.handle_collision()
                break

    def handle_collision(self):
        if self.collision_detected:
            return

        self.collision_detected = True
        collision_msg = Bool()
        collision_msg.data = True
        self.collision_pub.publish(collision_msg)

        self.cancel_current_trajectory()

        if self.recovery_enabled and self.last_safe_position is not None:
            if self.stop_timer is not None:
                self.stop_timer.cancel()
            self.stop_timer = self.create_timer(0.5, self.stop_at_current_position_once)

        if self.reset_timer is not None:
            self.reset_timer.cancel()
        self.reset_timer = self.create_timer(1.5, self.reset_collision_flag_once)

    def cancel_current_trajectory(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling active trajectory...')
            future = self.goal_handle.cancel_goal_async()

        stop_trajectory = JointTrajectory()
        stop_trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_position if self.current_position else [0.0] * 6
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000

        stop_trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = stop_trajectory

        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def stop_at_current_position(self):
        if self.current_position is None:
            return

        self.get_logger().info('Holding current position after collision')

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_position
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000

        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def stop_at_current_position_once(self):
        self.stop_at_current_position()
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warning('Stop trajectory rejected')
            return
        self.get_logger().debug('Stop trajectory accepted')

    def reset_collision_flag(self):
        self.collision_detected = False
        collision_msg = Bool()
        collision_msg.data = False
        self.collision_pub.publish(collision_msg)
        self.get_logger().info('Collision flag reset - ready')

    def reset_collision_flag_once(self):
        self.reset_collision_flag()
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.reset_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

