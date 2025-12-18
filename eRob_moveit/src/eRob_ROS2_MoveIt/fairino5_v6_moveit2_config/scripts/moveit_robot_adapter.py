#!/usr/bin/env python3

"""
MoveIt Robot Adapter - A ROS2 MoveIt-based implementation of the IRobot interface.

This adapter allows using ROS2 MoveIt as a drop-in replacement for the FairinoRobot
class, enabling motion planning and control through MoveIt instead of direct RPC.

This module can be imported standalone or integrated into existing robot control systems.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from threading import Lock, Event
from enum import Enum

# Try to import real ROS2 modules, fall back to stubs if not available
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Pose
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
    from shape_msgs.msg import SolidPrimitive
    ROS2_AVAILABLE = True
except ImportError:
    # Fall back to stubs for type checking
    ROS2_AVAILABLE = False
    print("Warning: ROS2 modules not available, using stubs")
    from core.model.robot.rclpy_stubs import (
        Node, MoveGroup, ActionClient, Pose,
        MotionPlanRequest, Constraints, PositionConstraint,
        OrientationConstraint, BoundingVolume
    )


class Axis(Enum):
    """Jog axis enumeration matching FairinoRobot"""
    X = 1
    Y = 2
    Z = 3
    RX = 4
    RY = 5
    RZ = 6


class Direction(Enum):
    """Jog direction enumeration matching FairinoRobot"""
    MINUS = 0
    PLUS = 1


class IRobot:
    """Base robot interface - to be implemented by robot adapters"""

    def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        raise NotImplementedError

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        raise NotImplementedError

    def get_current_position(self):
        raise NotImplementedError

    def get_current_velocity(self):
        raise NotImplementedError

    def get_current_acceleration(self):
        raise NotImplementedError

    def enable(self):
        raise NotImplementedError

    def disable(self):
        raise NotImplementedError

    def setDigitalOutput(self, portId, value):
        raise NotImplementedError

    def start_jog(self, axis, direction, step, vel, acc):
        raise NotImplementedError

    def stop_motion(self):
        raise NotImplementedError

    def resetAllErrors(self):
        raise NotImplementedError


class MoveItRobotAdapter(Node, IRobot):
    """
    ROS2 MoveIt-based robot adapter implementing the IRobot interface.

    This adapter provides a unified interface for robot control using MoveIt,
    allowing seamless integration with code designed for FairinoRobot.
    """

    def __init__(self, node_name='moveit_robot_adapter', group_name='fairino5_v6_group',
                 end_effector_link='wrist3_link', base_frame='base_link'):
        """
        Initialize the MoveIt robot adapter.

        Args:
            node_name (str): ROS2 node name
            group_name (str): MoveIt planning group name
            end_effector_link (str): End effector link name
            base_frame (str): Base frame for motion planning
        """
        super().__init__(node_name)

        self.group_name = group_name
        self.end_effector_link = end_effector_link
        self.base_frame = base_frame

        # Thread safety
        self.lock = Lock()
        self.goal_complete_event = Event()
        self.execution_lock = Lock()  # Prevent overlapping goal execution

        # Robot state
        self.current_position = None  # Cartesian position [x, y, z, rx, ry, rz]
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.enabled = False
        self.last_goal_result = None
        self.is_executing = False  # Track if a goal is currently executing

        # ROS2 action clients with reentrant callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10,
            callback_group=self.callback_group
        )

        # Wait for action server
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('MoveGroup action server not available after 5 seconds')
        else:
            self.get_logger().info('MoveGroup action server connected')

        # Default motion parameters (matching FairinoRobot defaults)
        self.default_vel_scaling = 0.3
        self.default_acc_scaling = 0.3

        self.get_logger().info(f'MoveItRobotAdapter initialized for group: {group_name}')

    def joint_state_callback(self, msg):
        """Update current robot state from joint_states topic"""
        with self.lock:
            if len(msg.position) >= 6:
                self.current_joint_positions = list(msg.position[:6])

                if hasattr(msg, 'velocity') and len(msg.velocity) >= 6:
                    self.current_joint_velocities = list(msg.velocity[:6])

                # Compute forward kinematics to get Cartesian position
                self.current_position = self._compute_fk(msg)

    def _compute_fk(self, joint_state_msg):
        """
        Compute forward kinematics to get Cartesian pose.
        Returns [x, y, z, rx, ry, rz] in mm and degrees.
        """
        if len(joint_state_msg.position) < 6:
            return None

        q = np.array(joint_state_msg.position[:6])

        # DH parameters forward kinematics (same as velocity_monitor.py)
        def rotZ(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        def rotX(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

        def trans(x, y, z):
            return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

        T = np.eye(4)
        T = T @ rotZ(q[0])
        T = T @ trans(0, 0, 0.152) @ rotX(np.pi/2) @ rotZ(q[1])
        T = T @ trans(-0.425, 0, 0) @ rotZ(q[2])
        T = T @ trans(-0.39501, 0, 0) @ rotZ(q[3])
        T = T @ trans(0, 0, 0.1021) @ rotX(np.pi/2) @ rotZ(q[4])
        T = T @ trans(0, 0, 0.102) @ rotX(-np.pi/2) @ rotZ(q[5])

        # Extract position (convert to mm)
        x_mm = T[0, 3] * 1000.0
        y_mm = T[1, 3] * 1000.0
        z_mm = T[2, 3] * 1000.0

        # Extract rotation (convert to euler angles in degrees)
        R = T[:3, :3]
        rot = Rotation.from_matrix(R)
        rx, ry, rz = rot.as_euler('xyz', degrees=True)

        return [x_mm, y_mm, z_mm, rx, ry, rz]

    def _velocity_to_scaling(self, vel):
        """Convert velocity percentage (0-100) to scaling factor (0.0-1.0)"""
        return np.clip(vel / 100.0, 0.01, 1.0)

    def _acceleration_to_scaling(self, acc):
        """Convert acceleration percentage (0-100) to scaling factor (0.0-1.0)"""
        return np.clip(acc / 100.0, 0.01, 1.0)

    def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        """
        Move to a Cartesian position using MoveIt.

        Args:
            position (list): [x, y, z, rx, ry, rz] in mm and degrees
            tool (int): Tool frame ID (not used in MoveIt adapter)
            user (int): User frame ID (not used in MoveIt adapter)
            vel (float): Velocity percentage (0-100)
            acc (float): Acceleration percentage (0-100)
            blendR (float): Blend radius (not used for single point moves)

        Returns:
            list: [error_code, message]
        """
        if not self.enabled:
            self.get_logger().warn('Robot is disabled. Call enable() first.')
            return [-1, 'Robot disabled']

        if len(position) != 6:
            self.get_logger().error(f'Invalid position format: {position}')
            return [-1, 'Invalid position format']

        # Extract position and orientation
        x_mm, y_mm, z_mm, rx, ry, rz = position
        x = x_mm / 1000.0
        y = y_mm / 1000.0
        z = z_mm / 1000.0

        # Convert velocity/acceleration to scaling factors
        vel_scaling = self._velocity_to_scaling(vel)
        acc_scaling = self._acceleration_to_scaling(acc)

        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.group_name
        goal.request.planner_id = 'LIN'  # Use Pilz LIN for Cartesian motion
        goal.request.pipeline_id = 'pilz_industrial_motion_planner'
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = vel_scaling
        goal.request.max_acceleration_scaling_factor = acc_scaling

        # Set workspace parameters
        goal.request.workspace_parameters.header.frame_id = self.base_frame
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = 0.010
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0

        # Create pose
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Convert euler angles to quaternion
        rot = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True)
        quat = rot.as_quat(canonical=False)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        # Create position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.003]  # 3mm tolerance
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        # Create orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0

        # Add constraints to goal
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal.request.goal_constraints.append(constraints)

        # Execute motion
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0

        self.get_logger().info(f'MoveCart to [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}, {rx:.1f}, {ry:.1f}, {rz:.1f}]')

        # Send goal and wait for result
        self.goal_complete_event.clear()
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

        # Wait for completion (blocking behavior like FairinoRobot)
        self.goal_complete_event.wait(timeout=30.0)

        return self.last_goal_result if self.last_goal_result else [0, 'Success']

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        """
        Execute linear motion with blending (uses same implementation as move_cartesian).

        For true linear interpolation with blending, use Cartesian path planning.
        """
        # For single point moves, this is equivalent to move_cartesian
        # Blend radius is handled in multi-point paths
        return self.move_cartesian(position, tool, user, vel, acc, blendR)

    def get_current_position(self):
        """
        Get the current TCP position.

        Returns:
            list: [x, y, z, rx, ry, rz] in mm and degrees, or None if unavailable
        """
        with self.lock:
            return self.current_position

    def get_current_velocity(self):
        """
        Get current joint velocities.

        Returns:
            list: Joint velocities in rad/s, or None if unavailable
        """
        with self.lock:
            return self.current_joint_velocities

    def get_current_acceleration(self):
        """
        Get current joint accelerations (not available from joint_states).

        Returns:
            None: Acceleration not available in standard ROS2 joint_states
        """
        self.get_logger().warn('Acceleration not available from joint_states topic')
        return None

    def enable(self):
        """Enable robot motion"""
        self.enabled = True
        self.get_logger().info('Robot enabled')

    def disable(self):
        """Disable robot motion"""
        self.enabled = False
        self.get_logger().info('Robot disabled')

    def setDigitalOutput(self, portId, value):
        """
        Set digital output (not implemented in MoveIt adapter).

        This would require a separate ROS2 topic/service for I/O control.
        """
        self.get_logger().warn(f'Digital I/O not implemented in MoveIt adapter')
        return [-1, 'Not implemented']

    def start_jog(self, axis, direction, step, vel, acc):
        """
        Start jogging in specified axis and direction.

        Args:
            axis (Axis): Axis to jog
            direction (Direction): Direction (PLUS or MINUS)
            step (float): Distance to move in mm
            vel (float): Velocity percentage
            acc (float): Acceleration percentage

        Returns:
            list: [error_code, message]
        """
        if not self.enabled:
            return [-1, 'Robot disabled']

        # Get current position
        current = self.get_current_position()
        if current is None:
            self.get_logger().error('Current position not available for jogging')
            return [-1, 'Position unavailable']

        # Calculate target position based on axis and direction
        target = current.copy()
        axis_idx = axis.value - 1  # Convert to 0-indexed

        if direction == Direction.PLUS:
            target[axis_idx] += step
        else:
            target[axis_idx] -= step

        # Execute move
        return self.move_cartesian(target, vel=vel, acc=acc)

    def stop_motion(self):
        """
        Stop all motion (emergency stop).

        Note: This cancels the current goal but does not provide true emergency stop.
        For safety-critical applications, use robot controller's hardware E-stop.
        """
        self.get_logger().warn('Stopping motion - canceling current goal')
        # Cancel all goals
        self.move_group_client.cancel_all_goals()
        return [0, 'Motion stopped']

    def resetAllErrors(self):
        """
        Reset errors (not applicable in MoveIt adapter).

        Error reset should be handled at the robot controller level.
        """
        self.get_logger().info('Error reset not applicable in MoveIt adapter')
        return [0, 'No errors']

    def _goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            self.last_goal_result = [-1, 'Goal rejected']
            self.goal_complete_event.set()
            return

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Callback when goal execution completes"""
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Goal succeeded')
            self.last_goal_result = [0, 'Success']
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code.val}')
            self.last_goal_result = [result.error_code.val, f'Error {result.error_code.val}']

        self.goal_complete_event.set()


# Example usage
if __name__ == '__main__':
    rclpy.init()

    # Create adapter instance
    robot = MoveItRobotAdapter()

    # Enable robot
    robot.enable()

    # Spin in the background thread
    from threading import Thread
    spin_thread = Thread(target=rclpy.spin, args=(robot,), daemon=True)
    spin_thread.start()

    # Wait for joint states
    import time
    time.sleep(2.0)

    # Get the current position
    current_pos = robot.get_current_position()
    if current_pos:
        print(f'Current position: {current_pos}')

        # Move to a new position (example)
        target = current_pos.copy()
        target[2] += 50  # Move 50mm up in Z

        print(f'Moving to: {target}')
        result = robot.move_cartesian(target, vel=30, acc=30)
        print(f'Move result: {result}')

    # Cleanup
    robot.destroy_node()
    rclpy.shutdown()