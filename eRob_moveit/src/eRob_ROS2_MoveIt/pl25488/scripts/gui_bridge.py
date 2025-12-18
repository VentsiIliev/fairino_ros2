#!/usr/bin/env python3
"""
SIMPLIFIED GUI BRIDGE - Pure message forwarder
No IK, no FK, no path planning, no computations.
Just forwards pose/joint commands to MoveIt's MoveGroup action.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest,
    MotionSequenceRequest,
    MotionSequenceItem,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    JointConstraint
)
from shape_msgs.msg import SolidPrimitive
import threading
from collections import deque
class GUIBridge(Node):
    """
    Minimal GUI bridge - just forwards commands to MoveIt.
    NO computations, NO IK/FK, NO path planning.
    MoveIt does ALL the work.
    """
    def __init__(self):
        super().__init__('gui_bridge')
        self._executing = False
        self._execution_lock = threading.Lock()
        self._command_queue = deque(maxlen=10)
        self._queue_lock = threading.Lock()

        # MoveIt action client
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for move_group...')
        self._move_group_client.wait_for_server()
        self.get_logger().info('✓ Connected to move_group')

        # Cartesian path service for multi-waypoint trajectories
        from rclpy.node import Client
        self._cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        # Parameters - high speed with S-curve jerk limiting for smooth motion
        self.velocity_scaling = self.declare_parameter('velocity_scaling', 0.9).value
        self.acceleration_scaling = self.declare_parameter('acceleration_scaling', 0.8).value
        # Subscribe to GUI commands
        self.create_subscription(Pose, '/gui_cartesian_command', self._cartesian_callback, 1)
        self.create_subscription(Float64MultiArray, '/gui_joint_command', self._joint_callback, 1)
        self.create_subscription(PoseArray, '/gui_cartesian_path', self._cartesian_path_callback, 1)

        # Subscribe to motion sequence commands (for smooth jogging with blend radii)
        from moveit_msgs.msg import MotionSequenceRequest
        self.create_subscription(MotionSequenceRequest, '/gui_motion_sequence', self._motion_sequence_callback, 1)

        # Subscribe to speed updates from GUI
        from std_msgs.msg import Float32MultiArray
        self.create_subscription(Float32MultiArray, '/gui_speed_scaling', self._speed_callback, 10)

        # Subscribe to digital I/O commands
        from std_msgs.msg import Int32MultiArray
        self.create_subscription(Int32MultiArray, '/gui_digital_output', self._digital_output_callback, 10)

        # Publisher for command completion feedback
        from std_msgs.msg import Bool
        self.command_complete_pub = self.create_publisher(Bool, '/gui_command_complete', 10)

        self.get_logger().info('GUI Bridge ready - forwarding messages to MoveIt')
        self.get_logger().info(f'Using Pilz LIN planner with TracIK')
        self.get_logger().info(f'Speed: vel={self.velocity_scaling:.2f}, accel={self.acceleration_scaling:.2f}')

    def _speed_callback(self, msg: 'Float32MultiArray'):
        """Update velocity/acceleration scaling from GUI"""
        if len(msg.data) >= 2:
            self.velocity_scaling = max(0.01, min(1.0, msg.data[0]))
            self.acceleration_scaling = max(0.01, min(1.0, msg.data[1]))
            self.get_logger().info(f'Speed updated: vel={self.velocity_scaling:.2f}, accel={self.acceleration_scaling:.2f}')

    def _digital_output_callback(self, msg):
        """
        Handle digital output commands.
        Message format: [port_id, value]

        Note: This is a placeholder. In a real system, this would interface with
        the robot's I/O controller (e.g., via ros2_control or custom driver).
        """
        if len(msg.data) >= 2:
            port_id = int(msg.data[0])
            value = int(msg.data[1])
            self.get_logger().info(f'Digital Output: port={port_id}, value={value}')
            # TODO: Forward to actual I/O controller
            # For now, just log the command

    def _cartesian_callback(self, msg: Pose):
        with self._execution_lock:
            if self._executing:
                # Queue the command instead of rejecting it
                with self._queue_lock:
                    self._command_queue.append(('pose', msg))
                    self.get_logger().info(f'Command queued (queue size: {len(self._command_queue)})')
                return
            self._executing = True

        x, y, z = msg.position.x, msg.position.y, msg.position.z
        max_reach = 0.7
        xy_distance = (x**2 + y**2)**0.5

        if xy_distance > max_reach:
            self.get_logger().error(f'❌ Pose UNREACHABLE: XY distance {xy_distance*1000:.0f}mm exceeds max reach {max_reach*1000:.0f}mm')
            with self._execution_lock:
                self._executing = False
            self._process_queue()
            return

        self.get_logger().info(f'Pose goal: X={x:.3f} Y={y:.3f} Z={z:.3f} (reach: {xy_distance*1000:.0f}mm)')
        self._send_pose_goal(msg)
    def _joint_callback(self, msg: Float64MultiArray):
        self.get_logger().warn('Joint commands are deprecated - use pose or path commands only')
        return

    def _cartesian_path_callback(self, msg: PoseArray):
        """Handle multi-waypoint Cartesian path (1000-2000 points)"""
        with self._execution_lock:
            if self._executing:
                with self._queue_lock:
                    self._command_queue.append(('path', msg.poses))
                    self.get_logger().info(f'Cartesian path queued: {len(msg.poses)} waypoints (queue size: {len(self._command_queue)})')
                return
            self._executing = True

        self.get_logger().info(f'Cartesian path: {len(msg.poses)} waypoints - computing smooth trajectory...')
        self._send_cartesian_path(msg.poses)

    def _motion_sequence_callback(self, msg: 'MotionSequenceRequest'):
        """Handle PILZ motion sequence with blend radii for smooth jogging"""
        with self._execution_lock:
            if self._executing:
                with self._queue_lock:
                    self._command_queue.append(('sequence', msg))
                    self.get_logger().info(f'Motion sequence queued: {len(msg.items)} items (queue size: {len(self._command_queue)})')
                return
            self._executing = True

        self.get_logger().info(f'Motion sequence: {len(msg.items)} segments - planning smooth blended trajectory...')
        self._send_motion_sequence(msg)

    def _send_pose_goal(self, pose: Pose):
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'manipulator'

        goal.request.planner_id = 'LIN'
        goal.request.pipeline_id = 'pilz_industrial_motion_planner'

        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 2.0

        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling

        goal.request.workspace_parameters.header.frame_id = 'base_link'
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = 0.010  # Changed from -0.1 to 0.010 (10mm min Z to prevent going below table)
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'tcp'
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.003]
        bounding_volume.primitives.append(sphere)
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        bounding_volume.primitive_poses.append(pose_stamped.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = 'base_link'
        orient_constraint.link_name = 'tcp'
        orient_constraint.orientation = pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0

        # FIX: Use actual velocity scaling instead of 0.0
        goal.request.max_cartesian_speed = max(0.01, self.velocity_scaling)  # Changed from 0.0


        future = self._move_group_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _send_cartesian_path(self, waypoints):
        """Execute Cartesian path - uses direct IK for large paths, service for small paths"""
        num_waypoints = len(waypoints)

        # For large paths (>2000 waypoints), use direct IK approach (much faster)
        if num_waypoints > 2000:
            self.get_logger().info(f'Large path ({num_waypoints} waypoints) - using direct IK execution')
            self._execute_cartesian_direct(waypoints)
        else:
            # For smaller paths, use MoveIt's Cartesian path service (more accurate)
            self.get_logger().info(f'Standard path ({num_waypoints} waypoints) - using Cartesian path service')
            self._execute_cartesian_service(waypoints)

    def _execute_cartesian_direct(self, waypoints):
        """Fast execution for large waypoint counts - compute IK directly without service"""
        from moveit_msgs.srv import GetPositionIK
        from moveit_msgs.msg import RobotTrajectory, RobotState, PositionIKRequest
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        # Create IK service client if not exists
        if not hasattr(self, '_ik_client'):
            self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Sample waypoints to reduce IK calls (every 10th point for very large paths)
        sample_rate = max(1, len(waypoints) // 500)  # Max 500 IK computations
        sampled_waypoints = waypoints[::sample_rate]

        if sample_rate > 1:
            # Always include last waypoint
            if waypoints[-1] not in sampled_waypoints:
                sampled_waypoints.append(waypoints[-1])
            self.get_logger().info(f'Sampled {len(waypoints)} → {len(sampled_waypoints)} waypoints (every {sample_rate}th point)')

        # Compute IK for all sampled waypoints
        trajectory = JointTrajectory()
        trajectory.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

        time_from_start = 0.0
        prev_positions = None

        for i, pose in enumerate(sampled_waypoints):
            # Create IK request
            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = 'manipulator'
            ik_req.ik_request.pose_stamped.header.frame_id = 'base_link'
            ik_req.ik_request.pose_stamped.pose = pose
            ik_req.ik_request.avoid_collisions = True
            ik_req.ik_request.timeout.sec = 0
            ik_req.ik_request.timeout.nanosec = 10000000  # 10ms timeout

            # Call IK synchronously
            if not self._ik_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().error('IK service not available')
                with self._execution_lock:
                    self._executing = False
                self._process_queue()
                return

            try:
                future = self._ik_client.call_async(ik_req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

                if future.result() and future.result().error_code.val == 1:
                    # IK succeeded
                    joint_state = future.result().solution.joint_state
                    positions = list(joint_state.position[:6])

                    # Calculate time based on distance from previous point
                    if prev_positions:
                        # Estimate time based on joint motion
                        max_joint_diff = max(abs(positions[j] - prev_positions[j]) for j in range(6))
                        # Time = distance / (max_velocity * scaling)
                        time_increment = max_joint_diff / (2.0 * self.velocity_scaling)  # rad / (rad/s)
                        time_from_start += max(time_increment, 0.05)  # Min 50ms between points
                    else:
                        time_from_start = 0.1  # First point

                    # Add trajectory point
                    point = JointTrajectoryPoint()
                    point.positions = positions
                    point.time_from_start = Duration(sec=int(time_from_start),
                                                    nanosec=int((time_from_start % 1.0) * 1e9))
                    trajectory.points.append(point)
                    prev_positions = positions
                else:
                    self.get_logger().warn(f'IK failed for waypoint {i}/{len(sampled_waypoints)}')
            except Exception as e:
                self.get_logger().error(f'IK computation failed: {e}')

        if len(trajectory.points) < 2:
            self.get_logger().error('Too few IK solutions - path execution failed')
            with self._execution_lock:
                self._executing = False
            self._process_queue()
            return

        self.get_logger().info(f'Computed {len(trajectory.points)} trajectory points from {len(sampled_waypoints)} waypoints')

        # Create RobotTrajectory and execute
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = trajectory
        self._execute_trajectory(robot_traj)

    def _execute_cartesian_service(self, waypoints):
        """Use MoveIt Cartesian path service for smaller, more precise paths"""
        from moveit_msgs.msg import RobotState
        from std_msgs.msg import Header

        num_waypoints = len(waypoints)

        # Adaptive parameters based on waypoint count
        if num_waypoints > 1000:
            max_step = 0.003  # 3mm
        elif num_waypoints > 500:
            max_step = 0.002  # 2mm
        else:
            max_step = 0.001  # 1mm - best quality

        self.get_logger().info(f'Using max_step={max_step*1000:.1f}mm for {num_waypoints} waypoints')

        # Build the request
        req = GetCartesianPath.Request()
        req.header = Header()
        req.header.frame_id = 'base_link'
        req.group_name = 'manipulator'
        req.start_state = RobotState()
        req.waypoints = list(waypoints)
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.link_name = 'tcp'

        # FIX: Add velocity and acceleration scaling to path computation
        req.max_velocity_scaling_factor = max(0.01, self.velocity_scaling)
        req.max_acceleration_scaling_factor = max(0.01, self.acceleration_scaling)

        # Call service
        if not self._cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Cartesian path service not available')
            with self._execution_lock:
                self._executing = False
            self._process_queue()
            return

        future = self._cartesian_path_client.call_async(req)
        future.add_done_callback(lambda f: self._cartesian_path_response(f, num_waypoints))

    def _cartesian_path_response(self, future, num_waypoints):
        """Handle Cartesian path computation result and execute trajectory"""
        try:
            response = future.result()

            # Check if path was successfully computed
            if response.fraction < 0.99:
                self.get_logger().error(
                    f'Cartesian path failed: only {response.fraction*100:.1f}% achievable'
                )
                with self._execution_lock:
                    self._executing = False
                self._process_queue()
                return

            self.get_logger().info(
                f'✓ Cartesian path computed: {len(response.solution.joint_trajectory.points)} '
                f'trajectory points for {num_waypoints} waypoints'
            )

            # Execute the computed trajectory
            self._execute_trajectory(response.solution)

        except Exception as e:
            self.get_logger().error(f'Cartesian path execution failed: {e}')
            with self._execution_lock:
                self._executing = False
            self._process_queue()

    def _execute_trajectory(self, trajectory):
        """Execute a pre-computed trajectory"""
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient as AC

        # Create execute trajectory action client if not exists
        if not hasattr(self, '_execute_traj_client'):
            self._execute_traj_client = AC(self, ExecuteTrajectory, '/execute_trajectory')
            self._execute_traj_client.wait_for_server()

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        # Velocity/acceleration scaling is now properly handled during path computation
        # No need for post-scaling which can cause timing issues and collisions

        future = self._execute_traj_client.send_goal_async(goal)
        future.add_done_callback(self._execute_traj_response)

    def _execute_traj_response(self, future):
        """Handle trajectory execution goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution rejected')

            from std_msgs.msg import Bool
            complete_msg = Bool()
            complete_msg.data = False
            self.command_complete_pub.publish(complete_msg)

            with self._execution_lock:
                self._executing = False
            self._process_queue()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._execute_traj_result)

    def _execute_traj_result(self, future):
        """Handle trajectory execution completion"""
        result = future.result().result

        from std_msgs.msg import Bool
        complete_msg = Bool()
        complete_msg.data = (result.error_code.val == 1)
        self.command_complete_pub.publish(complete_msg)

        if result.error_code.val == 1:
            self.get_logger().info('✓ Cartesian path executed successfully')
        else:
            self.get_logger().error(f'✗ Cartesian path execution failed: {result.error_code.val}')

        with self._execution_lock:
            self._executing = False

        self._process_queue()

    def _send_motion_sequence(self, sequence_msg: 'MotionSequenceRequest'):
        """
        Execute PILZ motion sequence with blend radii.
        Uses MoveIt's sequence capability for smooth multi-segment motion.
        """
        from moveit_msgs.action import MoveGroupSequence

        # Create sequence action client if not exists
        if not hasattr(self, '_sequence_client'):
            self._sequence_client = ActionClient(self, MoveGroupSequence, '/sequence_move_group')
            if not self._sequence_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('MoveGroupSequence action server not available')
                with self._execution_lock:
                    self._executing = False
                self._process_queue()
                return

        # Build sequence goal
        goal = MoveGroupSequence.Goal()
        goal.request = sequence_msg

        # Apply velocity/acceleration scaling to all items
        for item in goal.request.items:
            item.req.max_velocity_scaling_factor = self.velocity_scaling
            item.req.max_acceleration_scaling_factor = self.acceleration_scaling
            item.req.planner_id = 'LIN'
            item.req.pipeline_id = 'pilz_industrial_motion_planner'
            item.req.group_name = 'manipulator'

        self.get_logger().info(
            f'Sending motion sequence: {len(goal.request.items)} segments, '
            f'vel={self.velocity_scaling:.2f}, acc={self.acceleration_scaling:.2f}'
        )

        # Send goal
        future = self._sequence_client.send_goal_async(goal)
        future.add_done_callback(self._sequence_goal_response)

    def _sequence_goal_response(self, future):
        """Handle sequence goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Motion sequence rejected')

            from std_msgs.msg import Bool
            complete_msg = Bool()
            complete_msg.data = False
            self.command_complete_pub.publish(complete_msg)

            with self._execution_lock:
                self._executing = False
            self._process_queue()
            return

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._sequence_result)

    def _sequence_result(self, future):
        """Handle sequence execution completion"""
        result = future.result().result

        from std_msgs.msg import Bool
        complete_msg = Bool()
        complete_msg.data = (result.error_code.val == 1)
        self.command_complete_pub.publish(complete_msg)

        if result.error_code.val == 1:
            self.get_logger().info('✓ Motion sequence executed successfully')
        else:
            self.get_logger().error(f'✗ Motion sequence failed: {result.error_code.val}')

        with self._execution_lock:
            self._executing = False

        self._process_queue()

    def _goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')

            # Publish completion with failure status
            from std_msgs.msg import Bool
            complete_msg = Bool()
            complete_msg.data = False
            self.command_complete_pub.publish(complete_msg)

            with self._execution_lock:
                self._executing = False
            self._process_queue()
            return

        # Keep lock held - don't release until execution completes
        # This prevents "Cannot push a new trajectory" errors in MoveIt
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result)
    def _goal_result(self, future):
        result = future.result().result

        from std_msgs.msg import Bool
        complete_msg = Bool()
        complete_msg.data = (result.error_code.val == 1)
        self.command_complete_pub.publish(complete_msg)

        if result.error_code.val == 1:
            self.get_logger().info('✓ Success')
        else:
            self.get_logger().error(f'✗ Failed: {result.error_code.val}')

        # Release lock after trajectory execution completes
        with self._execution_lock:
            self._executing = False

        # Process next queued command if any
        self._process_queue()

    def _process_queue(self):
        """Process next command from queue if available"""
        with self._queue_lock:
            if not self._command_queue:
                return
            cmd_type, cmd_data = self._command_queue.popleft()

        with self._execution_lock:
            if self._executing:
                # Another command started, re-queue
                with self._queue_lock:
                    self._command_queue.appendleft((cmd_type, cmd_data))
                return
            self._executing = True

        self.get_logger().info(f'Processing queued command ({len(self._command_queue)} remaining)')

        if cmd_type == 'pose':
            self._send_pose_goal(cmd_data)
        elif cmd_type == 'path':
            self._send_cartesian_path(cmd_data)
        elif cmd_type == 'sequence':
            self._send_motion_sequence(cmd_data)
        else:
            self.get_logger().warn(f'Unknown command type: {cmd_type}')
            with self._execution_lock:
                self._executing = False
            self._process_queue()
def main(args=None):
    rclpy.init(args=args)
    node = GUIBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
