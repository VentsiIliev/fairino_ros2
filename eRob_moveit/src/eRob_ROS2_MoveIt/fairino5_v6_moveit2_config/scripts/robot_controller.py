#!/usr/bin/env python3
import time
from datetime import datetime
from collections import deque
from threading import Lock

import numpy as np
import rclpy
# TOTG integration: import the ApplyIPP service
from fairino5_v6_moveit2_config.srv import ApplyIPP
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, \
    RobotTrajectory, RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation

tool_registry = {
    # Format: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    # Position in millimeters, orientation in degrees
    "TOOL_0": [0, 0, 0, 0, 0, 0],
    "TOOL_1": [0.081, -7.250, 0, 0, 0, 0],
}

class RobotMonitor:
    """Compute FK, joint/cartesian velocities and accelerations from joint states."""

    def __init__(self, velocity_window_size=5, acceleration_window_size=5,tcp_transform=None):
        self.prev_positions = None
        self.prev_velocities = None
        self.prev_cartesian = None
        self.prev_cart_velocities = None
        self.prev_time = None

        self.velocity_window = deque(maxlen=velocity_window_size)
        self.acceleration_window = deque(maxlen=acceleration_window_size)
        self.cart_velocity_window = deque(maxlen=velocity_window_size)
        self.cart_acceleration_window = deque(maxlen=acceleration_window_size)
        self.latest_data = {
            'positions': np.zeros(6),
            'velocities': np.zeros(6),
            'accelerations': np.zeros(6),
            'vel_magnitude': 0.0,
            'acc_magnitude': 0.0,
            'cartesian': np.zeros(3),
            'cart_velocity': np.zeros(3),
            'cart_acceleration': np.zeros(3),
            'cart_vel_magnitude': 0.0,
            'cart_acc_magnitude': 0.0,
            'efforts': np.zeros(6)
        }
        # TCP offset from wrist3 (4x4 homogeneous transform)
        self.T_tcp = tcp_transform if tcp_transform is not None else np.eye(4)

    def update_joint_state(self, joint_positions, timestamp):
        joint_positions = np.array(joint_positions[:6])

        # Compute delta time
        dt = None
        if self.prev_time is not None:
            dt = (timestamp - self.prev_time).total_seconds()
            if dt <= 0.0:
                dt = None  # avoid division by zero

        # -------------------
        # Joint space updates
        # -------------------
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

        # -------------------
        # Cartesian updates
        # -------------------
        cartesian = self.compute_fk(joint_positions, tcp_transform=self.T_tcp) # 6D [x, y, z, rx, ry, rz]
        cart_vel = np.zeros(3)
        cart_acc = np.zeros(3)
        cart_vel_mag = 0.0
        cart_acc_mag = 0.0

        if cartesian is not None and self.prev_cartesian is not None and dt is not None:
            # Linear velocities only
            cart_vel_linear = (cartesian[:3] - self.prev_cartesian[:3]) / dt * 1000.0  # mm/s
            self.cart_velocity_window.append(cart_vel_linear)
            cart_vel = np.mean(self.cart_velocity_window, axis=0)

            # Linear accelerations
            if self.prev_cart_velocities is not None:
                cart_acc = (cart_vel - self.prev_cart_velocities) / dt
                self.cart_acceleration_window.append(cart_acc)
                cart_acc = np.mean(self.cart_acceleration_window, axis=0)

            cart_vel_mag = np.linalg.norm(cart_vel)
            cart_acc_mag = np.linalg.norm(cart_acc)
        # -------------------
        # Save previous states
        # -------------------
        self.prev_positions = joint_positions
        self.prev_velocities = velocities
        self.prev_cartesian = cartesian
        self.prev_cart_velocities = cart_vel  # linear only
        self.prev_time = timestamp

        # -------------------
        # Store latest data
        # -------------------
        self.latest_data = {
            'positions': joint_positions,
            'velocities': avg_velocities,
            'accelerations': avg_accelerations,
            'vel_magnitude': vel_mag,
            'acc_magnitude': acc_mag,
            'cartesian': cartesian if cartesian is not None else np.zeros(3),
            'cart_velocity': cart_vel,
            'cart_acceleration': cart_acc,
            'cart_vel_magnitude': cart_vel_mag,
            'cart_acc_magnitude': cart_acc_mag,
            'efforts': self.latest_data.get('efforts', np.zeros(6))  # preserve previous efforts safely
        }

        return self.latest_data.copy()

    @staticmethod
    def compute_fk(q, tcp_transform=None):
        """
        Forward Kinematics for 6-DOF robot with optional TCP offset.

        Args:
            q (array-like): 6 joint positions in radians [q1, q2, ..., q6]
            tcp_transform (np.ndarray or None): 4x4 homogeneous matrix from wrist3 to tool center point (TCP)

        Returns:
            np.ndarray: [x, y, z, rx, ry, rz] (position in meters, orientation in degrees)
        """
        from scipy.spatial.transform import Rotation

        def rotZ(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0, 0],
                             [s, c, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])

        def rotX(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0, 0],
                             [0, c, -s, 0],
                             [0, s, c, 0],
                             [0, 0, 0, 1]])

        def trans(x, y, z):
            return np.array([[1, 0, 0, x],
                             [0, 1, 0, y],
                             [0, 0, 1, z],
                             [0, 0, 0, 1]])

        # ----- Wrist3 FK -----
        T = np.eye(4)
        T = T @ rotZ(q[0])
        T = T @ trans(0, 0, 0.152) @ rotX(np.pi / 2) @ rotZ(q[1])
        T = T @ trans(-0.425, 0, 0) @ rotZ(q[2])
        T = T @ trans(-0.39501, 0, 0) @ rotZ(q[3])
        T = T @ trans(0, 0, 0.1021) @ rotX(np.pi / 2) @ rotZ(q[4])
        T = T @ trans(0, 0, 0.102) @ rotX(-np.pi / 2) @ rotZ(q[5])

        # Position and orientation BEFORE TCP
        pos_before = T[:3, 3]
        rot_before = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        # print(f"[RobotMonitor] BEFORE TCP -> Position: {pos_before}, Orientation: {rot_before}")

        # ----- Apply TCP offset if provided -----
        if tcp_transform is not None:
            T = T @ tcp_transform
        # Position and orientation AFTER TCP
        pos_after = T[:3, 3]
        rot_after = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        # print(f"[RobotMonitor] AFTER TCP  -> Position: {pos_after}, Orientation: {rot_after}")

        # Position
        pos = T[:3, 3]

        # Orientation (rotation matrix -> Euler angles in degrees)
        rot_matrix = T[:3, :3]
        r = Rotation.from_matrix(rot_matrix)
        euler_deg = r.as_euler('xyz', degrees=True)

        return np.concatenate([pos, euler_deg])


class WorkObject:
    """
    Represents a work object (coordinate frame) relative to the robot base.
    """
    def __init__(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        # Position in mm
        self.position = np.array([x, y, z], dtype=float)
        # Orientation in degrees
        self.orientation = np.array([rx, ry, rz], dtype=float)
        # Precompute homogeneous transform
        self._compute_transform()

    def _compute_transform(self):
        """
        Compute 4x4 homogeneous transform from base to work object.
        """
        rot = Rotation.from_euler('xyz', self.orientation, degrees=True)
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = self.position / 1000.0  # convert mm to meters
        self.transform = T

    def apply(self, pose, inverse=False):
        """
        Apply work object transform to a Cartesian pose.
        pose: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        inverse: if True, transform from base frame to workobject frame
        Returns new pose
        """
        # Convert pose to homogeneous matrix
        rot = Rotation.from_euler('xyz', pose[3:], degrees=True)
        T_pose = np.eye(4)
        T_pose[:3, :3] = rot.as_matrix()
        T_pose[:3, 3] = np.array(pose[:3]) / 1000.0  # mm -> m

        if inverse:
            T_result = np.linalg.inv(self.transform) @ T_pose
        else:
            T_result = self.transform @ T_pose

        # Extract position
        pos = T_result[:3, 3] * 1000.0  # back to mm

        # Extract orientation
        r = Rotation.from_matrix(T_result[:3, :3])
        euler = r.as_euler('xyz', degrees=True)

        return [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]]


class RobotController(Node):
    def __init__(self):
        super().__init__('velocity_monitor')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lock = Lock()

        self.monitor = None
        self.tcp_loaded = False
        self.T_ee_link = None  # wrist3 → ee_link (from URDF/TF, static)
        self.T_tool = np.eye(4)  # ee_link → TCP (from tool registry, switchable)
        # Timer to attempt loading TCP transform every 0.5 seconds
        self.create_timer(0.5, self.load_tcp_transform)

        # ROS clients
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cart_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ipp_client = self.create_client(ApplyIPP, '/apply_ipp')

        self.prev_cartesian = None

        self.get_logger().info('Waiting for move_group action server...')
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def wait_for_monitor(self, timeout_sec=5.0):
        """Wait until RobotMonitor (TCP) is initialized."""
        import time
        start_time = time.time()
        while self.monitor is None:
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for RobotMonitor (TCP not loaded)")
                return False
            time.sleep(0.1)
        return True

    def set_tool(self, tool_name):
        """
        Switch the active TCP/tool.
        Composes ee_link offset (from URDF) with tool offset (from registry).
        """
        if tool_name not in tool_registry:
            self.get_logger().warning(f"Tool {tool_name} not found in registry")
            return

        offset = tool_registry[tool_name]
        xyz = offset[:3]
        rpy = offset[3:]

        # Build tool offset transform (ee_link → TCP)
        self.T_tool = np.eye(4)
        self.T_tool[:3, :3] = Rotation.from_euler('xyz', rpy, degrees=True).as_matrix()
        self.T_tool[:3, 3] = np.array(xyz) / 1000.0  # Convert mm to meters

        # Compose: wrist3 → ee_link → TCP
        if self.monitor is not None and self.T_ee_link is not None:
            self.monitor.T_tcp = self.T_ee_link @ self.T_tool
            self.get_logger().info(f"Switched active tool to {tool_name}")
        else:
            self.get_logger().warning("RobotMonitor not initialized yet")


    def load_tcp_transform(self):
        if self.tcp_loaded:
            return  # Already loaded

        if self.tf_buffer.can_transform('wrist3_link', 'ee_link', rclpy.time.Time()):
            try:
                # Load ee_link offset from URDF/TF (wrist3 → ee_link)
                self.T_ee_link = self.get_tcp_transform('wrist3_link', 'ee_link')
                print(f"[RobotController] Loaded ee_link transform:\n{self.T_ee_link}")

                # Initialize with composed transform (wrist3 → ee_link → TCP)
                T_tcp_total = self.T_ee_link @ self.T_tool
                self.monitor = RobotMonitor(tcp_transform=T_tcp_total)
                self.tcp_loaded = True

                self.get_logger().info("TCP transform loaded and RobotMonitor initialized")
            except Exception as e:
                self.get_logger().warning(f"TCP transform lookup failed: {e}")


    def get_tcp_transform(self, from_frame='wrist3_link', to_frame='ee_link'):
        """Get 4x4 TCP transform from tf2 (base->tcp)."""
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                from_frame,  # target_frame
                to_frame,  # source_frame
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Build 4x4 homogeneous matrix
            t = trans.transform.translation
            q = trans.transform.rotation
            T = np.eye(4)
            T[:3, 3] = [t.x, t.y, t.z]
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            print(f"[RobotController] Loaded TCP transform:\n{T}")

            return T
        except Exception as e:
            self.get_logger().warning(f"Could not get TCP transform from tf: {e}")
            return np.eye(4)

    def send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, planner_id='LIN'):
        """Send a single Cartesian goal to the robot via MoveGroup action.

        Args:
            x_mm, y_mm, z_mm: Desired TCP position in millimeters
            rx, ry, rz: Desired TCP orientation in degrees
            planner_id: 'LIN' for linear motion (Cartesian straight line),
                       'PTP' for point-to-point motion (joint space)
        """

        # Wait for MoveGroup action server to be available
        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Move group action server not available')
            return

        # Convert millimeters to meters, as MoveIt works in meters
        x, y, z = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0

        # Build desired TCP pose as homogeneous transform
        T_tcp_desired = np.eye(4)
        T_tcp_desired[:3, :3] = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        T_tcp_desired[:3, 3] = [x, y, z]

        # Apply inverse tool transform to get ee_link pose
        # TCP_pose = ee_link_pose * T_tool  =>  ee_link_pose = TCP_pose * inv(T_tool)
        # MoveIt now plans for ee_link (per SRDF), so we only remove tool offset
        T_ee_link = T_tcp_desired @ np.linalg.inv(self.T_tool)

        # Extract position and orientation for ee_link
        ee_position = T_ee_link[:3, 3]
        ee_rotation = Rotation.from_matrix(T_ee_link[:3, :3])
        ee_quat = ee_rotation.as_quat(canonical=False)

        # Create a new MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()  # Request object holds all motion planning parameters

        # Specify which MoveIt group (robot arm) to move
        goal.request.group_name = 'fairino5_v6_group'

        # Planner settings
        goal.request.planner_id = planner_id  # 'LIN' for linear, 'PTP' for point-to-point
        goal.request.pipeline_id = 'pilz_industrial_motion_planner'  # Planning pipeline
        goal.request.num_planning_attempts = 1  # Number of attempts to plan
        goal.request.allowed_planning_time = 5.0  # Max time allowed for planning (seconds)

        # Scaling factors for velocity and acceleration
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale

        # Workspace bounds (defines the volume where the robot is allowed to plan)
        goal.request.workspace_parameters.header.frame_id = 'base_link'
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = 0.01
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0

        # Define target pose for ee_link
        pose = Pose()
        pose.position.x = ee_position[0]
        pose.position.y = ee_position[1]
        pose.position.z = ee_position[2]
        pose.orientation.x = ee_quat[0]
        pose.orientation.y = ee_quat[1]
        pose.orientation.z = ee_quat[2]
        pose.orientation.w = ee_quat[3]

        # --- Position Constraints ---
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'  # reference frame for constraint
        pos_constraint.link_name = 'ee_link'  # link that should satisfy the constraint

        # Tolerance region: a small sphere around the target pose
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.003])  # 3 mm radius
        # This sphere tells MoveIt: the robot must reach any point inside this 3mm sphere.
        # Smaller radius = more precise, larger radius = easier to plan.

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(sphere)  # attach sphere shape
        bounding_volume.primitive_poses.append(pose)  # attach sphere at the target pose
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0  # importance of this constraint

        # --- Orientation Constraints ---
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = 'base_link'  # reference frame
        orient_constraint.link_name = 'ee_link'  # link to orient
        orient_constraint.orientation = pose.orientation  # target orientation

        # Allowed deviation from target orientation (radians)
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0  # importance of orientation constraint

        # Combine position and orientation constraints
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal.request.goal_constraints.append(constraints)

        # --- Planning options ---
        goal.planning_options.plan_only = False  # execute path, not just plan
        goal.planning_options.replan = False  # do not replan if failed
        goal.planning_options.replan_attempts = 0  # number of replans
        goal.request.max_cartesian_speed = 0.5  # max Cartesian speed (m/s)

        # Log the goal for debugging
        self.get_logger().info(
            f'Sending Cartesian goal: X={x_mm}mm Y={y_mm}mm Z={z_mm}mm RX={rx}° RY={ry}° RZ={rz}°'
        )
        self.get_logger().info(f"[DEBUG] T_tcp_desired Z = {z_mm}")
        self.get_logger().info(f"[DEBUG] T_tool offset Z = {self.T_tool[2, 3] * 1000:.1f} mm")
        self.get_logger().info(f"[DEBUG] ee_link Z after inv(T_tool) = {T_ee_link[2, 3] * 1000:.1f} mm")

        # Send the goal asynchronously
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)  # handle response when done

    def _goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result)

    def _get_result(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code.val}')

    def jog_cartesian(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, vel_scale=0.1, acc_scale=0.1):
        """
        Jog the robot by a small step in BASE frame.
        dx_mm, dy_mm, dz_mm are relative increments.
        """

        # We need a valid current Cartesian position
        if self.prev_cartesian is None:
            self.get_logger().warning('No Cartesian position available for jog')
            return

        # Current TCP position (from FK with full T_tcp)
        x = self.prev_cartesian[0] * 1000.0
        y = self.prev_cartesian[1] * 1000.0
        z = self.prev_cartesian[2] * 1000.0
        rx = self.prev_cartesian[3]
        ry = self.prev_cartesian[4]
        rz = self.prev_cartesian[5]

        # Apply jog step (in base frame)
        x += dx_mm
        y += dy_mm
        z += dz_mm

        # Send new TCP position (maintaining current orientation)
        self.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale, acc_scale)

    def send_path_cartesian(self, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
        """Cartesian Path: Uses MoveIt's compute_cartesian_path service with adaptive step sizing.

        Args:
            waypoints_mm: List of TCP waypoints [x_mm, y_mm, z_mm] in millimeters
            rx, ry, rz: TCP orientation in degrees (same for all waypoints)
        """
        num_waypoints = len(waypoints_mm)
        self.get_logger().info(f'[Cartesian Path] Computing smooth path through {num_waypoints} waypoints')
        self.get_logger().info(f'[Cartesian Path] vel= {vel_scaling}, acc= {acc_scaling}')

        # Debug: Log first TCP waypoint
        if num_waypoints > 0:
            self.get_logger().info(f'[Cartesian Path] First TCP waypoint: X={waypoints_mm[0][0]:.1f}mm Y={waypoints_mm[0][1]:.1f}mm Z={waypoints_mm[0][2]:.1f}mm')
            self.get_logger().info(f'[Cartesian Path] TCP orientation: RX={rx}° RY={ry}° RZ={rz}°')
            self.get_logger().info(f'[Cartesian Path] T_tool offset from ee_link: {self.T_tool[2,3]*1000:.1f}mm in Z')

        if not self.cart_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('[Cartesian Path] compute_cartesian_path service not available')
            return

        # Build TCP orientation as rotation matrix
        tcp_rotation = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True)

        # Create waypoint poses for ee_link (applying inverse of tool transform only)
        # Since MoveIt now plans for ee_link, we only need to remove the tool offset
        waypoints = []
        for i, wp in enumerate(waypoints_mm):
            # Build the desired TCP pose
            T_tcp_desired = np.eye(4)
            T_tcp_desired[:3, :3] = tcp_rotation.as_matrix()
            T_tcp_desired[:3, 3] = [wp[0] / 1000.0, wp[1] / 1000.0, wp[2] / 1000.0]

            # Apply inverse of tool transform (ee_link → TCP)
            # to get ee_link pose, which MoveIt will plan for
            # Input: TCP coordinates → Output: ee_link coordinates
            T_ee_link = T_tcp_desired @ np.linalg.inv(self.T_tool)

            # Extract ee_link pose
            ee_position = T_ee_link[:3, 3]
            ee_rotation = Rotation.from_matrix(T_ee_link[:3, :3])
            ee_quat = ee_rotation.as_quat(canonical=False)

            # Debug: Log first waypoint transformation
            if i == 0:
                self.get_logger().info(f'[Cartesian Path] After inv(T_tool): ee_link Z={ee_position[2]*1000:.1f}mm')

            pose = Pose()
            pose.position.x = ee_position[0]
            pose.position.y = ee_position[1]
            pose.position.z = ee_position[2]
            pose.orientation.x = ee_quat[0]
            pose.orientation.y = ee_quat[1]
            pose.orientation.z = ee_quat[2]
            pose.orientation.w = ee_quat[3]
            waypoints.append(pose)

        # Adaptive step size based on path complexity
        # Finer steps for larger paths ensure smoother trajectories
        if num_waypoints > 10:
            max_step = 0.003  # 3mm for complex paths
        elif num_waypoints > 5:
            max_step = 0.002  # 2mm for medium paths
        else:
            max_step = 0.001  # 1mm for simple paths - best quality

        self.get_logger().info(f'[Cartesian Path] Waypoints prepared {waypoints}')

        # Create a service request
        request = GetCartesianPath.Request()
        request.header.frame_id = 'base_link'
        request.group_name = 'fairino5_v6_group'
        request.link_name = 'ee_link'  # Explicitly set link_name to match SRDF tip_link
        request.waypoints = waypoints
        request.max_step = max_step
        request.jump_threshold = 0.0  # Disabled - no jump checking
        request.avoid_collisions = True
        # Set velocity and acceleration scaling factors for TOTG
        request.max_velocity_scaling_factor = vel_scaling
        request.max_acceleration_scaling_factor = acc_scaling

        self.get_logger().info(f'[Cartesian Path] Using max_step={max_step * 1000:.1f}mm for {num_waypoints} waypoints')

        self.get_logger().info('[Cartesian Path] Requesting cartesian path computation...')
        future = self.cart_path_client.call_async(request)
        future.add_done_callback(lambda f: self._cartesian_path_response(f, vel_scaling, acc_scaling))

    def apply_ipp_totg(self, trajectory, vel_scaling=0.6, acc_scaling=0.4):
        """Call the IPP service to apply TOTG (Time Optimal Trajectory Generation)."""
        if not self.ipp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('[TOTG] IPP service not available, using original trajectory')
            return trajectory

        request = ApplyIPP.Request()
        request.trajectory = trajectory
        request.max_velocity_scaling = vel_scaling
        request.max_acceleration_scaling = acc_scaling

        self.get_logger().info(
            f'[TOTG] Applying time-optimal parameterization with vel={vel_scaling}, acc={acc_scaling}')

        future = self.ipp_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.get_logger().info('[TOTG] Time-optimal trajectory generated successfully')
            return future.result().trajectory
        else:
            self.get_logger().error('[TOTG] IPP service call failed, using original trajectory')
            return trajectory

    def _cartesian_path_response(self, future, vel_scaling, acc_scaling):
        try:
            response = future.result()
            fraction = response.fraction
            self.get_logger().info(f'[Cartesian Path] Path computed: {fraction * 100:.1f}% successful')

            if fraction < 0.9:
                self.get_logger().warning(f'[Cartesian Path] Only {fraction * 100:.1f}% of path could be computed')
                return

            if not self.execute_trajectory_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error('[Cartesian Path] ExecuteTrajectory action server not available')
                return

            # Get the computed trajectory
            trajectory = response.solution

            self.get_logger().info(
                f'[Cartesian Path] Original trajectory has {len(trajectory.joint_trajectory.points)} points')

            # Apply TOTG via IPP service for time-optimal execution
            trajectory = self.apply_ipp_totg(trajectory, vel_scaling, acc_scaling)

            self.get_logger().info(
                f'[Cartesian Path] Final trajectory has {len(trajectory.joint_trajectory.points)} points')

            # Execute the time-optimized trajectory
            execute_goal = ExecuteTrajectory.Goal()
            execute_goal.trajectory = trajectory

            future = self.execute_trajectory_client.send_goal_async(execute_goal)
            future.add_done_callback(self._execute_trajectory_response)

        except Exception as e:
            self.get_logger().error(f'[Cartesian Path] Service call failed: {e}')

    def _execute_trajectory_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[Cartesian Path] Trajectory execution rejected')
            return
        self.get_logger().info('[Cartesian Path] Trajectory execution accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._execute_trajectory_result)

    def _execute_trajectory_result(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('[Cartesian Path] Trajectory execution succeeded!')
        else:
            self.get_logger().error(
                f'[Cartesian Path] Trajectory execution failed with error code: {result.error_code.val}')

    def joint_state_callback(self, msg):
        """Process joint states via RobotMonitor and update UI."""
        with self.lock:

            if self.monitor is None:  # <-- guard added
                return  # TCP transform not loaded yet

            if len(msg.position) < 6:
                return

            timestamp = datetime.now()
            data = self.monitor.update_joint_state(msg.position, timestamp)

            # Update previous Cartesian for jogging
            self.prev_cartesian = data['cartesian']
            self.latest_data = data  # Store latest data


            # Send data to UI
            # self.ui_callback(data)

    def get_latest_data(self):
        """Return a copy of the latest joint/cartesian data."""
        with self.lock:
            if hasattr(self, 'latest_data'):
                return self.latest_data.copy()
            else:
                # Return empty/default structure if no data yet

                return None

class FairinoRos2Robot:
    def __init__(self, ip, node=None,workobject=None):
        self.ip = ip
        self.node = node  # embeds the RobotController node
        self.workobject = workobject  # WorkObject frame (optional)

    # ---------------- WorkObject Methods ----------------
    def set_workobject(self, workobject):
        """Set a WorkObject for the robot (coordinate frame)."""
        self.workobject = workobject

    def apply_workobject(self, pose):
        """
        Apply the current workobject transform to a pose.
        pose: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        """
        if self.workobject is None:
            return pose
        return self.workobject.apply(pose)

    # ---------------- Movement Methods ----------------
    def move_cartesian(self, position, tool=0, user=0, vel=100, acc=100, blendR=0, blocking=False):
        if len(position) != 6:
            raise ValueError("Position must be a list of 6 values: [x, y, z, rx, ry, rz]")

        # Apply workobject if exists
        position = self.apply_workobject(position)

        vel_scale = max(0.0, min(1.0, vel / 100.0))
        acc_scale = max(0.0, min(1.0, acc / 100.0))
        x, y, z, rx, ry, rz = position
        self.node.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale=vel_scale, acc_scale=acc_scale, planner_id='PTP')

        if blocking:
            return self.wait_for_position(position, threshold=1.0, timeout=30.0)
        return True

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=False):
        if len(position) != 6:
            raise ValueError("Position must be a list of 6 values: [x, y, z, rx, ry, rz]")

        position = self.apply_workobject(position)

        vel_scale = max(0.0, min(1.0, vel / 100.0))
        acc_scale = max(0.0, min(1.0, acc / 100.0))
        x, y, z, rx, ry, rz = position
        self.node.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale=vel_scale, acc_scale=acc_scale, planner_id='LIN')

        if blocking:
            return self.wait_for_position(position, threshold=1.0, timeout=30.0)
        return True

    def execute_path(self, path, rx=None, ry=None, rz=None, vel=0.6, acc=0.4, blocking=False):
        if not path or self.node is None:
            return -1
        self.node.get_logger().info(f"[EXECUTE_PATH] Received path with {len(path)} waypoints")

        waypoints_xyz = []

        for wp in path:
            if len(wp) == 3:
                waypoints_xyz.append([wp[0], wp[1], wp[2]])
                # Get current TCP orientation if not provided
                if rx is None or ry is None or rz is None:
                    result = self.get_current_position()
                    if result[0] == 0 and result[1] is not None:
                        current_pose = result[1]
                        rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
                    else:
                        # Fallback if current position unavailable
                        rx, ry, rz = 180.0, 0.0, 0.0
            elif len(wp) == 6:
                wx, wy, wz, wrx, wry, wrz = wp
                waypoints_xyz.append([wx, wy, wz])
                if rx is None:
                    rx = wrx
                if ry is None:
                    ry = wry
                if rz is None:
                    rz = wrz
            else:
                continue

        if not waypoints_xyz:
            return -1

        # Transform waypoints from workobject frame to base frame if workobject is set
        if self.workobject is not None:
            self.node.get_logger().info(f"[EXECUTE_PATH] Transforming waypoints from workobject to base frame")
            waypoints_base = []
            for wp_xyz in waypoints_xyz:
                # Combine XYZ with orientation for transformation
                wp_full = [wp_xyz[0], wp_xyz[1], wp_xyz[2], rx, ry, rz]
                # Transform from workobject to base frame
                wp_base = self.apply_workobject(wp_full)
                waypoints_base.append([wp_base[0], wp_base[1], wp_base[2]])
            waypoints_xyz = waypoints_base
            # Also transform orientation to base frame
            orientation_full = [0, 0, 0, rx, ry, rz]  # dummy position, only orientation matters
            orientation_base = self.apply_workobject(orientation_full)
            rx, ry, rz = orientation_base[3], orientation_base[4], orientation_base[5]

        self.node.get_logger().info(f"[EXECUTE_PATH] Extracted {len(waypoints_xyz)} XYZ waypoints")
        self.node.get_logger().info(f"[EXECUTE_PATH] First waypoint: {waypoints_xyz[0]}")
        self.node.get_logger().info(f"[EXECUTE_PATH] Orientation (base frame): RX={rx}° RY={ry}° RZ={rz}°")

        self.node.send_path_cartesian(
            waypoints_mm=waypoints_xyz,
            rx=rx,
            ry=ry,
            rz=rz,
            vel_scaling=vel,
            acc_scaling=acc
        )

        if blocking:
            last_waypoint = waypoints_xyz[-1] + [rx, ry, rz]
            return self.wait_for_position(last_waypoint, threshold=1.0, timeout=60.0)
        return 0

    # ---------------- Status Methods ----------------
    def get_current_position(self, frame="base_link"):
        """
        Return the current Cartesian position, optionally transformed to the work object frame.

        Output: (success_code, [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg])
        """
        if self.node is None:
            return -1, None

        data = self.node.get_latest_data()
        if data is None or 'positions' not in data:
            return -1, None

        q = data['positions']
        fk = self.node.monitor.compute_fk(q, tcp_transform=self.node.monitor.T_tcp)
        fk[:3] *= 1000.0  # meters -> mm

        pose = fk.tolist()

        # Apply workobject if it exists
        if self.workobject is not None:
            pose = self.workobject.apply(pose, inverse=True)  # transform from base to workobject frame

        return 0, pose

    def get_current_velocity(self):
        if self.node is None:
            return -1, (0.0, 0.0, 0.0)
        data = self.node.get_latest_data()
        if data is None or 'cart_velocity' not in data:
            return -1, (0.0, 0.0, 0.0)
        return 0, tuple(data['cart_velocity'].tolist())

    def get_current_acceleration(self):
        if self.node is None:
            return -1, (0.0, 0.0, 0.0)
        data = self.node.get_latest_data()
        if data is None or 'cart_acceleration' not in data:
            return -1, (0.0, 0.0, 0.0)
        return 0, tuple(data['cart_acceleration'].tolist())

    def wait_for_position(self, target_position, threshold=1.0, timeout=30.0, check_interval=0.01):
        import time, math
        start_time = time.time()
        if len(target_position) >= 3:
            target_xyz = target_position[:3]
        else:
            return False

        while True:
            if time.time() - start_time > timeout:
                return False
            result = self.get_current_position()
            if result is None or result[0] != 0:
                time.sleep(check_interval)
                continue
            _, current_position = result
            if current_position is None:
                time.sleep(check_interval)
                continue
            current_xyz = current_position[:3]
            distance = math.sqrt(sum((current_xyz[i] - target_xyz[i]) ** 2 for i in range(3)))
            if distance < threshold:
                return True
            time.sleep(check_interval)

    # ---------------- Jog / Control / Misc ----------------
    def start_jog(self, axis, direction, step, vel, acc):
        """
        Jog the robot in the workobject frame.
        axis: 0=X, 1=Y, 2=Z (in workobject frame)
        direction: +1 or -1
        step: distance in mm
        """
        if self.node is None or self.node.prev_cartesian is None:
            return -1
        if axis not in [0, 1, 2] or direction not in [1, -1]:
            return -1

        # Get current position in workobject frame (with tool offset applied)
        result = self.get_current_position()
        if result is None or result[0] != 0:
            return -1
        _, current_pos_wobj = result
        if current_pos_wobj is None or len(current_pos_wobj) < 6:
            return -1

        # Apply delta in workobject frame
        x, y, z, rx, ry, rz = current_pos_wobj
        deltas = [0.0, 0.0, 0.0]
        deltas[axis] = step * direction

        new_pos_wobj = [x + deltas[0], y + deltas[1], z + deltas[2], rx, ry, rz]

        # Transform to base frame (workobject.apply() does this)
        new_pos_base = self.apply_workobject(new_pos_wobj)

        # Send command
        vel_scale = max(0.0, min(1.0, vel / 100.0))
        acc_scale = max(0.0, min(1.0, acc / 100.0))

        try:
            x_base, y_base, z_base, rx_base, ry_base, rz_base = new_pos_base
            self.node.send_cartesian_goal(x_base, y_base, z_base, rx_base, ry_base, rz_base,
                                         vel_scale=vel_scale, acc_scale=acc_scale, planner_id='LIN')
            return 0
        except Exception as e:
            print(f"Jog error: {e}")
            return -1

    def enable(self): pass
    def disable(self): pass
    def printSdkVersion(self): pass
    def setDigitalOutput(self, portId, value): pass
    def stop_motion(self): pass
    def resetAllErrors(self): pass

