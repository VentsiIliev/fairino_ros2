#!/usr/bin/env python3
import time
from datetime import datetime
from collections import deque
from threading import Lock

import numpy as np
import rclpy
from fairino5_v6_moveit2_config.srv import ApplyIPP
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MotionSequenceItem, MotionSequenceRequest
from moveit_msgs.srv import GetCartesianPath, GetMotionSequence
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros


from utils.transformation_utils import TransformationUtils
from utils.work_object import WorkObject
from safety_wall_manager import SafetyWallManager
from enums import RobotAxis,Direction

# ============ Safety Workspace Boundaries ============
# Safety workspace will be extracted from mounting_surface link mesh geometry
# Fallback values if mesh extraction fails
SAFETY_WORKSPACE_FALLBACK = {
    'x_min': -0.37,
    'x_max': 0.5,
    'y_min': -0.8,
    'y_max': 0.6,
    'z_min': 0.0,
    'z_max': 1.1,
}

SAFETY_MARGIN = 0.01

tool_registry = {
    # Format: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    # Position in millimeters, orientation in degrees
    "TOOL_0": [0, 0, 0, 0, 0, 0],
    "TOOL_1": [0.081, -7.250, 0, 0, 0, 0],
}

# Tool ID to name mapping for move commands
tool_id_map = {
    0: "TOOL_0",
    1: "TOOL_1",
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
        # ----- Wrist3 FK -----
        T = np.eye(4)
        T = T @ TransformationUtils.rot_z(q[0])
        T = T @ TransformationUtils.trans(0, 0, 0.152) @ TransformationUtils.rot_x(np.pi / 2) @ TransformationUtils.rot_z(q[1])
        T = T @ TransformationUtils.trans(-0.425, 0, 0) @ TransformationUtils.rot_z(q[2])
        T = T @ TransformationUtils.trans(-0.39501, 0, 0) @ TransformationUtils.rot_z(q[3])
        T = T @ TransformationUtils.trans(0, 0, 0.1021) @ TransformationUtils.rot_x(np.pi / 2) @ TransformationUtils.rot_z(q[4])
        T = T @ TransformationUtils.trans(0, 0, 0.102) @ TransformationUtils.rot_x(-np.pi / 2) @ TransformationUtils.rot_z(q[5])

        # Position and orientation BEFORE TCP
        pos_before = T[:3, 3]
        rot_before = TransformationUtils.matrix_to_euler(T[:3, :3])
        # print(f"[RobotMonitor] BEFORE TCP -> Position: {pos_before}, Orientation: {rot_before}")

        # ----- Apply TCP offset if provided -----
        if tcp_transform is not None:
            T = T @ tcp_transform
        # Position and orientation AFTER TCP
        pos_after = T[:3, 3]
        rot_after = TransformationUtils.matrix_to_euler(T[:3, :3])
        # print(f"[RobotMonitor] AFTER TCP -> Position: {pos_after}, Orientation: {rot_after}")

        # Extract final position and orientation
        pos = T[:3, 3]
        euler_deg = TransformationUtils.matrix_to_euler(T[:3, :3])

        return np.concatenate([pos, euler_deg])



class RobotController(Node):
    def __init__(self):
        import time
        start_time = time.time()

        super().__init__('velocity_monitor')
        self.get_logger().info('[Init] RobotController starting...')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lock = Lock()
        self.execution_lock = Lock()

        self.monitor = None
        self.tcp_loaded = False
        self.T_ee_link = None
        self.T_tool = np.eye(4)
        # Reduced frequency - TCP transform typically available quickly
        self.tcp_load_timer = self.create_timer(1.0, self.load_tcp_transform)

        t1 = time.time()
        workspace = self._extract_workspace_from_urdf()
        self.get_logger().info(f'[Init] Workspace extraction took {time.time()-t1:.2f}s')

        self.safety_manager = SafetyWallManager(
            node=self,
            workspace=workspace,
            margin=SAFETY_MARGIN,
            enabled=True,
            marker_publish_interval=2.0
        )

        # Defer initial safety wall publishing to speed up initialization
        # Will be published on first use or after 1 second
        self._safety_init_timer = self.create_timer(1.0, self._delayed_safety_init)

        # ROS clients
        self.controller_client = ActionClient(self, FollowJointTrajectory, '/fairino5_controller/follow_joint_trajectory')
        self.cart_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ipp_client = self.create_client(ApplyIPP, '/apply_ipp')

        self.prev_cartesian = None
        self.current_joint_state = None  # Store latest joint state for trajectory planning

        # Track active goal handles for motion cancellation
        self.active_execute_send_future = None
        self.active_controller_goal = None
        self.is_executing = False

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        from action_msgs.msg import GoalStatusArray
        self.create_subscription(GoalStatusArray, '/fairino5_controller/follow_joint_trajectory/_action/status',
                               self._controller_status_callback, 10)

        self.get_logger().info(f'[Init] RobotController ready ({time.time()-start_time:.2f}s total)')

    def _delayed_safety_init(self):
        """Publish safety walls after a short delay to speed up startup."""
        self.safety_manager.force_update()
        self.get_logger().info('[Init] Safety walls published')

        # Cancel this timer after first execution (one-shot behavior)
        if hasattr(self, '_safety_init_timer'):
            self._safety_init_timer.cancel()
            self.destroy_timer(self._safety_init_timer)

    def _extract_workspace_from_urdf(self, max_retries=3, retry_delay=0.1):
        """Extract workspace boundaries from mounting_surface collision mesh in URDF."""
        import xml.etree.ElementTree as ET
        import os
        from ament_index_python.packages import get_package_share_directory
        import time

        for attempt in range(max_retries):
            try:
                robot_description = None

                try:
                    from rclpy.parameter import Parameter
                    params = self.get_parameters_by_prefix('')
                    if 'robot_description' in params:
                        robot_description = params['robot_description']
                except:
                    pass

                if not robot_description:
                    try:
                        from rcl_interfaces.srv import GetParameters
                        client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
                        if client.wait_for_service(timeout_sec=0.3):
                            request = GetParameters.Request()
                            request.names = ['robot_description']
                            future = client.call_async(request)
                            import rclpy
                            rclpy.spin_until_future_complete(self, future, timeout_sec=0.3)
                            if future.done():
                                response = future.result()
                                if response and len(response.values) > 0:
                                    robot_description = response.values[0].string_value
                    except Exception as e:
                        self.get_logger().debug(f"Could not fetch from robot_state_publisher: {e}")

                if not robot_description:
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        self.get_logger().info("Using fallback workspace (robot_description not available yet)")
                        return SAFETY_WORKSPACE_FALLBACK.copy()

                root = ET.fromstring(robot_description)

                for link in root.findall('.//link[@name="mounting_surface"]'):
                    collision = link.find('collision')
                    if collision is not None:
                        mesh_elem = collision.find('.//mesh')
                        if mesh_elem is not None:
                            mesh_filename = mesh_elem.get('filename', '')

                            if mesh_filename.startswith('package://'):
                                package_path = mesh_filename.replace('package://', '')
                                parts = package_path.split('/', 1)
                                if len(parts) == 2:
                                    package_name, relative_path = parts
                                    try:
                                        package_dir = get_package_share_directory(package_name)
                                        mesh_path = os.path.join(package_dir, relative_path)

                                        # Quick file existence check before loading
                                        if not os.path.exists(mesh_path):
                                            self.get_logger().info(f"Mesh file not found, using fallback workspace")
                                            return SAFETY_WORKSPACE_FALLBACK.copy()

                                        # Load mesh (this can be slow)
                                        import trimesh
                                        mesh = trimesh.load(mesh_path)
                                        bounds = mesh.bounds

                                        workspace = {
                                            'x_min': float(bounds[0][0]),
                                            'x_max': float(bounds[1][0]),
                                            'y_min': float(bounds[0][1]),
                                            'y_max': float(bounds[1][1]),
                                            'z_min': float(bounds[0][2]),
                                            'z_max': SAFETY_WORKSPACE_FALLBACK['z_max'],
                                        }

                                        self.get_logger().info(f"✓ Extracted workspace from mesh")
                                        return workspace
                                    except Exception as e:
                                        self.get_logger().debug(f"Mesh load failed: {e}")

                self.get_logger().info("Using fallback workspace (mounting_surface not in URDF)")
                return SAFETY_WORKSPACE_FALLBACK.copy()

            except Exception as e:
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    self.get_logger().info(f"Using fallback workspace (URDF parsing failed)")

        return SAFETY_WORKSPACE_FALLBACK.copy()

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

    def get_tool_transform(self, tool_id):
        """
        Get tool transform matrix from tool ID.

        Args:
            tool_id (int): Tool ID (0, 1, etc.)

        Returns:
            4x4 tool transform matrix (ee_link → TCP)
        """
        tool_name = tool_id_map.get(tool_id, "TOOL_0")
        if tool_name not in tool_registry:
            self.get_logger().warning(f"Tool {tool_name} not found in registry, using TOOL_0")
            tool_name = "TOOL_0"

        offset = tool_registry[tool_name]
        xyz = offset[:3]
        rpy = offset[3:]

        # Build tool offset transform (ee_link → TCP)
        T_tool = np.eye(4)
        T_tool[:3, :3] = TransformationUtils.euler_to_matrix(rpy)
        T_tool[:3, 3] = np.array(xyz) / 1000.0  # Convert mm to meters

        return T_tool

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
        self.T_tool[:3, :3] = TransformationUtils.euler_to_matrix(rpy)
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

                # Initialize with a composed transform (wrist3 → ee_link → TCP)
                T_tcp_total = self.T_ee_link @ self.T_tool
                self.monitor = RobotMonitor(tcp_transform=T_tcp_total)
                self.tcp_loaded = True

                self.get_logger().info("TCP transform loaded and RobotMonitor initialized")

                # Destroy timer to prevent further callbacks
                if hasattr(self, 'tcp_load_timer'):
                    self.tcp_load_timer.cancel()
                    self.destroy_timer(self.tcp_load_timer)

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
            # Convert TF2 TransformStamped to 4x4 homogeneous matrix
            T = TransformationUtils.tf2_to_transform(trans)
            print(f"[RobotController] Loaded TCP transform:\n{T}")
            return T
        except Exception as e:
            self.get_logger().warning(f"Could not get TCP transform from tf: {e}")
            return np.eye(4)

    def send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, planner_id='LIN', tool_transform=None):
        """
        Send a single-point Cartesian goal using MoveIt's compute_cartesian_path service.
        This provides smooth motion through Pilz planner without MoveGroup action.

        Args:
            x_mm, y_mm, z_mm: TCP target position in mm
            rx, ry, rz: TCP target orientation in degrees
            vel_scale: Velocity scaling (0.0-1.0)
            acc_scale: Acceleration scaling (0.0-1.0)
            planner_id: Ignored (kept for API compatibility, always uses cartesian path)
            tool_transform: Tool offset transform (optional)
        """
        self.safety_manager.force_update()

        if self.is_executing:
            self.get_logger().warning('[MOVE] Previous trajectory still executing, ignoring new goal')
            return

        # Use single-waypoint cartesian path (smoother than MoveGroup action)
        self.send_path_cartesian(
            waypoints_mm=[[x_mm, y_mm, z_mm]],
            rx=rx, ry=ry, rz=rz,
            vel_scaling=vel_scale,
            acc_scaling=acc_scale
        )

    def jog_cartesian(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, vel_scale=0.1, acc_scale=0.1):
        """
        Jog the robot by a small step in BASE frame.
        dx_mm, dy_mm, dz_mm are relative increments.
        """
        self.safety_manager.force_update()

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

        # Pre-validate jog target safety
        is_safe, msg = self.safety_manager.check_position_safety(
            x/1000.0, y/1000.0, z/1000.0  # Convert mm to m
        )
        if not is_safe:
            self.get_logger().error(f'[SAFETY] Jog target rejected: {msg}')
            return

        # Send new TCP position (maintaining current orientation)
        self.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale, acc_scale)

    def send_path_cartesian(self, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
        """Cartesian Path: Uses MoveIt's compute_cartesian_path service with adaptive step sizing.

        Args:
            waypoints_mm: List of TCP waypoints [x_mm, y_mm, z_mm] in millimeters
            rx, ry, rz: TCP orientation in degrees (same for all waypoints)
        """
        self.safety_manager.force_update()
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

        # Create waypoint poses for ee_link (applying inverse of tool transform only)
        # Since MoveIt now plans for ee_link, we only need to remove the tool offset
        waypoints = []
        for i, wp in enumerate(waypoints_mm):
            # Build the desired TCP pose
            tcp_pose = [wp[0], wp[1], wp[2], rx, ry, rz]
            T_tcp_desired = TransformationUtils.pose_to_transform(tcp_pose)

            # Apply inverse of tool transform (ee_link → TCP)
            # to get ee_link pose, which MoveIt will plan for
            # Input: TCP coordinates → Output: ee_link coordinates
            T_ee_link = TransformationUtils.remove_tcp_offset(T_tcp_desired, self.T_tool)

            # Extract ee_link pose
            ee_position = T_ee_link[:3, 3]
            ee_quat = TransformationUtils.matrix_to_quaternion(T_ee_link[:3, :3])

            # Pre-validate waypoint safety
            is_safe, msg = self.safety_manager.check_position_safety(
                ee_position[0], ee_position[1], ee_position[2]
            )
            if not is_safe:
                self.get_logger().error(f'[SAFETY] Waypoint {i+1} rejected: {msg}')
                return  # Reject entire path if any waypoint unsafe
            if "Warning" in msg and i == 0:  # Only warn once
                self.get_logger().warning(f'[SAFETY] {msg}')

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
            max_step = 0.0015  # 1.5 mm
        elif num_waypoints > 5:
            max_step = 0.001  # 1 mm
        else:
            max_step = 0.0008  # 0.8 mm

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

        # ✅ CRITICAL FIX: Set start state to robot's CURRENT position
        # Without this, MoveIt uses an empty/default state causing trajectory mismatch
        if hasattr(self, 'current_joint_state') and self.current_joint_state is not None:
            request.start_state.joint_state = self.current_joint_state
            request.start_state.is_diff = False  # Use as absolute state, not differential
        else:
            self.get_logger().warning('[Cartesian Path] No current joint state available - trajectory may not execute correctly')

        self.get_logger().info(f'[Cartesian Path] Using max_step={max_step * 1000:.1f}mm for {num_waypoints} waypoints')

        self.get_logger().info('[Cartesian Path] Requesting cartesian path computation...')
        future = self.cart_path_client.call_async(request)
        future.add_done_callback(lambda f: self._cartesian_path_response(f, vel_scaling, acc_scaling))

    def send_path_pilz_sequence(self, waypoints_mm, rx, ry, rz, vel_scaling=0.6, acc_scaling=0.4, blend_radius=0.01):
        """Execute smooth Cartesian path using Pilz sequence with blend radius.

        This uses Pilz Industrial Motion Planner's sequence capability to create smooth
        blended motion through waypoints without stopping at each point.

        Args:
            waypoints_mm: List of TCP waypoints [x_mm, y_mm, z_mm] in millimeters
            rx, ry, rz: TCP orientation in degrees (same for all waypoints)
            vel_scaling: Velocity scaling factor (0.0-1.0)
            acc_scaling: Acceleration scaling factor (0.0-1.0)
            blend_radius: Blend radius in meters (default 10mm) - radius for smoothing corners
        """
        from moveit_msgs.srv import GetMotionSequence

        if not hasattr(self, 'sequence_client'):
            self.sequence_client = self.create_client(GetMotionSequence, '/plan_sequence_path')

        if not self.sequence_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('[Pilz Sequence] plan_sequence_path service not available')
            return

        num_waypoints = len(waypoints_mm)
        self.get_logger().info(f'[Pilz Sequence] Planning smooth path through {num_waypoints} waypoints with adaptive blend radius')

        # First pass: Convert all waypoints to ee_link poses and collect them
        ee_poses = []
        for i, wp in enumerate(waypoints_mm):
            # Build the desired TCP pose
            tcp_pose = [wp[0], wp[1], wp[2], rx, ry, rz]
            T_tcp_desired = TransformationUtils.pose_to_transform(tcp_pose)

            # Apply inverse tool transform to get ee_link pose
            T_ee_link = TransformationUtils.remove_tcp_offset(T_tcp_desired, self.T_tool)
            ee_position = T_ee_link[:3, 3]
            ee_quat = TransformationUtils.matrix_to_quaternion(T_ee_link[:3, :3])

            # Safety check
            is_safe, msg = self.safety_manager.check_position_safety(
                ee_position[0], ee_position[1], ee_position[2]
            )
            if not is_safe:
                self.get_logger().error(f'[SAFETY] Waypoint {i+1} rejected: {msg}')
                return

            ee_poses.append((ee_position, ee_quat))

        # Calculate adaptive blend radii based on distance between consecutive waypoints
        blend_radii = []
        for i in range(len(ee_poses) - 1):
            # Distance to next waypoint
            dx = ee_poses[i+1][0][0] - ee_poses[i][0][0]
            dy = ee_poses[i+1][0][1] - ee_poses[i][0][1]
            dz = ee_poses[i+1][0][2] - ee_poses[i][0][2]
            dist = (dx**2 + dy**2 + dz**2) ** 0.5

            # Blend radius = 30% of segment length, clamped to [0.0001m, 0.005m] (0.1mm to 5mm)
            # This ensures no overlap while maintaining smoothness
            adaptive_blend = max(0.0001, min(0.005, dist * 0.3))
            blend_radii.append(adaptive_blend)

        blend_radii.append(0.0)  # Last waypoint has zero blend radius

        # Log blend radius statistics
        if len(blend_radii) > 1:
            valid_radii = [r for r in blend_radii[:-1] if r > 0]
            if valid_radii:
                self.get_logger().info(
                    f'[Pilz Sequence] Adaptive blend radii: min={min(valid_radii)*1000:.2f}mm, '
                    f'max={max(valid_radii)*1000:.2f}mm, avg={sum(valid_radii)/len(valid_radii)*1000:.2f}mm'
                )

        # Second pass: Create sequence items with adaptive blend radii
        sequence_items = []
        for i, (ee_position, ee_quat) in enumerate(ee_poses):
            # Create pose goal
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = 'base_link'
            pose_goal.pose.position.x = ee_position[0]
            pose_goal.pose.position.y = ee_position[1]
            pose_goal.pose.position.z = ee_position[2]
            pose_goal.pose.orientation.x = ee_quat[0]
            pose_goal.pose.orientation.y = ee_quat[1]
            pose_goal.pose.orientation.z = ee_quat[2]
            pose_goal.pose.orientation.w = ee_quat[3]

            # Create motion sequence item
            item = MotionSequenceItem()
            item.req.planner_id = 'LIN'  # Use linear motion
            item.req.group_name = 'fairino5_v6_group'
            item.req.max_velocity_scaling_factor = vel_scaling
            item.req.max_acceleration_scaling_factor = acc_scaling
            item.req.goal_constraints.append(self._create_pose_constraint(pose_goal))

            # Use adaptive blend radius
            item.blend_radius = blend_radii[i]

            sequence_items.append(item)

        # Create sequence request
        request = GetMotionSequence.Request()
        request.request.items = sequence_items

        self.get_logger().info('[Pilz Sequence] Sending sequence request...')
        future = self.sequence_client.call_async(request)
        future.add_done_callback(lambda f: self._pilz_sequence_response(f))

    def _create_pose_constraint(self, pose_stamped):
        """Helper to create MoveIt constraints from PoseStamped"""
        constraints = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_stamped.header
        pos_constraint.link_name = 'ee_link'

        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])  # 1mm tolerance
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(pose_stamped.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose_stamped.header
        orient_constraint.link_name = 'ee_link'
        orient_constraint.orientation = pose_stamped.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.01
        orient_constraint.absolute_y_axis_tolerance = 0.01
        orient_constraint.absolute_z_axis_tolerance = 0.01
        orient_constraint.weight = 1.0

        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)

        return constraints

    def apply_ipp_totg(self, trajectory, vel_scaling=0.6, acc_scaling=0.4, callback=None):
        """Call the IPP service to apply TOTG (ASYNC).

        Args:
            trajectory: RobotTrajectory to time-parameterize
            vel_scaling: Velocity scaling factor [0.0-1.0]
            acc_scaling: Acceleration scaling factor [0.0-1.0]
            callback: Function to call with (trajectory_or_None) when done
        """
        self.get_logger().info('[TOTG] Checking if IPP service is available...')

        if not self.ipp_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('[TOTG] ✗ IPP service /apply_ipp NOT available after 5s!')
            self.get_logger().error('[TOTG]    Is ipp_helper node running? Check: ros2 node list | grep ipp')
            if callback:
                callback(None)
            return

        self.get_logger().info('[TOTG] ✓ IPP service is available')

        request = ApplyIPP.Request()
        request.trajectory = trajectory.joint_trajectory
        request.max_velocity_scaling = float(vel_scaling)
        request.max_acceleration_scaling = float(acc_scaling)

        self.get_logger().info(
            f'[TOTG] Requesting time-optimal parameterization (vel={vel_scaling}, acc={acc_scaling})')

        # Call ASYNC to avoid blocking the executor
        future = self.ipp_client.call_async(request)

        # Handle response when ready
        def handle_ipp_response(fut):
            try:
                response = fut.result()

                if response is None:
                    self.get_logger().error('[TOTG] ✗ Response is None')
                    if callback:
                        callback(None)
                    return

                # ✅ FIX: Extract JointTrajectory from RobotTrajectory response
                if not hasattr(response, 'trajectory'):
                    self.get_logger().error('[TOTG] ✗ Response has no trajectory attribute')
                    if callback:
                        callback(None)
                    return

                # Response is RobotTrajectory, extract joint_trajectory
                if not hasattr(response.trajectory, 'joint_trajectory'):
                    self.get_logger().error('[TOTG] ✗ Response.trajectory has no joint_trajectory attribute')
                    if callback:
                        callback(None)
                    return

                joint_traj = response.trajectory.joint_trajectory
                num_points = len(joint_traj.points)

                if num_points == 0:
                    self.get_logger().error('[TOTG] ✗ Empty trajectory - TOTG failed')
                    if callback:
                        callback(None)
                    return

                self.get_logger().info(
                    f'[TOTG] ✓ Generated {num_points} time-parameterized points')

                # Validate that timestamps are present
                has_timestamps = False
                for pt in joint_traj.points:
                    if pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0:
                        has_timestamps = True
                        break

                if not has_timestamps:
                    self.get_logger().error('[TOTG] ✗ Response has no timestamps - INVALID trajectory')
                    if callback:
                        callback(None)
                    return

                # ✅ Success: Update trajectory with time-parameterized result
                trajectory.joint_trajectory = joint_traj
                if callback:
                    callback(trajectory)

            except Exception as e:
                self.get_logger().error(f'[TOTG] ✗ Service call failed: {e}')
                import traceback
                self.get_logger().error(f'[TOTG] Traceback: {traceback.format_exc()}')
                if callback:
                    callback(None)

        future.add_done_callback(handle_ipp_response)

    def _cartesian_path_response(self, future, vel_scaling, acc_scaling):
        self.safety_manager.force_update()
        try:
            response = future.result()
            fraction = response.fraction
            self.get_logger().info(f'[Cartesian Path] Path computed: {fraction * 100:.1f}% successful')

            if fraction < 0.9:
                self.get_logger().warning(f'[Cartesian Path] Only {fraction * 100:.1f}% of path could be computed')
                return

            # Get the computed trajectory
            trajectory = response.solution

            self.get_logger().info(
                f'[Cartesian Path] Computed trajectory has {len(trajectory.joint_trajectory.points)} points')

            # ✅ CRITICAL: Apply TOTG for time-optimal velocity profile (ASYNC)
            # This ensures smooth continuous motion without stops at waypoints
            def on_totg_done(result_trajectory):
                if result_trajectory is None:
                    self.get_logger().error('[Cartesian Path] ✗ TOTG failed - aborting execution')
                    self.get_logger().error('[Cartesian Path] Cannot execute trajectory without time parameterization')
                    return

                # Send trajectory directly to the controller
                self._send_trajectory_to_controller(result_trajectory.joint_trajectory)

            self.apply_ipp_totg(trajectory, vel_scaling, acc_scaling, callback=on_totg_done)

        except Exception as e:
            self.get_logger().error(f'[Cartesian Path] Service call failed: {e}')

    def _pilz_sequence_response(self, future):
        """Handle Pilz sequence planning response and execute the blended trajectory."""
        self.safety_manager.force_update()
        try:
            response = future.result()

            if response.response.error_code.val != 1:
                self.get_logger().error(f'[Pilz Sequence] Planning failed with error code: {response.response.error_code.val}')
                return

            # Get the planned trajectory
            trajectory = response.response.planned_trajectories[0] if response.response.planned_trajectories else None

            if trajectory is None or len(trajectory.joint_trajectory.points) == 0:
                self.get_logger().error('[Pilz Sequence] No trajectory returned from planner')
                return

            self.get_logger().info(
                f'[Pilz Sequence] Sequence planned successfully: {len(trajectory.joint_trajectory.points)} points')

            # Send blended trajectory directly to controller
            self._send_trajectory_to_controller(trajectory.joint_trajectory)

        except Exception as e:
            self.get_logger().error(f'[Pilz Sequence] Service call failed: {e}')


    def _send_trajectory_to_controller(self, joint_trajectory):
        """Send trajectory DIRECTLY to the low-level controller for maximum smoothness.

        Bypasses MoveIt's ExecuteTrajectory action which can enforce unwanted path constraints.
        The controller's spline interpolation will smoothly blend through all waypoints.
        """
        if not self.execution_lock.acquire(blocking=False):
            self.get_logger().warning('[Controller] Trajectory already executing, ignoring')
            return

        if self.is_executing:
            self.get_logger().warning('[Controller] Previous trajectory still active')
            self.execution_lock.release()
            return

        if not self.controller_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('[Controller] fairino5_controller not available')
            self.execution_lock.release()
            return

        # ✅ CRITICAL VALIDATION: Check for timestamps
        if len(joint_trajectory.points) == 0:
            self.get_logger().error('[Controller] ✗ Empty trajectory - aborting')
            self.execution_lock.release()
            return

        has_valid_timestamps = False
        for pt in joint_trajectory.points:
            if pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0:
                has_valid_timestamps = True
                break

        if not has_valid_timestamps:
            self.get_logger().error('[Controller] ✗ Trajectory has NO timestamps - aborting')
            self.get_logger().error('[Controller] TOTG must have failed. Cannot execute.')
            self.execution_lock.release()
            return

        self.is_executing = True

        # Create controller goal with NO path tolerances
        # Only enforce goal position (final point)
        from control_msgs.msg import JointTolerance

        goal_tolerance = []
        for name in joint_trajectory.joint_names:
            tol = JointTolerance()
            tol.name = name
            tol.position = 0.01  # 0.01 rad final position tolerance
            tol.velocity = 0.0   # Must stop at end
            tol.acceleration = 0.0
            goal_tolerance.append(tol)

        controller_goal = FollowJointTrajectory.Goal()
        controller_goal.trajectory = joint_trajectory
        controller_goal.path_tolerance = []  # EMPTY = smooth blending, no stops
        controller_goal.goal_tolerance = goal_tolerance

        # Calculate trajectory duration and set VERY generous time tolerance
        # This prevents spurious aborts when controller is slightly behind schedule
        if len(joint_trajectory.points) > 0:
            last_point = joint_trajectory.points[-1]
            traj_duration_sec = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            # Tolerance = 2x trajectory duration or 5s minimum
            # This gives controller plenty of room for execution delays
            time_tolerance_sec = max(5.0, traj_duration_sec * 2.0)

            # DEBUG: Log trajectory details
            self.get_logger().info(f'[Controller] Trajectory duration: {traj_duration_sec:.2f}s, timeout: {time_tolerance_sec:.1f}s')
            self.get_logger().info(f'[Controller] First point positions: {[round(p, 3) for p in joint_trajectory.points[0].positions]}')
            self.get_logger().info(f'[Controller] Last point positions: {[round(p, 3) for p in last_point.positions]}')
            self.get_logger().info(f'[Controller] First point time: {joint_trajectory.points[0].time_from_start.sec + joint_trajectory.points[0].time_from_start.nanosec/1e9:.3f}s')
            self.get_logger().info(f'[Controller] Last point time: {traj_duration_sec:.3f}s')
        else:
            time_tolerance_sec = 5.0

        controller_goal.goal_time_tolerance.sec = int(time_tolerance_sec)
        controller_goal.goal_time_tolerance.nanosec = int((time_tolerance_sec % 1.0) * 1e9)

        self.get_logger().info(
            f'[Controller] Sending {len(joint_trajectory.points)} points directly to controller '
            f'(spline interpolation, no path tolerance, goal_time_tolerance={time_tolerance_sec:.1f}s)')

        future = self.controller_client.send_goal_async(controller_goal)
        self.active_execute_send_future = future
        future.add_done_callback(self._controller_goal_response)


    def _controller_goal_response(self, future):
        """Handle controller goal acceptance."""
        self.active_execute_send_future = None
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('[Controller] Trajectory execution rejected by fairino5_controller')
                self.active_controller_goal = None
                self.is_executing = False
                self.execution_lock.release()
                return

            self.get_logger().info('[Controller] Trajectory accepted by fairino5_controller')
            self.active_controller_goal = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._controller_goal_result)
        except Exception as e:
            self.get_logger().error(f'[Controller] Goal response error: {e}')
            self.is_executing = False
            self.execution_lock.release()

    def _controller_goal_result(self, future):
        """Handle controller goal completion."""
        try:
            result = future.result().result
            if result.error_code == 0:
                self.get_logger().info('[Controller] Trajectory execution succeeded!')
            else:
                self.get_logger().error(f'[Controller] Trajectory execution failed with error: {result.error_code}')
        except Exception as e:
            self.get_logger().error(f'[Controller] Result error: {e}')
        finally:
            self.active_controller_goal = None
            if self.execution_lock.locked():
                self.execution_lock.release()



    def _controller_status_callback(self, msg):
        """Monitor controller action status and track active goals."""
        from action_msgs.msg import GoalStatus

        for status in msg.status_list:
            if status.status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                pass
            elif status.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                   GoalStatus.STATUS_CANCELED]:
                if self.active_controller_goal is not None:
                    self.active_controller_goal = None
                self.is_executing = False

    def joint_state_callback(self, msg):
        """Process joint states via RobotMonitor and update UI."""
        with self.lock:

            if self.monitor is None:  # <-- guard added
                return  # TCP transform not loaded yet

            if len(msg.position) < 6:
                return

            # Store current joint state for trajectory planning
            self.current_joint_state = msg

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

    def stop_motion(self):
        """
        Cancel all active motion goals (Controller only).

        Returns:
            bool: True if motion was stopped, False if no active goals
        """
        stopped = False

        # Cancel pending send operation
        if self.active_execute_send_future is not None:
            self.get_logger().info('[STOP] Cancelling pending send operation...')
            try:
                self.active_execute_send_future.cancel()
                self.active_execute_send_future = None
                stopped = True
            except Exception as e:
                self.get_logger().error(f'[STOP] Failed to cancel send future: {e}')

        # Cancel the actual hardware controller goal (fairino5_controller)
        if self.active_controller_goal is not None:
            self.get_logger().info('[STOP] Cancelling active Controller goal (fairino5_controller)...')
            try:
                future = self.active_controller_goal.cancel_goal_async()
                self.active_controller_goal = None
                stopped = True
            except Exception as e:
                self.get_logger().error(f'[STOP] Failed to cancel controller goal: {e}')

        if stopped:
            self.get_logger().warning('[STOP] Robot motion cancelled!')
        else:
            self.get_logger().info('[STOP] No active goals to cancel')

        return stopped

    def is_motion_active(self):
        """Return True if any motion goal is active."""
        return any([
            self.active_controller_goal is not None,
            self.active_execute_send_future is not None
        ])


class FairinoRos2Robot:
    """
    ROS2-based robot controller with interface compatible with FairinoRobot.
    Provides motion control, I/O operations, and coordinate frame management.
    """

    def __init__(self, ip, node=None, workobject=None):
        """
        Initializes the ROS2 robot wrapper.

        Args:
            ip (str): IP address of the robot controller (for compatibility, not used in ROS2)
            node (RobotController): ROS2 node for robot control (optional)
            workobject (WorkObject): Default work object frame (optional)
        """
        self.ip = ip
        self.node = node  # embeds the RobotController node
        self.workobject = workobject  # Default WorkObject frame (user=0)
        self.workobject_registry = {0: workobject}  # Registry of work objects by user ID

    # ---------------- WorkObject Methods ----------------
    def set_workobject(self, workobject, user_id=0):
        """
        Set a WorkObject for the robot (coordinate frame).

        Args:
            workobject (WorkObject): Work object to set
            user_id (int): User frame ID (default 0)
        """
        self.workobject_registry[user_id] = workobject
        if user_id == 0:
            self.workobject = workobject

    def get_workobject(self, user_id=0):
        """
        Get a WorkObject by user ID.

        Args:
            user_id (int): User frame ID

        Returns:
            WorkObject or None
        """
        return self.workobject_registry.get(user_id)

    def apply_workobject(self, pose, user_id=0):
        """
        Apply workobject transform to a pose (from user frame to base frame).

        Args:
            pose: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
            user_id (int): User frame ID (0 = default workobject)

        Returns:
            Transformed pose in base frame
        """
        workobject = self.get_workobject(user_id)
        if workobject is None:
            return pose
        return workobject.apply(pose)

    # ---------------- Movement Methods ----------------
    def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=False):
        """
        Moves the robot in Cartesian space (point-to-point motion).

        Args:
            position (list): Target Cartesian position [x, y, z, rx, ry, rz] in tool frame
            tool (int): Tool frame ID (position is relative to this tool frame)
            user (int): User frame ID (0 = default workobject)
            vel (float): Velocity (percentage 0-100)
            acc (float): Acceleration (percentage 0-100)
            blendR (float): Blend radius (not used in ROS2 implementation)
            blocking (bool): Wait for motion to complete (not used in ROS2 async implementation)

        Returns:
            int: 0 on success, error code otherwise
        """
        self.node.get_logger().info(f"[MOVE_CARTESIAN] Target position: {position} with tool={tool}, user={user}, vel={vel}, acc={acc}, blendR={blendR}")
        if len(position) != 6:
            return -1  # Invalid position format

        try:
            # Transform position from user frame to base frame
            position_base = self.apply_workobject(position, user_id=user)

            # Get tool transform for the specified tool ID
            tool_transform = self.node.get_tool_transform(tool)

            vel_scale = max(0.0, min(1.0, vel / 100.0))
            acc_scale = max(0.0, min(1.0, acc / 100.0))
            x, y, z, rx, ry, rz = position_base
            self.node.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale=vel_scale, acc_scale=acc_scale, planner_id='PTP', tool_transform=tool_transform)
            if blocking:
                self.node.get_logger().info(f"[MOVE_LINER] Blocking until position reached: {position_base}")
                self.wait_for_position(position, threshold=1.0, timeout=60.0)
            return 0  # Success
        except Exception as e:
            print(f"move_cartesian error: {e}")
            return -1  # Error

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True):
        """
        Executes a linear movement with blending.

        Args:
            position (list): Target position [x, y, z, rx, ry, rz] in tool frame
            tool (int): Tool frame ID (position is relative to this tool frame)
            user (int): User frame ID (0 = default workobject)
            vel (float): Velocity (percentage 0-100)
            acc (float): Acceleration (percentage 0-100)
            blendR (float): Blend radius (not used in ROS2 implementation)
            blocking (bool): Wait for motion to complete (not used in ROS2 async implementation)

        Returns:
            int: 0 on success, error code otherwise
        """
        if len(position) != 6:
            return -1  # Invalid position format

        try:
            # Transform position from user frame to base frame
            position_base = self.apply_workobject(position, user_id=user)

            # Get tool transform for the specified tool ID
            tool_transform = self.node.get_tool_transform(tool)

            vel_scale = max(0.0, min(1.0, vel / 100.0))
            acc_scale = max(0.0, min(1.0, acc / 100.0))
            x, y, z, rx, ry, rz = position_base
            self.node.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale=vel_scale, acc_scale=acc_scale, planner_id='LIN', tool_transform=tool_transform)

            if blocking:
                self.node.get_logger().info(f"[MOVE_LINER] Blocking until position reached: {position_base}")
                return self.node.wait_for_position(position_base, threshold=1.0, timeout=60.0)

            return 0  # Success
        except Exception as e:
            print(f"move_liner error: {e}")
            return -1  # Error

    def execute_path(self, path, rx=None, ry=None, rz=None, vel=0.6, acc=0.4, blocking=True):
        """Execute path with automatic selection of best execution strategy.

        Strategy selection based on path density:
        - Dense paths (avg spacing < 2mm): Use compute_cartesian_path (continuous contour)
        - Sparse paths (avg spacing >= 2mm): Always use compute_cartesian_path for consistency

        This ensures smooth motion without stops for all path types.
        """
        if not path or self.node is None:
            return -1

        self.node.get_logger().info(f"[EXECUTE_PATH] Received path with {len(path)} waypoints")

        waypoints_xyz = []

        for wp in path:
            if len(wp) == 3:
                waypoints_xyz.append([wp[0], wp[1], wp[2]])
                # Get current TCP orientation if not provided
                if rx is None or ry is None or rz is None:
                    current_pose = self.get_current_position()
                    if current_pose is not None:
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

        # Transform waypoints from workobject frame to base frame if the workobject is set
        if self.workobject is not None:
            self.node.get_logger().info(f"[EXECUTE_PATH] Transforming waypoints from work object to base frame")
            waypoints_base = []
            for wp_xyz in waypoints_xyz:
                # Combine XYZ with orientation for transformation
                wp_full = [wp_xyz[0], wp_xyz[1], wp_xyz[2], rx, ry, rz]
                # Transform from workobject to base frame using WorkObject.apply()
                wp_base = self.workobject.apply(wp_full)
                waypoints_base.append([wp_base[0], wp_base[1], wp_base[2]])
            waypoints_xyz = waypoints_base
            # Also transform orientation to base frame
            orientation_full = [0, 0, 0, rx, ry, rz]  # dummy position, only orientation matters
            orientation_base = self.workobject.apply(orientation_full)
            rx, ry, rz = orientation_base[3], orientation_base[4], orientation_base[5]

        # Calculate average distance between consecutive waypoints
        if len(waypoints_xyz) > 1:
            total_dist = 0.0
            for i in range(len(waypoints_xyz) - 1):
                dx = waypoints_xyz[i+1][0] - waypoints_xyz[i][0]
                dy = waypoints_xyz[i+1][1] - waypoints_xyz[i][1]
                dz = waypoints_xyz[i+1][2] - waypoints_xyz[i][2]
                total_dist += (dx**2 + dy**2 + dz**2) ** 0.5
            avg_spacing = total_dist / (len(waypoints_xyz) - 1)
        else:
            avg_spacing = 0.0

        self.node.get_logger().info(f"[EXECUTE_PATH] {len(waypoints_xyz)} waypoints, avg spacing: {avg_spacing:.2f}mm")
        self.node.get_logger().info(f"[EXECUTE_PATH] First waypoint: {waypoints_xyz[0]}")
        self.node.get_logger().info(f"[EXECUTE_PATH] Orientation (base frame): RX={rx}° RY={ry}° RZ={rz}°")

        # ✅ ALWAYS use compute_cartesian_path for consistency
        # Controller's spline interpolation + TOTG provides smooth continuous motion
        self.node.get_logger().info("[EXECUTE_PATH] Using MoveIt compute_cartesian_path (continuous trajectory)")
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
            self.node.get_logger().info(f"[EXECUTE_PATH] Blocking until final position reached: {last_waypoint}")
            return self.wait_for_position(last_waypoint, threshold=1.0, timeout=60.0)
        return 0

    # ---------------- Status Methods ----------------
    def get_current_position(self):
        """
        Retrieves the current TCP (tool center point) position.

        Returns:
            list: Current robot TCP pose [x, y, z, rx, ry, rz] or None on error
        """
        if self.node is None:
            return None

        data = self.node.get_latest_data()
        if data is None or 'positions' not in data:
            return None

        try:
            q = data['positions']
            fk = self.node.monitor.compute_fk(q, tcp_transform=self.node.monitor.T_tcp)
            fk[:3] *= 1000.0  # meters -> mm
            pose = fk.tolist()

            # Transform from base to workobject frame if a workobject exists
            if self.workobject is not None:
                pose = self.workobject.apply(pose, inverse=True)

            return pose
        except Exception as e:
            print(f"get_current_position error: {e}")
            return None

    def get_current_velocity(self):
        """
        Retrieves the current Cartesian velocity.

        Returns:
            tuple: Current velocity (vx, vy, vz) or None on error
        """
        if self.node is None:
            return None
        data = self.node.get_latest_data()
        if data is None or 'cart_velocity' not in data:
            return None
        return tuple(data['cart_velocity'].tolist())

    def get_current_acceleration(self):
        """
        Retrieves the current Cartesian acceleration.

        Returns:
            tuple: Current acceleration (ax, ay, az) or None on error
        """
        if self.node is None:
            return None
        data = self.node.get_latest_data()
        if data is None or 'cart_acceleration' not in data:
            return None
        return tuple(data['cart_acceleration'].tolist())

    def wait_for_position(self, target_position, threshold=1.0, timeout=30.0, check_interval=0.01):
        """Internal helper to wait for robot to reach target position.

        Args:
            target_position: Target position in BASE frame [x, y, z, rx, ry, rz] (in mm)
            threshold: Distance threshold in mm
            timeout: Timeout in seconds
            check_interval: Check interval in seconds
        """
        import time, math
        start_time = time.time()
        if len(target_position) >= 3:
            target_xyz = target_position[:3]
        else:
            return False

        while True:
            if time.time() - start_time > timeout:
                return False

            # Get current position in BASE frame (not workobject frame)
            if self.node is None:
                time.sleep(check_interval)
                continue

            data = self.node.get_latest_data()
            if data is None or 'positions' not in data:
                time.sleep(check_interval)
                continue

            try:
                # Get FK in BASE frame (do NOT apply workobject inverse transform)
                q = data['positions']
                fk = self.node.monitor.compute_fk(q, tcp_transform=self.node.monitor.T_tcp)
                fk[:3] *= 1000.0  # meters -> mm
                current_xyz = fk[:3].tolist()

                distance = math.sqrt(sum((current_xyz[i] - target_xyz[i]) ** 2 for i in range(3)))
                self.node.get_logger().info(f"Target position (base): {target_xyz}")
                self.node.get_logger().info(f"Current position (base): {current_xyz}")
                self.node.get_logger().info(f"Distance: {distance}")
                if distance < threshold:
                    return True
            except Exception as e:
                self.node.get_logger().warning(f"wait_for_position error: {e}")

            time.sleep(check_interval)

    # ---------------- Jog / Control / Misc ----------------
    def start_jog(self, axis: RobotAxis, direction: Direction, step, vel, acc):
        """
        Starts jogging the robot in a specified axis and direction.
        """
        if self.node.is_motion_active():
            self.node.get_logger().warn("Cannot start new jog: previous motion still active")
            return 0

        self.node.get_logger().info(
            f"Starting jog: axis={axis}, direction={direction}, step={step}mm, vel={vel}%, acc={acc}%"
        )

        if self.node is None or self.node.prev_cartesian is None:
            return -1

        # Handle enum types
        axis_val = axis.value if hasattr(axis, 'value') else axis
        dir_val = direction.value if hasattr(direction, 'value') else direction

        if axis_val not in [1, 2, 3, 4, 5, 6] or dir_val not in [1, -1]:
            return -1

        # Get current position in workobject frame
        current_pos_wobj = self.get_current_position()
        if current_pos_wobj is None or len(current_pos_wobj) < 6:
            return -1

        x, y, z, rx, ry, rz = current_pos_wobj

        # Map enum value to 0-based index for deltas
        axis_index = axis_val - 1  # X=0, Y=1, Z=2, RX=3, etc.

        # Initialize full 6-element delta array
        deltas = [0.0] * 6
        deltas[axis_index] = step * dir_val

        # Apply delta to current position
        new_pos_wobj = [
            x + deltas[0],
            y + deltas[1],
            z + deltas[2],
            rx + deltas[3],
            ry + deltas[4],
            rz + deltas[5]
        ]

        # Transform to base frame
        new_pos_base = self.apply_workobject(new_pos_wobj)

        # Send command
        vel_scale = max(0.0, min(1.0, vel / 100.0))
        acc_scale = max(0.0, min(1.0, acc / 100.0))

        try:
            x_base, y_base, z_base, rx_base, ry_base, rz_base = new_pos_base
            self.node.send_cartesian_goal(
                x_base, y_base, z_base,
                rx_base, ry_base, rz_base,
                vel_scale=vel_scale, acc_scale=acc_scale,
                planner_id='LIN'
            )
            return 0
        except Exception as e:
            self.node.get_logger().error(f"Jog error: {e}")
            return -1

    def enable(self):
        """
        Enables the robot, allowing motion.
        Note: In ROS2 implementation, robot is always enabled when node is active.
        """
        if self.node is not None:
            self.node.get_logger().info("Robot enable called (ROS2 robot is always enabled)")
        return 0

    def disable(self):
        """
        Disables the robot, preventing motion.
        Note: In ROS2 implementation, use stop_motion() instead.
        """
        if self.node is not None:
            self.node.get_logger().info("Robot disable called (use stop_motion for ROS2)")
        return 0

    def printSdkVersion(self):
        """
        Prints the current SDK version.
        Note: ROS2 implementation uses ROS2 version info.
        """
        version = "ROS2 Fairino Robot Controller v1.0"
        print(version)
        return version

    def setDigitalOutput(self, portId, value):
        """
        Sets a digital output pin on the robot.

        Args:
            portId (int): Output port number
            value (int): Value to set (0 or 1)

        Returns:
            int: 0 on success, -1 on error

        Note: Not implemented in the ROS2 version - requires hardware interface
        """
        print(f"setDigitalOutput: port {portId} -> {value} (not implemented in ROS2)")
        return -1

    def stop_motion(self):
        """
        Stops all current robot motion by cancelling active action goals.

        Returns:
            int: 0 on success, -1 on error
        """
        if self.node is None:
            return -1

        # Cancel all active motion goals
        stopped = self.node.stop_motion()
        return 0 if stopped else -1

    def resetAllErrors(self):
        """
        Resets all current error states on the robot.

        Returns:
            int: 0 on success, -1 on error

        Note: Not applicable in ROS2 version
        """
        print("resetAllErrors called (not applicable in ROS2)")
        return 0

