#!/usr/bin/env python3
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
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray

from utils.transformation_utils import TransformationUtils
from safety.safety_wall_manager import SafetyWallManager
from status.robot_monitor import RobotMonitor

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
        self.get_logger().info(f'[Init] Workspace extraction took {time.time() - t1:.2f}s')

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
        self.controller_client = ActionClient(self, FollowJointTrajectory,
                                              '/fairino5_controller/follow_joint_trajectory')
        self.cart_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ipp_client = self.create_client(ApplyIPP, '/apply_ipp')

        self.prev_cartesian = None
        self.current_joint_state = None  # Store latest joint state for trajectory planning

        # Track active goal handles for motion cancellation
        self.active_execute_send_future = None
        self.active_controller_goal = None
        self.is_executing = False

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(GoalStatusArray, '/fairino5_controller/follow_joint_trajectory/_action/status',
                                 self._controller_status_callback, 10)

        self.get_logger().info(f'[Init] RobotController ready ({time.time() - start_time:.2f}s total)')

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
                self.monitor = RobotMonitor(ros_node=self, tcp_transform=T_tcp_total)
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

    def send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, planner_id='LIN',
                            tool_transform=None):
        from motion.trajectory_planner import send_cartesian_goal
        send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, planner_id, tool_transform)

    def jog_cartesian(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, vel_scale=0.1, acc_scale=0.1):
        from motion.jog_controller import jog_cartesian
        jog_cartesian(self, dx_mm, dy_mm, dz_mm, vel_scale, acc_scale)

    def send_path_cartesian(self, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
        from motion.trajectory_planner import send_path_cartesian
        send_path_cartesian(self, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling)

    def _controller_status_callback(self, msg):
        """Monitor controller action status and track active goals."""

        for status in msg.status_list:
            if status.status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                pass
            elif status.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                   GoalStatus.STATUS_CANCELED]:
                if self.active_controller_goal is not None:
                    self.active_controller_goal = None
                self.is_executing = False

    def joint_state_callback(self, msg):
        """Process joint states and store for trajectory planning."""
        with self.lock:

            if self.monitor is None:  # <-- guard added
                return  # RobotMonitor not initialized yet

            if len(msg.position) < 6:
                return

            # Store current joint state for trajectory planning
            self.current_joint_state = msg

            # Get latest data from RobotMonitor (updated via topic subscriptions)
            data = self.monitor.get_latest_data()

            if data is not None:
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
