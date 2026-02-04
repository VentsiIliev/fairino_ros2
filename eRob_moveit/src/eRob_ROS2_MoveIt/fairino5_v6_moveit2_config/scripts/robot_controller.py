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
from safety.dynamics_collision_detector import DynamicsCollisionDetector
from status.robot_monitor import RobotMonitor
from status.robot_status_publisher import RobotStatusPublisher
from motion.motion_queue import MotionQueue
from utils.workspace_extractor import _extract_workspace_from_urdf
# ============ Safety Workspace Boundaries ============
# Safety workspace will be extracted from mounting_surface link mesh geometry
# Fallback values if mesh extraction fails


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

        # Motion queue for sequential execution
        self.motion_queue = MotionQueue(max_size=10)

        # Status publisher - broadcasts execution state and queue size
        self.status_publisher = RobotStatusPublisher(
            node=self,
            motion_queue=self.motion_queue,
            topic_name='/robot_status',
            publish_rate=10.0  # 10 Hz
        )

        self.monitor = None
        self.tcp_loaded = False
        self.T_ee_link = None
        self.T_tool = np.eye(4)
        # Reduced frequency - TCP transform typically available quickly
        self.tcp_load_timer = self.create_timer(1.0, self.load_tcp_transform)

        t1 = time.time()
        workspace = _extract_workspace_from_urdf(self,max_retries=60,retry_delay=2.0)

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
        self.current_joint_state = None  # Store the latest joint state for trajectory planning

        # Track active goal handles for motion cancellation
        self.active_execute_send_future = None
        self.active_controller_goal = None
        self.is_executing = False

        # Dynamics-based collision detector - uses inverse dynamics to isolate external torques
        # τ_external = τ_measured - τ_expected(q, dq, ddq)
        self.collision_detector = DynamicsCollisionDetector(
            urdf_path='/home/ilv/ros2_ws/src/fairino_description/urdf/fairino5_v6.urdf',
            base_link='base_link',
            tip_link='wrist3_link',
            num_joints=6,
            external_torque_rate_thresholds=np.array([15.0, 15.0, 12.0, 8.0, 6.0, 4.0]),
            confirmation_samples=3,
            recovery_time=1.0,
            logger=self.get_logger(),
            include_gravity=False,
        )
        self.collision_detector.set_on_collision(self._on_collision_detected)
        self.collision_detector.enable()
        self.collision_detector.arm()  # Always armed for testing
        self.collision_stop_enabled = False  # Set to True to auto-stop on collision
        self.collision_always_armed = True # Keep detector armed even when not moving
        self.get_logger().info('[Init] Dynamics collision detector initialized (ALWAYS ARMED for testing)')

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
                self.monitor = RobotMonitor(ros_node=self, tcp_transform=T_tcp_total, stable_update_rate_hz=50.0)
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
        return send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, planner_id, tool_transform)

    def jog_cartesian(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, vel_scale=0.1, acc_scale=0.1):
        from motion.jog_controller import jog_cartesian
        return jog_cartesian(self, dx_mm, dy_mm, dz_mm, vel_scale, acc_scale)

    def send_path_cartesian(self, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
        from motion.trajectory_planner import send_path_cartesian
        return send_path_cartesian(self, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling)

    def _on_collision_detected(self):
        """Callback when collision detector triggers."""
        self.get_logger().error('[COLLISION] Collision detected!')
        if self.collision_stop_enabled:
            self.get_logger().error('[COLLISION] Stopping motion immediately!')
            self.stop_motion()

    def _controller_status_callback(self, msg):
        """Monitor controller action status and track active goals."""

        for status in msg.status_list:
            if status.status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                # Arm collision detector when motion starts
                if not self.collision_detector.armed:
                    self.collision_detector.arm()
            elif status.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                   GoalStatus.STATUS_CANCELED]:
                if self.active_controller_goal is not None:
                    self.active_controller_goal = None
                self.is_executing = False
                # Only disarm if not in "always armed" mode
                if not self.collision_always_armed:
                    self.collision_detector.disarm()

    def joint_state_callback(self, msg):
        """Process joint states and store for trajectory planning."""
        import time

        # Feed data to dynamics collision detector (outside lock for speed)
        if len(msg.effort) >= 6 and len(msg.position) >= 6:
            # Get velocities and accelerations from monitor if available
            if self.monitor is not None:
                data = self.monitor.get_latest_data()
                velocities = data.get('velocities', np.zeros(6))
                accelerations = data.get('accelerations', np.zeros(6))
            else:
                # Fallback: use velocity from joint_states if available
                velocities = np.array(msg.velocity[:6]) if len(msg.velocity) >= 6 else np.zeros(6)
                accelerations = np.zeros(6)

            self.collision_detector.update(
                measured_efforts=np.array(msg.effort[:6]),
                positions=np.array(msg.position[:6]),
                velocities=velocities,
                accelerations=accelerations,
                timestamp=time.time()
            )

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

                # Also store efforts in monitor data
                if len(msg.effort) >= 6:
                    data['efforts'] = np.array(msg.effort[:6])

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
        Cancel all active motion goals (Controller only) and clear queue.

        Returns:
            int: 0 if motion was stopped, -1 if no active goals
        """
        stopped = False
        queue_cleared = 0

        # Cancel pending send operation
        if self.active_execute_send_future is not None:
            self.get_logger().info('[STOP] Cancelling pending trajectory submission...')
            try:
                self.active_execute_send_future.cancel()
                self.active_execute_send_future = None
                stopped = True
                self.get_logger().info('[STOP] ✓ Pending submission cancelled')
            except Exception as e:
                self.get_logger().error(f'[STOP] Failed to cancel send future: {e}')

        # Cancel the actual hardware controller goal (fairino5_controller)
        if self.active_controller_goal is not None:
            self.get_logger().info('[STOP] Cancelling active controller trajectory...')
            try:
                future = self.active_controller_goal.cancel_goal_async()
                self.active_controller_goal = None
                self.is_executing = False  # Clear execution flag
                stopped = True
                self.get_logger().warning('[STOP] ✓ Robot motion cancelled!')
            except Exception as e:
                self.get_logger().error(f'[STOP] Failed to cancel controller goal: {e}')

        # Clear motion queue
        queue_cleared = self.motion_queue.clear()
        if queue_cleared > 0:
            self.get_logger().warning(f'[STOP] Cleared {queue_cleared} queued motions')

        if stopped or queue_cleared > 0:
            self.get_logger().warning('[STOP] 🛑 Robot motion stopped and queue cleared')
            return 0  # Success
        else:
            self.get_logger().info('[STOP] No active motion to cancel')
            return -1  # No motion was active

    def is_motion_active(self):
        """Return True if any motion goal is active."""
        return any([
            self.active_controller_goal is not None,
            self.active_execute_send_future is not None
        ])

    # ============ Collision Detection API ============

    def enable_collision_detection(self):
        """Enable effort-based collision detection."""
        self.collision_detector.enable()
        self.get_logger().info('[CollisionDetector] Enabled')

    def disable_collision_detection(self):
        """Disable effort-based collision detection."""
        self.collision_detector.disable()
        self.get_logger().info('[CollisionDetector] Disabled')

    def enable_collision_stop(self):
        """Enable automatic motion stop when collision detected."""
        self.collision_stop_enabled = True
        self.get_logger().info('[CollisionDetector] Auto-stop ENABLED')

    def disable_collision_stop(self):
        """Disable automatic motion stop (detection still active for monitoring)."""
        self.collision_stop_enabled = False
        self.get_logger().info('[CollisionDetector] Auto-stop DISABLED (monitoring only)')

    def set_collision_always_armed(self, always_armed: bool):
        """
        Set whether collision detector should always be armed.

        Args:
            always_armed: True to keep armed even when not moving (for testing)
        """
        self.collision_always_armed = always_armed
        if always_armed:
            self.collision_detector.arm()
            self.get_logger().info('[CollisionDetector] ALWAYS ARMED mode enabled')
        else:
            self.get_logger().info('[CollisionDetector] Normal mode - armed only during motion')

    def set_collision_thresholds(self, rate_thresholds):
        """
        Set collision detection rate thresholds.

        Args:
            rate_thresholds: Per-joint rate thresholds (N·m/sample), array of 6 values
        """
        self.collision_detector.set_thresholds(np.array(rate_thresholds))

    def reset_collision_state(self):
        """Reset collision detector state."""
        self.collision_detector.arm()
        self.get_logger().info('[CollisionDetector] State reset and re-armed')

    def get_collision_status(self):
        """Get collision detector status for debugging."""
        return self.collision_detector.get_status()

    def set_sensitivity_preset(self, preset_name: str):
        """
        Set collision detection sensitivity preset.

        Args:
            preset_name: One of 'ULTRA_SENSITIVE_2KG', 'HIGH_SENSITIVE_4KG',
                        'MEDIUM_SENSITIVE_5KG', 'STANDARD_6KG', 'LOW_SENSITIVE_8KG'

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.collision_detector.set_sensitivity_preset(preset_name)
            self.get_logger().info(f'[CollisionDetector] Sensitivity changed to: {preset_name}')
            return True
        except Exception as e:
            self.get_logger().error(f'[CollisionDetector] Failed to set preset {preset_name}: {e}')
            return False

    def get_current_sensitivity_preset(self):
        """Get the currently active sensitivity preset name."""
        return self.collision_detector.get_current_preset()
