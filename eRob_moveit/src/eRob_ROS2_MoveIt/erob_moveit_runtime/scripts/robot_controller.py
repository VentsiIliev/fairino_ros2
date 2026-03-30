#!/usr/bin/env python3
import json
import subprocess
import threading
import time
from threading import Lock

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from control_msgs.msg import DynamicJointState
from erob_moveit_runtime.srv import ApplyIPP
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MotionSequenceItem, MotionSequenceRequest
from moveit_msgs.srv import GetCartesianPath, GetMotionSequence
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header, String
import tf2_ros
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from utils.transformation_utils import TransformationUtils
from safety.safety_wall_manager import SafetyWallManager
from safety.collision_detection import KDLInverseDynamicsModel, create_dynamics_collision_detector
from status.robot_monitor import RobotMonitor
from status.robot_state_store import RobotStateStore
from status.robot_status_publisher import RobotStatusPublisher
from motion.execution.motion_queue import MotionQueue
from motion.execution.motion_coordinator import MotionCoordinator
from motion.execution.trajectory_executor import TrajectoryExecutor
from motion.execution.trajectory_optimizer import build_trajectory_optimizer
from motion.planning.planner_context import PlannerContext
from motion.planning.planner_support_service import PlannerSupportService
from utils.workspace_extractor import _extract_workspace_from_urdf
# All tunable constants live in config.py
from config import (
    SAFETY_MARGIN_M as SAFETY_MARGIN,
    WALL_XY_OFFSET_M as WALL_XY_OFFSET,
    TOOL_REGISTRY as tool_registry,
    TOOL_ID_MAP as tool_id_map,
    MOTION_QUEUE_MAX_SIZE,
    MARKER_PUBLISH_INTERVAL_S,
    TOPIC_ROBOT_STATUS,
    STATUS_PUBLISH_RATE_HZ,
    WS_EXTRACT_MAX_RETRIES,
    WS_EXTRACT_RETRY_DELAY,
    ACTION_FOLLOW_TRAJECTORY,
    SERVICE_CARTESIAN_PATH,
    SERVICE_APPLY_IPP,
    COLLISION_TIP_LINK,
    BASE_LINK,
    WRIST_LINK,
    EE_LINK,
    URDF_PATH,
    NUM_JOINTS,
    COLLISION_RATE_THRESHOLDS,
    COLLISION_SUSTAINED_THRESHOLDS,
    COLLISION_CONFIRMATION_SAMPLES,
    COLLISION_RECOVERY_TIME_S,
    TRAJECTORY_OPTIMIZER,
    SAFETY_WALLS_ENABLED,
    ETHERCAT_WATCHDOG_ENABLED,
    ETHERCAT_EXPECTED_SLAVES,
    ETHERCAT_WATCHDOG_POLL_S,
    ETHERCAT_WATCHDOG_CMD_TIMEOUT_S,
    MOTION_ERROR_HARDWARE_NOT_READY,
    DRAG_MODE_ENABLED_DEFAULT,
    DRAG_MODE_UPDATE_RATE_HZ,
    DRAG_MODE_MODE_COMMAND_TOPIC,
    DRAG_MODE_EFFORT_COMMAND_TOPIC,
    DRAG_MODE_TORQUE_OFFSET_COMMAND_TOPIC,
    DRAG_MODE_CSP_VALUE,
    DRAG_MODE_CST_VALUE,
    DRAG_MODE_COMPENSATION_SCALE,
    DRAG_MODE_DAMPING_NM_PER_RAD_S,
    DRAG_MODE_MAX_EFFORT_NM,
    DRAG_MODE_MAX_TORQUE_OFFSET_NM,
    DRAG_MODE_MODE_SETTLE_TIMEOUT_S,
    JOINT_NAMES,
)


class RobotController(Node):
    _MANIPULATOR_CONTROLLER_NAME = 'manipulator_controller'
    _DRAG_CONTROLLER_NAMES = (
        'drag_mode_controller',
        'drag_effort_controller',
        'drag_torque_offset_controller',
    )

    def __init__(self):
        import time
        start_time = time.time()

        super().__init__('velocity_monitor')
        self.get_logger().info('[Init] RobotController starting...')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Motion queue for sequential execution
        self.motion_queue = MotionQueue(max_size=MOTION_QUEUE_MAX_SIZE)
        self._motion = MotionCoordinator(node=self, motion_queue=self.motion_queue)
        self.lock = self._motion.lock
        self.execution_lock = self._motion.execution_lock

        # Status publisher - broadcasts execution state and queue size
        self.status_publisher = RobotStatusPublisher(
            node=self,
            motion_queue=self.motion_queue,
            topic_name=TOPIC_ROBOT_STATUS,
            publish_rate=STATUS_PUBLISH_RATE_HZ
        )

        self.monitor = None
        self.tcp_loaded = False
        self.T_ee_link = None
        self.T_tool = np.eye(4)
        # Reduced frequency - TCP transform typically available quickly
        self.tcp_load_timer = self.create_timer(1.0, self.load_tcp_transform)

        t1 = time.time()
        workspace = _extract_workspace_from_urdf(self, max_retries=WS_EXTRACT_MAX_RETRIES, retry_delay=WS_EXTRACT_RETRY_DELAY)

        self.get_logger().info(f'[Init] Workspace extraction took {time.time() - t1:.2f}s')

        workspace = {
            'x_min': workspace['x_min'] - WALL_XY_OFFSET,
            'x_max': workspace['x_max'] + WALL_XY_OFFSET,
            'y_min': workspace['y_min'] - WALL_XY_OFFSET,
            'y_max': workspace['y_max'] + WALL_XY_OFFSET,
            'z_min': workspace['z_min'],
            'z_max': workspace['z_max'],
        }
        self.get_logger().info(
            f'[Init] Workspace after XY offset ({WALL_XY_OFFSET*1000:.0f}mm): '
            f'X[{workspace["x_min"]:.3f},{workspace["x_max"]:.3f}] '
            f'Y[{workspace["y_min"]:.3f},{workspace["y_max"]:.3f}] '
            f'Z[{workspace["z_min"]:.3f},{workspace["z_max"]:.3f}]')

        self.safety_manager = SafetyWallManager(
            node=self,
            workspace=workspace,
            margin=SAFETY_MARGIN,
            enabled=SAFETY_WALLS_ENABLED,
            marker_publish_interval=MARKER_PUBLISH_INTERVAL_S
        )

        # Defer initial safety wall publishing to speed up initialization
        # Will be published on first use or after 1 second
        self._safety_init_timer = self.create_timer(1.0, self._delayed_safety_init)

        # ROS clients
        self.controller_client = ActionClient(self, FollowJointTrajectory, ACTION_FOLLOW_TRAJECTORY)
        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
        )
        self.cart_path_client = self.create_client(GetCartesianPath, SERVICE_CARTESIAN_PATH)
        self.ipp_client = self.create_client(ApplyIPP, SERVICE_APPLY_IPP)
        self.trajectory_executor = TrajectoryExecutor(
            node=self,
            coordinator=self._motion,
            motion_queue=self.motion_queue,
            controller_client=self.controller_client,
        )
        self.trajectory_optimizer = build_trajectory_optimizer(
            TRAJECTORY_OPTIMIZER,
            node=self,
            fallback_name="TOTG",
        )
        self.state_store = RobotStateStore()
        self.planner_support = PlannerSupportService(node=self)
        self.planner_context = PlannerContext(
            node=self,
            state_store=self.state_store,
            motion_coordinator=self._motion,
            motion_queue=self.motion_queue,
            safety_manager=self.safety_manager,
            cart_path_client=self.cart_path_client,
            ipp_client=self.ipp_client,
            trajectory_executor=self.trajectory_executor,
            planner_support=self.planner_support,
            trajectory_optimizer=self.trajectory_optimizer,
        )
        self.planner_context.T_tool = self.T_tool

        self.urdf_path = URDF_PATH

        # Dynamics-based collision detector - uses inverse dynamics to isolate external torques
        # τ_external = τ_measured - τ_expected(q, dq, ddq)
        self.collision_detector = create_dynamics_collision_detector(
            urdf_path=self.urdf_path,
            base_link=BASE_LINK,
            tip_link=COLLISION_TIP_LINK,
            num_joints=NUM_JOINTS,
            external_torque_rate_thresholds=np.array(COLLISION_RATE_THRESHOLDS),
            external_torque_sustained_thresholds=np.array(COLLISION_SUSTAINED_THRESHOLDS),
            enable_sustained_check=False,  # Disable to avoid false positives during high acceleration
            confirmation_samples=COLLISION_CONFIRMATION_SAMPLES,
            recovery_time=COLLISION_RECOVERY_TIME_S,
            logger=self.get_logger(),
            include_gravity=False,
        )
        self.collision_detector.set_on_collision(self._on_collision_detected)
        self.collision_detector.disable()  # TEMPORARILY DISABLED — re-enable with self.collision_detector.enable()
        self.collision_detector.disarm()  # Not computing — gravity model is 6.6x off, causes false positives
        self.collision_stop_enabled = False  # Set to True to auto-stop on collision
        self.collision_always_armed = False
        self.get_logger().info('[Init] Dynamics collision detector initialized (DISABLED — gravity model not calibrated)')

        self._ethercat_fault_lock = Lock()
        self._ethercat_motion_fault = False
        self._ethercat_fault_reason = ""
        self._ethercat_fault_stop_issued = False
        self._ethercat_watchdog_enabled = bool(ETHERCAT_WATCHDOG_ENABLED)
        self._ethercat_watchdog_running = False
        self._ethercat_watchdog_thread = None
        if self._ethercat_watchdog_enabled:
            self._ethercat_watchdog_running = True
            self._ethercat_watchdog_thread = threading.Thread(
                target=self._ethercat_watchdog_loop,
                daemon=True,
                name="EthercatWatchdog",
            )
            self._ethercat_watchdog_thread.start()
            self.get_logger().info('[EtherCAT] Runtime OP watchdog enabled')
        else:
            self.get_logger().info('[EtherCAT] Runtime OP watchdog disabled for this robot backend')

        self._drag_lock = Lock()
        self._drag_enabled = bool(DRAG_MODE_ENABLED_DEFAULT)
        self._drag_update_dt = 1.0 / max(float(DRAG_MODE_UPDATE_RATE_HZ), 1.0)
        self._drag_mode_csp = float(DRAG_MODE_CSP_VALUE)
        self._drag_mode_cst = float(DRAG_MODE_CST_VALUE)
        self._drag_compensation_scale = float(DRAG_MODE_COMPENSATION_SCALE)
        self._drag_mode_settle_timeout_s = float(DRAG_MODE_MODE_SETTLE_TIMEOUT_S)
        self._drag_damping_nm_per_rad_s = np.array(DRAG_MODE_DAMPING_NM_PER_RAD_S, dtype=float)
        self._drag_max_effort_nm = np.array(DRAG_MODE_MAX_EFFORT_NM, dtype=float)
        self._drag_max_torque_offset_nm = np.array(DRAG_MODE_MAX_TORQUE_OFFSET_NM, dtype=float)
        self._drag_external_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_expected_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_friction_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_friction_coulomb_nm = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_friction_viscous_nm_per_rad_s = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_velocity = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_mode_display = np.full(NUM_JOINTS, self._drag_mode_csp, dtype=float)
        self._drag_last_mode_command = np.full(NUM_JOINTS, self._drag_mode_csp, dtype=float)
        self._drag_last_effort_command = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_torque_offset_command = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_transition_ts = 0.0
        self._drag_joint_order = list(JOINT_NAMES)
        self._drag_bias_model = KDLInverseDynamicsModel(
            urdf_path=self.urdf_path,
            base_link=BASE_LINK,
            tip_link=COLLISION_TIP_LINK,
            num_joints=NUM_JOINTS,
            logger=self.get_logger(),
            include_gravity=True,
        )
        self._drag_mode_pub = self.create_publisher(
            Float64MultiArray,
            DRAG_MODE_MODE_COMMAND_TOPIC,
            10,
        )
        self._drag_effort_pub = self.create_publisher(
            Float64MultiArray,
            DRAG_MODE_EFFORT_COMMAND_TOPIC,
            10,
        )
        self._drag_torque_offset_pub = self.create_publisher(
            Float64MultiArray,
            DRAG_MODE_TORQUE_OFFSET_COMMAND_TOPIC,
            10,
        )
        self.create_subscription(
            DynamicJointState,
            '/zeroerr/collision_monitor/state',
            self._drag_monitor_callback,
            10,
        )
        self.create_subscription(
            String,
            '/zeroerr/collision_monitor/config_state',
            self._drag_config_callback,
            10,
        )
        self._drag_timer = self.create_timer(self._drag_update_dt, self._drag_mode_step)
        self.get_logger().info(
            f"[DragMode] Initialized real CST path (enabled={self._drag_enabled}, dt={self._drag_update_dt:.3f}s)"
        )

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(GoalStatusArray, ACTION_FOLLOW_TRAJECTORY + '/_action/status',
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

    def enable_safety_walls(self) -> dict:
        """Enable safety walls and republish them to MoveIt/RViz."""
        self.safety_manager.enable_safety()
        return self.safety_manager.get_status()

    def disable_safety_walls(self) -> dict:
        """Disable safety walls and remove them from MoveIt/RViz."""
        self.safety_manager.disable_safety()
        return self.safety_manager.get_status()

    def get_safety_walls_status(self) -> dict:
        """Return current safety wall status."""
        return self.safety_manager.get_status()

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
        self.planner_context.T_tool = self.T_tool

        # Topic publishes EE_LINK already; only apply tool offset
        if self.monitor is not None:
            self.monitor.T_tcp = self.T_tool
            self.get_logger().info(f"Switched active tool to {tool_name}")
        else:
            self.get_logger().warning("RobotMonitor not initialized yet")

    def load_tcp_transform(self):
        if self.tcp_loaded:
            return  # Already loaded

        if self.tf_buffer.can_transform(WRIST_LINK, EE_LINK, rclpy.time.Time()):
            try:
                # Load ee_link offset from URDF/TF (wrist3 → ee_link) — kept for reference/logging only.
                # The /cartesian_position topic already publishes the EE_LINK (tcp) frame, so the
                # monitor must only apply the user tool offset (T_tool), not T_ee_link again.
                self.T_ee_link = self.get_tcp_transform(WRIST_LINK, EE_LINK)
                print(f"[RobotController] Loaded ee_link transform:\n{self.T_ee_link}")

                # Monitor receives EE_LINK pose from topic → only apply tool offset on top
                self.monitor = RobotMonitor(ros_node=self, tcp_transform=self.T_tool, stable_update_rate_hz=50.0)
                self.tcp_loaded = True

                self.get_logger().info("TCP transform loaded and RobotMonitor initialized")

                # Destroy timer to prevent further callbacks
                if hasattr(self, 'tcp_load_timer'):
                    self.tcp_load_timer.cancel()
                    self.destroy_timer(self.tcp_load_timer)

            except Exception as e:
                self.get_logger().warning(f"TCP transform lookup failed: {e}")

    def get_tcp_transform(self, from_frame=WRIST_LINK, to_frame=EE_LINK):
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

    def execute(self, strategy, queue_if_busy=True):
        with self._drag_lock:
            drag_enabled = self._drag_enabled
        if drag_enabled:
            self.get_logger().error('[DragMode] Rejecting motion while CST drag mode is enabled')
            return MOTION_ERROR_HARDWARE_NOT_READY
        if not self.is_hardware_ready_for_motion():
            self.get_logger().error(
                f'[EtherCAT] Rejecting motion: {self.get_hardware_fault_reason()}'
            )
            return MOTION_ERROR_HARDWARE_NOT_READY
        return self._motion.execute(strategy, queue_if_busy=queue_if_busy)

    def send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                            tool_transform=None, queue_if_busy=True):
        from motion.strategies import SingleTargetStrategy
        return self.execute(SingleTargetStrategy(
            x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
            tool_transform=tool_transform), queue_if_busy=queue_if_busy)

    def _on_collision_detected(self):
        """Callback when collision detector triggers."""
        self.get_logger().error('[COLLISION] Collision detected!')
        if self.collision_stop_enabled:
            self.get_logger().error('[COLLISION] Stopping motion immediately!')
            self.stop_motion()

    def _controller_status_callback(self, msg):
        """Monitor controller action status for side effects only.

        Normal motion lifecycle cleanup is owned by the trajectory result callback.
        Clearing active goal state here races with the async result future and makes
        successful completions look like stale/cancelled goals.
        """

        for status in msg.status_list:
            if status.status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                # Arm collision detector when motion starts
                if not self.collision_detector.armed:
                    self.collision_detector.arm()
            elif status.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                   GoalStatus.STATUS_CANCELED]:
                # Result callback owns active goal + execution cleanup. Only handle
                # detector state here to avoid racing normal completion.
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
            if len(msg.position) < 6:
                return

            # Joint-state caching must not depend on RobotMonitor readiness.
            self.current_joint_state = msg

            if self.monitor is None:
                return  # RobotMonitor not initialized yet

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
        return self.state_store.get_latest_data()

    def request_cartesian_path(self, request):
        return self.planner_context.request_cartesian_path(request)

    def wait_for_cartesian_path_service(self, timeout_sec=1.0):
        return self.planner_context.wait_for_cartesian_path_service(timeout_sec=timeout_sec)

    def force_safety_update(self):
        self.planner_context.force_safety_update()

    def check_position_safety(self, x, y, z):
        return self.planner_context.check_position_safety(x, y, z)

    def set_last_requested_delta_mm(self, value):
        self.planner_context.set_last_requested_delta_mm(value)

    def get_last_requested_delta_mm(self):
        return self.planner_context.get_last_requested_delta_mm()

    def set_last_full_waypoints(self, value):
        self.planner_context.set_last_full_waypoints(value)

    def get_last_full_waypoints(self):
        return self.planner_context.get_last_full_waypoints()

    def stage_pending_path(self, trajectory, vel_scaling, acc_scaling):
        self.planner_context.stage_pending_path(trajectory, vel_scaling, acc_scaling)

    def consume_pending_path(self):
        return self.planner_context.consume_pending_path()

    def clear_pending_path(self):
        self.planner_context.clear_pending_path()

    def submit_motion_task(self, task_function, task_args=None):
        self.planner_context.submit_motion_task(task_function, task_args)

    def mark_current_motion_complete(self, result):
        self.planner_context.mark_current_motion_complete(result)

    def clear_motion_queue(self):
        self.planner_context.clear_motion_queue()

    def get_fk_client(self):
        return self.planner_context.get_fk_client()

    def get_ik_client(self):
        return self.planner_context.get_ik_client()

    def get_state_validity_client(self):
        return self.planner_context.get_state_validity_client()

    def stop_motion(self):
        return self._motion.stop_motion()

    def enable_drag_mode(self) -> dict:
        stop_result = self.stop_motion()
        switch_ok = self._set_drag_controller_ownership(True)
        with self._drag_lock:
            self._drag_enabled = bool(switch_ok)
            self._drag_last_transition_ts = time.monotonic()
        return {
            "enabled": bool(switch_ok),
            "stopped": bool(isinstance(stop_result, dict) and stop_result.get("stopped", False)),
            "mode": "cst",
            "state": "ENABLING" if switch_ok else "ERROR",
            "controller_switch_ok": bool(switch_ok),
        }

    def disable_drag_mode(self) -> dict:
        with self._drag_lock:
            self._drag_enabled = False
            self._drag_last_transition_ts = time.monotonic()
        stop_result = self.stop_motion()
        self._publish_drag_commands(
            mode_command=np.full(NUM_JOINTS, self._drag_mode_csp, dtype=float),
            effort_command=np.zeros(NUM_JOINTS, dtype=float),
            torque_offset_command=np.zeros(NUM_JOINTS, dtype=float),
        )
        switch_ok = self._set_drag_controller_ownership(False)
        self._send_hold_position_trajectory()
        return {
            "enabled": False,
            "stopped": bool(isinstance(stop_result, dict) and stop_result.get("stopped", False)),
            "mode": "csp",
            "state": "DISABLING" if switch_ok else "ERROR",
            "controller_switch_ok": bool(switch_ok),
        }

    def get_drag_mode_status(self) -> dict:
        with self._drag_lock:
            enabled = self._drag_enabled
            mode_display = self._drag_mode_display.tolist()
            last_mode = self._drag_last_mode_command.tolist()
            last_effort = self._drag_last_effort_command.tolist()
            last_offset = self._drag_last_torque_offset_command.tolist()
            return {
                "enabled": enabled,
                "active_mode": "cst" if enabled else "csp",
                "requested_mode_value": self._drag_mode_cst if enabled else self._drag_mode_csp,
                "mode_display": mode_display,
                "mode_match": all(abs(value - (self._drag_mode_cst if enabled else self._drag_mode_csp)) < 0.5 for value in mode_display),
                "settle_timeout_s": self._drag_mode_settle_timeout_s,
                "manipulator_controller_expected_active": not enabled,
                "compensation_scale": self._drag_compensation_scale,
                "damping_nm_per_rad_s": self._drag_damping_nm_per_rad_s.tolist(),
                "max_effort_nm": self._drag_max_effort_nm.tolist(),
                "max_torque_offset_nm": self._drag_max_torque_offset_nm.tolist(),
                "external_tau_nm": self._drag_external_tau.tolist(),
                "expected_tau_nm": self._drag_expected_tau.tolist(),
                "friction_tau_nm": self._drag_friction_tau.tolist(),
                "last_mode_command": last_mode,
                "last_effort_command_nm": last_effort,
                "last_torque_offset_command_nm": last_offset,
            }

    def _set_drag_controller_ownership(self, drag_enabled: bool) -> bool:
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('[DragMode] /controller_manager/switch_controller not available')
            return False

        request = SwitchController.Request()
        request.strictness = 2
        request.activate_asap = True
        request.timeout = Duration(sec=2, nanosec=0)
        request.activate_controllers = list(self._DRAG_CONTROLLER_NAMES)
        if drag_enabled:
            request.deactivate_controllers = [self._MANIPULATOR_CONTROLLER_NAME]
        else:
            request.activate_controllers.append(self._MANIPULATOR_CONTROLLER_NAME)
            request.deactivate_controllers = []

        future = self.switch_controller_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            self.get_logger().error('[DragMode] Timed out switching controllers')
            return False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'[DragMode] Controller switch failed: {exc}')
            return False
        if not response.ok:
            self.get_logger().error('[DragMode] Controller manager rejected drag controller switch')
            return False
        self.get_logger().info(
            f"[DragMode] Controller ownership set for {'drag' if drag_enabled else 'trajectory'} mode"
        )
        return True

    def is_hardware_ready_for_motion(self) -> bool:
        with self._ethercat_fault_lock:
            return not self._ethercat_motion_fault

    def get_hardware_fault_reason(self) -> str:
        with self._ethercat_fault_lock:
            return self._ethercat_fault_reason or 'EtherCAT hardware fault'

    def is_motion_active(self):
        """Return True if any motion goal is active."""
        return self._motion.is_motion_active()

    def has_pending_motion(self):
        """Return True if any queued motion is waiting to execute."""
        return self._motion.has_pending_motion()

    def destroy_node(self):
        self._ethercat_watchdog_running = False
        watchdog = getattr(self, '_ethercat_watchdog_thread', None)
        if watchdog is not None and watchdog.is_alive():
            watchdog.join(timeout=1.0)
        return super().destroy_node()

    def _drag_monitor_callback(self, msg: DynamicJointState):
        joint_index = {name: idx for idx, name in enumerate(self._drag_joint_order)}
        external_tau = np.zeros(NUM_JOINTS, dtype=float)
        mode_display = np.full(NUM_JOINTS, self._drag_mode_csp, dtype=float)
        for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
            index = joint_index.get(joint_name)
            if index is None:
                continue
            for name, value in zip(interface_value.interface_names, interface_value.values):
                if name == 'external_torque':
                    if np.isfinite(value):
                        external_tau[index] = float(value)
                elif name == 'mode_display' and np.isfinite(value):
                    mode_display[index] = float(value)
        with self._drag_lock:
            self._drag_external_tau = external_tau
            self._drag_mode_display = mode_display

    def _drag_config_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        coulomb = payload.get("friction_coulomb_nm")
        viscous = payload.get("friction_viscous_nm_per_rad_s")
        if not isinstance(coulomb, list) or not isinstance(viscous, list):
            return
        try:
            coulomb_np = np.array(coulomb[:NUM_JOINTS], dtype=float)
            viscous_np = np.array(viscous[:NUM_JOINTS], dtype=float)
        except Exception:
            return
        if coulomb_np.size != NUM_JOINTS or viscous_np.size != NUM_JOINTS:
            return
        with self._drag_lock:
            self._drag_friction_coulomb_nm = coulomb_np
            self._drag_friction_viscous_nm_per_rad_s = viscous_np

    def _drag_mode_step(self):
        with self._drag_lock:
            drag_enabled = self._drag_enabled
            target_mode = self._drag_mode_cst if drag_enabled else self._drag_mode_csp

        if not self.is_hardware_ready_for_motion():
            return

        joint_state = self.current_joint_state
        if joint_state is None or len(joint_state.position) < NUM_JOINTS:
            return

        positions = np.array(joint_state.position[:NUM_JOINTS], dtype=float)
        velocities = (
            np.array(joint_state.velocity[:NUM_JOINTS], dtype=float)
            if len(joint_state.velocity) >= NUM_JOINTS
            else np.zeros(NUM_JOINTS, dtype=float)
        )
        friction_tau = self._compute_drag_friction_torque(velocities)
        expected_tau = self._drag_bias_model.compute_bias_torque(positions, velocities)

        with self._drag_lock:
            self._drag_velocity = velocities
            self._drag_expected_tau = expected_tau
            self._drag_friction_tau = friction_tau

        torque_offset = self._drag_compensation_scale * (expected_tau + friction_tau)
        torque_offset = np.clip(
            torque_offset,
            -self._drag_max_torque_offset_nm,
            self._drag_max_torque_offset_nm,
        )
        effort_command = np.clip(
            -self._drag_damping_nm_per_rad_s * velocities,
            -self._drag_max_effort_nm,
            self._drag_max_effort_nm,
        )

        if drag_enabled and (self.is_motion_active() or self.has_pending_motion()):
            return

        if not drag_enabled:
            torque_offset = np.zeros(NUM_JOINTS, dtype=float)
            effort_command = np.zeros(NUM_JOINTS, dtype=float)

        self._publish_drag_commands(
            mode_command=np.full(NUM_JOINTS, target_mode, dtype=float),
            effort_command=effort_command,
            torque_offset_command=torque_offset,
        )

    def _compute_drag_friction_torque(self, velocities: np.ndarray) -> np.ndarray:
        coulomb = getattr(self, "_drag_friction_coulomb_nm", np.zeros(NUM_JOINTS, dtype=float))
        viscous = getattr(self, "_drag_friction_viscous_nm_per_rad_s", np.zeros(NUM_JOINTS, dtype=float))
        return np.sign(velocities) * coulomb + viscous * velocities

    def _publish_drag_commands(
        self,
        mode_command: np.ndarray,
        effort_command: np.ndarray,
        torque_offset_command: np.ndarray,
    ) -> None:
        mode_msg = Float64MultiArray()
        mode_msg.data = mode_command.tolist()
        effort_msg = Float64MultiArray()
        effort_msg.data = effort_command.tolist()
        offset_msg = Float64MultiArray()
        offset_msg.data = torque_offset_command.tolist()

        self._drag_mode_pub.publish(mode_msg)
        self._drag_effort_pub.publish(effort_msg)
        self._drag_torque_offset_pub.publish(offset_msg)

        with self._drag_lock:
            self._drag_last_mode_command = mode_command.copy()
            self._drag_last_effort_command = effort_command.copy()
            self._drag_last_torque_offset_command = torque_offset_command.copy()

    def _send_hold_position_trajectory(self) -> None:
        joint_state = self.current_joint_state
        if joint_state is None or len(joint_state.position) < NUM_JOINTS:
            return
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        traj.header.stamp = self.get_clock().now().to_msg()

        start_pt = JointTrajectoryPoint()
        start_pt.positions = list(joint_state.position[:NUM_JOINTS])
        start_pt.time_from_start = Duration(sec=0, nanosec=0)

        end_pt = JointTrajectoryPoint()
        end_pt.positions = list(joint_state.position[:NUM_JOINTS])
        end_pt.time_from_start = Duration(sec=0, nanosec=200_000_000)

        traj.points = [start_pt, end_pt]
        self.trajectory_executor.send_trajectory_to_controller(traj)

    def _ethercat_watchdog_loop(self):
        while self._ethercat_watchdog_running:
            self._poll_ethercat_health_once()
            time.sleep(ETHERCAT_WATCHDOG_POLL_S)

    def _poll_ethercat_health_once(self):
        try:
            result = subprocess.run(
                ['ethercat', 'slaves'],
                capture_output=True,
                text=True,
                timeout=ETHERCAT_WATCHDOG_CMD_TIMEOUT_S,
                check=False,
            )
        except Exception as exc:
            self._set_ethercat_fault(
                True,
                f'ethercat slaves command failed: {exc}',
            )
            return

        if result.returncode != 0:
            self._set_ethercat_fault(
                True,
                f'ethercat slaves exited with code {result.returncode}',
            )
            return

        slave_lines = [
            line for line in result.stdout.splitlines()
            if line.strip() and line.lstrip()[:1].isdigit()
        ]
        total_count = len(slave_lines)
        op_count = sum(1 for line in slave_lines if ' OP ' in f' {line} ')
        if total_count != ETHERCAT_EXPECTED_SLAVES or op_count != ETHERCAT_EXPECTED_SLAVES:
            self._set_ethercat_fault(
                True,
                f'EtherCAT not fully OP ({op_count}/{total_count} OP, expected {ETHERCAT_EXPECTED_SLAVES})',
            )
            return

        self._set_ethercat_fault(False, '')

    def _set_ethercat_fault(self, faulted: bool, reason: str):
        should_stop_motion = False
        with self._ethercat_fault_lock:
            previous_faulted = self._ethercat_motion_fault
            previous_reason = self._ethercat_fault_reason
            self._ethercat_motion_fault = faulted
            self._ethercat_fault_reason = reason
            if faulted:
                should_stop_motion = not self._ethercat_fault_stop_issued
                self._ethercat_fault_stop_issued = True
            else:
                self._ethercat_fault_stop_issued = False

        if faulted and (not previous_faulted or previous_reason != reason):
            self.get_logger().error(f'[EtherCAT] Motion interlock active: {reason}')
        elif not faulted and previous_faulted:
            self.get_logger().info('[EtherCAT] All slaves back in OP — motion interlock cleared')

        if faulted and should_stop_motion:
            try:
                stop_result = self.stop_motion()
                self.get_logger().error(
                    f'[EtherCAT] Issued emergency motion stop due to slave state fault: {stop_result}'
                )
            except Exception as exc:
                self.get_logger().error(
                    f'[EtherCAT] Failed to stop motion after slave state fault: {exc}'
                )

    @property
    def active_execute_send_future(self):
        return self._motion.active_execute_send_future

    @active_execute_send_future.setter
    def active_execute_send_future(self, value):
        self._motion.active_execute_send_future = value

    @property
    def active_controller_goal(self):
        return self._motion.active_controller_goal

    @active_controller_goal.setter
    def active_controller_goal(self, value):
        self._motion.active_controller_goal = value

    @property
    def is_executing(self):
        return self._motion.is_executing

    @is_executing.setter
    def is_executing(self, value):
        self._motion.is_executing = value

    @property
    def plan_generation(self):
        return self._motion.plan_generation

    @plan_generation.setter
    def plan_generation(self, value):
        self._motion.plan_generation = value

    @property
    def last_move_result(self):
        return self._motion.last_move_result

    @last_move_result.setter
    def last_move_result(self, value):
        self._motion.last_move_result = value

    @property
    def last_submitted_task_id(self):
        return self._motion.last_submitted_task_id

    @last_submitted_task_id.setter
    def last_submitted_task_id(self, value):
        self._motion.last_submitted_task_id = value

    @property
    def prev_cartesian(self):
        return self.state_store.get_prev_cartesian()

    @prev_cartesian.setter
    def prev_cartesian(self, value):
        self.state_store.set_prev_cartesian(value)

    @property
    def current_joint_state(self):
        return self.state_store.get_current_joint_state()

    @current_joint_state.setter
    def current_joint_state(self, value):
        self.state_store.set_current_joint_state(value)

    @property
    def latest_data(self):
        return self.state_store.get_latest_data()

    @latest_data.setter
    def latest_data(self, value):
        self.state_store.set_latest_data(value)

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
