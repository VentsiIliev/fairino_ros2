#!/usr/bin/env python3
import json
import subprocess
import threading
import time
from threading import Lock
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from control_msgs.msg import DynamicJointState
from erob_moveit_runtime.srv import ApplyIPP
from geometry_msgs.msg import Point, Pose, TransformStamped
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
from visualization_msgs.msg import Marker, MarkerArray
import config
from backend.runtime_adapter import create_runtime_adapter

from utils.transformation_utils import TransformationUtils
from safety.safety_wall_manager import SafetyWallManager
from safety.collision_detection import KDLInverseDynamicsModel
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

ZEROERR_DRIVE_ERROR_CODES = {
    0x0000: "no active drive error",
    0x2214: "motor current is over current",
    0x2250: "sum of motor three phase current exceeds the limit",
    0x2341: "U phase over current",
    0x2342: "V phase over current",
    0x2343: "W phase over current",
    0x3210: "bus voltage is overvoltage",
    0x3220: "bus voltage is undervoltage",
    0x4110: "power component temperature is too high",
    0x7121: "blocked motor rotation",
    0x7305: "load-side single-turn encoder read data is incorrect",
    0x7306: "motor-side single-turn encoder read data is incorrect",
    0x730D: "battery warning error",
    0x730F: "battery low voltage",
    0x7311: "sampled motor-end position error exceeds the limit",
    0x7314: "battery reconnection detected; reset load-side encoder to clear alarm",
    0x7315: "sampled load-side position error exceeds the limit",
    0x7350: "motor-side encoder type is not supported",
    0x7374: "multi-turn position error",
    0x7377: "reset pin error detected",
    0x737A: "load-side single-turn encoder startup error",
    0x737E: "motor-side single-turn encoder startup error",
    0x8130: "CAN heartbeat error",
    0x8400: "velocity error exceeds the limit value",
    0x8401: "motor velocity exceeds the limit value",
    0x8500: "position error exceeds the limit value",
    0xA000: "master station offline / EtherCAT communication abnormal",
    0xF004: "EtherCAT initialization error",
    0xF005: "STO function is activated",
    0xF006: "multi-turn circle count error",
    0xF008: "bus voltage below minimum allowable supply voltage (19V)",
}
# All tunable constants live in config.py
from config import (
    SAFETY_MARGIN_M as SAFETY_MARGIN,
    WALL_XY_OFFSET_M as WALL_XY_OFFSET,
    TOOL_REGISTRY as tool_registry,
    TOOL_ID_MAP as tool_id_map,
    MOTION_QUEUE_MAX_SIZE,
    MARKER_PUBLISH_INTERVAL_S,
    TOPIC_ROBOT_STATUS,
    TOPIC_ACTIVE_TOOL_MARKERS,
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
    ETHERCAT_RECOVERY_ENABLED,
    ETHERCAT_RECOVERY_MIN_INTERVAL_S,
    ETHERCAT_RECOVERY_CMD_TIMEOUT_S,
    MOTION_ERROR_HARDWARE_NOT_READY,
    MOTION_ERROR_DRIVE_NOT_ENABLED,
    DRAG_MODE_ENABLED_DEFAULT,
    DRAG_MODE_UPDATE_RATE_HZ,
    DRAG_MODE_MODE_COMMAND_TOPIC,
    DRAG_MODE_EFFORT_COMMAND_TOPIC,
    DRAG_MODE_TORQUE_OFFSET_COMMAND_TOPIC,
    DRAG_MODE_ENABLE_SET_COMMAND_TOPIC,
    DRAG_MODE_DISABLE_SET_COMMAND_TOPIC,
    DRIVE_ENABLE_SET_COMMAND_TOPIC,
    DRIVE_DISABLE_SET_COMMAND_TOPIC,
    DRAG_MODE_CSP_VALUE,
    DRAG_MODE_CST_VALUE,
    DRAG_MODE_COMPENSATION_SCALE,
    DRAG_MODE_DAMPING_NM_PER_RAD_S,
    DRAG_MODE_MAX_EFFORT_NM,
    DRAG_MODE_MAX_TORQUE_OFFSET_NM,
    DRAG_MODE_MODE_SETTLE_TIMEOUT_S,
    DRAG_MODE_DISABLE_PULSE_S,
    DRAG_MODE_ENABLE_PULSE_S,
    DRAG_MODE_CONFIG_PATH,
    DRAG_MODE_JOINT_MODELS,
    DRAG_MODE_MODEL_NAMES,
    DRAG_MODE_MODEL_RATED_CURRENT_MA,
    DRAG_MODE_MODEL_OUTPUT_TORQUE_CONSTANT_NM_PER_A,
    JOINT_NAMES,
)


class RobotController(Node):
    _MANIPULATOR_CONTROLLER_NAME = 'manipulator_controller'
    _DRAG_CONTROLLER_NAMES = (
        'drag_mode_controller',
        'drag_effort_controller',
        'drag_torque_offset_controller',
        'drag_enable_set_controller',
        'drag_disable_set_controller',
    )
    _DRIVE_SET_CONTROLLER_NAMES = (
        'drive_enable_set_controller',
        'drive_disable_set_controller',
    )

    def __init__(self):
        import time
        start_time = time.time()

        super().__init__('velocity_monitor')
        self.get_logger().info('[Init] RobotController starting...')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.active_tcp_tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.active_tool_marker_pub = self.create_publisher(
            MarkerArray,
            TOPIC_ACTIVE_TOOL_MARKERS,
            10,
        )
        self.active_tool_marker_timer = self.create_timer(
            0.1,
            self._publish_active_tool_visualization,
        )

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
        self.T_monitor_tool = np.eye(4)
        self.active_tool_name = "TOOL_0"
        self.runtime_adapter = create_runtime_adapter()
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
        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
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

        self._ethercat_fault_lock = Lock()
        self._ethercat_motion_fault = False
        self._ethercat_fault_reason = ""
        self._ethercat_fault_stop_issued = False
        self._ethercat_last_recovery_attempt_ts = 0.0
        self._collision_monitor_fault_enabled = True
        self._collision_fault_lock = Lock()
        self._collision_motion_fault = False
        self._collision_fault_reason = ""
        self._collision_following_error_thresholds = np.zeros(NUM_JOINTS, dtype=float)
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
        self._drive_enable_lock = Lock()
        self._drive_operation_enabled_requested = False
        self._drag_enabled = bool(DRAG_MODE_ENABLED_DEFAULT)
        self._drag_update_dt = 1.0 / max(float(DRAG_MODE_UPDATE_RATE_HZ), 1.0)
        self._drag_mode_csp = float(DRAG_MODE_CSP_VALUE)
        self._drag_mode_cst = float(DRAG_MODE_CST_VALUE)
        self._drag_compensation_scale = float(DRAG_MODE_COMPENSATION_SCALE)
        joint_comp_scale_cfg = globals().get(
            "DRAG_MODE_JOINT_COMPENSATION_SCALE",
            [1.0] * NUM_JOINTS,
        )
        self._drag_joint_compensation_scale = np.array(joint_comp_scale_cfg, dtype=float)
        if self._drag_joint_compensation_scale.size != NUM_JOINTS:
            self.get_logger().warning(
                f"[DragMode] Invalid DRAG_MODE_JOINT_COMPENSATION_SCALE size="
                f"{self._drag_joint_compensation_scale.size}, expected {NUM_JOINTS}; using ones"
            )
            self._drag_joint_compensation_scale = np.ones(NUM_JOINTS, dtype=float)
        self._drag_mode_settle_timeout_s = float(DRAG_MODE_MODE_SETTLE_TIMEOUT_S)
        self._drag_disable_pulse_s = float(DRAG_MODE_DISABLE_PULSE_S)
        self._drag_enable_pulse_s = float(DRAG_MODE_ENABLE_PULSE_S)
        self._drag_damping_nm_per_rad_s = np.array(DRAG_MODE_DAMPING_NM_PER_RAD_S, dtype=float)
        self._drag_max_effort_nm = np.array(DRAG_MODE_MAX_EFFORT_NM, dtype=float)
        self._drag_max_torque_offset_nm = np.array(DRAG_MODE_MAX_TORQUE_OFFSET_NM, dtype=float)
        self._drag_external_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_expected_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_gravity_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_non_gravity_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_friction_tau = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_friction_coulomb_nm = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_friction_viscous_nm_per_rad_s = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_velocity = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_mode_display = np.full(NUM_JOINTS, self._drag_mode_csp, dtype=float)
        self._drag_statusword = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_error_code = np.zeros(NUM_JOINTS, dtype=float)
        self._drive_effort_actual = np.zeros(NUM_JOINTS, dtype=float)
        self._drive_motor_actual_current = np.zeros(NUM_JOINTS, dtype=float)
        self._drive_following_error_actual = np.zeros(NUM_JOINTS, dtype=float)
        self._drive_startup_snapshot_logged = False
        self._drive_pre_motion_snapshot_logged = False
        self._drag_last_mode_command = np.full(NUM_JOINTS, self._drag_mode_csp, dtype=float)
        self._drag_last_effort_command = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_torque_offset_command = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_effort_command_raw = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_torque_offset_command_raw = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_enable_set_command = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_disable_set_command = np.zeros(NUM_JOINTS, dtype=float)
        self._drag_last_transition_ts = 0.0
        self._drag_last_diag_log_ts = 0.0
        self._drag_joint_order = list(JOINT_NAMES)
        self._drag_config_path = Path(DRAG_MODE_CONFIG_PATH)
        self._drag_joint_models = list(DRAG_MODE_JOINT_MODELS)
        self._drag_model_rated_current_ma = {
            str(model): float(value)
            for model, value in zip(DRAG_MODE_MODEL_NAMES, DRAG_MODE_MODEL_RATED_CURRENT_MA)
        }
        self._drag_model_output_torque_constant = {
            str(model): float(value)
            for model, value in zip(DRAG_MODE_MODEL_NAMES, DRAG_MODE_MODEL_OUTPUT_TORQUE_CONSTANT_NM_PER_A)
        }
        self._drag_bias_model = KDLInverseDynamicsModel(
            urdf_path=self.urdf_path,
            base_link=BASE_LINK,
            tip_link=COLLISION_TIP_LINK,
            num_joints=NUM_JOINTS,
            logger=self.get_logger(),
            include_gravity=True,
        )
        self._drag_nograv_model = KDLInverseDynamicsModel(
            urdf_path=self.urdf_path,
            base_link=BASE_LINK,
            tip_link=COLLISION_TIP_LINK,
            num_joints=NUM_JOINTS,
            logger=self.get_logger(),
            include_gravity=False,
        )
        self._drag_mode_pub = None
        self._drag_effort_pub = None
        self._drag_torque_offset_pub = None
        self._drag_enable_set_pub = None
        self._drag_disable_set_pub = None
        self._drive_enable_set_pub = None
        self._drive_disable_set_pub = None
        self._drag_timer = None
        if self.runtime_adapter.supports_drag_mode:
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
            self._drag_enable_set_pub = self.create_publisher(
                Float64MultiArray,
                DRAG_MODE_ENABLE_SET_COMMAND_TOPIC,
                10,
            )
            self._drag_disable_set_pub = self.create_publisher(
                Float64MultiArray,
                DRAG_MODE_DISABLE_SET_COMMAND_TOPIC,
                10,
            )
            self._drive_enable_set_pub = self.create_publisher(
                Float64MultiArray,
                DRIVE_ENABLE_SET_COMMAND_TOPIC,
                10,
            )
            self._drive_disable_set_pub = self.create_publisher(
                Float64MultiArray,
                DRIVE_DISABLE_SET_COMMAND_TOPIC,
                10,
            )
            self.create_subscription(
                DynamicJointState,
                '/zeroerr/collision_monitor/state',
                self._drag_monitor_callback,
                10,
            )
            self.create_subscription(
                DynamicJointState,
                '/dynamic_joint_states',
                self._drag_drive_state_callback,
                10,
            )
            self.create_subscription(
                String,
                '/zeroerr/collision_monitor/config_state',
                self._drag_config_callback,
                10,
            )
            self.create_subscription(
                DynamicJointState,
                '/zeroerr/collision_monitor/state',
                self._collision_monitor_state_callback,
                10,
            )
            self.create_subscription(
                String,
                '/zeroerr/collision_monitor/config_state',
                self._collision_monitor_config_callback,
                10,
            )
            self._drag_timer = self.create_timer(self._drag_update_dt, self._drag_mode_step)
            self._load_persisted_drag_config()
            self.get_logger().info(
                f"[DragMode] Initialized real CST path (enabled={self._drag_enabled}, dt={self._drag_update_dt:.3f}s)"
            )
        else:
            self.get_logger().info("[DragMode] Disabled for this robot backend")

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

    def _publish_active_tool_visualization(self):
        """Publish the current active TCP as a TF frame and RViz marker."""
        if self.monitor is None:
            return
        pose = self.monitor.get_cartesian_position()
        source_pose = self.monitor.get_cartesian_source_position()
        if pose is None or len(pose) < 6:
            return

        try:
            T_tcp = TransformationUtils.pose_to_transform(pose[:6])
            q = TransformationUtils.matrix_to_quaternion(T_tcp[:3, :3])
            T_source = None
            if source_pose is not None and len(source_pose) >= 6:
                T_source = TransformationUtils.pose_to_transform(source_pose[:6])
        except Exception as exc:
            self.get_logger().warning(f"[ActiveTCP] Failed to build marker pose: {exc}")
            return

        now = self.get_clock().now().to_msg()
        self._publish_active_tcp_tf(now, T_tcp, q)
        self._publish_active_tcp_markers(now, T_tcp, q, T_source)

    def _publish_active_tcp_tf(self, stamp, T_tcp, quaternion):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = BASE_LINK
        transform.child_frame_id = "active_tcp"
        transform.transform.translation.x = float(T_tcp[0, 3])
        transform.transform.translation.y = float(T_tcp[1, 3])
        transform.transform.translation.z = float(T_tcp[2, 3])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.active_tcp_tf_broadcaster.sendTransform(transform)

    def _publish_active_tcp_markers(self, stamp, T_tcp, quaternion, T_source=None):
        marker_array = MarkerArray()
        origin = T_tcp[:3, 3]
        rotation = T_tcp[:3, :3]
        source_origin = T_source[:3, 3] if T_source is not None else None
        if source_origin is not None:
            marker_array.markers.append(self._make_source_sphere_marker(stamp, source_origin))
            marker_array.markers.append(self._make_source_to_tcp_marker(stamp, source_origin, origin))
        marker_array.markers.append(self._make_tcp_sphere_marker(stamp, origin, quaternion))
        marker_array.markers.extend(self._make_tcp_axis_markers(stamp, origin, rotation))
        marker_array.markers.append(self._make_tcp_text_marker(stamp, origin, source_origin))
        self.active_tool_marker_pub.publish(marker_array)

    def _base_marker(self, stamp, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = BASE_LINK
        marker.ns = "active_tcp"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=500_000_000)
        return marker

    def _make_tcp_sphere_marker(self, stamp, origin, quaternion) -> Marker:
        marker = self._base_marker(stamp, 0, Marker.SPHERE)
        marker.pose.position.x = float(origin[0])
        marker.pose.position.y = float(origin[1])
        marker.pose.position.z = float(origin[2])
        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])
        marker.scale.x = 0.018
        marker.scale.y = 0.018
        marker.scale.z = 0.018
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _make_source_sphere_marker(self, stamp, origin) -> Marker:
        marker = self._base_marker(stamp, 5, Marker.SPHERE)
        marker.pose.position.x = float(origin[0])
        marker.pose.position.y = float(origin[1])
        marker.pose.position.z = float(origin[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.012
        marker.scale.y = 0.012
        marker.scale.z = 0.012
        marker.color.r = 0.85
        marker.color.g = 0.85
        marker.color.b = 0.85
        marker.color.a = 1.0
        return marker

    def _make_source_to_tcp_marker(self, stamp, source_origin, tcp_origin) -> Marker:
        marker = self._base_marker(stamp, 6, Marker.LINE_STRIP)
        marker.points = [
            Point(x=float(source_origin[0]), y=float(source_origin[1]), z=float(source_origin[2])),
            Point(x=float(tcp_origin[0]), y=float(tcp_origin[1]), z=float(tcp_origin[2])),
        ]
        marker.scale.x = 0.003
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _make_tcp_axis_markers(self, stamp, origin, rotation) -> list[Marker]:
        colors = (
            (1.0, 0.1, 0.1),
            (0.1, 0.9, 0.1),
            (0.2, 0.4, 1.0),
        )
        markers = []
        for index, color in enumerate(colors):
            marker = self._base_marker(stamp, index + 1, Marker.LINE_STRIP)
            start = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
            end_vec = origin + rotation[:, index] * 0.06
            end = Point(x=float(end_vec[0]), y=float(end_vec[1]), z=float(end_vec[2]))
            marker.points = [start, end]
            marker.scale.x = 0.004
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            markers.append(marker)
        return markers

    def _make_tcp_text_marker(self, stamp, origin, source_origin=None) -> Marker:
        marker = self._base_marker(stamp, 4, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = float(origin[0])
        marker.pose.position.y = float(origin[1])
        marker.pose.position.z = float(origin[2] + 0.04)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.03
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        if source_origin is None:
            marker.text = f"{self.active_tool_name} active TCP"
        else:
            offset_mm = float(np.linalg.norm((origin - source_origin) * 1000.0))
            marker.text = f"{self.active_tool_name} active TCP ({offset_mm:.1f} mm)"
        return marker

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

    def _get_monitor_tcp_transform(self):
        """Return the transform RobotMonitor should apply on /cartesian_position."""
        return self.runtime_adapter.get_monitor_tcp_transform(self)

    def _build_registry_tool_transform(self, tool_name):
        offset = tool_registry[tool_name]
        xyz = offset[:3]
        rpy = offset[3:]

        T_tool = np.eye(4)
        T_tool[:3, :3] = TransformationUtils.euler_to_matrix(rpy)
        T_tool[:3, 3] = np.array(xyz) / 1000.0
        return T_tool

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

        registry_transform = self._build_registry_tool_transform(tool_name)
        return self.runtime_adapter.get_planning_tool_transform(self, registry_transform)

    def set_tool(self, tool_name):
        """
        Switch the active TCP/tool.
        Composes ee_link offset (from URDF) with tool offset (from registry).
        """
        if tool_name not in tool_registry:
            self.get_logger().warning(f"Tool {tool_name} not found in registry")
            return

        registry_transform = self._build_registry_tool_transform(tool_name)
        self.T_monitor_tool = registry_transform
        self.T_tool = self.runtime_adapter.get_planning_tool_transform(self, registry_transform)
        self.planner_context.T_tool = self.T_tool
        self.active_tool_name = tool_name

        # Monitor transform depends on the backend's /cartesian_position source frame.
        if self.monitor is not None:
            self.monitor.set_tcp_transform(self._get_monitor_tcp_transform())
            self.get_logger().info(f"Switched active tool to {tool_name}")
        else:
            self.get_logger().warning("RobotMonitor not initialized yet")

    def load_tcp_transform(self):
        if self.tcp_loaded:
            return  # Already loaded

        if self.tf_buffer.can_transform(WRIST_LINK, EE_LINK, rclpy.time.Time()):
            try:
                # Load the wrist3 -> ee_link fixed transform from TF/URDF.
                self.T_ee_link = self.get_tcp_transform(WRIST_LINK, EE_LINK)
                print(f"[RobotController] Loaded ee_link transform:\n{self.T_ee_link}")
                if self.active_tool_name in tool_registry:
                    registry_transform = self._build_registry_tool_transform(self.active_tool_name)
                    self.T_monitor_tool = registry_transform
                    self.T_tool = self.runtime_adapter.get_planning_tool_transform(self, registry_transform)
                    self.planner_context.T_tool = self.T_tool

                # Monitor receives backend-specific Cartesian source frames:
                # - ZeroErr/generic: ee_link pose
                # - Fairino: flange pose from native controller state
                self.monitor = RobotMonitor(
                    ros_node=self,
                    tcp_transform=self._get_monitor_tcp_transform(),
                    stable_update_rate_hz=50.0,
                )
                self.monitor.set_stable_update_callback(self._handle_monitor_update)
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
        if not self.is_drive_operation_enabled_for_motion():
            self.get_logger().error(
                f'[DriveEnable] Rejecting motion: {self.get_drive_enable_fault_reason()}'
            )
            return MOTION_ERROR_DRIVE_NOT_ENABLED
        if not self.is_hardware_ready_for_motion():
            self.get_logger().error(
                f'[EtherCAT] Rejecting motion: {self.get_hardware_fault_reason()}'
            )
            return MOTION_ERROR_HARDWARE_NOT_READY
        if not self.is_motion_stack_ready():
            self.get_logger().error(
                f'[Motion] Rejecting motion: {self.get_motion_stack_fault_reason()}'
            )
            return MOTION_ERROR_HARDWARE_NOT_READY
        return self._motion.execute(strategy, queue_if_busy=queue_if_busy)

    def send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                            tool_transform=None, queue_if_busy=True, avoid_collisions=None):
        from motion.strategies import SingleTargetStrategy
        from config import resolve_avoid_collisions
        return self.execute(SingleTargetStrategy(
            x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
            tool_transform=tool_transform,
            avoid_collisions=resolve_avoid_collisions(avoid_collisions)), queue_if_busy=queue_if_busy)

    def _collision_monitor_config_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        thresholds = payload.get("following_error_thresholds")
        if not isinstance(thresholds, list):
            return
        try:
            threshold_array = np.array(thresholds[:NUM_JOINTS], dtype=float)
        except Exception:
            return
        if threshold_array.size != NUM_JOINTS:
            return
        with self._collision_fault_lock:
            self._collision_following_error_thresholds = threshold_array

    def _collision_monitor_state_callback(self, msg: DynamicJointState):
        if not self._collision_monitor_fault_enabled:
            return

        joint_index = {name: idx for idx, name in enumerate(JOINT_NAMES)}

        with self._collision_fault_lock:
            if self._collision_motion_fault:
                return
            thresholds = self._collision_following_error_thresholds.copy()

        for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
            index = joint_index.get(joint_name)
            if index is None:
                continue
            following_error = None
            external_torque = None
            torque_sensor = None
            contact_latched = False
            dynamics_latched = False

            for name, value in zip(interface_value.interface_names, interface_value.values):
                if name == 'following_error_actual' and np.isfinite(value):
                    following_error = abs(float(value))
                elif name == 'external_torque' and np.isfinite(value):
                    external_torque = abs(float(value))
                elif name == 'torque_sensor' and np.isfinite(value):
                    torque_sensor = float(value)
                elif name == 'contact_latched':
                    contact_latched = bool(value >= 0.5)
                elif name == 'dynamics_latched':
                    dynamics_latched = bool(value >= 0.5)

            if not (contact_latched or dynamics_latched):
                continue

            threshold = float(thresholds[index]) if index < thresholds.size else np.nan

            if not np.isfinite(threshold):
                threshold = float("nan")

            collision_type = 'dynamics_latched' if dynamics_latched else 'contact_latched'
            context = []
            if following_error is not None:
                if np.isfinite(threshold):
                    context.append(f'following_error={following_error:.1f} threshold={threshold:.1f}')
                else:
                    context.append(f'following_error={following_error:.1f}')
            if external_torque is not None:
                context.append(f'external_torque={external_torque:.2f}')
            if torque_sensor is not None:
                context.append(f'torque_sensor={torque_sensor:.2f}')
            reason = f'Collision detected on {joint_name}: {collision_type}=true'
            if context:
                reason += ' | ' + ', '.join(context)
            self._trip_collision_motion_fault(reason)

            return

    def _trip_collision_motion_fault(self, reason: str):
        with self._collision_fault_lock:
            if self._collision_motion_fault:
                return
            self._collision_motion_fault = True
            self._collision_fault_reason = reason

        self.get_logger().error(f'[CollisionMonitor] {reason}')
        try:
            stop_result = self.stop_motion()
            self.get_logger().error(
                f'[CollisionMonitor] Motion stopped, queue cleared, and new motion blocked | stop_result={stop_result}'
            )
        except Exception as exc:
            self.get_logger().error(
                f'[CollisionMonitor] Failed to stop motion after collision latch: {exc}'
            )

    def _controller_status_callback(self, msg):
        """Monitor controller action status for side effects only.

        Normal motion lifecycle cleanup is owned by the trajectory result callback.
        Clearing active goal state here races with the async result future and makes
        successful completions look like stale/cancelled goals.
        """
        return

    def joint_state_callback(self, msg):
        """Process joint states and store for trajectory planning."""
        if len(msg.effort) >= NUM_JOINTS:
            with self._drag_lock:
                self._drive_effort_actual = np.array(msg.effort[:NUM_JOINTS], dtype=float)

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
                    self.latest_data = data

            # Send data to UI
            # self.ui_callback(data)

        self._maybe_log_drive_state_snapshot('startup')

    def get_latest_data(self):
        """Return a copy of the latest joint/cartesian data."""
        data = self.state_store.get_latest_data()
        if data is not None:
            return data
        if self.monitor is not None:
            return self.monitor.get_latest_data()
        return None

    def _handle_monitor_update(self, data):
        """Persist RobotMonitor stable snapshots into the shared planner/status store."""
        if data is None:
            return
        self.latest_data = data
        cartesian = data.get('cartesian')
        if cartesian is not None:
            self.prev_cartesian = cartesian

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

    def stage_pending_path(self, trajectory, vel_scaling, acc_scaling, trajectory_optimizer_name=None):
        self.planner_context.stage_pending_path(
            trajectory,
            vel_scaling,
            acc_scaling,
            trajectory_optimizer_name=trajectory_optimizer_name,
        )

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
        self.get_logger().error(
            '[DragMode] Enable rejected: drag mode is disabled because it is not fully implemented and tested'
        )
        return {
            "enabled": False,
            "stopped": False,
            "mode": "disabled",
            "state": "DISABLED_FOR_SAFETY",
            "controller_switch_ok": False,
        }

    def disable_drag_mode(self) -> dict:
        if not self.runtime_adapter.supports_drag_mode:
            return {
                "enabled": False,
                "stopped": False,
                "mode": "unsupported",
                "state": "UNSUPPORTED",
                "controller_switch_ok": False,
            }
        with self._drag_lock:
            self._drag_enabled = False
            self._drag_last_transition_ts = time.monotonic()
            self._drag_last_diag_log_ts = 0.0
        stop_result = self.stop_motion()
        self.get_logger().info(f'[DragMode] Disable requested | stop_result={stop_result}')
        self._publish_drag_commands(
            mode_command=np.full(NUM_JOINTS, self._drag_mode_csp, dtype=float),
            effort_command=np.zeros(NUM_JOINTS, dtype=float),
            torque_offset_command=np.zeros(NUM_JOINTS, dtype=float),
            enable_set_command=np.zeros(NUM_JOINTS, dtype=float),
            disable_set_command=np.zeros(NUM_JOINTS, dtype=float),
        )
        switch_ok = self._set_drag_controller_ownership(False)
        self._send_hold_position_trajectory(reason='drag mode disabled')
        self.get_logger().info(
            f"[DragMode] Disable result | enabled=False controller_switch_ok={bool(switch_ok)}"
        )
        return {
            "enabled": False,
            "stopped": bool(isinstance(stop_result, dict) and stop_result.get("stopped", False)),
            "mode": "csp",
            "state": "DISABLING" if switch_ok else "ERROR",
            "controller_switch_ok": bool(switch_ok),
        }

    def set_drive_operation_enabled(self, enabled: bool) -> dict:
        if not self.runtime_adapter.supports_drag_mode:
            return {
                "enabled": False,
                "requested_enabled": bool(enabled),
                "state": "UNSUPPORTED",
                "controller_switch_ok": False,
            }
        if enabled and not self.is_hardware_ready_for_motion():
            with self._drive_enable_lock:
                self._drive_operation_enabled_requested = False
            reason = self.get_hardware_fault_reason()
            self.get_logger().error(f'[DriveEnable] Enable rejected: {reason}')
            return {
                "success": False,
                "enabled": False,
                "requested_enabled": False,
                "state": "HARDWARE_NOT_READY",
                "controller_switch_ok": False,
                "error": reason,
            }
        switch_ok = self._ensure_drive_set_controllers_active()
        if not switch_ok:
            return {
                "enabled": False,
                "requested_enabled": bool(enabled),
                "state": "ERROR",
                "controller_switch_ok": False,
                "error": "failed to activate enable_set/disable_set controllers",
            }

        if enabled:
            self._send_hold_position_trajectory(reason='drive enable')
            time.sleep(0.25)

        pulse = np.ones(NUM_JOINTS, dtype=float)
        zeros = np.zeros(NUM_JOINTS, dtype=float)
        enable_msg = Float64MultiArray()
        disable_msg = Float64MultiArray()
        if enabled:
            enable_msg.data = pulse.tolist()
            disable_msg.data = zeros.tolist()
        else:
            enable_msg.data = zeros.tolist()
            disable_msg.data = pulse.tolist()
        self._drive_enable_set_pub.publish(enable_msg)
        self._drive_disable_set_pub.publish(disable_msg)
        time.sleep(0.05)
        enable_msg.data = zeros.tolist()
        disable_msg.data = zeros.tolist()
        self._drive_enable_set_pub.publish(enable_msg)
        self._drive_disable_set_pub.publish(disable_msg)
        with self._drive_enable_lock:
            self._drive_operation_enabled_requested = bool(enabled)
        self.get_logger().info(
            f"[DriveEnable] {'Enable' if enabled else 'Disable'} operation requested via enable_set/disable_set"
        )
        return {
            "success": True,
            "requested_enabled": bool(enabled),
            "state": "ENABLE_REQUESTED" if enabled else "DISABLE_REQUESTED",
            "controller_switch_ok": True,
            "mode": "csp",
        }

    def is_drive_operation_enabled_for_motion(self) -> bool:
        with self._drive_enable_lock:
            requested_enabled = bool(self._drive_operation_enabled_requested)
        if not requested_enabled:
            return False
        with self._drag_lock:
            statusword = [int(round(value)) for value in self._drag_statusword.tolist()]
        if len(statusword) < NUM_JOINTS:
            return False
        return all(
            self._decode_statusword_state(value) == 'operation_enabled'
            for value in statusword[:NUM_JOINTS]
        )

    def get_drive_enable_fault_reason(self) -> str:
        with self._drive_enable_lock:
            requested_enabled = bool(self._drive_operation_enabled_requested)
        if not requested_enabled:
            return "drive operation is not enabled; call POST /drive/enable before motion"
        with self._drag_lock:
            statusword = [int(round(value)) for value in self._drag_statusword.tolist()]
        statusword_state = [
            self._decode_statusword_state(value)
            for value in statusword[:NUM_JOINTS]
        ]
        return (
            "drive enable was requested, but not all drives report operation_enabled "
            f"(status_state={statusword_state})"
        )

    def get_drive_operation_status(self) -> dict:
        with self._drive_enable_lock:
            requested_enabled = bool(self._drive_operation_enabled_requested)
        with self._drag_lock:
            statusword = [int(round(value)) for value in self._drag_statusword.tolist()]
        statusword_state = [
            self._decode_statusword_state(value)
            for value in statusword[:NUM_JOINTS]
        ]
        actual_enabled = (
            len(statusword_state) == NUM_JOINTS
            and all(state == 'operation_enabled' for state in statusword_state)
        )
        return {
            "success": True,
            "requested_enabled": requested_enabled,
            "actual_enabled": actual_enabled,
            "motion_allowed_by_drive_enable": requested_enabled and actual_enabled,
            "state": "OPERATION_ENABLED" if actual_enabled else (
                "ENABLE_REQUESTED" if requested_enabled else "DISABLED"
            ),
            "statusword": statusword,
            "status_state": statusword_state,
        }

    def get_drag_mode_status(self) -> dict:
        if not self.runtime_adapter.supports_drag_mode:
            return {
                "enabled": False,
                "active_mode": "unsupported",
                "state": "UNSUPPORTED",
            }
        with self._drag_lock:
            enabled = self._drag_enabled
            mode_display = self._drag_mode_display.tolist()
            statusword = self._drag_statusword.tolist()
            error_code = self._drag_error_code.tolist()
            last_mode = self._drag_last_mode_command.tolist()
            last_effort = self._drag_last_effort_command.tolist()
            last_offset = self._drag_last_torque_offset_command.tolist()
            last_effort_raw = self._drag_last_effort_command_raw.tolist()
            last_offset_raw = self._drag_last_torque_offset_command_raw.tolist()
            return {
                "enabled": enabled,
                "active_mode": "cst" if enabled else "csp",
                "requested_mode_value": self._drag_mode_cst if enabled else self._drag_mode_csp,
                "mode_display": mode_display,
                "statusword": statusword,
                "error_code": error_code,
                "mode_match": all(abs(value - (self._drag_mode_cst if enabled else self._drag_mode_csp)) < 0.5 for value in mode_display),
                "settle_timeout_s": self._drag_mode_settle_timeout_s,
                "manipulator_controller_expected_active": not enabled,
                "compensation_scale": self._drag_compensation_scale,
                "joint_compensation_scale": self._drag_joint_compensation_scale.tolist(),
                "damping_nm_per_rad_s": self._drag_damping_nm_per_rad_s.tolist(),
                "max_effort_nm": self._drag_max_effort_nm.tolist(),
                "max_torque_offset_nm": self._drag_max_torque_offset_nm.tolist(),
                "drive_command_unit": "per_thousand_of_rated_current",
                "drive_command_objects": {
                    "target_torque": "0x6071",
                    "torque_offset": "0x60B2",
                },
                "external_tau_nm": self._drag_external_tau.tolist(),
                "expected_tau_nm": self._drag_expected_tau.tolist(),
                "gravity_tau_nm": self._drag_gravity_tau.tolist(),
                "non_gravity_tau_nm": self._drag_non_gravity_tau.tolist(),
                "friction_tau_nm": self._drag_friction_tau.tolist(),
                "last_mode_command": last_mode,
                "last_effort_command_nm": last_effort,
                "last_torque_offset_command_nm": last_offset,
                "last_effort_command_raw": last_effort_raw,
                "last_torque_offset_command_raw": last_offset_raw,
                "last_enable_set_command": self._drag_last_enable_set_command.tolist(),
                "last_disable_set_command": self._drag_last_disable_set_command.tolist(),
            }

    def get_drag_mode_config(self) -> dict:
        if not self.runtime_adapter.supports_drag_mode:
            return {"supported": False}
        with self._drag_lock:
            return {
                "compensation_scale": float(self._drag_compensation_scale),
                "joint_compensation_scale": self._drag_joint_compensation_scale.tolist(),
                "damping_nm_per_rad_s": self._drag_damping_nm_per_rad_s.tolist(),
                "max_effort_nm": self._drag_max_effort_nm.tolist(),
                "max_torque_offset_nm": self._drag_max_torque_offset_nm.tolist(),
                "settle_timeout_s": float(self._drag_mode_settle_timeout_s),
                "disable_pulse_s": float(self._drag_disable_pulse_s),
                "enable_pulse_s": float(self._drag_enable_pulse_s),
                "drive_command_unit": "per_thousand_of_rated_current",
                "drive_command_objects": {
                    "target_torque": "0x6071",
                    "torque_offset": "0x60B2",
                },
            }

    def update_drag_mode_config(self, payload: dict) -> dict:
        if not self.runtime_adapter.supports_drag_mode:
            raise ValueError("Drag mode is not supported for this robot backend")
        if not isinstance(payload, dict):
            raise ValueError("Drag config payload must be an object")

        with self._drag_lock:
            if "compensation_scale" in payload:
                self._drag_compensation_scale = float(payload["compensation_scale"])
            if "joint_compensation_scale" in payload:
                joint_scale = np.array(payload["joint_compensation_scale"], dtype=float)
                if joint_scale.size != NUM_JOINTS:
                    raise ValueError(f"joint_compensation_scale must contain {NUM_JOINTS} values")
                self._drag_joint_compensation_scale = joint_scale
            if "settle_timeout_s" in payload:
                self._drag_mode_settle_timeout_s = float(payload["settle_timeout_s"])
            if "disable_pulse_s" in payload:
                self._drag_disable_pulse_s = float(payload["disable_pulse_s"])
            if "enable_pulse_s" in payload:
                self._drag_enable_pulse_s = float(payload["enable_pulse_s"])

            if "damping_nm_per_rad_s" in payload:
                damping = np.array(payload["damping_nm_per_rad_s"], dtype=float)
                if damping.size != NUM_JOINTS:
                    raise ValueError(f"damping_nm_per_rad_s must contain {NUM_JOINTS} values")
                self._drag_damping_nm_per_rad_s = damping

            if "max_effort_nm" in payload:
                max_effort = np.array(payload["max_effort_nm"], dtype=float)
                if max_effort.size != NUM_JOINTS:
                    raise ValueError(f"max_effort_nm must contain {NUM_JOINTS} values")
                self._drag_max_effort_nm = max_effort

            if "max_torque_offset_nm" in payload:
                max_offset = np.array(payload["max_torque_offset_nm"], dtype=float)
                if max_offset.size != NUM_JOINTS:
                    raise ValueError(f"max_torque_offset_nm must contain {NUM_JOINTS} values")
                self._drag_max_torque_offset_nm = max_offset

        updated = self.get_drag_mode_config()
        try:
            self._drag_config_path.parent.mkdir(parents=True, exist_ok=True)
            self._drag_config_path.write_text(
                json.dumps(updated, indent=2, sort_keys=True),
                encoding="utf-8",
            )
        except Exception as exc:
            self.get_logger().warning(f"[DragMode] Failed to persist config to {self._drag_config_path}: {exc}")
        self.get_logger().info(f"[DragMode] Runtime config updated | {updated}")
        return updated

    def _load_persisted_drag_config(self) -> None:
        path = self._drag_config_path
        if not path.exists():
            return
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.update_drag_mode_config(payload)
            self.get_logger().info(f"[DragMode] Loaded persisted config from {path}")
        except Exception as exc:
            self.get_logger().warning(f"[DragMode] Failed to load persisted config from {path}: {exc}")

    def _set_drag_controller_ownership(self, drag_enabled: bool) -> bool:
        controller_states = self._get_controller_states()
        if controller_states is None:
            return False
        self.get_logger().info(
            f"[DragMode] Controller states before switch: {controller_states}"
        )

        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('[DragMode] /controller_manager/switch_controller not available')
            return False

        request = SwitchController.Request()
        request.strictness = 2
        request.activate_asap = True
        request.timeout = Duration(sec=2, nanosec=0)
        if drag_enabled:
            request.activate_controllers = [
                name for name in self._DRAG_CONTROLLER_NAMES
                if controller_states.get(name) != 'active'
            ]
            request.deactivate_controllers = (
                [self._MANIPULATOR_CONTROLLER_NAME]
                if controller_states.get(self._MANIPULATOR_CONTROLLER_NAME) == 'active'
                else []
            )
        else:
            request.activate_controllers = []
            if controller_states.get(self._MANIPULATOR_CONTROLLER_NAME) != 'active':
                request.activate_controllers.append(self._MANIPULATOR_CONTROLLER_NAME)
            request.deactivate_controllers = [
                name for name in self._DRAG_CONTROLLER_NAMES
                if controller_states.get(name) == 'active'
            ]

        if not request.activate_controllers and not request.deactivate_controllers:
            self.get_logger().info('[DragMode] Controller ownership already in desired state')
            return True

        self.get_logger().info(
            f"[DragMode] switch_controller request | drag_enabled={drag_enabled} "
            f"activate={request.activate_controllers} deactivate={request.deactivate_controllers}"
        )

        future = self.switch_controller_client.call_async(request)
        response = self._wait_for_service_future(future, timeout_s=3.0)
        if response is None:
            self.get_logger().error('[DragMode] Timed out switching controllers')
            return False
        if not response.ok:
            self.get_logger().error('[DragMode] Controller manager rejected drag controller switch')
            return False
        updated_states = self._get_controller_states()
        if updated_states is not None:
            self.get_logger().info(
                f"[DragMode] Controller states after switch: {updated_states}"
            )
        self.get_logger().info(
            f"[DragMode] Controller ownership set for {'drag' if drag_enabled else 'trajectory'} mode"
        )
        return True

    def _ensure_drive_set_controllers_active(self) -> bool:
        controller_states = self._get_controller_states()
        if controller_states is None:
            return False
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('[DriveEnable] /controller_manager/switch_controller not available')
            return False

        request = SwitchController.Request()
        request.strictness = 2
        request.activate_asap = True
        request.timeout = Duration(sec=2, nanosec=0)
        request.activate_controllers = [
            name for name in self._DRIVE_SET_CONTROLLER_NAMES
            if controller_states.get(name) != 'active'
        ]
        request.deactivate_controllers = []

        if not request.activate_controllers:
            return True

        self.get_logger().info(
            f"[DriveEnable] Activating set controllers: {request.activate_controllers}"
        )
        future = self.switch_controller_client.call_async(request)
        response = self._wait_for_service_future(future, timeout_s=3.0)
        if response is None:
            self.get_logger().error('[DriveEnable] Timed out activating set controllers')
            return False
        if not response.ok:
            self.get_logger().error('[DriveEnable] Controller manager rejected set controller activation')
            return False
        return True

    def _get_controller_states(self) -> dict[str, str] | None:
        if not self.list_controllers_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('[DragMode] /controller_manager/list_controllers not available')
            return None
        future = self.list_controllers_client.call_async(ListControllers.Request())
        response = self._wait_for_service_future(future, timeout_s=3.0)
        if response is None:
            self.get_logger().error('[DragMode] Timed out listing controllers')
            return None
        return {controller.name: controller.state for controller in response.controller}

    def _wait_for_service_future(self, future, timeout_s: float):
        event = threading.Event()
        result = {"value": None, "error": None}

        def _done_callback(done_future):
            try:
                result["value"] = done_future.result()
            except Exception as exc:
                result["error"] = exc
            finally:
                event.set()

        future.add_done_callback(_done_callback)
        if not event.wait(timeout_s):
            return None
        if result["error"] is not None:
            self.get_logger().error(f'[DragMode] Service future failed: {result["error"]}')
            return None
        return result["value"]

    def is_hardware_ready_for_motion(self) -> bool:
        with self._ethercat_fault_lock:
            return not self._ethercat_motion_fault

    def get_hardware_fault_reason(self) -> str:
        with self._ethercat_fault_lock:
            return self._ethercat_fault_reason or 'EtherCAT hardware fault'

    def is_motion_stack_ready(self) -> bool:
        with self._collision_fault_lock:
            if self._collision_motion_fault:
                return False
        if self.current_joint_state is None:
            return False
        if self.prev_cartesian is None or len(self.prev_cartesian) < 6:
            return False
        if not self.wait_for_cartesian_path_service(timeout_sec=0.05):
            return False
        ik_client = self.get_ik_client()
        if ik_client is None or not ik_client.wait_for_service(timeout_sec=0.05):
            return False
        fk_client = self.get_fk_client()
        if fk_client is None or not fk_client.wait_for_service(timeout_sec=0.05):
            return False
        state_validity_client = self.get_state_validity_client()
        if state_validity_client is None or not state_validity_client.wait_for_service(timeout_sec=0.05):
            return False
        if not self.controller_client.wait_for_server(timeout_sec=0.05):
            return False
        return True

    def get_motion_stack_fault_reason(self) -> str:
        with self._collision_fault_lock:
            if self._collision_motion_fault:
                return self._collision_fault_reason or 'collision monitor latched a motion fault'
        if self.current_joint_state is None:
            return 'joint_states not available yet'
        if self.prev_cartesian is None or len(self.prev_cartesian) < 6:
            return 'current Cartesian pose not available yet'
        if not self.wait_for_cartesian_path_service(timeout_sec=0.05):
            return 'MoveIt compute_cartesian_path service not available'
        ik_client = self.get_ik_client()
        if ik_client is None or not ik_client.wait_for_service(timeout_sec=0.05):
            return 'MoveIt IK service not available'
        fk_client = self.get_fk_client()
        if fk_client is None or not fk_client.wait_for_service(timeout_sec=0.05):
            return 'MoveIt FK service not available'
        state_validity_client = self.get_state_validity_client()
        if state_validity_client is None or not state_validity_client.wait_for_service(timeout_sec=0.05):
            return 'MoveIt state validity service not available'
        if not self.controller_client.wait_for_server(timeout_sec=0.05):
            return 'FollowJointTrajectory action server not available'
        return 'motion stack not ready'

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

    def _drag_drive_state_callback(self, msg: DynamicJointState):
        joint_index = {name: idx for idx, name in enumerate(self._drag_joint_order)}
        statusword = np.zeros(NUM_JOINTS, dtype=float)
        error_code = np.zeros(NUM_JOINTS, dtype=float)
        motor_actual_current = np.zeros(NUM_JOINTS, dtype=float)
        following_error_actual = np.zeros(NUM_JOINTS, dtype=float)
        mode_display = self._drag_mode_display.copy()
        for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
            index = joint_index.get(joint_name)
            if index is None:
                continue
            for name, value in zip(interface_value.interface_names, interface_value.values):
                if name == 'statusword' and np.isfinite(value):
                    statusword[index] = float(value)
                elif name == 'error_code' and np.isfinite(value):
                    error_code[index] = float(value)
                elif name == 'motor_actual_current' and np.isfinite(value):
                    motor_actual_current[index] = float(value)
                elif name == 'following_error_actual' and np.isfinite(value):
                    following_error_actual[index] = float(value)
                elif name == 'mode_display' and np.isfinite(value):
                    mode_display[index] = float(value)
        with self._drag_lock:
            self._drag_statusword = statusword
            self._drag_error_code = error_code
            self._drive_motor_actual_current = motor_actual_current
            self._drive_following_error_actual = following_error_actual
            self._drag_mode_display = mode_display

    def _format_drive_state_snapshot(self, label: str):
        with self._drag_lock:
            statusword = [int(round(value)) for value in self._drag_statusword.tolist()]
            error_code = [int(round(value)) for value in self._drag_error_code.tolist()]
            mode_display = [int(round(value)) for value in self._drag_mode_display.tolist()]
            effort = [round(value, 3) for value in self._drive_effort_actual.tolist()]
            motor_current = [round(value, 3) for value in self._drive_motor_actual_current.tolist()]
            following_error = [round(value, 3) for value in self._drive_following_error_actual.tolist()]
        statusword_bits = [self._decode_statusword_bits(value) for value in statusword]
        statusword_state = [self._decode_statusword_state(value) for value in statusword]
        error_text = [self._decode_drive_error_code(value) for value in error_code]
        return (
            f'[DriveState] {label}: '
            f'statusword={statusword} '
            f'status_state={statusword_state} '
            f'status_bits={statusword_bits} '
            f'error_code={[f"0x{value:04X}" for value in error_code]} '
            f'error_text={error_text} '
            f'mode_display={mode_display} '
            f'effort={effort} '
            f'motor_actual_current={motor_current} '
            f'following_error_actual={following_error}'
        )

    def _decode_statusword_bits(self, statusword: int) -> str:
        flags = (
            ("rtso", 0),
            ("so", 1),
            ("oe", 2),
            ("f", 3),
            ("ve", 4),
            ("qs", 5),
            ("sod", 6),
            ("w", 7),
            ("rm", 9),
            ("tr", 10),
            ("ila", 11),
        )
        active = [name for name, bit in flags if statusword & (1 << bit)]
        return "+".join(active) if active else "-"

    def _decode_statusword_state(self, statusword: int) -> str:
        state_code = statusword & 0x006F
        state_map = {
            0x0000: 'not_ready_to_switch_on',
            0x0040: 'switch_on_disabled',
            0x0021: 'ready_to_switch_on',
            0x0023: 'switched_on',
            0x0027: 'operation_enabled',
            0x0007: 'quick_stop_active',
            0x000F: 'fault_reaction_active',
            0x0008: 'fault',
        }
        return state_map.get(state_code, f'unknown(0x{state_code:04X})')

    def _decode_drive_error_code(self, error_code: int) -> str:
        return ZEROERR_DRIVE_ERROR_CODES.get(error_code, "unknown drive error code")

    def _maybe_log_drive_state_snapshot(self, label: str):
        with self._drag_lock:
            if label == 'startup':
                if self._drive_startup_snapshot_logged:
                    return
                self._drive_startup_snapshot_logged = True
            elif label == 'before_first_motion':
                if self._drive_pre_motion_snapshot_logged:
                    return
                self._drive_pre_motion_snapshot_logged = True
            else:
                return
        self.get_logger().info(self._format_drive_state_snapshot(label))

    def log_drive_state_before_first_motion(self):
        self._maybe_log_drive_state_snapshot('before_first_motion')

    def _drag_mode_step(self):
        with self._drag_lock:
            drag_enabled = self._drag_enabled
            target_mode = self._drag_mode_cst if drag_enabled else self._drag_mode_csp
            mode_display = self._drag_mode_display.copy()
            statusword = self._drag_statusword.copy()
            error_code = self._drag_error_code.copy()
            last_diag_log_ts = self._drag_last_diag_log_ts
            transition_ts = self._drag_last_transition_ts

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
        non_gravity_tau = self._drag_nograv_model.compute_bias_torque(positions, velocities)
        gravity_tau = expected_tau - non_gravity_tau

        with self._drag_lock:
            self._drag_velocity = velocities
            self._drag_expected_tau = expected_tau
            self._drag_gravity_tau = gravity_tau
            self._drag_non_gravity_tau = non_gravity_tau
            self._drag_friction_tau = friction_tau

        torque_offset = (
            self._drag_compensation_scale
            * self._drag_joint_compensation_scale
            * (expected_tau + friction_tau)
        )
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

        enable_set_command = np.zeros(NUM_JOINTS, dtype=float)
        disable_set_command = np.zeros(NUM_JOINTS, dtype=float)
        now = time.monotonic()
        elapsed = max(0.0, now - transition_ts)
        if drag_enabled:
            if elapsed < self._drag_disable_pulse_s:
                disable_set_command.fill(1.0)
                torque_offset = np.zeros(NUM_JOINTS, dtype=float)
                effort_command = np.zeros(NUM_JOINTS, dtype=float)
            elif elapsed < (self._drag_disable_pulse_s + self._drag_enable_pulse_s):
                enable_set_command.fill(1.0)
                torque_offset = np.zeros(NUM_JOINTS, dtype=float)
                effort_command = np.zeros(NUM_JOINTS, dtype=float)

        self._publish_drag_commands(
            mode_command=np.full(NUM_JOINTS, target_mode, dtype=float),
            effort_command=effort_command,
            torque_offset_command=torque_offset,
            enable_set_command=enable_set_command,
            disable_set_command=disable_set_command,
        )
        effort_command_raw = self._drag_nm_to_drive_units(effort_command)
        torque_offset_command_raw = self._drag_nm_to_drive_units(torque_offset)

        if now - last_diag_log_ts >= 1.0:
            mode_match = bool(np.all(np.abs(mode_display - target_mode) < 0.5))
            if drag_enabled and not mode_match:
                self.get_logger().warning(
                    '[DragMode] Mode mismatch while drag enabled | '
                    f'requested={target_mode} mode_display={mode_display.tolist()} '
                    f'statusword={statusword.tolist()} error_code={error_code.tolist()} '
                    f'enable_set={enable_set_command.tolist()} disable_set={disable_set_command.tolist()} '
                    f'gravity_tau={gravity_tau.tolist()} non_gravity_tau={non_gravity_tau.tolist()} '
                    f'effort_cmd={effort_command.tolist()} effort_raw={effort_command_raw.tolist()} '
                    f'offset_cmd={torque_offset.tolist()} offset_raw={torque_offset_command_raw.tolist()}'
                )
            else:
                # Keep drag diagnostics quiet during normal operation; mismatch/fault cases above
                # still log as warnings with full context.
                pass
            with self._drag_lock:
                self._drag_last_diag_log_ts = now

    def _compute_drag_friction_torque(self, velocities: np.ndarray) -> np.ndarray:
        coulomb = getattr(self, "_drag_friction_coulomb_nm", np.zeros(NUM_JOINTS, dtype=float))
        viscous = getattr(self, "_drag_friction_viscous_nm_per_rad_s", np.zeros(NUM_JOINTS, dtype=float))
        return np.sign(velocities) * coulomb + viscous * velocities

    def _publish_drag_commands(
        self,
        mode_command: np.ndarray,
        effort_command: np.ndarray,
        torque_offset_command: np.ndarray,
        enable_set_command: np.ndarray,
        disable_set_command: np.ndarray,
    ) -> None:
        mode_msg = Float64MultiArray()
        mode_msg.data = mode_command.tolist()
        effort_command_raw = self._drag_nm_to_drive_units(effort_command)
        effort_msg = Float64MultiArray()
        effort_msg.data = effort_command_raw.tolist()
        torque_offset_command_raw = self._drag_nm_to_drive_units(torque_offset_command)
        offset_msg = Float64MultiArray()
        offset_msg.data = torque_offset_command_raw.tolist()
        enable_msg = Float64MultiArray()
        enable_msg.data = enable_set_command.tolist()
        disable_msg = Float64MultiArray()
        disable_msg.data = disable_set_command.tolist()

        self._drag_mode_pub.publish(mode_msg)
        self._drag_effort_pub.publish(effort_msg)
        self._drag_torque_offset_pub.publish(offset_msg)
        self._drag_enable_set_pub.publish(enable_msg)
        self._drag_disable_set_pub.publish(disable_msg)

        with self._drag_lock:
            self._drag_last_mode_command = mode_command.copy()
            self._drag_last_effort_command = effort_command.copy()
            self._drag_last_torque_offset_command = torque_offset_command.copy()
            self._drag_last_effort_command_raw = effort_command_raw.copy()
            self._drag_last_torque_offset_command_raw = torque_offset_command_raw.copy()
            self._drag_last_enable_set_command = enable_set_command.copy()
            self._drag_last_disable_set_command = disable_set_command.copy()

    def _drag_nm_to_drive_units(self, torque_nm: np.ndarray) -> np.ndarray:
        raw = np.zeros(NUM_JOINTS, dtype=float)
        for index in range(min(NUM_JOINTS, len(self._drag_joint_models))):
            model = self._drag_joint_models[index]
            rated_current_ma = self._drag_model_rated_current_ma.get(model)
            output_torque_constant = self._drag_model_output_torque_constant.get(model)
            if (
                rated_current_ma is None
                or output_torque_constant is None
                or rated_current_ma <= 0.0
                or output_torque_constant <= 0.0
            ):
                raw[index] = 0.0
                continue
            raw_per_nm = 1_000_000.0 / (rated_current_ma * output_torque_constant)
            raw[index] = torque_nm[index] * raw_per_nm
        return raw

    def _send_hold_position_trajectory(self, reason: str = 'hold position') -> bool:
        joint_state = self.current_joint_state
        if joint_state is None:
            self.get_logger().warning(f'[HoldPosition] Cannot send hold trajectory for {reason}: no joint state yet')
            return False
        state_names = list(getattr(joint_state, 'name', []) or [])
        state_positions = list(getattr(joint_state, 'position', []) or [])
        if not state_names or len(state_names) != len(state_positions):
            self.get_logger().warning(
                f'[HoldPosition] Cannot send hold trajectory for {reason}: invalid joint state '
                f'names={len(state_names)} positions={len(state_positions)}'
            )
            return False
        position_by_name = {
            name: position
            for name, position in zip(state_names, state_positions)
        }
        missing = [name for name in JOINT_NAMES if name not in position_by_name]
        if missing:
            self.get_logger().warning(
                f'[HoldPosition] Cannot send hold trajectory for {reason}: missing joints {missing}'
            )
            return False
        positions = [float(position_by_name[name]) for name in JOINT_NAMES]
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        traj.header.stamp = self.get_clock().now().to_msg()

        start_pt = JointTrajectoryPoint()
        start_pt.positions = list(positions)
        start_pt.velocities = [0.0] * NUM_JOINTS
        start_pt.accelerations = [0.0] * NUM_JOINTS
        start_pt.time_from_start = Duration(sec=0, nanosec=0)

        end_pt = JointTrajectoryPoint()
        end_pt.positions = list(positions)
        end_pt.velocities = [0.0] * NUM_JOINTS
        end_pt.accelerations = [0.0] * NUM_JOINTS
        end_pt.time_from_start = Duration(sec=0, nanosec=200_000_000)

        traj.points = [start_pt, end_pt]
        self.get_logger().warning(
            f'[HoldPosition] Sending hold trajectory for {reason}: '
            f'positions={[round(value, 6) for value in positions]}'
        )
        self.trajectory_executor.send_trajectory_to_controller(traj)
        return True

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
                with self._drive_enable_lock:
                    self._drive_operation_enabled_requested = False
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
        if faulted:
            self._maybe_attempt_ethercat_recovery(reason)

    def _maybe_attempt_ethercat_recovery(self, reason: str):
        if not bool(ETHERCAT_RECOVERY_ENABLED):
            return
        now = time.monotonic()
        min_interval = max(float(ETHERCAT_RECOVERY_MIN_INTERVAL_S), 0.1)
        with self._ethercat_fault_lock:
            if now - self._ethercat_last_recovery_attempt_ts < min_interval:
                return
            self._ethercat_last_recovery_attempt_ts = now

        try:
            self.get_logger().warning(
                f'[EtherCAT] Requesting slave recovery to OP after fault: {reason}'
            )
            result = subprocess.run(
                ['ethercat', 'states', 'OP'],
                capture_output=True,
                text=True,
                timeout=max(float(ETHERCAT_RECOVERY_CMD_TIMEOUT_S), 0.1),
                check=False,
            )
        except Exception as exc:
            self.get_logger().error(f'[EtherCAT] Recovery command failed: {exc}')
            return

        stdout = (result.stdout or '').strip()
        stderr = (result.stderr or '').strip()
        if result.returncode == 0:
            detail = f' stdout={stdout!r}' if stdout else ''
            self.get_logger().warning(
                f'[EtherCAT] Recovery command issued: ethercat states OP{detail}'
            )
        else:
            self.get_logger().error(
                '[EtherCAT] Recovery command failed: '
                f'code={result.returncode} stdout={stdout!r} stderr={stderr!r}'
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
