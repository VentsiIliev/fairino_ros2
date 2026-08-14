#!/usr/bin/env python3
import os
import subprocess
import threading
import time
from threading import Lock

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from erob_moveit_runtime.srv import ApplyIPP
from geometry_msgs.msg import Point, Pose, TransformStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, MotionSequenceItem, MotionSequenceRequest, PlanningScene
from moveit_msgs.srv import GetCartesianPath, GetMotionSequence
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import tf2_ros
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
import config
from backend.runtime_adapter import create_runtime_adapter

from utils.transformation_utils import TransformationUtils
from safety.safety_wall_manager import SafetyWallManager
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
from robot_runtime_context import RobotRuntimeContext

# All tunable constants live in config.py
from config import (
    SAFETY_MARGIN_M as SAFETY_MARGIN,
    WALL_XY_OFFSET_M as WALL_XY_OFFSET,
    TOOL_REGISTRY as tool_registry,
    TOOL_ID_MAP as tool_id_map,
    MOTION_QUEUE_MAX_SIZE,
    MARKER_PUBLISH_INTERVAL_S,
    TOPIC_ROBOT_STATUS,
    ROBOT_STATUS_PUBLISH_ENABLED,
    TOPIC_ACTIVE_TOOL_MARKERS,
    STATUS_PUBLISH_RATE_HZ,
    WS_EXTRACT_MAX_RETRIES,
    WS_EXTRACT_RETRY_DELAY,
    SERVICE_CARTESIAN_PATH,
    SERVICE_MOTION_SEQUENCE,
    SERVICE_APPLY_IPP,
    SERVICE_APPLY_RUCKIG,
    URDF_PATH,
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
    MONITOR_UPDATE_RATE_HZ,
    RUNTIME_JOINT_STATE_INPUT_RATE_HZ,
    RUNTIME_DYNAMIC_STATE_INPUT_RATE_HZ,
    STARTUP_AUTO_ENABLE_DRIVES,
    STARTUP_AUTO_ENABLE_DRIVES_TIMEOUT_S,
    STARTUP_AUTO_ENABLE_DRIVES_RETRY_PERIOD_S,
    STARTUP_AUTO_ENABLE_DRIVES_VERIFY_TIMEOUT_S,
    MOTION_ERROR_HARDWARE_NOT_READY,
    MOTION_ERROR_DRIVE_NOT_ENABLED,
)


class RobotController(Node):
    _MANIPULATOR_CONTROLLER_NAME = 'manipulator_controller'

    def __init__(
            self,
            robot_context: RobotRuntimeContext | None = None,
    ):
        import time
        start_time = time.time()

        super().__init__('velocity_monitor')
        self.robot_context = (
            robot_context
            if robot_context is not None
            else RobotRuntimeContext.from_config()
        )

        self.get_logger().info(
            f'[Init] Runtime robot: {self.robot_context.name} '
            f'group={self.robot_context.planning_group} '
            f'base={self.robot_context.base_link} '
            f'action={self.robot_context.action_follow_trajectory}'
        )

        self.get_logger().info('[Init] RobotController starting...')

        self._fake_hardware = os.environ.get('ZEROERR_USE_FAKE_HARDWARE', '').strip().lower() in {
            '1', 'true', 'yes', 'on'
        }
        if self._fake_hardware:
            self.get_logger().info('[FakeHardware] Runtime EtherCAT and drive-enable logic disabled')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.active_tcp_tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.active_tool_marker_pub = self.create_publisher(
            MarkerArray,
            TOPIC_ACTIVE_TOOL_MARKERS,
            10,
        )
        self.active_tool_collision_pub = self.create_publisher(
            PlanningScene,
            config.TOPIC_PLANNING_SCENE,
            10,
        )
        self.active_tool_marker_timer = None
        rviz_enabled = os.environ.get("ZEROERR_USE_RVIZ", "false").strip().lower() in {
            "1", "true", "yes", "on",
        }
        active_tool_marker_hz = float(getattr(config, "ACTIVE_TOOL_MARKER_PUBLISH_HZ", 1.0) or 0.0)
        if rviz_enabled and active_tool_marker_hz > 0.0:
            self.active_tool_marker_timer = self.create_timer(
                1.0 / active_tool_marker_hz,
                self._publish_active_tool_visualization,
            )
            self.get_logger().info(
                f'[ActiveTCP] Visualization marker publishing at {active_tool_marker_hz:.1f} Hz'
            )
        else:
            reason = "RViz disabled" if not rviz_enabled else "configured rate <= 0"
            self.get_logger().info(f'[ActiveTCP] Visualization marker publishing disabled ({reason})')

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
            publish_rate=STATUS_PUBLISH_RATE_HZ,
            enabled=ROBOT_STATUS_PUBLISH_ENABLED,
        )

        self.monitor = None
        self.tcp_loaded = False
        self.T_ee_link = None
        self.T_tool = np.eye(4)
        self.T_monitor_tool = np.eye(4)
        self.active_tool_name = "TOOL_0"
        self._active_tool_collision_timer = None
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
        self.controller_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.robot_context.action_follow_trajectory,
        )

        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
        )
        self.cart_path_client = self.create_client(GetCartesianPath, SERVICE_CARTESIAN_PATH)
        self.sequence_client = self.create_client(GetMotionSequence, SERVICE_MOTION_SEQUENCE)
        self.ipp_client = self.create_client(ApplyIPP, SERVICE_APPLY_IPP)
        self.ruckig_client = self.create_client(ApplyIPP, SERVICE_APPLY_RUCKIG)
        self.trajectory_executor = TrajectoryExecutor(
            node=self,
            coordinator=self._motion,
            motion_queue=self.motion_queue,
            controller_client=self.controller_client,
            action_follow_trajectory=self.robot_context.action_follow_trajectory,
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
            sequence_client=self.sequence_client,
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
        self._startup_auto_enable_thread = None
        self._motion_interlock_lock = Lock()
        self._motion_interlock_active = False
        self._motion_interlock_reason = ""
        self._runtime_joint_input_period = (
            1.0 / float(RUNTIME_JOINT_STATE_INPUT_RATE_HZ)
            if float(RUNTIME_JOINT_STATE_INPUT_RATE_HZ) > 0.0
            else 0.0
        )
        self._runtime_dynamic_input_period = (
            1.0 / float(RUNTIME_DYNAMIC_STATE_INPUT_RATE_HZ)
            if float(RUNTIME_DYNAMIC_STATE_INPUT_RATE_HZ) > 0.0
            else 0.0
        )
        self._last_runtime_joint_input_ts = 0.0
        self._last_runtime_dynamic_input_ts = 0.0
        self._ethercat_watchdog_enabled = bool(ETHERCAT_WATCHDOG_ENABLED) and not self._fake_hardware
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
            if self._fake_hardware:
                self.get_logger().info('[EtherCAT] Runtime OP watchdog disabled in fake hardware mode')
            else:
                self.get_logger().info('[EtherCAT] Runtime OP watchdog disabled for this robot backend')

        self.runtime_adapter.initialize_runtime_controller(self)

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(
            GoalStatusArray,
            self.robot_context.action_follow_trajectory + '/_action/status',
            self._controller_status_callback,
            10,
        )

        self.get_logger().info(f'[Init] RobotController ready ({time.time() - start_time:.2f}s total)')

        if bool(STARTUP_AUTO_ENABLE_DRIVES) and not self._fake_hardware:
            self._startup_auto_enable_thread = threading.Thread(
                target=self._startup_auto_enable_drives_loop,
                daemon=True,
                name="StartupAutoEnableDrives",
            )
            self._startup_auto_enable_thread.start()
        elif self._fake_hardware:
            self.get_logger().info('[DriveEnable] Startup auto-enable skipped in fake hardware mode')

    @property
    def is_executing(self):
        return self._motion.is_executing

    @is_executing.setter
    def is_executing(self, value):
        self._motion.is_executing = bool(value)

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

    def _delayed_safety_init(self):
        """Publish safety walls after a short delay to speed up initialization."""
        self.safety_manager.force_update()
        self.get_logger().info('[Init] Safety walls published')
        self._publish_mounting_surface_collision()
        self._publish_active_tool_collision()

        if hasattr(self, '_safety_init_timer'):
            self._safety_init_timer.cancel()
            self.destroy_timer(self._safety_init_timer)

    def _active_tool_collision_enabled(self) -> bool:
        return bool(getattr(config, "ACTIVE_TOOL_COLLISION_ENABLED", False))

    def _mounting_surface_collision_enabled(self) -> bool:
        return bool(getattr(config, "MOUNTING_SURFACE_COLLISION_OBJECT_ENABLED", False))

    def _publish_mounting_surface_collision(self):
        if not self._mounting_surface_collision_enabled():
            return

        try:
            obj = CollisionObject()
            obj.id = str(
                getattr(config, "MOUNTING_SURFACE_COLLISION_OBJECT_ID", "mounting_surface_collision")
            )
            obj.header.frame_id = str(
                getattr(config, "MOUNTING_SURFACE_COLLISION_OBJECT_FRAME", "mounting_surface")
            )
            obj.header.stamp = self.get_clock().now().to_msg()

            box_values = list(getattr(config, "MOUNTING_SURFACE_COLLISION_BOX_M", []) or [])
            if len(box_values) != 3:
                box_values = [0.115, 0.190, 0.1475]

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(value) for value in box_values]

            origin_values = list(
                getattr(
                    config,
                    "MOUNTING_SURFACE_COLLISION_ORIGIN",
                    [100.0, 576.45, 158.0, 0.0, 0.0, 0.0],
                ) or []
            )
            if len(origin_values) != 6:
                origin_values = [100.0, 576.45, 158.0, 0.0, 0.0, 0.0]

            T_collision = TransformationUtils.pose_to_transform(origin_values)
            quat = TransformationUtils.matrix_to_quaternion(T_collision[:3, :3])
            pose = Pose()
            pose.position.x = float(T_collision[0, 3])
            pose.position.y = float(T_collision[1, 3])
            pose.position.z = float(T_collision[2, 3])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])

            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD

            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects.append(obj)
            self.active_tool_collision_pub.publish(scene)
            self.get_logger().info(
                f"[MountingSurfaceCollision] Published box id={obj.id} frame={obj.header.frame_id} "
                f"dims={primitive.dimensions}"
            )
        except Exception as exc:
            self.get_logger().warning(f"[MountingSurfaceCollision] Failed to publish collision object: {exc}")

    def _publish_active_tool_collision(self):
        if not self._active_tool_collision_enabled():
            return

        try:
            attached = AttachedCollisionObject()
            attached.link_name = str(
                getattr(
                    config,
                    "ACTIVE_TOOL_COLLISION_LINK",
                    self.robot_context.ee_link,
                )
                or self.robot_context.ee_link
            )
            attached.touch_links = [
                str(link)
                for link in getattr(config, "ACTIVE_TOOL_COLLISION_TOUCH_LINKS", []) or []
            ]

            obj = CollisionObject()
            obj.id = str(getattr(config, "ACTIVE_TOOL_COLLISION_ID", "active_tool_collision"))
            obj.header.frame_id = attached.link_name
            obj.header.stamp = self.get_clock().now().to_msg()

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [
                float(getattr(config, "ACTIVE_TOOL_COLLISION_LENGTH_M", 0.17)),
                float(getattr(config, "ACTIVE_TOOL_COLLISION_RADIUS_M", 0.012)),
            ]

            origin_values = list(getattr(config, "ACTIVE_TOOL_COLLISION_ORIGIN", [0, 0, 0, 0, 0, 0]) or [])
            if len(origin_values) != 6:
                origin_values = [0, 0, 0, 0, 0, 0]
            T_collision = TransformationUtils.pose_to_transform(origin_values)
            if bool(getattr(config, "ACTIVE_TOOL_COLLISION_USE_ACTIVE_TOOL_TRANSFORM", True)):
                T_collision = self.T_tool @ T_collision

            quat = TransformationUtils.matrix_to_quaternion(T_collision[:3, :3])
            pose = Pose()
            pose.position.x = float(T_collision[0, 3])
            pose.position.y = float(T_collision[1, 3])
            pose.position.z = float(T_collision[2, 3])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])

            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD
            attached.object = obj

            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects.append(attached)
            self.active_tool_collision_pub.publish(scene)
            self.get_logger().info(
                f"[ActiveToolCollision] Attached cylinder id={obj.id} link={attached.link_name} "
                f"tool={self.active_tool_name} length={primitive.dimensions[0]:.3f}m "
                f"radius={primitive.dimensions[1]:.3f}m"
            )
        except Exception as exc:
            self.get_logger().warning(f"[ActiveToolCollision] Failed to publish attached cylinder: {exc}")

    def _publish_active_tool_visualization(self):
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
        transform.header.frame_id = self.robot_context.base_link
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
        collision_marker = self._make_active_tool_collision_marker(stamp)
        if collision_marker is not None:
            marker_array.markers.append(collision_marker)
        self.active_tool_marker_pub.publish(marker_array)

    def _make_source_sphere_marker(self, stamp, origin):
        marker = Marker()
        marker.header.frame_id = self.robot_context.base_link
        marker.header.stamp = stamp
        marker.ns = "active_tool"
        marker.id = 90
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(origin[0])
        marker.pose.position.y = float(origin[1])
        marker.pose.position.z = float(origin[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.012
        marker.scale.y = 0.012
        marker.scale.z = 0.012
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _make_source_to_tcp_marker(self, stamp, source_origin, tcp_origin):
        marker = Marker()
        marker.header.frame_id = self.robot_context.base_link
        marker.header.stamp = stamp
        marker.ns = "active_tool"
        marker.id = 91
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.002
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        p0 = Point()
        p0.x, p0.y, p0.z = map(float, source_origin)
        p1 = Point()
        p1.x, p1.y, p1.z = map(float, tcp_origin)
        marker.points = [p0, p1]
        return marker

    def _make_tcp_sphere_marker(self, stamp, origin, quaternion):
        marker = Marker()
        marker.header.frame_id = self.robot_context.base_link
        marker.header.stamp = stamp
        marker.ns = "active_tool"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
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
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _make_tcp_axis_markers(self, stamp, origin, rotation):
        markers = []
        colors = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        axis_length = 0.06
        for idx in range(3):
            marker = Marker()
            marker.header.frame_id = self.robot_context.base_link
            marker.header.stamp = stamp
            marker.ns = "active_tool"
            marker.id = idx + 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.004
            marker.scale.y = 0.008
            marker.scale.z = 0.012
            marker.color.r, marker.color.g, marker.color.b = colors[idx]
            marker.color.a = 1.0

            start = Point()
            start.x = float(origin[0])
            start.y = float(origin[1])
            start.z = float(origin[2])
            end = Point()
            end_vec = origin + rotation[:, idx] * axis_length
            end.x = float(end_vec[0])
            end.y = float(end_vec[1])
            end.z = float(end_vec[2])
            marker.points = [start, end]
            markers.append(marker)
        return markers

    def _make_active_tool_collision_marker(self, stamp):
        if not self._active_tool_collision_enabled():
            return None
        origin_values = list(getattr(config, "ACTIVE_TOOL_COLLISION_ORIGIN", [0, 0, 0, 0, 0, 0]) or [])
        if len(origin_values) != 6:
            origin_values = [0, 0, 0, 0, 0, 0]
        T_collision = TransformationUtils.pose_to_transform(origin_values)
        if bool(getattr(config, "ACTIVE_TOOL_COLLISION_USE_ACTIVE_TOOL_TRANSFORM", True)):
            T_collision = self.T_tool @ T_collision
        quat = TransformationUtils.matrix_to_quaternion(T_collision[:3, :3])

        marker = Marker()
        marker.header.frame_id = str(
            getattr(
                config,
                "ACTIVE_TOOL_COLLISION_LINK",
                self.robot_context.ee_link,
            )
            or self.robot_context.ee_link
        )
        marker.header.stamp = stamp
        marker.ns = "active_tool_collision"
        marker.id = 200
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(T_collision[0, 3])
        marker.pose.position.y = float(T_collision[1, 3])
        marker.pose.position.z = float(T_collision[2, 3])
        marker.pose.orientation.x = float(quat[0])
        marker.pose.orientation.y = float(quat[1])
        marker.pose.orientation.z = float(quat[2])
        marker.pose.orientation.w = float(quat[3])
        marker.scale.x = 2.0 * float(getattr(config, "ACTIVE_TOOL_COLLISION_RADIUS_M", 0.012))
        marker.scale.y = 2.0 * float(getattr(config, "ACTIVE_TOOL_COLLISION_RADIUS_M", 0.012))
        marker.scale.z = float(getattr(config, "ACTIVE_TOOL_COLLISION_LENGTH_M", 0.17))
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.35
        return marker

    def _make_tcp_label_marker(self, stamp, origin, source_origin=None):
        marker = Marker()
        marker.header.frame_id = self.robot_context.base_link
        marker.header.stamp = stamp
        marker.ns = "active_tool"
        marker.id = 10
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
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
        self.safety_manager.enable_safety()
        return self.safety_manager.get_status()

    def disable_safety_walls(self) -> dict:
        self.safety_manager.disable_safety()
        return self.safety_manager.get_status()

    def get_safety_walls_status(self) -> dict:
        return self.safety_manager.get_status()

    def wait_for_monitor(self, timeout_sec=5.0):
        import time
        start_time = time.time()
        while self.monitor is None:
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for RobotMonitor (TCP not loaded)")
                return False
            time.sleep(0.1)
        return True

    def _get_monitor_tcp_transform(self):
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
        tool_name = tool_id_map.get(tool_id, "TOOL_0")
        if tool_name not in tool_registry:
            self.get_logger().warning(f"Tool {tool_name} not found in registry, using TOOL_0")
            tool_name = "TOOL_0"

        registry_transform = self._build_registry_tool_transform(tool_name)
        return self.runtime_adapter.get_planning_tool_transform(self, registry_transform)

    def set_tool(self, tool_name):
        if tool_name not in tool_registry:
            self.get_logger().warning(f"Tool {tool_name} not found in registry")
            return

        registry_transform = self._build_registry_tool_transform(tool_name)
        self.T_monitor_tool = registry_transform
        self.T_tool = self.runtime_adapter.get_planning_tool_transform(self, registry_transform)
        self.planner_context.T_tool = self.T_tool
        self.active_tool_name = tool_name

        if self.monitor is not None:
            self.monitor.set_tcp_transform(self._get_monitor_tcp_transform())
            self.get_logger().info(f"Switched active tool to {tool_name}")
        else:
            self.get_logger().warning("RobotMonitor not initialized yet")
        self._publish_active_tool_collision()

    def set_active_tool(self, tool_id):
        tool_name = tool_id_map.get(tool_id, "TOOL_0")
        if tool_name not in tool_registry:
            self.get_logger().warning(f"Tool {tool_name} not found in registry")
            return False
        self.set_tool(tool_name)
        return True

    def load_tcp_transform(self):
        if self.tcp_loaded:
            return

        if self.tf_buffer.can_transform(
                self.robot_context.wrist_link,
                self.robot_context.ee_link,
                rclpy.time.Time(),
        ):
            try:
                self.T_ee_link = self.get_tcp_transform(
                    self.robot_context.wrist_link,
                    self.robot_context.ee_link,
                )
                print(f"[RobotController] Loaded ee_link transform:\n{self.T_ee_link}")
                if self.active_tool_name in tool_registry:
                    registry_transform = self._build_registry_tool_transform(self.active_tool_name)
                    self.T_monitor_tool = registry_transform
                    self.T_tool = self.runtime_adapter.get_planning_tool_transform(self, registry_transform)
                    self.planner_context.T_tool = self.T_tool

                self.monitor = RobotMonitor(
                    ros_node=self,
                    tcp_transform=self._get_monitor_tcp_transform(),
                    stable_update_rate_hz=MONITOR_UPDATE_RATE_HZ,
                )
                self.monitor.set_stable_update_callback(self._handle_monitor_update)
                self.tcp_loaded = True

                self.get_logger().info("TCP transform loaded and RobotMonitor initialized")
                self._publish_active_tool_collision()

                if hasattr(self, 'tcp_load_timer'):
                    self.tcp_load_timer.cancel()
                    self.destroy_timer(self.tcp_load_timer)

            except Exception as e:
                self.get_logger().warning(f"TCP transform lookup failed: {e}")

    def get_tcp_transform(
            self,
            from_frame=None,
            to_frame=None,
    ):
        from_frame = (
            str(from_frame).strip()
            if from_frame is not None
            else self.robot_context.wrist_link
        )

        to_frame = (
            str(to_frame).strip()
            if to_frame is not None
            else self.robot_context.ee_link
        )

        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            T = TransformationUtils.tf2_to_transform(trans)
            print(f"[RobotController] Loaded TCP transform:\n{T}")
            return T
        except Exception as e:
            self.get_logger().warning(
                f"Could not get TCP transform from tf: {e}"
            )
            return np.eye(4)

    def execute(self, strategy, queue_if_busy=True):
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

    def _controller_status_callback(self, msg):
        return

    def joint_state_callback(self, msg):
        if self._runtime_joint_input_period > 0.0:
            now = time.monotonic()
            if now - self._last_runtime_joint_input_ts < self._runtime_joint_input_period:
                return
            self._last_runtime_joint_input_ts = now

        self.runtime_adapter.on_joint_state(self, msg)

        with self.lock:
            if len(msg.position) < 6:
                return

            self.current_joint_state = msg

            if self.monitor is None:
                return

            data = self.monitor.get_latest_data()

            if data is not None:
                self.prev_cartesian = data['cartesian']
                self.latest_data = data

                if len(msg.effort) >= 6:
                    data['efforts'] = np.array(msg.effort[:6])
                    self.latest_data = data

        self.runtime_adapter.log_drive_state_snapshot(self, 'startup')

    def get_latest_data(self):
        data = self.state_store.get_latest_data()
        if data is not None:
            return data
        if self.monitor is not None:
            return self.monitor.get_latest_data()
        return None

    def _handle_monitor_update(self, data):
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

    def get_contour_ik_client(self):
        return self.planner_context.get_contour_ik_client()

    def get_linked_lin_client(self):
        return self.planner_context.get_linked_lin_client()

    def get_trajectory_state_validation_client(self):
        return self.planner_context.get_trajectory_state_validation_client()

    def get_state_validity_client(self):
        return self.planner_context.get_state_validity_client()

    def get_ptp_client(self):
        return self.planner_context.get_ptp_client()

    def stop_motion(self):
        return self._motion.stop_motion()

    def set_drive_operation_enabled(self, enabled: bool) -> dict:
        if self._fake_hardware:
            return {
                'success': True,
                'supported': False,
                'enabled': True,
                'requested_enabled': bool(enabled),
                'actual_enabled': True,
                'motion_allowed_by_drive_enable': True,
                'state': 'NOT_REQUIRED_FAKE_HARDWARE',
            }
        return self.runtime_adapter.set_drive_operation_enabled(self, enabled)

    def is_drive_operation_enabled_for_motion(self) -> bool:
        if self._fake_hardware:
            return True
        return self.runtime_adapter.is_drive_operation_enabled_for_motion(self)

    def get_drive_enable_fault_reason(self) -> str:
        if self._fake_hardware:
            return 'drive enable is not required in fake hardware mode'
        return self.runtime_adapter.get_drive_enable_fault_reason(self)

    def get_drive_operation_status(self) -> dict:
        if self._fake_hardware:
            return {
                'success': True,
                'supported': False,
                'requested_enabled': False,
                'actual_enabled': True,
                'motion_allowed_by_drive_enable': True,
                'state': 'NOT_REQUIRED_FAKE_HARDWARE',
            }
        return self.runtime_adapter.get_drive_operation_status(self)

    def _startup_auto_enable_drives_loop(self):
        if self._fake_hardware:
            self.get_logger().info('[DriveEnable] Startup auto-enable bypassed in fake hardware mode')
            return

        timeout_s = max(float(STARTUP_AUTO_ENABLE_DRIVES_TIMEOUT_S), 0.1)
        retry_period_s = max(float(STARTUP_AUTO_ENABLE_DRIVES_RETRY_PERIOD_S), 0.1)
        verify_timeout_s = max(float(STARTUP_AUTO_ENABLE_DRIVES_VERIFY_TIMEOUT_S), 0.1)
        deadline = time.monotonic() + timeout_s
        last_reason = ""

        self.get_logger().info(
            f'[DriveEnable] Startup auto-enable enabled; waiting up to {timeout_s:.1f}s'
        )
        while rclpy.ok() and time.monotonic() < deadline:
            if self.is_drive_operation_enabled_for_motion():
                self.get_logger().info('[DriveEnable] Startup auto-enable skipped: drives already operation_enabled')
                return

            if not self.is_hardware_ready_for_motion():
                last_reason = self.get_hardware_fault_reason()
                self.get_logger().info(f'[DriveEnable] Startup auto-enable waiting: {last_reason}')
                time.sleep(retry_period_s)
                continue

            if self.current_joint_state is None:
                last_reason = 'joint_states not available yet'
                self.get_logger().info(f'[DriveEnable] Startup auto-enable waiting: {last_reason}')
                time.sleep(retry_period_s)
                continue

            if self._get_controller_states() is None:
                last_reason = 'controller manager services not available yet'
                self.get_logger().info(f'[DriveEnable] Startup auto-enable waiting: {last_reason}')
                time.sleep(retry_period_s)
                continue

            self.get_logger().info('[DriveEnable] Startup auto-enable requesting operation enable')
            result = self.set_drive_operation_enabled(True)
            if not result.get('success', False):
                last_reason = str(result.get('error') or result)
                self.get_logger().warning(f'[DriveEnable] Startup auto-enable request failed: {last_reason}')
                time.sleep(retry_period_s)
                continue

            verify_deadline = time.monotonic() + verify_timeout_s
            while rclpy.ok() and time.monotonic() < verify_deadline:
                if self.is_drive_operation_enabled_for_motion():
                    self.get_logger().info('[DriveEnable] Startup auto-enable complete: all drives operation_enabled')
                    return
                last_reason = self.get_drive_enable_fault_reason()
                time.sleep(0.1)

            self.get_logger().warning(
                f'[DriveEnable] Startup auto-enable verification timed out: {last_reason}'
            )
            time.sleep(retry_period_s)

        self.get_logger().error(
            f'[DriveEnable] Startup auto-enable failed after {timeout_s:.1f}s: {last_reason or "timeout"}'
        )

    def _get_controller_states(self) -> dict[str, str] | None:
        if not self.list_controllers_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('[DriveEnable] /controller_manager/list_controllers not available')
            return None
        future = self.list_controllers_client.call_async(ListControllers.Request())
        response = self._wait_for_service_future(future, timeout_s=3.0)
        if response is None:
            self.get_logger().error('[DriveEnable] Timed out listing controllers')
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
            self.get_logger().error(f'[DriveEnable] Service future failed: {result["error"]}')
            return None
        return result["value"]

    def is_hardware_ready_for_motion(self) -> bool:
        if self._fake_hardware:
            with self._motion_interlock_lock:
                return not self._motion_interlock_active
        with self._motion_interlock_lock:
            if self._motion_interlock_active:
                return False
        with self._ethercat_fault_lock:
            return not self._ethercat_motion_fault

    def get_hardware_fault_reason(self) -> str:
        with self._motion_interlock_lock:
            if self._motion_interlock_active:
                return self._motion_interlock_reason or 'motion interlock active'
        if self._fake_hardware:
            return 'fake hardware mode'
        with self._ethercat_fault_lock:
            return self._ethercat_fault_reason or 'EtherCAT hardware fault'

    def trip_motion_interlock(self, reason: str):
        reason = str(reason or 'motion interlock active')
        with self._motion_interlock_lock:
            if self._motion_interlock_active and self._motion_interlock_reason == reason:
                return
            self._motion_interlock_active = True
            self._motion_interlock_reason = reason
        if not self._fake_hardware:
            self.runtime_adapter.reset_drive_operation_request(self)
        self.get_logger().error(f'[MotionInterlock] Motion interlock active: {reason}')

    def reset_motion_interlock(self) -> dict:
        with self._motion_interlock_lock:
            was_active = self._motion_interlock_active
            reason = self._motion_interlock_reason
            self._motion_interlock_active = False
            self._motion_interlock_reason = ""
        if was_active:
            self.get_logger().warning(f'[MotionInterlock] Motion interlock reset by operator: {reason}')
        return {
            "success": True,
            "reset": bool(was_active),
            "previous_reason": reason,
        }

    def get_motion_interlock_status(self) -> dict:
        with self._motion_interlock_lock:
            return {
                "active": bool(self._motion_interlock_active),
                "reason": self._motion_interlock_reason,
            }

    def is_motion_stack_ready(self) -> bool:
        try:
            return self.get_motion_stack_fault_reason() is None
        except Exception as exc:
            self.get_logger().warning(f'[MotionStackReady] readiness check failed: {exc}')
            return False

    def get_motion_stack_fault_reason(self) -> str | None:
        if self.current_joint_state is None:
            return 'joint_states not available yet'
        if self.prev_cartesian is None or len(self.prev_cartesian) < 6:
            return 'current Cartesian pose not available yet'
        try:
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
            ptp_client = self.get_ptp_client()
            if ptp_client is None or not ptp_client.wait_for_service(timeout_sec=0.05):
                return 'PTP helper service not available'
            contour_ik_client = self.get_contour_ik_client()
            if contour_ik_client is None or not contour_ik_client.wait_for_service(timeout_sec=0.05):
                return 'Contour IK helper service not available'
            linked_lin_client = self.get_linked_lin_client()
            if linked_lin_client is None or not linked_lin_client.wait_for_service(timeout_sec=0.05):
                return 'Linked LIN helper service not available'
            if self.ipp_client is None or not self.ipp_client.wait_for_service(timeout_sec=0.05):
                return 'TOTG/IPP optimizer service not available'
            if self.ruckig_client is None or not self.ruckig_client.wait_for_service(timeout_sec=0.05):
                return 'Ruckig optimizer service not available'
            if not self.controller_client.wait_for_server(timeout_sec=0.05):
                return 'FollowJointTrajectory action server not available'
        except Exception as exc:
            return f'motion stack readiness check failed: {exc}'
        return None

    def is_motion_active(self):
        return self._motion.is_motion_active()

    def has_pending_motion(self):
        return self._motion.has_pending_motion()

    def destroy_node(self):
        backend = getattr(self, 'robot', None)
        servo = getattr(backend, 'cartesian_servo', None)
        if servo is not None and hasattr(servo, 'shutdown'):
            try:
                servo.shutdown()
            except Exception as exc:
                self.get_logger().warning(f'Cartesian Servo shutdown failed: {exc}')
        self._ethercat_watchdog_running = False
        watchdog = getattr(self, '_ethercat_watchdog_thread', None)
        if watchdog is not None and watchdog.is_alive():
            watchdog.join(timeout=1.0)
        startup_enable = getattr(self, '_startup_auto_enable_thread', None)
        if startup_enable is not None and startup_enable.is_alive():
            startup_enable.join(timeout=1.0)
        return super().destroy_node()

    def _format_drive_state_snapshot(self, label: str):
        if self._fake_hardware:
            return None
        return self.runtime_adapter.format_drive_state_snapshot(self, label)

    def log_drive_state_before_first_motion(self):
        if self._fake_hardware:
            return
        self.runtime_adapter.log_drive_state_before_first_motion(self)

    def get_unwind_drive_state(self, unwind_check):
        if self._fake_hardware:
            return None
        return self.runtime_adapter.get_unwind_drive_state(self, unwind_check)

    def get_all_drive_states(self):
        if self._fake_hardware:
            return []
        return self.runtime_adapter.get_all_drive_states(self)

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
    def plan_generation(self):
        return self._motion.plan_generation

    @plan_generation.setter
    def plan_generation(self, value):
        self._motion.plan_generation = value

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

    def _send_hold_position_trajectory(
        self,
        reason: str = 'hold position',
        suppress_drive_disable_cancel: bool = False,
    ) -> bool:
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
        joint_names = list(self.robot_context.joint_names)
        joint_count = len(joint_names)
        missing = [name for name in joint_names if name not in position_by_name]
        if missing:
            self.get_logger().warning(
                f'[HoldPosition] Cannot send hold trajectory for {reason}: missing joints {missing}'
            )
            return False
        positions = [float(position_by_name[name]) for name in joint_names]
        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = self.get_clock().now().to_msg()

        start_pt = JointTrajectoryPoint()
        start_pt.positions = list(positions)
        start_pt.velocities = [0.0] * joint_count
        start_pt.accelerations = [0.0] * joint_count
        start_pt.time_from_start = Duration(sec=0, nanosec=0)

        end_pt = JointTrajectoryPoint()
        end_pt.positions = list(positions)
        end_pt.velocities = [0.0] * joint_count
        end_pt.accelerations = [0.0] * joint_count
        end_pt.time_from_start = Duration(sec=0, nanosec=200_000_000)

        traj.points = [start_pt, end_pt]
        self.get_logger().warning(
            f'[HoldPosition] Sending hold trajectory for {reason}: '
            f'positions={[round(value, 6) for value in positions]}'
        )
        self.trajectory_executor.send_trajectory_to_controller(
            traj,
            suppress_drive_disable_cancel=suppress_drive_disable_cancel,
        )
        return True

    def _ethercat_watchdog_loop(self):
        if self._fake_hardware:
            return
        while self._ethercat_watchdog_running:
            self._poll_ethercat_health_once()
            time.sleep(ETHERCAT_WATCHDOG_POLL_S)

    def _poll_ethercat_health_once(self):
        if self._fake_hardware:
            return
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

        lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        if result.returncode != 0:
            stderr = result.stderr.strip()
            self._set_ethercat_fault(
                True,
                f'ethercat slaves failed rc={result.returncode}: {stderr or "unknown error"}',
            )
            return

        if len(lines) != ETHERCAT_EXPECTED_SLAVES:
            self._set_ethercat_fault(
                True,
                f'expected {ETHERCAT_EXPECTED_SLAVES} EtherCAT slaves, found {len(lines)}',
            )
            return

        non_op = [line for line in lines if ' OP ' not in f' {line} ']
        if non_op:
            self._set_ethercat_fault(
                True,
                'EtherCAT slaves not OP: ' + '; '.join(non_op),
            )
            return

        self._set_ethercat_fault(False, '')

    def _set_ethercat_fault(self, faulted: bool, reason: str):
        if self._fake_hardware:
            return
        with self._ethercat_fault_lock:
            previous_fault = self._ethercat_motion_fault
            previous_reason = self._ethercat_fault_reason
            self._ethercat_motion_fault = bool(faulted)
            self._ethercat_fault_reason = str(reason or '')
            if not faulted:
                self._ethercat_fault_stop_issued = False

        if faulted:
            if not previous_fault or previous_reason != self._ethercat_fault_reason:
                self.get_logger().error(f'[EtherCAT] Motion fault active: {self._ethercat_fault_reason}')
            self._attempt_ethercat_recovery()
            should_stop = False
            with self._ethercat_fault_lock:
                if not self._ethercat_fault_stop_issued:
                    self._ethercat_fault_stop_issued = True
                    should_stop = True
            if should_stop:
                try:
                    self.stop_motion()
                except Exception as exc:
                    self.get_logger().error(f'[EtherCAT] Failed to stop motion after EtherCAT fault: {exc}')
        elif previous_fault:
            self.get_logger().info('[EtherCAT] Motion fault cleared; all slaves are OP')

    def _attempt_ethercat_recovery(self):
        if self._fake_hardware:
            return
        if not ETHERCAT_RECOVERY_ENABLED:
            return
        now = time.monotonic()
        with self._ethercat_fault_lock:
            if now - self._ethercat_last_recovery_attempt_ts < ETHERCAT_RECOVERY_MIN_INTERVAL_S:
                return
            self._ethercat_last_recovery_attempt_ts = now

        self.get_logger().warning('[EtherCAT] Attempting one recovery pass')
        try:
            subprocess.run(
                ['ethercat', 'states', '-p', '0-5', 'INIT'],
                capture_output=True,
                text=True,
                timeout=ETHERCAT_RECOVERY_CMD_TIMEOUT_S,
                check=False,
            )
            subprocess.run(
                ['ethercat', 'states', '-p', '0-5', 'PREOP'],
                capture_output=True,
                text=True,
                timeout=ETHERCAT_RECOVERY_CMD_TIMEOUT_S,
                check=False,
            )
            subprocess.run(
                ['ethercat', 'states', '-p', '0-5', 'SAFEOP'],
                capture_output=True,
                text=True,
                timeout=ETHERCAT_RECOVERY_CMD_TIMEOUT_S,
                check=False,
            )
            subprocess.run(
                ['ethercat', 'states', '-p', '0-5', 'OP'],
                capture_output=True,
                text=True,
                timeout=ETHERCAT_RECOVERY_CMD_TIMEOUT_S,
                check=False,
            )
        except Exception as exc:
            self.get_logger().warning(f'[EtherCAT] Recovery command failed: {exc}')
