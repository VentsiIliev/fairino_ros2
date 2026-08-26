#!/usr/bin/env python3
from enums import RobotAxis, Direction
from time import perf_counter
import math
import time
import traceback
import config
from backend.i_robot_backend import IRobotBackend
from motion.jog.planned_jog import PlannedJogCapability
from motion.jog.servo_jog import ServoJogCapability
from motion.servo.cartesian_servo.i_cartesian_servo import CartesianServo, CartesianServoFrame
from utils.work_object import WorkObject
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from concurrent.futures import ThreadPoolExecutor
from threading import Event, Lock
from uuid import uuid4


class MoveItRobotBackend(IRobotBackend):
    """
    Shared MoveIt-backed robot transport used by the REST API regardless of robot hardware.
    Provides motion control, I/O operations, and coordinate frame management.
    """

    def __init__(self, ip, node=None, workobject=None,cartesian_servo=None):
        """
        Initializes the ROS2 robot wrapper.

        Args:
            ip (str): IP address of the robot controller (for compatibility, not used in ROS2)
            node (RobotController): ROS2 node for robot control (optional)
            workobject (WorkObject): Default work object frame (optional)
        """
        self.ip = ip
        self.node = node  # embeds the RobotController node
        self.workobject_registry = self._build_workobject_registry(workobject)
        self.active_workobject_user_id = int(getattr(config, "DEFAULT_WORKOBJECT_ID", 0) or 0)
        self.active_workobject_name = self._workobject_name_for_user(self.active_workobject_user_id)
        self.workobject = self.workobject_registry.get(self.active_workobject_user_id)
        self._cartesian_servo = cartesian_servo
        self._planned_jog = PlannedJogCapability(self)
        self._servo_jog = ServoJogCapability(self)
        self._prepared_ordered_lock = Lock()
        self._prepared_ordered = {}
        self._prepared_ordered_executor = ThreadPoolExecutor(max_workers=1)

    # ---------------- WorkObject Methods ----------------
    def _build_workobject_registry(self, default_workobject=None):
        registry = {}
        id_map = dict(getattr(config, "WORKOBJECT_ID_MAP", {}) or {})
        named_registry = dict(getattr(config, "WORKOBJECT_REGISTRY", {}) or {})
        for user_id, name in id_map.items():
            values = named_registry.get(name)
            if values is None:
                continue
            try:
                registry[int(user_id)] = WorkObject(*values)
            except Exception:
                if self.node is not None:
                    self.node.get_logger().warning(f"Invalid workobject registry entry {name}: {values}")
        if default_workobject is not None:
            registry.setdefault(0, default_workobject)
        registry.setdefault(0, None)
        return registry

    def _workobject_name_for_user(self, user_id):
        try:
            name = config.resolve_workobject_name(int(user_id))
        except Exception:
            name = f"WOBJ_{int(user_id)}"
        return name

    def set_workobject(self, workobject, user_id=0):
        """
        Set a WorkObject for the robot (coordinate frame).

        Args:
            workobject (WorkObject): Work object to set
            user_id (int): User frame ID (default 0)
        """
        user_id = int(user_id)
        if workobject is not None and not isinstance(workobject, WorkObject):
            workobject = WorkObject(*workobject)
        self.workobject_registry[user_id] = workobject
        if user_id == self.active_workobject_user_id:
            self.workobject = workobject

    def get_workobject(self, user_id=0):
        """
        Get a WorkObject by user ID.

        Args:
            user_id (int): User frame ID

        Returns:
            WorkObject or None
        """
        return self.workobject_registry.get(int(user_id))

    def set_active_workobject(self, user_id=0):
        user_id = int(user_id)
        if user_id not in self.workobject_registry:
            name = config.resolve_workobject_name(user_id)
            values = getattr(config, "WORKOBJECT_REGISTRY", {}).get(name)
            if values is None:
                raise ValueError(f"user_id {user_id} maps to unknown workobject {name!r}")
            self.workobject_registry[user_id] = WorkObject(*values)
        self.active_workobject_user_id = user_id
        self.active_workobject_name = self._workobject_name_for_user(user_id)
        self.workobject = self.workobject_registry.get(user_id)
        return self.active_workobject_name

    def get_active_workobject(self):
        return {
            "user_id": int(self.active_workobject_user_id),
            "workobject_name": str(self.active_workobject_name),
            "origin": self._workobject_to_list(self.workobject),
        }

    @staticmethod
    def _workobject_to_list(workobject):
        if workobject is None:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return [
            float(workobject.x),
            float(workobject.y),
            float(workobject.z),
            float(workobject.rx),
            float(workobject.ry),
            float(workobject.rz),
        ]

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

    def _reject_if_drive_not_enabled(self, label):
        if self.node is None:
            return None
        if not self.node.is_drive_operation_enabled_for_motion():
            self.node.get_logger().error(
                f"[{label}] Rejected: {self.node.get_drive_enable_fault_reason()}"
            )
            return config.MOTION_ERROR_DRIVE_NOT_ENABLED
        return None

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True, trajectory_optimizer=None, allow_collision_recovery=False):
        started_at = perf_counter()
        if len(position) != 6:
            return -1
        drive_error = self._reject_if_drive_not_enabled("MOVE_LINER")
        if drive_error is not None:
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[MOVE_LINER] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        try:
            from motion.move_linear_timing import ensure as ensure_move_linear_timing, mark as mark_move_linear_timing
            ensure_move_linear_timing(self.node, source="backend.move_liner")
            mark_move_linear_timing(
                self.node,
                "backend_received",
                blocking=bool(blocking),
                tool=tool,
                user=user,
                vel=float(vel),
                acc=float(acc),
            )
            position_base = self.apply_workobject(position, user_id=user)
            tool_transform = self.node.get_tool_transform(tool)
            vel_scale = max(0.0, min(1.0, vel / 100.0))
            acc_scale = max(0.0, min(1.0, acc / 100.0))
            avoid_collisions = not bool(allow_collision_recovery)
            x, y, z, rx, ry, rz = position_base
            from motion.strategies import SingleTargetStrategy
            if trajectory_optimizer is not None:
                result = self.node.execute(SingleTargetStrategy(
                    x, y, z, rx, ry, rz, vel_scale, acc_scale,
                    tool_transform=tool_transform,
                    avoid_collisions=avoid_collisions,
                    trajectory_optimizer=trajectory_optimizer,
                ))
            else:
                result = self.node.execute(SingleTargetStrategy(
                    x, y, z, rx, ry, rz, vel_scale, acc_scale,
                    tool_transform=tool_transform,
                    avoid_collisions=avoid_collisions,
                ))
            self.node.get_logger().info(
                f"[TIMING] backend_move_linear submitted blocking={bool(blocking)} "
                f"result={result} elapsed_s={perf_counter() - started_at:.3f}"
            )
            if result != 0:
                if blocking and result > 0:
                    task_id = getattr(self.node, 'last_submitted_task_id', None)
                    if task_id is not None:
                        waited = self.node.motion_queue.wait_for_task(task_id, config.BLOCKING_MOVE_TIMEOUT_S)
                        if waited is None:
                            self.node.get_logger().error(
                                f"[MOVE_LINER] Timed out waiting for queued move task #{task_id} to complete")
                            return -1
                        return waited
                self.node.get_logger().info(f"[MOVE_LINER] execute() returned {result}")
                return result

            if blocking:
                import time
                wait_started_at = perf_counter()
                deadline = time.time() + config.BLOCKING_MOVE_TIMEOUT_S
                while self.node.is_executing and time.time() < deadline:
                    time.sleep(0.05)
                if self.node.is_executing:
                    self.node.get_logger().error(
                        f"[MOVE_LINER] Timed out waiting for move to complete after {config.BLOCKING_MOVE_TIMEOUT_S}s")
                    return -1
                self.node.get_logger().info(
                    f"[TIMING] backend_move_linear blocking_wait elapsed_s={perf_counter() - wait_started_at:.3f} "
                    f"total_elapsed_s={perf_counter() - started_at:.3f}"
                )
                return self.node.last_move_result

            return 0  # non-blocking: fire-and-forget
        except Exception as e:
            print(f"move_liner error: {e}")
            return -1

    def move_ptp(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True, trajectory_optimizer=None):
        started_at = perf_counter()
        if len(position) != 6:
            return -1
        drive_error = self._reject_if_drive_not_enabled("MOVE_PTP")
        if drive_error is not None:
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[MOVE_PTP] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        try:
            position_base = self.apply_workobject(position, user_id=user)
            tool_transform = self.node.get_tool_transform(tool)
            vel_scale = max(0.0, min(1.0, vel / 100.0))
            acc_scale = max(0.0, min(1.0, acc / 100.0))
            x, y, z, rx, ry, rz = position_base
            from motion.strategies import PtpTargetStrategy
            result = self.node.execute(PtpTargetStrategy(
                x, y, z, rx, ry, rz, vel_scale, acc_scale,
                tool_transform=tool_transform,
                trajectory_optimizer=trajectory_optimizer,
            ))
            self.node.get_logger().info(
                f"[TIMING] backend_move_ptp submitted blocking={bool(blocking)} "
                f"result={result} elapsed_s={perf_counter() - started_at:.3f}"
            )
            if result != 0:
                if blocking and result > 0:
                    task_id = getattr(self.node, 'last_submitted_task_id', None)
                    if task_id is not None:
                        waited = self.node.motion_queue.wait_for_task(task_id, config.BLOCKING_MOVE_TIMEOUT_S)
                        if waited is None:
                            self.node.get_logger().error(
                                f"[MOVE_PTP] Timed out waiting for queued move task #{task_id} to complete")
                            return -1
                        return waited
                self.node.get_logger().info(f"[MOVE_PTP] execute() returned {result}")
                return result

            if blocking:
                import time
                wait_started_at = perf_counter()
                deadline = time.time() + config.BLOCKING_MOVE_TIMEOUT_S
                while self.node.is_executing and time.time() < deadline:
                    time.sleep(0.05)
                if self.node.is_executing:
                    self.node.get_logger().error(
                        f"[MOVE_PTP] Timed out waiting for move to complete after {config.BLOCKING_MOVE_TIMEOUT_S}s")
                    return -1
                self.node.get_logger().info(
                    f"[TIMING] backend_move_ptp blocking_wait elapsed_s={perf_counter() - wait_started_at:.3f} "
                    f"total_elapsed_s={perf_counter() - started_at:.3f}"
                )
                return self.node.last_move_result

            return 0
        except Exception as e:
            print(f"move_ptp error: {e}")
            return -1

    def execute_path(
            self,
            path,
            rx=None,
            ry=None,
            rz=None,
            vel=0.6,
            acc=0.4,
            blocking=True,
            trajectory_optimizer=None,
            orientation_mode="constant",
    ):
        started_at = perf_counter()
        """Execute path with automatic selection of best execution strategy.

        Strategy selection based on path density:
        - Dense paths (avg spacing < 2mm): Use compute_cartesian_path (continuous contour)
        - Sparse paths (avg spacing >= 2mm): Always use compute_cartesian_path for consistency

        This ensures smooth motion without stops for all path types.
        """
        if not path or self.node is None:
            return -1
        drive_error = self._reject_if_drive_not_enabled("EXECUTE_PATH")
        if drive_error is not None:
            return drive_error
        if not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[EXECUTE_PATH] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY

        self.node.get_logger().info(f"[EXECUTE_PATH] Received path with {len(path)} waypoints")

        orientation_mode = str(orientation_mode or "constant").strip().lower()
        # Convert velocity/acceleration from percentage (0-100) to scaling factor (0.0-1.0)
        vel_scale = max(0.0, min(1.0, vel / 100.0))
        acc_scale = max(0.0, min(1.0, acc / 100.0))
        self.node.get_logger().info(
            f"[EXECUTE_PATH] Velocity conversion: {vel}% → {vel_scale:.3f} scaling, {acc}% → {acc_scale:.3f} scaling"
        )
        waypoints_pose = []

        for wp in path:
            if len(wp) == 3:
                waypoints_pose.append([wp[0], wp[1], wp[2]])
                # Get current TCP orientation if not provided
                if rx is None or ry is None or rz is None:
                    current_pose = self.get_current_position()
                    if current_pose is not None:
                        rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
                    else:
                        # Fallback if current position unavailable
                        rx, ry, rz = config.DEFAULT_ORIENTATION
            elif len(wp) == 6:
                wx, wy, wz, wrx, wry, wrz = wp
                if orientation_mode == "per_waypoint":
                    waypoints_pose.append([wx, wy, wz, wrx, wry, wrz])
                else:
                    waypoints_pose.append([wx, wy, wz])
                if rx is None:
                    rx = wrx
                if ry is None:
                    ry = wry
                if rz is None:
                    rz = wrz
            else:
                continue

        if not waypoints_pose:
            return -1

        # Transform waypoints from workobject frame to base frame if the workobject is set
        if self.workobject is not None:
            self.node.get_logger().info(f"[EXECUTE_PATH] Transforming waypoints from work object to base frame")
            waypoints_base = []
            for wp in waypoints_pose:
                if len(wp) >= 6 and orientation_mode == "per_waypoint":
                    wp_full = list(wp[:6])
                else:
                    wp_full = [wp[0], wp[1], wp[2], rx, ry, rz]
                # Transform from workobject to base frame using WorkObject.apply()
                wp_base = self.workobject.apply(wp_full)
                if orientation_mode == "per_waypoint":
                    waypoints_base.append(list(wp_base[:6]))
                else:
                    waypoints_base.append([wp_base[0], wp_base[1], wp_base[2]])
            waypoints_pose = waypoints_base
            if orientation_mode != "per_waypoint":
                # Also transform orientation to base frame
                orientation_full = [0, 0, 0, rx, ry, rz]  # dummy position, only orientation matters
                orientation_base = self.workobject.apply(orientation_full)
                rx, ry, rz = orientation_base[3], orientation_base[4], orientation_base[5]

        # Calculate average distance between consecutive waypoints
        if len(waypoints_pose) > 1:
            total_dist = 0.0
            for i in range(len(waypoints_pose) - 1):
                dx = waypoints_pose[i + 1][0] - waypoints_pose[i][0]
                dy = waypoints_pose[i + 1][1] - waypoints_pose[i][1]
                dz = waypoints_pose[i + 1][2] - waypoints_pose[i][2]
                total_dist += (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
            avg_spacing = total_dist / (len(waypoints_pose) - 1)
        else:
            avg_spacing = 0.0

        self.node.get_logger().info(f"[EXECUTE_PATH] {len(waypoints_pose)} waypoints, avg spacing: {avg_spacing:.2f}mm")
        self.node.get_logger().info(f"[EXECUTE_PATH] First waypoint: {waypoints_pose[0][:3]}")
        if orientation_mode == "per_waypoint" and len(waypoints_pose[0]) >= 6:
            self.node.get_logger().info(
                "[EXECUTE_PATH] Per-waypoint orientation enabled; first waypoint orientation: "
                f"RX={waypoints_pose[0][3]}° RY={waypoints_pose[0][4]}° RZ={waypoints_pose[0][5]}°"
            )
        else:
            self.node.get_logger().info(f"[EXECUTE_PATH] Orientation (base frame): RX={rx}° RY={ry}° RZ={rz}°")

        # A single waypoint is not a real path. Route it through the single-target
        # pipeline so adaptive interpolation / micro-move Jacobian fallback are applied
        # before MoveIt + TOTG are involved.
        if len(waypoints_pose) == 1:
            self.node.get_logger().info(
                "[EXECUTE_PATH] Single waypoint detected — delegating to single-target planner"
            )
            from motion.strategies import SingleTargetStrategy

            target = waypoints_pose[0]
            result = self.node.execute(
                SingleTargetStrategy(target[0], target[1], target[2], rx, ry, rz, vel_scale, acc_scale)
            )
        else:
            selected_optimizer = trajectory_optimizer
            if not selected_optimizer:
                selected_optimizer = str(
                    getattr(config, "PATH_TRAJECTORY_OPTIMIZER", "") or ""
                ).strip().upper() or None

            # ✅ ALWAYS use compute_cartesian_path for consistency
            # Controller's spline interpolation + TOTG provides smooth continuous motion
            self.node.get_logger().info(
                "[EXECUTE_PATH] Using MoveIt compute_cartesian_path (continuous trajectory)"
                + (f" with {selected_optimizer}" if selected_optimizer else "")
            )
            from motion.strategies import PathStrategy
            result = self.node.execute(
                PathStrategy(
                    waypoints_pose,
                    rx,
                    ry,
                    rz,
                    vel_scale,
                    acc_scale,
                    selected_optimizer,
                    orientation_mode=orientation_mode,
                )
            )
        self.node.get_logger().info(
            f"[TIMING] backend_execute_path submitted blocking={bool(blocking)} "
            f"result={result} waypoints={len(waypoints_pose)} elapsed_s={perf_counter() - started_at:.3f}"
        )

        # Return error code if planning/submission failed
        if result < 0:
            # Error codes: -2, -3, -5 etc
            return result

        if result > 0:
            if blocking:
                task_id = getattr(self.node, 'last_submitted_task_id', None)
                if task_id is not None:
                    waited = self.node.motion_queue.wait_for_task(task_id, config.BLOCKING_MOVE_TIMEOUT_S)
                    if waited is None:
                        self.node.get_logger().error(
                            f"[EXECUTE_PATH] Timed out waiting for queued path task #{task_id} to complete")
                        return -1
                    return waited
            # Positive = queued position (don't block on queued commands)
            self.node.get_logger().info(f"[EXECUTE_PATH] Command queued at position {result}")
            return result

        # result == 0: executing immediately
        if blocking:
            import time
            wait_started_at = perf_counter()
            deadline = time.time() + config.BLOCKING_MOVE_TIMEOUT_S
            while self.node.is_executing and time.time() < deadline:
                time.sleep(0.05)
            if self.node.is_executing:
                self.node.get_logger().error(
                    f"[EXECUTE_PATH] Timed out waiting for move to complete after {config.BLOCKING_MOVE_TIMEOUT_S}s")
                return -1
            self.node.get_logger().info(
                f"[TIMING] backend_execute_path blocking_wait elapsed_s={perf_counter() - wait_started_at:.3f} "
                f"total_elapsed_s={perf_counter() - started_at:.3f}"
            )
            return self.node.last_move_result

        return 0

    def execute_sequence(self, segments, tool=0, user=0, blocking=True):
        started_at = perf_counter()
        if not segments:
            return -1
        drive_error = self._reject_if_drive_not_enabled("EXECUTE_SEQUENCE")
        if drive_error is not None:
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[EXECUTE_SEQUENCE] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        try:
            tool_transform = self.node.get_tool_transform(tool)
            sequence_segments = []
            for segment in segments:
                position_base = self.apply_workobject(segment["position"], user_id=user)
                sequence_segments.append({
                    "position": position_base,
                    "vel": float(segment["vel"]),
                    "acc": float(segment["acc"]),
                    "motion_type": str(segment.get("motion_type", "linear")),
                    "blend_radius": float(segment.get("blend_radius", 0.0)),
                })
            from motion.strategies import SequenceStrategy
            result = self.node.execute(SequenceStrategy(
                sequence_segments,
                tool_transform=tool_transform,
            ))
            self.node.get_logger().info(
                f"[TIMING] backend_execute_sequence submitted blocking={bool(blocking)} "
                f"result={result} segments={len(sequence_segments)} elapsed_s={perf_counter() - started_at:.3f}"
            )
            if result != 0:
                if blocking and result > 0:
                    task_id = getattr(self.node, 'last_submitted_task_id', None)
                    if task_id is not None:
                        waited = self.node.motion_queue.wait_for_task(task_id, config.BLOCKING_MOVE_TIMEOUT_S)
                        if waited is None:
                            self.node.get_logger().error(
                                f"[EXECUTE_SEQUENCE] Timed out waiting for queued sequence task #{task_id} to complete")
                            return -1
                        return waited
                return result

            if blocking:
                import time
                wait_started_at = perf_counter()
                deadline = time.time() + config.BLOCKING_MOVE_TIMEOUT_S
                while self.node.is_executing and time.time() < deadline:
                    time.sleep(0.05)
                if self.node.is_executing:
                    self.node.get_logger().error(
                        f"[EXECUTE_SEQUENCE] Timed out waiting for sequence after {config.BLOCKING_MOVE_TIMEOUT_S}s")
                    return -1
                self.node.get_logger().info(
                    f"[TIMING] backend_execute_sequence blocking_wait elapsed_s={perf_counter() - wait_started_at:.3f} "
                    f"total_elapsed_s={perf_counter() - started_at:.3f}"
                )
                return self.node.last_move_result

            return 0
        except Exception as e:
            print(f"execute_sequence error: {e}")
            return -1

    def get_ordered_motion_chain_status(self):
        if self.node is None:
            return {"active": False}
        status = getattr(self.node, "_ordered_motion_chain_status", None)
        if not isinstance(status, dict):
            return {"active": False}
        return dict(status)

    def prepare_ordered_motion_chain(self, segments, start_position, tool=0, user=0,
                                     trajectory_optimizer=None,
                                     allow_servo_during_prepare=False):
        if not isinstance(start_position, (list, tuple)) or len(start_position) != 6:
            raise ValueError("start_position must contain 6 values")
        servo_status = self.cartesian_servo.get_status() if self.cartesian_servo is not None else None
        if servo_status is not None and str(servo_status.state.value) == "running":
            raise RuntimeError("cannot prepare an ordered motion chain while Cartesian Servo is running")
        with self._prepared_ordered_lock:
            active = [item for item in self._prepared_ordered.values()
                      if item.get("future") is None or not item["future"].done()]
            if active:
                raise RuntimeError("an ordered motion chain is already prepared or executing")
            plan_id = uuid4().hex
            authorized = Event()
            record = {
                "plan_id": plan_id,
                "state": "planning",
                "authorized": authorized,
                "created_at": time.time(),
                "result": None,
                "future": None,
                "start_position": [float(value) for value in start_position],
                "tool": int(tool),
                "user": int(user),
                "allow_servo_during_prepare": bool(allow_servo_during_prepare),
            }
            self._prepared_ordered[plan_id] = record

            def run_prepared():
                try:
                    result = self.execute_ordered_motion_chain(
                        segments=segments,
                        tool=tool,
                        user=user,
                        blocking=True,
                        trajectory_optimizer=trajectory_optimizer,
                        start_position=list(start_position),
                        execution_authorized=authorized,
                    )
                    record["result"] = int(result)
                    motion_error = getattr(self.node, "last_motion_error", None)
                    if result != 0 and motion_error:
                        record["error"] = str(motion_error)
                    record["state"] = "completed" if result == 0 else "failed"
                    return result
                except Exception as exc:
                    record["error"] = str(exc)
                    record["state"] = "failed"
                    record["result"] = -1
                    raise

            record["future"] = self._prepared_ordered_executor.submit(run_prepared)
            return self._prepared_status(record)

    def execute_prepared_ordered_motion_chain(self, plan_id, blocking=True):
        with self._prepared_ordered_lock:
            record = self._prepared_ordered.get(str(plan_id))
            if record is None:
                raise KeyError(f"unknown prepared plan {plan_id!r}")
            if record["state"] in {"completed", "failed", "discarded"}:
                raise RuntimeError(f"prepared plan is {record['state']}")
            if self.node.is_motion_active() or self.node.has_pending_motion():
                raise RuntimeError("cannot execute a prepared chain while another motion is active or queued")
            servo_status = self.cartesian_servo.get_status() if self.cartesian_servo is not None else None
            if servo_status is not None and str(servo_status.state.value) == "running":
                raise RuntimeError("cannot execute a prepared chain while Cartesian Servo is running")
            current_position = self.get_current_position(user_id=record["user"])
            if current_position is None or len(current_position) < 6:
                raise RuntimeError("cannot verify prepared chain start position")
            expected = record["start_position"]
            xyz_error_mm = math.sqrt(sum(
                (float(current_position[index]) - float(expected[index])) ** 2
                for index in range(3)
            ))
            angular_errors_deg = [
                abs((float(current_position[index]) - float(expected[index]) + 180.0) % 360.0 - 180.0)
                for index in range(3, 6)
            ]
            position_tolerance_mm = float(getattr(
                config, "PREPARED_CHAIN_START_POSITION_TOLERANCE_MM", 2.0
            ))
            orientation_tolerance_deg = float(getattr(
                config, "PREPARED_CHAIN_START_ORIENTATION_TOLERANCE_DEG", 2.0
            ))
            if (
                xyz_error_mm > position_tolerance_mm
                or max(angular_errors_deg, default=0.0) > orientation_tolerance_deg
            ):
                error = (
                    "prepared chain start mismatch: "
                    f"xyz_error_mm={xyz_error_mm:.3f} "
                    f"max_orientation_error_deg={max(angular_errors_deg, default=0.0):.3f} "
                    f"limits=({position_tolerance_mm:.3f}mm,{orientation_tolerance_deg:.3f}deg)"
                )
                # This plan can never be validly executed from its captured
                # start state. Cancel its authorization waiter before raising;
                # otherwise the record remains in planning forever and blocks
                # every later Servo request.
                setattr(self.node, "_ordered_motion_chain_stop_requested", True)
                record["state"] = "discarding"
                record["result"] = -1
                record["error"] = error
                record["authorized"].set()
                raise RuntimeError(error)
            record["state"] = "executing"
            record["authorized"].set()
            future = record["future"]
            status = self._prepared_status(record)
        if blocking:
            future.result()
            return self.get_prepared_ordered_motion_chain(plan_id)
        return status

    def has_active_prepared_ordered_motion_chain(self):
        # A client can disappear after prepare and before execute, leaving the
        # planning worker blocked on its authorization event indefinitely.
        # Expire only old, never-authorized records and only while the robot is
        # physically idle. Executing records are never cleared here.
        expiry_s = max(5.0, float(getattr(config, "PREPARED_CHAIN_ORPHAN_TTL_S", 30.0)))
        now = time.time()
        stale = []
        with self._prepared_ordered_lock:
            active = [
                record for record in self._prepared_ordered.values()
                if record["state"] not in {"completed", "failed", "discarded"}
                and (record.get("future") is None or not record["future"].done())
            ]
            robot_idle = not self.node.is_motion_active() and not self.node.has_pending_motion()
            if robot_idle:
                stale = [
                    record for record in active
                    if record["state"] == "planning"
                    and not record["authorized"].is_set()
                    and now - float(record.get("created_at", now)) >= expiry_s
                ]
                if stale:
                    # Publish cancellation before releasing authorization so a
                    # waking worker can never enter execution in between.
                    setattr(self.node, "_ordered_motion_chain_stop_requested", True)
                for record in stale:
                    record["state"] = "discarding"
                    record["result"] = -1
                    record["error"] = "prepared plan expired before authorization"
                    record["authorized"].set()

        if stale:
            for record in stale:
                future = record.get("future")
                if future is not None:
                    try:
                        future.result(timeout=1.0)
                    except Exception:
                        # Cancellation/failure is expected; timeout is handled
                        # by the fail-closed active-record check below.
                        pass
                if future is None or future.done():
                    with self._prepared_ordered_lock:
                        if record["state"] == "discarding":
                            record["state"] = "discarded"
                self.node.get_logger().warning(
                    "[OrderedChain] Discarded orphan prepared plan plan_id=%s age_s=%.3f",
                    record["plan_id"],
                    now - float(record.get("created_at", now)),
                )
            if not self.node.is_motion_active() and not self.node.has_pending_motion():
                setattr(self.node, "_ordered_motion_chain_stop_requested", False)

        with self._prepared_ordered_lock:
            remaining = [
                record for record in self._prepared_ordered.values()
                if record["state"] not in {"completed", "failed", "discarded"}
                and (record.get("future") is None or not record["future"].done())
                and not (
                    record.get("allow_servo_during_prepare", False)
                    and record["state"] == "planning"
                    and not record["authorized"].is_set()
                )
            ]
        if remaining:
            self.node.get_logger().warning(
                "[OrderedChain] Active prepared plans block Servo: %s",
                [
                    {
                        "plan_id": record["plan_id"],
                        "state": record["state"],
                        "age_s": round(now - float(record.get("created_at", now)), 3),
                        "authorized": record["authorized"].is_set(),
                    }
                    for record in remaining
                ],
            )
        return bool(remaining)

    def discard_prepared_ordered_motion_chain(self, plan_id):
        with self._prepared_ordered_lock:
            record = self._prepared_ordered.get(str(plan_id))
            if record is None:
                raise KeyError(f"unknown prepared plan {plan_id!r}")
            if record["state"] == "executing":
                raise RuntimeError("cannot discard a prepared plan after execution started")
            if record["state"] in {"completed", "failed", "discarded"}:
                return self._prepared_status(record)
            # Publish cancellation before waking the planning worker. Keep the
            # record non-terminal until that worker has actually exited so a
            # later Servo request cannot race an unfinished prepared chain.
            setattr(self.node, "_ordered_motion_chain_stop_requested", True)
            record["state"] = "discarding"
            record["result"] = -1
            record["error"] = "prepared plan discarded before authorization"
            record["authorized"].set()
            future = record.get("future")

        if future is not None:
            try:
                future.result(timeout=1.0)
            except Exception:
                pass

        with self._prepared_ordered_lock:
            if future is None or future.done():
                record["state"] = "discarded"
            status = self._prepared_status(record)
        if not self.node.is_motion_active() and not self.node.has_pending_motion():
            setattr(self.node, "_ordered_motion_chain_stop_requested", False)
        return status

    def get_prepared_ordered_motion_chain(self, plan_id):
        with self._prepared_ordered_lock:
            record = self._prepared_ordered.get(str(plan_id))
            if record is None:
                raise KeyError(f"unknown prepared plan {plan_id!r}")
            status = self._prepared_status(record)
        status["pipeline"] = self.get_ordered_motion_chain_status()
        return status

    @staticmethod
    def _prepared_status(record):
        return {
            "plan_id": record["plan_id"],
            "state": record["state"],
            "result": record.get("result"),
            "error": record.get("error"),
            "created_at": record["created_at"],
        }

    def _set_ordered_motion_chain_status(self, **updates):
        if self.node is None:
            return
        from motion.scheduling.status_adapter import normalize_ordered_chain_status

        status = normalize_ordered_chain_status(
            getattr(self.node, "_ordered_motion_chain_status", None),
            updates,
            updated_at=time.time(),
        )
        setattr(self.node, "_ordered_motion_chain_status", status)

    def execute_ordered_motion_chain(self, segments, tool=0, user=0, blocking=True,
                                     trajectory_optimizer=None, start_position=None,
                                     execution_authorized=None):
        started_at = perf_counter()
        if self.node is None or not segments:
            return -1
        # Do not let an earlier controller cancellation leak into this motion.
        self.node.last_motion_error = None
        from motion.move_linear_timing import begin as begin_motion_timing, clear as clear_motion_timing, \
            mark as mark_motion_timing

        begin_motion_timing(self.node, source="ordered_motion_chain")
        mark_motion_timing(
            self.node,
            "backend_received",
            segments=len(segments),
            blocking=bool(blocking),
            tool=tool,
            user=user,
        )
        scheduler_segment_states = {}
        from motion.scheduling.status_adapter import (
            ordered_chain_starting_status,
            ordered_chain_terminal_status,
        )

        # Resolve named per-segment limit profiles before any planning work.
        # A requested profile that is missing or malformed is a hard request error.
        config.apply_limit_profiles(segments, self.node.get_logger())

        try:
            from motion.scheduling.motion_adapters import (
                OrderedMotionBatchValidationFailure,
                validate_ordered_motion_batch_from_mappings,
            )

            batch_validation = validate_ordered_motion_batch_from_mappings(
                segments,
                blocking=blocking,
                tool=tool,
                user=user,
                trajectory_optimizer=trajectory_optimizer,
                default_velocity=config.DEFAULT_VEL_PERCENT,
                default_acceleration=config.DEFAULT_ACC_PERCENT,
            )
            scheduler_group_status = batch_validation.scheduler_group_status
            scheduler_group_states = batch_validation.scheduler_group_states
            scheduler_segment_states = batch_validation.scheduler_segment_states
            self.node.get_logger().info(batch_validation.log_message())
            mark_motion_timing(
                self.node,
                "ordered_motion_batch_adapter_validated",
                **batch_validation.timing_fields(),
            )
        except Exception as adapter_error:
            scheduler_group_status = ()
            scheduler_group_states = {}
            batch_validation_failure = OrderedMotionBatchValidationFailure(
                error=str(adapter_error)
            )
            self.node.get_logger().warning(batch_validation_failure.log_message())
            mark_motion_timing(
                self.node,
                "ordered_motion_batch_adapter_failed",
                **batch_validation_failure.timing_fields(),
            )
        setattr(self.node, "_ordered_motion_chain_stop_requested", False)
        self._set_ordered_motion_chain_status(
            **ordered_chain_starting_status(
                total_segments=len(segments),
                scheduler_group_status=scheduler_group_status,
                scheduler_group_states=scheduler_group_states,
                scheduler_segment_states=scheduler_segment_states,
            )
        )
        drive_error = self._reject_if_drive_not_enabled("EXECUTE_ORDERED_MOTION_CHAIN")
        if drive_error is not None:
            self._set_ordered_motion_chain_status(
                **ordered_chain_terminal_status(phase="rejected", result=drive_error)
            )
            mark_motion_timing(self.node, "ordered_chain_rejected", result=int(drive_error), reason="drive_not_enabled")
            clear_motion_timing(self.node, force=True)
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[EXECUTE_ORDERED_MOTION_CHAIN] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            self._set_ordered_motion_chain_status(
                **ordered_chain_terminal_status(
                    phase="rejected",
                    result=config.MOTION_ERROR_HARDWARE_NOT_READY,
                )
            )
            mark_motion_timing(
                self.node,
                "ordered_chain_rejected",
                result=int(config.MOTION_ERROR_HARDWARE_NOT_READY),
                reason="hardware_not_ready",
            )
            clear_motion_timing(self.node, force=True)
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        try:
            self.node.get_logger().info(
                f"[OrderedChain] Executing ordered motion chain with {len(segments)} segments"
            )
            result = self._execute_ordered_motion_chain_pipelined(
                segments,
                tool=tool,
                user=user,
                trajectory_optimizer=trajectory_optimizer,
                scheduler_group_status=scheduler_group_status,
                scheduler_segment_states=scheduler_segment_states,
                start_position=start_position,
                execution_authorized=execution_authorized,
            )
            if result != 0:
                self._set_ordered_motion_chain_status(
                    **ordered_chain_terminal_status(phase="failed", result=result)
                )
                return result
            self.node.get_logger().info(
                f"[TIMING] ordered_motion_chain_total result=0 elapsed_s={perf_counter() - started_at:.3f}"
            )
            mark_motion_timing(self.node, "ordered_chain_done", result=0, total_elapsed_s=perf_counter() - started_at)
            self._set_ordered_motion_chain_status(
                **ordered_chain_terminal_status(phase="completed", result=0)
            )
            return 0
        except Exception as e:
            details = traceback.format_exc()
            if self.node is not None:
                self.node.get_logger().error(f"execute_ordered_motion_chain error: {e}\n{details}")
            else:
                print(f"execute_ordered_motion_chain error: {e}\n{details}")
            self._set_ordered_motion_chain_status(
                **ordered_chain_terminal_status(phase="error", result=-1, error=str(e))
            )
            return -1
        finally:
            clear_motion_timing(self.node, force=True)

    def _execute_ordered_motion_chain_pipelined(
            self,
            segments,
            tool=0,
            user=0,
            trajectory_optimizer=None,
            scheduler_group_status=(),
            scheduler_segment_states=None,
            start_position=None,
            execution_authorized=None):
        from motion.execution.ordered_execution import (
            OrderedPlannedSequenceHooks,
            OrderedPlannedSegmentExecutorConfig,
            build_ordered_execution_hook_bundle,
            build_ordered_planned_segment_executor,
        )
        from motion.execution.trajectory_executor import _send_trajectory_to_controller
        from motion.planning.segment_planning import (
            _build_follow_path_trajectory,
            _optimize_sync,
            _plan_segment,
            _robot_state_from_trajectory_end,
            _wait_execution_complete,
        )
        from motion.planning.ordered_initial_state import build_ordered_initial_planning_state
        from motion.planning.ordered_segment_planner import (
            OrderedSegmentPlannerHooks,
            build_ordered_segment_planner_callback,
        )
        from motion.planning.ordered_unwind_planner import (
            build_ordered_unwind_direct_ik_planner,
        )
        from motion.move_linear_timing import mark as mark_motion_timing
        from motion.scheduling.ordered_scheduler_bridge import build_ordered_scheduler_runtime
        from motion.scheduling.ordered_planning_worker import (
            OrderedPlanningWorkerHooks,
            build_ordered_planning_worker_factory,
        )
        from motion.scheduling.ordered_pipeline_runner import (
            OrderedPipelineRunnerConfig,
            run_ordered_planning_and_execution,
        )
        from motion.scheduling.status_adapter import (
            ordered_chain_executing_status,
            ordered_chain_segment_finished_status,
            ordered_chain_stopped_status,
        )

        planning_node = getattr(self.node, "planner_context", self.node)
        from motion.blending.blend_builder import build_ordered_blend_builder
        ordered_blend_builder = build_ordered_blend_builder(planning_node, config)
        scheduler_runtime = build_ordered_scheduler_runtime(
            segments=segments,
            scheduler_group_status=scheduler_group_status,
            scheduler_segment_states=scheduler_segment_states,
            set_ordered_motion_chain_status=self._set_ordered_motion_chain_status,
        )
        #
        # Refresh safety/collision state once before preplanning
        # the complete ordered chain.
        #
        # planning_node.force_safety_update()
        tool_transform = self.node.get_tool_transform(tool)
        explicit_start_cartesian = None
        explicit_start_state = None
        if start_position is not None:
            from motion.planning.planner_utils import _to_pose_list
            from motion.planning.single_target import _resolve_start_state

            explicit_start_cartesian = list(
                self.apply_workobject(start_position, user_id=user)[:6]
            )
            start_poses, start_error = _to_pose_list(
                planning_node,
                [explicit_start_cartesian],
                tool_transform,
            )
            if start_error is not None or not start_poses:
                raise ValueError("prepared chain start_position failed safety validation")
            explicit_start_state = _resolve_start_state(planning_node, start_poses[0])
            if explicit_start_state is None:
                raise RuntimeError("prepared chain start_position IK could not be resolved")
        initial_state = build_ordered_initial_planning_state(
            node=self.node,
            planning_node=planning_node,
            config_obj=config,
            trajectory_optimizer=trajectory_optimizer,
            start_cartesian=explicit_start_cartesian,
            start_state=explicit_start_state,
        )
        mark_motion_timing(
            self.node,
            "ordered_initial_state_ready",
            duration_s=initial_state.duration_s,
            optimizer=initial_state.selected_optimizer or "default",
        )

        plan_unwind_direct_ik_trajectory = build_ordered_unwind_direct_ik_planner(
            planning_node=planning_node,
            config_obj=config,
            tool_transform=tool_transform,
            rotational_path_waypoints_base=self._rotational_path_waypoints_base,
            optimize_sync=_optimize_sync,
        )

        plan_ordered_segment_callback = build_ordered_segment_planner_callback(
            OrderedSegmentPlannerHooks(
                node=self.node,
                planning_node=planning_node,
                config_obj=config,
                tool_transform=tool_transform,
                user=user,
                selected_optimizer=initial_state.selected_optimizer,
                total_segments=len(segments),
                apply_workobject=self.apply_workobject,
                mark_motion_timing=mark_motion_timing,
                plan_segment=_plan_segment,
                optimize_sync=_optimize_sync,
                build_follow_path_trajectory=_build_follow_path_trajectory,
                robot_state_from_trajectory_end=_robot_state_from_trajectory_end,
                clamp_percentage=self.node.trajectory_executor._clamp_percentage,
                canonical_angle=self.node.trajectory_executor._canonical_angle,
                plan_unwind_direct_ik_trajectory=plan_unwind_direct_ik_trajectory,
            )
        )
        raw_plan_ordered_segment_callback = plan_ordered_segment_callback

        def plan_ordered_segment_callback(index, segment, *args, **kwargs):
            planned = raw_plan_ordered_segment_callback(index, segment, *args, **kwargs)
            if segment.get('limit_profile'):
                planned['limit_profile'] = segment['limit_profile']
                planned['_joint_rate_limits_rad_s'] = dict(segment.get('_joint_rate_limits_rad_s', {}))
            return planned

        execution_hooks = build_ordered_execution_hook_bundle(
            node=self.node,
            logger=planning_node.get_logger(),
            config_obj=config,
            set_ordered_motion_chain_status=self._set_ordered_motion_chain_status,
            ordered_chain_executing_status=ordered_chain_executing_status,
            ordered_chain_segment_finished_status=ordered_chain_segment_finished_status,
            mark_motion_timing=mark_motion_timing,
            publish_scheduler_updates=scheduler_runtime.publish_scheduler_updates,
            send_trajectory=_send_trajectory_to_controller,
            wait_execution_complete=_wait_execution_complete,
            verify_explicit_unwind_complete=(
                self.node.trajectory_executor._verify_explicit_unwind_complete
            ),
        )

        scheduler_bridge = scheduler_runtime.bridge
        plan_timeout_s = float(getattr(config, "CUSTOM_SEQUENCE_PLAN_TIMEOUT_S", 10.0)) + 30.0

        _execute_planned_segment = build_ordered_planned_segment_executor(
            execution_hooks=execution_hooks,
            scheduler_bridge=scheduler_bridge,
            config=OrderedPlannedSegmentExecutorConfig(
                min_timeout_s=float(getattr(config, "EXECUTOR_TIME_MIN_S", 5.0)),
                timeout_multiplier=float(
                    getattr(config, "EXECUTOR_TIME_MULTIPLIER", 2.0)
                ),
                motion_error_result=config.MOTION_ERROR_CONTROLLER_EXECUTION_FAILED,
                default_velocity_percent=config.DEFAULT_VEL_PERCENT,
                default_acceleration_percent=config.DEFAULT_ACC_PERCENT,
                verification_failure_result=-6,
            ),
            runtime_unwind=self._unwind_joint6_with_rotational_path,
            now_s=time.time,
        )

        planning_worker_factory = build_ordered_planning_worker_factory(
            hooks=OrderedPlanningWorkerHooks(
                node=self.node,
                mark_motion_timing=mark_motion_timing,
                publish_planned=scheduler_runtime.publish_planned,
                plan_ordered_segment=plan_ordered_segment_callback,
                blend_builder=ordered_blend_builder,
                optimize_sync=lambda moveit_trajectory, vel_scale, acc_scale, **kwargs: _optimize_sync(
                    planning_node,
                    moveit_trajectory,
                    vel_scale,
                    acc_scale,
                    **kwargs,
                ),
                robot_state_from_trajectory_end=_robot_state_from_trajectory_end,
                apply_workobject=self.apply_workobject,
                tool_transform=tool_transform,
                user=user,
            ),
            segments=segments,
            start_cartesian=initial_state.start_cartesian,
            start_state=initial_state.start_state,
            scheduler_bridge=scheduler_bridge,
            selected_optimizer=initial_state.selected_optimizer,
        )

        return run_ordered_planning_and_execution(
            node=self.node,
            mark_motion_timing=mark_motion_timing,
            planning_worker_factory=planning_worker_factory,
            sequence_hooks=OrderedPlannedSequenceHooks(
                node=self.node,
                logger=planning_node.get_logger(),
                set_ordered_motion_chain_status=self._set_ordered_motion_chain_status,
                ordered_chain_stopped_status=ordered_chain_stopped_status,
                mark_motion_timing=mark_motion_timing,
                execute_planned_segment=_execute_planned_segment,
            ),
            scheduler_bridge=scheduler_bridge,
            segments_count=len(segments),
            config=OrderedPipelineRunnerConfig(
                plan_timeout_s=plan_timeout_s,
                previous_execution_suppress=initial_state.previous_execution_suppress,
                stopped_result=-14,
            ),
            execution_authorized=execution_authorized,
        )

    def unwind_joint6(self, blocking=True, queue_if_busy=True, vel=None, acc=None):
        if self.node is None:
            return -1
        last_ordered_unwind_failure = getattr(self.node, "_last_ordered_unwind_failure_time", None)
        if last_ordered_unwind_failure is not None:
            suppress_s = max(
                0.0,
                float(getattr(config, "EXECUTOR_SUPPRESS_UNWIND_AFTER_ORDERED_FAILURE_S", 10.0)),
            )
            elapsed_s = time.time() - float(last_ordered_unwind_failure)
            if elapsed_s <= suppress_s:
                result = int(getattr(self.node, "_last_ordered_unwind_failure_result", -6))
                self.node.get_logger().error(
                    "[UNWIND_J6] Standalone unwind suppressed after ordered unwind failure "
                    f"{elapsed_s:.3f}s ago; refusing automatic duplicate unwind result={result}"
                )
                return result
            setattr(self.node, "_last_ordered_unwind_failure_time", None)
            setattr(self.node, "_last_ordered_unwind_failure_result", None)
        drive_error = self._reject_if_drive_not_enabled("UNWIND_J6")
        if drive_error is not None:
            return drive_error
        if not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[UNWIND_J6] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY

        result = self._unwind_joint6_with_rotational_path(
            vel=vel,
            acc=acc,
            queue_if_busy=queue_if_busy,
        )

        if result < 0:
            return result

        if result > 0:
            if blocking:
                task_id = getattr(self.node, 'last_submitted_task_id', None)
                if task_id is not None:
                    waited = self.node.motion_queue.wait_for_task(
                        task_id,
                        config.BLOCKING_MOVE_TIMEOUT_S,
                    )
                    if waited is None:
                        self.node.get_logger().error(
                            f"[UNWIND_J6] Timed out waiting for queued unwind task #{task_id} to complete"
                        )
                        return -1
                    return waited
            return result

        if blocking:
            deadline = time.time() + config.BLOCKING_MOVE_TIMEOUT_S
            while self.node.is_executing and time.time() < deadline:
                time.sleep(0.05)
            if self.node.is_executing:
                self.node.get_logger().error(
                    f"[UNWIND_J6] Timed out waiting for unwind to complete after {config.BLOCKING_MOVE_TIMEOUT_S}s"
                )
                return -1
            return self.node.last_move_result

        return 0

    def joint_jog(self, joint, direction, step, vel, acc, blocking=True):
        if self.node is None:
            return -1
        drive_error = self._reject_if_drive_not_enabled("JOINT_JOG")
        if drive_error is not None:
            return drive_error
        if not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[JOINT_JOG] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY

        joint_names = list(getattr(config, "JOINT_NAMES", []) or [])
        joint_name = str(joint or "").strip()
        aliases = {f"J{index}": f"Joint_{index}" for index in range(1, len(joint_names) + 1)}
        joint_name = aliases.get(joint_name.upper(), joint_name)
        if joint_name not in joint_names:
            self.node.get_logger().error(
                f"[JOINT_JOG] Joint {joint!r} is not configured; valid={joint_names}"
            )
            return -1

        current_positions = self.node.trajectory_executor._get_latest_joint_state_in_trajectory_order(joint_names)
        if current_positions is None:
            self.node.get_logger().error("[JOINT_JOG] Latest joint state unavailable")
            return -1

        try:
            direction_value = direction.value if hasattr(direction, "value") else int(direction)
        except (TypeError, ValueError):
            direction_name = str(direction).strip().upper()
            direction_value = 1 if direction_name in {"PLUS", "POSITIVE", "+", "1"} else -1

        delta_rad = math.radians(abs(float(step))) * (1.0 if direction_value >= 0 else -1.0)
        joint_index = joint_names.index(joint_name)
        start_positions = [float(value) for value in current_positions]
        target_positions = list(start_positions)
        target_positions[joint_index] += delta_rad

        vel_percent = self.node.trajectory_executor._clamp_percentage(vel)
        rate_limits = dict(getattr(config, "SINGLE_TARGET_JOINT_RATE_LIMITS_RAD_S", {}) or {})
        max_rate = float(rate_limits.get(joint_name, rate_limits.get(joint_name.lower(), 1.2)) or 1.2)
        max_rate = max(0.05, max_rate * max(0.01, vel_percent / 100.0))
        duration_s = max(0.2, abs(delta_rad) / max_rate)
        sec = int(duration_s)
        nanosec = int((duration_s - sec) * 1_000_000_000)

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = self.node.get_clock().now().to_msg()

        start_pt = JointTrajectoryPoint()
        start_pt.positions = start_positions
        start_pt.velocities = [0.0] * len(joint_names)
        start_pt.accelerations = [0.0] * len(joint_names)
        start_pt.time_from_start = Duration(sec=0, nanosec=0)

        end_pt = JointTrajectoryPoint()
        end_pt.positions = target_positions
        end_pt.velocities = [0.0] * len(joint_names)
        end_pt.accelerations = [0.0] * len(joint_names)
        end_pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points = [start_pt, end_pt]

        self.node.get_logger().info(
            f"[JOINT_JOG] {joint_name} {start_positions[joint_index]:.6f} -> "
            f"{target_positions[joint_index]:.6f} rad step={float(step):.3f}deg "
            f"direction={int(direction_value):+d} vel={vel_percent:.1f}% duration={duration_s:.3f}s"
        )
        self.node.trajectory_executor.send_trajectory_to_controller(traj)

        if blocking:
            return self._wait_for_motion_idle_result(
                float(getattr(config, "BLOCKING_MOVE_TIMEOUT_S", 60.0)),
                "[JOINT_JOG]",
            )
        return 0

    def _unwind_joint6_with_rotational_path(self, vel=None, acc=None, queue_if_busy=True):
        joint_names = list(getattr(config, 'JOINT_NAMES', []) or [])
        joint_name = str(getattr(config, 'EXECUTOR_POST_UNWIND_JOINT_NAME', 'Joint_6')).strip()
        if joint_name not in joint_names:
            self.node.get_logger().error(f'[UNWIND_J6] Joint {joint_name!r} is not configured')
            return -1

        axis_index = int(getattr(config, 'EXECUTOR_POST_UNWIND_ROTATION_AXIS_INDEX', 5))
        if axis_index < 3 or axis_index > 5:
            self.node.get_logger().error(f'[UNWIND_J6] Invalid unwind rotation axis index: {axis_index}')
            return -1

        joint_index = joint_names.index(joint_name)
        current_positions = self.node.trajectory_executor._get_latest_joint_state_in_trajectory_order(joint_names)
        if current_positions is None:
            self.node.get_logger().error('[UNWIND_J6] Latest joint state unavailable')
            return -1

        initial_value = float(current_positions[joint_index])
        final_target = self.node.trajectory_executor._canonical_angle(initial_value)
        min_delta = float(getattr(config, 'EXECUTOR_POST_UNWIND_MIN_DELTA_RAD', 0.5))
        remaining = final_target - initial_value
        if abs(remaining) < min_delta:
            self.node.get_logger().info(
                '[UNWIND_J6] Rotational-path unwind skipped - no unwind needed '
                f'({joint_name} current={initial_value:.4f}rad '
                f'target={final_target:.4f}rad '
                f'delta={remaining:.4f}rad '
                f'min_delta={min_delta:.4f}rad)'
            )
            self.node.last_move_result = 0
            return 0

        vel_percent = self.node.trajectory_executor._clamp_percentage(vel)
        acc_percent = self.node.trajectory_executor._clamp_percentage(acc)
        vel_scale = vel_percent / 100.0
        acc_scale = acc_percent / 100.0
        sign = float(getattr(config, 'EXECUTOR_POST_UNWIND_ROTATION_AXIS_SIGN', 1.0))
        if abs(sign) < 1e-9:
            sign = 1.0
        max_step_deg = max(1.0, abs(float(getattr(config, 'EXECUTOR_POST_UNWIND_ROTATIONAL_SEGMENT_DEG', 180.0))))
        total_delta_deg = math.degrees(remaining) * sign
        segment_count = max(1, int(math.ceil(abs(total_delta_deg) / max_step_deg)))
        self.node.get_logger().info(
            '[UNWIND_J6] Executing rotational-path unwind: '
            f'{joint_name} {initial_value:.3f} -> {final_target:.3f} rad '
            f'delta={remaining:.3f} rad cart_axis={axis_index} cart_delta={total_delta_deg:.3f}deg '
            f'segments={segment_count} max_segment={max_step_deg:.1f}deg '
            f'vel={vel_percent:.1f}% acc={acc_percent:.1f}%'
        )

        for segment_index in range(1, segment_count + 1):
            active_user_id = int(self.active_workobject_user_id)
            current_pos_wobj = self.get_current_position(user_id=active_user_id)
            if current_pos_wobj is None or len(current_pos_wobj) < 6:
                self.node.get_logger().error('[UNWIND_J6] Current Cartesian pose unavailable')
                return -1

            current_positions = self.node.trajectory_executor._get_latest_joint_state_in_trajectory_order(joint_names)
            if current_positions is None:
                self.node.get_logger().error('[UNWIND_J6] Latest joint state unavailable')
                return -1
            current_value = float(current_positions[joint_index])
            remaining = final_target - current_value
            remaining_deg = math.degrees(remaining) * sign
            if abs(remaining) < min_delta:
                break

            segment_delta_deg = math.copysign(
                min(abs(remaining_deg), max_step_deg),
                remaining_deg,
            )
            target_pos_wobj = list(current_pos_wobj[:6])
            target_pos_wobj[axis_index] = float(target_pos_wobj[axis_index]) + segment_delta_deg
            self.node.get_logger().info(
                f'[UNWIND_J6] Rotational unwind segment {segment_index}/{segment_count}: '
                f'{current_value:.3f} -> {final_target:.3f} rad, cart_delta={segment_delta_deg:.3f}deg'
            )

            result = self._send_rotational_unwind_path(
                current_pos_wobj,
                target_pos_wobj,
                axis_index,
                vel_scale,
                acc_scale,
                joint_name=joint_name,
                joint_start=current_value,
                joint_target=current_value + math.radians(segment_delta_deg) / sign,
                user_id=active_user_id,
            )
            if result != 0:
                return result

            result = self._wait_for_motion_idle_result(
                float(getattr(config, 'BLOCKING_MOVE_TIMEOUT_S', 60.0)),
                '[UNWIND_J6]',
            )
            if result != 0:
                return result

        check = {
            'joint_names': joint_names,
            'joint_name': joint_name,
            'joint_index': joint_index,
            'target_value': final_target,
        }
        if self.node.trajectory_executor._verify_explicit_unwind_complete(check):
            return 0
        return -6

    def _wait_for_motion_idle_result(self, timeout_s, label):
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            if (
                    not self.node.is_executing
                    and not self.node.is_motion_active()
                    and not self.node.has_pending_motion()
            ):
                return int(getattr(self.node, 'last_move_result', -1))
            time.sleep(0.01)
        self.node.get_logger().error(
            f'{label} Timed out waiting for motion to complete after {float(timeout_s):.2f}s'
        )
        return -1

    @staticmethod
    def _force_joint_branch_on_trajectory(joint_trajectory, joint_name, start_value, target_value, logger=None):
        if joint_trajectory is None or not getattr(joint_trajectory, 'points', None):
            return
        joint_names = list(getattr(joint_trajectory, 'joint_names', []) or [])
        if joint_name not in joint_names:
            return
        joint_index = joint_names.index(joint_name)
        points = list(joint_trajectory.points)
        if len(points) < 2:
            return

        start_value = float(start_value)
        target_value = float(target_value)
        delta = target_value - start_value
        for point_index, point in enumerate(points):
            positions = list(point.positions)
            if joint_index >= len(positions):
                continue
            fraction = point_index / max(1, len(points) - 1)
            positions[joint_index] = start_value + delta * fraction
            point.positions = positions

        if logger is not None:
            logger.info(
                '[UNWIND_J6] Forced unwind joint branch before optimization: '
                f'{joint_name} {start_value:.3f} -> {target_value:.3f} rad '
                f'points={len(points)}'
            )

    def _send_rotational_unwind_path(
            self,
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            vel_scale,
            acc_scale,
            joint_name=None,
            joint_start=None,
            joint_target=None,
            user_id=0,
    ):
        if bool(getattr(config, 'EXECUTOR_POST_UNWIND_USE_DIRECT_IK', False)):
            result = self._send_rotational_unwind_direct_ik_path(
                current_pos_wobj,
                target_pos_wobj,
                rotation_index,
                vel_scale,
                acc_scale,
                joint_name=joint_name,
                joint_start=joint_start,
                joint_target=joint_target,
                user_id=user_id,
            )
            if result == 0:
                return 0
            if not bool(getattr(config, 'EXECUTOR_POST_UNWIND_DIRECT_IK_FALLBACK_CARTESIAN', True)):
                return result
            self.node.get_logger().warning(
                f'[UNWIND_J6] Direct IK unwind path failed with result={result}; falling back to Cartesian path'
            )
        return self._send_rotational_jog_path(
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            vel_scale,
            acc_scale,
            user_id=user_id,
        )

    def _send_rotational_unwind_direct_ik_path(
            self,
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            vel_scale,
            acc_scale,
            joint_name=None,
            joint_start=None,
            joint_target=None,
            user_id=0,
    ):
        started_at = perf_counter()
        direct_ik_step_deg = max(0.1, float(getattr(config, 'EXECUTOR_POST_UNWIND_DIRECT_IK_STEP_DEG', 4.0)))
        waypoints_base = self._rotational_path_waypoints_base(
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            max_step_override_deg=direct_ik_step_deg,
            user_id=user_id,
        )
        if len(waypoints_base) < 2:
            return -1

        try:
            from motion.execution.trajectory_executor import _send_trajectory_to_controller
            from motion.planning.segment_planning import _optimize_sync
            from motion.planning.direct_contour_ik import _build_direct_contour_trajectory, _log_report
            from motion.planning.planner_utils import _begin_execution, _to_pose_list
        except Exception as exc:
            self.node.get_logger().warning(f'[UNWIND_J6] Direct IK imports unavailable: {exc}')
            return -1

        planning_node = getattr(self.node, 'planner_context', self.node)
        poses, err = _to_pose_list(
            planning_node,
            waypoints_base,
            planning_node.T_tool,
            check_last_only=True,
        )
        if err:
            return err

        try:
            ik_result = _build_direct_contour_trajectory(planning_node, poses)
        except Exception as exc:
            self.node.get_logger().warning(f'[UNWIND_J6] Direct IK unwind exception: {exc}')
            return -1

        ik_result.report.timings['total_before_optimizer_s'] = perf_counter() - started_at
        _log_report(planning_node, ik_result.report)
        if not ik_result.report.ok:
            return -6

        if joint_name is not None and joint_start is not None and joint_target is not None:
            self._force_joint_branch_on_trajectory(
                ik_result.trajectory.joint_trajectory,
                str(joint_name),
                float(joint_start),
                float(joint_target),
                logger=self.node.get_logger(),
            )

        optimizer_name = str(
            getattr(config, 'EXECUTOR_POST_UNWIND_DIRECT_IK_OPTIMIZER', '') or '').strip().upper() or None
        try:
            optimized, optimize_elapsed = _optimize_sync(
                planning_node,
                ik_result.trajectory,
                vel_scale,
                acc_scale,
                optimizer_name=optimizer_name,
            )
        except Exception as exc:
            self.node.get_logger().error(f'[UNWIND_J6] Direct IK time parameterization failed: {exc}')
            return -7

        joint_trajectory = optimized.joint_trajectory
        generation = _begin_execution(planning_node)
        planning_node._last_cartesian_request_kind = 'unwind_direct_ik'
        planning_node._last_cartesian_request_waypoints = len(waypoints_base)
        planning_node._last_cartesian_request_started_at = started_at
        self.node.get_logger().info(
            f'[TIMING] unwind_direct_ik waypoints={len(waypoints_base)} '
            f'points={len(getattr(joint_trajectory, "points", []) or [])} '
            f'optimize_s={optimize_elapsed:.3f} elapsed_s={perf_counter() - started_at:.3f} '
            f'generation={generation}'
        )
        _send_trajectory_to_controller(planning_node, joint_trajectory)
        return 0

    def _rotational_path_waypoints_base(
            self,
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            max_step_override_deg=None,
            apply_workobject_to_waypoints=True,
            user_id=0,
    ):
        angular_delta = float(target_pos_wobj[rotation_index]) - float(current_pos_wobj[rotation_index])
        if max_step_override_deg is None:
            max_step = self._rotational_jog_max_step_deg()
        else:
            max_step = max(0.1, float(max_step_override_deg))
        step_count = max(2, int(math.ceil(abs(angular_delta) / max_step)))
        waypoints_base = []
        for step_index in range(1, step_count + 1):
            alpha = step_index / float(step_count)
            waypoint = list(current_pos_wobj[:6])
            waypoint[3] = float(current_pos_wobj[3]) + (float(target_pos_wobj[3]) - float(current_pos_wobj[3])) * alpha
            waypoint[4] = float(current_pos_wobj[4]) + (float(target_pos_wobj[4]) - float(current_pos_wobj[4])) * alpha
            waypoint[5] = float(current_pos_wobj[5]) + (float(target_pos_wobj[5]) - float(current_pos_wobj[5])) * alpha
            if apply_workobject_to_waypoints:
                waypoint = self.apply_workobject(waypoint, user_id=user_id)
            waypoints_base.append(list(waypoint[:6]))
        return waypoints_base

    def enable_safety_walls(self):
        """Enable safety walls in the ROS controller."""
        if self.node is None:
            return {"enabled": False, "error": "Robot controller unavailable"}
        return self.node.enable_safety_walls()

    def disable_safety_walls(self):
        """Disable safety walls in the ROS controller."""
        if self.node is None:
            return {"enabled": False, "error": "Robot controller unavailable"}
        return self.node.disable_safety_walls()

    def get_safety_walls_status(self):
        """Return safety wall status from the ROS controller."""
        if self.node is None:
            return {"enabled": False, "error": "Robot controller unavailable"}
        return self.node.get_safety_walls_status()

    def get_motion_passage_status(self, passage_id=None):
        """Return reusable motion-passage state from the ROS controller."""
        if self.node is None:
            return {"error": "Robot controller unavailable"}
        return self.node.get_motion_passage_status(passage_id)

    def set_motion_passage_closed(self, passage_id, closed):
        """Add or remove a motion-passage collision lid."""
        if self.node is None:
            return {"error": "Robot controller unavailable"}
        return self.node.set_motion_passage_closed(passage_id, bool(closed))

    # ---------------- Status Methods ----------------
    def get_current_position(self, user_id=None):
        """
        Retrieves the current TCP (tool center point) position.

        Returns:
            list: Current robot TCP pose [x, y, z, rx, ry, rz] in mm/degrees or None on error
        """
        if self.node is None:
            return None

        # For live current-position queries, prefer the monitor's latest snapshot
        # over the controller's 50 Hz stable store to avoid reporting lag.
        if getattr(self.node, "monitor", None) is not None:
            data = self.node.monitor.get_latest_data()
        else:
            data = self.node.get_latest_data()
        if data is None or 'cartesian' not in data:
            return None

        try:
            # Use cartesian from robot_monitor.py (already in mm from C++ node)
            pose = data['cartesian'].tolist()

            # Transform from base to the requested workobject frame, or to the
            # active workobject frame for state/current-position queries.
            resolved_user_id = self.active_workobject_user_id if user_id is None else int(user_id)
            active_workobject = self.get_workobject(resolved_user_id)
            if active_workobject is not None:
                pose = active_workobject.apply(pose, inverse=True)

            return pose
        except Exception as e:
            print(f"get_current_position error: {e}")
            return None

    def get_current_base_tcp_position(self):
        """Return the active TCP pose in robot-base coordinates.

        This deliberately does not apply any WorkObject transform. It is the
        calibration-safe pose used to define WorkObjects with any calibrated
        active tool.
        """
        if self.node is None:
            return None
        if getattr(self.node, "monitor", None) is not None:
            data = self.node.monitor.get_latest_data()
        else:
            data = self.node.get_latest_data()
        if data is None or "cartesian" not in data:
            return None
        try:
            return data["cartesian"].tolist()
        except Exception as e:
            print(f"get_current_base_tcp_position error: {e}")
            return None

    def get_current_flange_position(self):
        """
        Retrieves the current unmodified Cartesian source pose.

        For Fairino this is the robot flange pose reported by the native
        controller before ee_link/tool transforms are applied by RobotMonitor.

        Returns:
            list: Current flange/source pose [x, y, z, rx, ry, rz] in mm/degrees or None on error
        """
        if self.node is None or getattr(self.node, "monitor", None) is None:
            return None

        data = self.node.monitor.get_latest_data()
        if data is None or 'cartesian_source' not in data:
            return None

        try:
            return data['cartesian_source'].tolist()
        except Exception as e:
            print(f"get_current_flange_position error: {e}")
            return None

    def get_current_joints(self):
        """
        Retrieves current joint positions.

        Returns:
            dict: Joint positions in radians and degrees, or None on error.
        """
        if self.node is None:
            return None

        joint_state = getattr(self.node, "current_joint_state", None)
        if joint_state is None:
            return None

        try:
            state_names = list(getattr(joint_state, "name", []) or [])
            state_positions = list(getattr(joint_state, "position", []) or [])
            if not state_names or len(state_names) != len(state_positions):
                return None
            position_by_name = {
                str(name): float(position)
                for name, position in zip(state_names, state_positions)
            }
            names = [str(name) for name in getattr(config, "JOINT_NAMES", [])]
            if not names:
                names = state_names[:6]
            missing = [name for name in names[:6] if name not in position_by_name]
            if missing:
                self.node.get_logger().warning(
                    f"[STATE] Current joint state missing configured joints {missing}; "
                    f"available={state_names}"
                )
                return None
            names = names[:6]
            radians = [position_by_name[name] for name in names]
            degrees = [math.degrees(value) for value in radians]
            return {
                "names": names,
                "radians": radians,
                "degrees": degrees,
            }
        except Exception as e:
            print(f"get_current_joints error: {e}")
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

    def wait_for_position(self, target_position, threshold=config.BLOCKING_POS_THRESHOLD_MM,
                          timeout=config.BLOCKING_MOVE_TIMEOUT_S,
                          check_interval=config.BLOCKING_CHECK_INTERVAL_S):
        import time, math
        if len(target_position) < 3:
            return False
        target_xyz = target_position[:3]
        start_time = time.time()
        current_xyz = None
        distance = None

        while True:
            if time.time() - start_time > timeout:
                self.node.get_logger().error(
                    f'[WAIT_POS] Timeout ({timeout}s) — never reached {[f"{v:.2f}" for v in target_xyz]}'
                    + (f' | current={[f"{v:.2f}" for v in current_xyz]}'
                       f' | delta={distance:.2f}mm' if current_xyz is not None else ' | no position data'))
                return False

            if self.node is None:
                time.sleep(check_interval)
                continue

            data = self.node.get_latest_data()
            if data is None or 'cartesian' not in data:
                time.sleep(check_interval)
                continue

            try:
                current_xyz = data['cartesian'][:3].tolist()
                distance = math.sqrt(sum((current_xyz[i] - target_xyz[i]) ** 2 for i in range(3)))
                if distance < threshold:
                    self.node.get_logger().info(f'[WAIT_POS] ✓ Reached (dist={distance:.3f}mm)')
                    return True
            except Exception:
                pass

            time.sleep(check_interval)

    # ---------------- Jog / Control / Misc ----------------
    def start_jog(self, axis: RobotAxis, direction: Direction, step, vel, acc, *, frame=None, tool=0, user=0):
        if frame is None:
            frame = CartesianServoFrame.USER
        else:
            frame = CartesianServoFrame(frame)
        return self._planned_jog.start_jog(
            axis,
            direction,
            step,
            vel,
            acc,
            frame=frame,
            tool=tool,
            user=user,
        )

    def start_servo_jog(
        self,
        axis: RobotAxis,
        direction: Direction,
        vel=None,
        acc=None,
        *,
        frame=None,
        tool=0,
        user=0,
        linear_mm_s=None,
        angular_deg_s=None,
        disable_collision_checking=False,
    ):
        if frame is None:
            frame = CartesianServoFrame.USER
        else:
            frame = CartesianServoFrame(frame)
        return self._servo_jog.start_continuous_jog(
            axis,
            direction,
            vel,
            acc,
            frame=frame,
            tool=tool,
            user=user,
            linear_mm_s=linear_mm_s,
            angular_deg_s=angular_deg_s,
            disable_collision_checking=disable_collision_checking,
        )

    def stop_servo_jog(self, *, restore_collision_checking=True):
        return self._servo_jog.stop_continuous_jog(
            restore_collision_checking=restore_collision_checking
        )

    def _start_planned_jog(self, axis: RobotAxis, direction: Direction, step, vel, acc, *, frame=None, tool=0, user=0):
        drive_error = self._reject_if_drive_not_enabled("JOG")
        if drive_error is not None:
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                f"[JOG] Rejected: {self.node.get_hardware_fault_reason()}"
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        self.node.get_logger().info(
            f"Starting jog: axis={axis}, direction={direction}, step={step}mm, vel={vel}%, acc={acc}%"
        )

        if self.node is None or self.node.prev_cartesian is None:
            return -1

        if self.node.is_motion_active() or self.node.has_pending_motion():
            self.node.get_logger().info('[JOG] Busy or queued motion pending — ignoring')
            return -1

        axis_val = axis.value if hasattr(axis, 'value') else axis
        dir_val = direction.value if hasattr(direction, 'value') else direction

        if axis_val not in [1, 2, 3, 4, 5, 6] or dir_val not in [1, -1]:
            return -1

        current_pos_wobj = self.get_current_position(user_id=user)
        if current_pos_wobj is None or len(current_pos_wobj) < 6:
            return -1

        x, y, z, rx, ry, rz = current_pos_wobj
        deltas = [0.0] * 6
        deltas[axis_val - 1] = step * dir_val
        target_pos_wobj = [
            x + deltas[0], y + deltas[1], z + deltas[2],
            rx + deltas[3], ry + deltas[4], rz + deltas[5]
        ]

        new_pos_base = self.apply_workobject(target_pos_wobj, user_id=user)

        vel_scale = max(0.0, min(1.0, vel / 100.0))
        acc_scale = max(0.0, min(1.0, acc / 100.0))

        try:
            if axis_val >= 4 and self._should_interpolate_rotational_jog(deltas[axis_val - 1]):
                result = self._send_rotational_jog_path(
                    current_pos_wobj,
                    target_pos_wobj,
                    axis_val - 1,
                    vel_scale,
                    acc_scale,
                    user_id=user,
                )
            else:
                x_b, y_b, z_b, rx_b, ry_b, rz_b = new_pos_base
                from config import resolve_avoid_collisions
                avoid_collisions = resolve_avoid_collisions(getattr(config, 'JOG_AVOID_COLLISIONS', True))
                result = self.node.send_cartesian_goal(
                    x_b, y_b, z_b, rx_b, ry_b, rz_b,
                    vel_scale=vel_scale, acc_scale=acc_scale,
                    queue_if_busy=False,
                    avoid_collisions=avoid_collisions,
                )
            if result != 0:
                return result

            import time
            timeout_s = self._jog_blocking_timeout_s(axis_val, deltas, vel)
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                if (
                        not self.node.is_executing
                        and not self.node.is_motion_active()
                        and not self.node.has_pending_motion()
                ):
                    return self.node.last_move_result
                time.sleep(0.01)

            self.node.get_logger().error(
                f'[JOG] Timed out waiting for jog motion to complete after {timeout_s:.2f}s'
            )
            return -1
        except Exception as e:
            self.node.get_logger().error(f"Jog error: {e}")
            return -1

    @property
    def cartesian_servo(self) -> CartesianServo | None :
        return self._cartesian_servo

    @staticmethod
    def _rotational_jog_max_step_deg():
        return max(0.1, float(getattr(config, 'JOG_MAX_ORIENTATION_STEP_DEG', 5.0)))

    def _should_interpolate_rotational_jog(self, angular_delta_deg):
        return abs(float(angular_delta_deg)) > self._rotational_jog_max_step_deg()

    def _jog_blocking_timeout_s(self, axis_val, deltas, vel_percent):
        base_timeout_s = float(getattr(config, 'JOG_BLOCKING_TIMEOUT_S', 5.0))
        axis_index = int(axis_val) - 1
        if axis_index < 3:
            return base_timeout_s

        angular_delta_deg = abs(float(deltas[axis_index]))
        if not self._should_interpolate_rotational_jog(angular_delta_deg):
            return base_timeout_s

        velocity_percent = max(1.0, abs(float(vel_percent)))
        estimated_motion_s = angular_delta_deg / velocity_percent * 0.4
        return max(base_timeout_s, estimated_motion_s + 3.0)

    def _send_rotational_jog_path(self, current_pos_wobj, target_pos_wobj, rotation_index, vel_scale, acc_scale, user_id=0):
        angular_delta = float(target_pos_wobj[rotation_index]) - float(current_pos_wobj[rotation_index])
        max_step = self._rotational_jog_max_step_deg()
        waypoints_base = self._rotational_path_waypoints_base(
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            user_id=user_id,
        )
        target_base = waypoints_base[-1]
        selected_optimizer = str(
            getattr(config, "PATH_TRAJECTORY_OPTIMIZER", "") or ""
        ).strip().upper() or None
        self.node.get_logger().info(
            f'[JOG] Rotational jog path: axis_index={rotation_index} '
            f'delta={angular_delta:.3f}deg waypoints={len(waypoints_base)} '
            f'max_step={max_step:.3f}deg optimizer={selected_optimizer or "default"}'
        )
        self.node.get_logger().info(
            '[JOG] Rotational jog user frame: '
            f'start_xyz={[round(v, 3) for v in current_pos_wobj[:3]]} '
            f'start_rpy={[round(v, 3) for v in current_pos_wobj[3:6]]} '
            f'target_rpy={[round(v, 3) for v in target_pos_wobj[3:6]]}'
        )
        self.node.get_logger().info(
            '[JOG] Rotational jog base target: '
            f'xyz={[round(v, 3) for v in target_base[:3]]} '
            f'rpy={[round(v, 3) for v in target_base[3:6]]}'
        )
        from motion.strategies import PathStrategy
        return self.node.execute(
            PathStrategy(
                waypoints_base,
                target_base[3],
                target_base[4],
                target_base[5],
                vel_scale,
                acc_scale,
                trajectory_optimizer=selected_optimizer,
                orientation_mode='per_waypoint',
            ),
            queue_if_busy=False,
        )

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
        version = "ROS2 MoveIt Robot Backend v1.0"
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

        Note: In the ROS2 runtime this publishes to the Fairino hardware bridge
        on the existing /set_do topic.
        """
        if self.node is None:
            return -1

        try:
            from std_msgs.msg import Int32MultiArray

            port = int(portId)
            status = int(value)
            if status not in (0, 1):
                raise ValueError(f"Digital output value must be 0 or 1, got {value!r}")

            publisher = getattr(self.node, "_digital_output_pub", None)
            if publisher is None:
                publisher = self.node.create_publisher(Int32MultiArray, "/set_do", 10)
                setattr(self.node, "_digital_output_pub", publisher)
                self.node.get_logger().info("[DIGITAL_OUTPUT] Created /set_do publisher")

            msg = Int32MultiArray()
            msg.data = [port, status]
            publisher.publish(msg)
            self.node.get_logger().info(
                f"[DIGITAL_OUTPUT] Published /set_do -> port={port} value={status}"
            )
            return 0
        except Exception as exc:
            self.node.get_logger().error(
                f"[DIGITAL_OUTPUT] Failed to publish /set_do: {exc}"
            )
            return -1

    def stop_motion(self):
        """
        Stops all current robot motion by cancelling active action goals.

        Returns:
            dict: structured stop result from the controller node.
        """
        if self.node is None:
            return {
                "state": "ERROR",
                "result": -2,
                "success": False,
                "stopped": False,
                "error": "robot node not available",
            }

        setattr(self.node, "_ordered_motion_chain_stop_requested", True)
        with self._prepared_ordered_lock:
            for record in self._prepared_ordered.values():
                if record["state"] not in {"completed", "failed", "discarded"}:
                    record["authorized"].set()
        try:
            self.stop_servo_jog()
        except Exception as exc:
            self.node.get_logger().warning(f"[SERVO_JOG] stop during stop_motion failed: {exc}")
        return self.node.stop_motion()

    def resetAllErrors(self):
        """
        Resets all current error states on the robot.

        Returns:
            int: 0 on success, -1 on error

        Note: Not applicable in ROS2 version
        """
        print("resetAllErrors called (not applicable in ROS2)")
        return 0
