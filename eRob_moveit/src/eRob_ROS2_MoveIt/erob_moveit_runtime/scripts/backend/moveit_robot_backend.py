#!/usr/bin/env python3
from enums import RobotAxis, Direction
from time import perf_counter
from threading import Event, Lock
import math
import inspect
import time
import traceback
import config
from backend.i_robot_backend import IRobotBackend


class MoveItRobotBackend(IRobotBackend):
    """
    Shared MoveIt-backed robot transport used by the REST API regardless of robot hardware.
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

    def _reject_if_drive_not_enabled(self, label):
        if self.node is None:
            return None
        if not self.node.is_drive_operation_enabled_for_motion():
            self.node.get_logger().error(
                f"[{label}] Rejected: {self.node.get_drive_enable_fault_reason()}"
            )
            return config.MOTION_ERROR_DRIVE_NOT_ENABLED
        return None

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True, trajectory_optimizer=None):
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
            x, y, z, rx, ry, rz = position_base
            from motion.strategies import SingleTargetStrategy
            if trajectory_optimizer is not None:
                result = self.node.execute(SingleTargetStrategy(
                    x, y, z, rx, ry, rz, vel_scale, acc_scale,
                    tool_transform=tool_transform,
                    trajectory_optimizer=trajectory_optimizer,
                ))
            else:
                result = self.node.execute(SingleTargetStrategy(
                    x, y, z, rx, ry, rz, vel_scale, acc_scale,
                    tool_transform=tool_transform,
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

    def _set_ordered_motion_chain_status(self, **updates):
        if self.node is None:
            return
        status = dict(getattr(self.node, "_ordered_motion_chain_status", {}) or {})
        status.update(updates)
        if status.get("active") is False:
            status.update({
                "current_segment_index": None,
                "current_segment_number": None,
                "current_segment_label": None,
                "current_segment_type": None,
                "current_segment_protected": False,
                "preplanned_ready_count": 0,
                "next_preplanned_segment_index": None,
                "next_preplanned_segment_number": None,
                "next_preplanned_segment_label": None,
                "next_preplanned_segment_type": None,
            })
        status["updated_at"] = time.time()
        setattr(self.node, "_ordered_motion_chain_status", status)

    def execute_ordered_motion_chain(self, segments, tool=0, user=0, blocking=True, trajectory_optimizer=None):
        started_at = perf_counter()
        if self.node is None or not segments:
            return -1
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
        setattr(self.node, "_ordered_motion_chain_stop_requested", False)
        self._set_ordered_motion_chain_status(
            active=True,
            phase="starting",
            total_segments=len(segments),
            current_segment_index=None,
            current_segment_number=None,
            current_segment_label=None,
            current_segment_type=None,
            current_segment_protected=False,
            planned_segments_count=0,
            executed_segments_count=0,
            preplanned_ready_count=0,
            next_preplanned_segment_index=None,
            next_preplanned_segment_number=None,
            next_preplanned_segment_label=None,
            next_preplanned_segment_type=None,
            last_planned_segment_index=None,
            last_planned_segment_number=None,
            last_planned_segment_label=None,
            last_planned_segment_type=None,
            result=None,
        )
        drive_error = self._reject_if_drive_not_enabled("EXECUTE_ORDERED_MOTION_CHAIN")
        if drive_error is not None:
            self._set_ordered_motion_chain_status(active=False, phase="rejected", result=int(drive_error))
            mark_motion_timing(self.node, "ordered_chain_rejected", result=int(drive_error), reason="drive_not_enabled")
            clear_motion_timing(self.node, force=True)
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[EXECUTE_ORDERED_MOTION_CHAIN] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            self._set_ordered_motion_chain_status(
                active=False,
                phase="rejected",
                result=int(config.MOTION_ERROR_HARDWARE_NOT_READY),
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
            )
            if result != 0:
                self._set_ordered_motion_chain_status(active=False, phase="failed", result=int(result))
                return result
            self.node.get_logger().info(
                f"[TIMING] ordered_motion_chain_total result=0 elapsed_s={perf_counter() - started_at:.3f}"
            )
            mark_motion_timing(self.node, "ordered_chain_done", result=0, total_elapsed_s=perf_counter() - started_at)
            self._set_ordered_motion_chain_status(active=False, phase="completed", result=0)
            return 0
        except Exception as e:
            details = traceback.format_exc()
            if self.node is not None:
                self.node.get_logger().error(f"execute_ordered_motion_chain error: {e}\n{details}")
            else:
                print(f"execute_ordered_motion_chain error: {e}\n{details}")
            self._set_ordered_motion_chain_status(active=False, phase="error", result=-1, error=str(e))
            return -1
        finally:
            clear_motion_timing(self.node, force=True)

    def _execute_ordered_motion_chain_pipelined(self, segments, tool=0, user=0, trajectory_optimizer=None):
        from concurrent.futures import ThreadPoolExecutor
        from copy import deepcopy
        from moveit_msgs.msg import RobotState, RobotTrajectory
        from queue import Empty, Queue
        from motion.execution.trajectory_executor import _send_trajectory_to_controller
        from motion.planning.segment_planning import (
            _build_follow_path_trajectory,
            _optimize_sync,
            _plan_segment,
            _robot_state_from_trajectory_end,
            _wait_execution_complete,
        )
        from motion.planning.direct_contour_ik import _build_direct_contour_trajectory, _log_report
        from motion.planning.planner_utils import _to_pose_list
        from motion.move_linear_timing import mark as mark_motion_timing

        planning_node = getattr(self.node, "planner_context", self.node)
        #
        # Refresh safety/collision state once before preplanning
        # the complete ordered chain.
        #
        # planning_node.force_safety_update()
        init_started = perf_counter()
        tool_transform = self.node.get_tool_transform(tool)
        start_cartesian = list(planning_node.prev_cartesian[:6])
        clean_joint_state = deepcopy(planning_node.current_joint_state)
        clean_joint_state.header.stamp = planning_node.get_clock().now().to_msg()
        clean_joint_state.velocity = [0.0] * (len(clean_joint_state.name) or len(clean_joint_state.position))
        clean_joint_state.effort = []
        start_state = RobotState()
        start_state.joint_state = clean_joint_state
        start_state.is_diff = False
        selected_optimizer = trajectory_optimizer or (
                str(getattr(config, "PATH_TRAJECTORY_OPTIMIZER", "") or "").strip().upper() or None
        )
        previous_execution_suppress = bool(getattr(self.node, "_suppress_post_success_unwind", False))
        mark_motion_timing(
            self.node,
            "ordered_initial_state_ready",
            duration_s=perf_counter() - init_started,
            optimizer=selected_optimizer or "default",
        )

        def _joint_positions_by_name(state):
            names = list(getattr(state.joint_state, "name", []) or [])
            values = list(getattr(state.joint_state, "position", []) or [])
            return {name: float(value) for name, value in zip(names, values)}

        def _force_unwind_joint_branch(joint_trajectory, joint_name, start_value, target_value):
            if joint_trajectory is None or not getattr(joint_trajectory, "points", None):
                return
            joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
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

            planning_node.get_logger().info(
                "[UNWIND_J6] Forced ordered unwind joint branch before optimization: "
                f"{joint_name} {start_value:.3f} -> {target_value:.3f} rad "
                f"points={len(points)}"
            )

        def _plan_unwind_direct_ik_trajectory(
                current_pos_wobj,
                target_pos_wobj,
                rotation_index,
                vel_scale,
                acc_scale,
                seed_state,
                joint_name=None,
                joint_start=None,
                joint_target=None,
        ):
            segment_started = perf_counter()
            direct_ik_step_deg = max(0.1, float(getattr(config, "EXECUTOR_POST_UNWIND_DIRECT_IK_STEP_DEG", 4.0)))
            waypoints_base = self._rotational_path_waypoints_base(
                current_pos_wobj,
                target_pos_wobj,
                rotation_index,
                max_step_override_deg=direct_ik_step_deg,
                apply_workobject_to_waypoints=False,
            )
            poses, err = _to_pose_list(
                planning_node,
                waypoints_base,
                tool_transform,
                check_last_only=True,
            )
            if err:
                raise RuntimeError(f"unwind pose conversion failed with result {err}")
            ik_result = _build_direct_contour_trajectory(planning_node, poses, seed_state=seed_state)
            ik_result.report.timings["total_before_optimizer_s"] = perf_counter() - segment_started
            _log_report(planning_node, ik_result.report)
            if not ik_result.report.ok:
                raise RuntimeError(
                    f"unwind direct IK failed: {ik_result.report.failure_reason} {ik_result.report.details}")
            if joint_name is not None and joint_start is not None and joint_target is not None:
                _force_unwind_joint_branch(
                    ik_result.trajectory.joint_trajectory,
                    str(joint_name),
                    float(joint_start),
                    float(joint_target),
                )
            optimizer_name = str(
                getattr(config, "EXECUTOR_POST_UNWIND_DIRECT_IK_OPTIMIZER", "") or "").strip().upper() or None
            optimized, optimize_elapsed = _optimize_sync(
                planning_node,
                ik_result.trajectory,
                vel_scale,
                acc_scale,
                optimizer_name=optimizer_name,
            )
            planning_node.get_logger().info(
                f"[TIMING] ordered_chain_unwind_plan_piece waypoints={len(waypoints_base)} "
                f"points={len(getattr(optimized.joint_trajectory, 'points', []) or [])} "
                f"optimize_s={optimize_elapsed:.3f} elapsed_s={perf_counter() - segment_started:.3f}"
            )
            return optimized.joint_trajectory

        def _ordered_trajectory_point_match_error(joint_trajectory, point):
            live_state = getattr(self.node, "current_joint_state", None)
            if live_state is None:
                return None, "no live joint state"
            state_names = list(getattr(live_state, "name", []) or [])
            state_positions = list(getattr(live_state, "position", []) or [])
            if not state_names or len(state_names) != len(state_positions):
                return None, "invalid live joint state"

            live_by_name = {name: float(value) for name, value in zip(state_names, state_positions)}
            joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
            planned_positions = list(getattr(point, "positions", []) or [])
            if not joint_names or len(planned_positions) < len(joint_names):
                return None, "invalid planned trajectory point"

            worst = None
            for joint_name, planned_value in zip(joint_names, planned_positions):
                if joint_name not in live_by_name:
                    return None, f"live joint state missing {joint_name}"
                actual_value = live_by_name[joint_name]
                error = abs(actual_value - float(planned_value))
                if worst is None or error > worst[0]:
                    worst = (error, joint_name, actual_value, float(planned_value))
            if worst is None:
                return None, "no joints to compare"
            return worst, None

        def _wait_ordered_trajectory_point_match(label, joint_trajectory, point, phase):
            if not bool(getattr(config, "EXECUTOR_ORDERED_START_MATCH_ENABLED", True)):
                planning_node.get_logger().info(
                    f"[OrderedChain] {phase} state match check disabled for '{label}'"
                )
                mark_motion_timing(self.node, "ordered_state_match_skipped", label=label, phase=phase)
                return True

            tolerance_rad = max(0.0, float(getattr(config, "EXECUTOR_ORDERED_START_MATCH_TOL_RAD", 0.02)))
            timeout_s = max(0.0, float(getattr(config, "EXECUTOR_ORDERED_START_MATCH_TIMEOUT_S", 0.35)))
            mark_motion_timing(
                self.node,
                "ordered_state_match_start",
                label=label,
                phase=phase,
                tolerance_rad=tolerance_rad,
                timeout_s=timeout_s,
            )
            planning_node.get_logger().info(
                f"[OrderedChain] Waiting for {phase} state match for '{label}' "
                f"tolerance={tolerance_rad:.4f}rad timeout_s={timeout_s:.3f}"
            )
            started = perf_counter()
            last_worst = None
            last_reason = None
            while True:
                worst, reason = _ordered_trajectory_point_match_error(joint_trajectory, point)
                if worst is not None:
                    last_worst = worst
                    if worst[0] <= tolerance_rad:
                        match_elapsed = perf_counter() - started
                        planning_node.get_logger().info(
                            f"[OrderedChain] {phase} state matched for '{label}': "
                            f"max_error={worst[0]:.4f}rad joint={worst[1]} "
                            f"elapsed_s={match_elapsed:.3f}"
                        )
                        mark_motion_timing(
                            self.node,
                            "ordered_state_match_done",
                            label=label,
                            phase=phase,
                            matched=True,
                            duration_s=match_elapsed,
                            max_error_rad=worst[0],
                            joint=worst[1],
                        )
                        return True
                else:
                    last_reason = reason

                if perf_counter() - started >= timeout_s:
                    match_elapsed = perf_counter() - started
                    if last_worst is not None:
                        error, joint_name, actual_value, planned_value = last_worst
                        planning_node.get_logger().error(
                            f"[OrderedChain] {phase} state mismatch for '{label}': "
                            f"max_error={error:.4f}rad tolerance={tolerance_rad:.4f}rad "
                            f"joint={joint_name} actual={actual_value:.6f} planned={planned_value:.6f} "
                            f"timeout_s={timeout_s:.3f}"
                        )
                        mark_motion_timing(
                            self.node,
                            "ordered_state_match_done",
                            label=label,
                            phase=phase,
                            matched=False,
                            duration_s=match_elapsed,
                            max_error_rad=error,
                            joint=joint_name,
                        )
                    else:
                        planning_node.get_logger().error(
                            f"[OrderedChain] {phase} state mismatch for '{label}': {last_reason or 'unknown'} "
                            f"timeout_s={timeout_s:.3f}"
                        )
                        mark_motion_timing(
                            self.node,
                            "ordered_state_match_done",
                            label=label,
                            phase=phase,
                            matched=False,
                            duration_s=match_elapsed,
                            reason=last_reason or "unknown",
                        )
                    return False
                time.sleep(0.01)

        def _plan_ordered_segment(
                index,
                segment,
                current_cartesian,
                current_state,
                *,
                defer_optimization=False,
        ):
            segment_type = str(
                segment.get("type") or ""
            ).strip().lower()

            label = str(
                segment.get("label")
                or f"segment_{index + 1}"
            )

            plan_started = perf_counter()

            #
            # blendR belongs to THIS segment and describes
            # the transition from this segment into the next.
            #
            # blendR == 0:
            #     ordinary non-blended move
            #
            # blendR > 0:
            #     this trajectory may be blended with the next
            #     trajectory by the ordered planning worker.
            #
            blend_r = max(
                0.0,
                float(
                    segment.get(
                        "blendR",
                        0.0,
                    )
                    or 0.0
                ),
            )

            vel_percent = float(
                segment.get(
                    "vel",
                    config.DEFAULT_VEL_PERCENT,
                )
            )

            acc_percent = float(
                segment.get(
                    "acc",
                    config.DEFAULT_ACC_PERCENT,
                )
            )

            vel_scale = max(
                0.0,
                min(
                    1.0,
                    vel_percent / 100.0,
                ),
            )

            acc_scale = max(
                0.0,
                min(
                    1.0,
                    acc_percent / 100.0,
                ),
            )

            mark_motion_timing(
                self.node,
                "ordered_segment_plan_start",
                index=index + 1,
                label=label,
                segment_type=segment_type,
                blendR=blend_r,
                defer_optimization=bool(
                    defer_optimization
                ),
            )

            # ============================================================
            # LINEAR
            # ============================================================
            if segment_type == "linear":
                target_base = self.apply_workobject(
                    list(segment["position"][:6]),
                    user_id=user,
                )

                plan_kwargs = {
                    "index": index,
                    "segment": {
                        "label": label,
                        "position": list(target_base[:6]),
                        "vel": vel_percent,
                        "acc": acc_percent,
                        "motion_type": "linear",
                    },
                    "start_cartesian": list(current_cartesian[:6]),
                    "start_state": current_state,
                    "tool_transform": tool_transform,
                }

                #
                # Newer segment_planning.py versions accept defer_optimization.
                # Keep compatibility with the current repo version too: if the
                # parameter is not present, plan normally and strip timing below.
                #
                if "defer_optimization" in inspect.signature(_plan_segment).parameters:
                    plan_kwargs["defer_optimization"] = bool(defer_optimization)

                planned = _plan_segment(
                    planning_node,
                    **plan_kwargs,
                )

                if defer_optimization and "defer_optimization" not in inspect.signature(_plan_segment).parameters:
                    #
                    # TOTG/Ruckig does not change the geometric joint positions.
                    # For blend construction we only need those positions, so
                    # discard per-segment timing/velocity/acceleration here.
                    #
                    raw_linear = deepcopy(planned.joint_trajectory)
                    for point in raw_linear.points:
                        point.velocities = []
                        point.accelerations = []
                        point.effort = []
                        point.time_from_start.sec = 0
                        point.time_from_start.nanosec = 0
                    planned.joint_trajectory = raw_linear
                    planned.final_state = _robot_state_from_trajectory_end(raw_linear)
                    planned.optimize_elapsed_s = 0.0

                plan_elapsed = (
                        perf_counter() - plan_started
                )

                mark_motion_timing(
                    self.node,
                    "ordered_segment_plan_done",
                    index=index + 1,
                    label=label,
                    segment_type=segment_type,
                    duration_s=plan_elapsed,
                    points=len(
                        getattr(
                            planned.joint_trajectory,
                            "points",
                            [],
                        )
                        or []
                    ),
                    optimize_s=float(
                        getattr(
                            planned,
                            "optimize_elapsed_s",
                            0.0,
                        )
                        or 0.0
                    ),
                    blendR=blend_r,
                    deferred=bool(
                        defer_optimization
                    ),
                )

                return {
                    "type": segment_type,
                    "label": label,

                    "start_position": list(current_cartesian[:6]),

                    "target_position":
                        planned.target_position,

                    "final_state":
                        planned.final_state,

                    "trajectory":
                        planned.joint_trajectory,

                    "plan_elapsed_s":
                        plan_elapsed,

                    "optimize_elapsed_s":
                        planned.optimize_elapsed_s,

                    "protected": bool(
                        segment.get(
                            "protected",
                            False,
                        )
                    ),

                    #
                    # Needed by blend planner.
                    #
                    "blendR": blend_r,
                    "vel_scale": vel_scale,
                    "acc_scale": acc_scale,

                    #
                    # Useful for diagnostics.
                    #
                    "optimization_deferred": bool(
                        defer_optimization
                    ),
                }

            # ============================================================
            # PTP
            # ============================================================
            if segment_type == "ptp":
                from motion.planning.ptp_target import (
                    plan_ptp_trajectory,
                )

                target_base = self.apply_workobject(
                    list(segment["position"][:6]),
                    user_id=user,
                )

                response = plan_ptp_trajectory(
                    planning_node,
                    target_base,
                    current_state.joint_state,
                    tool_transform=tool_transform,
                )

                if not bool(response.success):
                    raise RuntimeError(
                        f"Ordered PTP segment "
                        f"{label!r} rejected: "
                        f"{response.message}"
                    )

                #
                # Native PTP planner says the target is
                # already reached.
                #
                if bool(response.noop):
                    plan_elapsed = (
                            perf_counter() -
                            plan_started
                    )

                    mark_motion_timing(
                        self.node,
                        "ordered_segment_plan_done",
                        index=index + 1,
                        label=label,
                        segment_type=segment_type,
                        duration_s=plan_elapsed,
                        points=0,
                        native_ptp_ms=float(
                            response.total_time_ms
                        ),
                        ik_ms=float(
                            response.ik_time_ms
                        ),
                        validation_ms=float(
                            response.validation_time_ms
                        ),
                        noop=True,
                        blendR=blend_r,
                    )

                    return {
                        "type": segment_type,
                        "label": label,

                        "start_position": list(current_cartesian[:6]),

                        "target_position": list(
                            target_base[:6]
                        ),

                        "final_state":
                            current_state,

                        "trajectory": None,

                        "noop": True,

                        "plan_elapsed_s":
                            plan_elapsed,

                        "optimize_elapsed_s": 0.0,

                        "protected": bool(
                            segment.get(
                                "protected",
                                False,
                            )
                        ),

                        "blendR": blend_r,
                        "vel_scale": vel_scale,
                        "acc_scale": acc_scale,

                        "optimization_deferred": bool(
                            defer_optimization
                        ),
                    }

                #
                # The C++ PTP helper returns an untimed geometric
                # joint trajectory.
                #
                raw_joint_trajectory = (
                    response.trajectory
                )

                if not getattr(
                        raw_joint_trajectory,
                        "points",
                        None,
                ):
                    raise RuntimeError(
                        f"Ordered PTP segment "
                        f"{label!r} returned an "
                        "empty trajectory"
                    )

                #
                # CRITICAL FOR BLENDING
                #
                # If this trajectory will be blended with another
                # trajectory, do NOT time-parameterize it now.
                #
                # We need:
                #
                #     raw A
                #       +
                #     blend
                #       +
                #     raw B
                #
                # and THEN one optimizer pass over the whole path.
                #
                if defer_optimization:
                    final_joint_trajectory = (
                        raw_joint_trajectory
                    )

                    optimize_elapsed = 0.0

                else:
                    moveit_trajectory = (
                        RobotTrajectory()
                    )

                    moveit_trajectory.joint_trajectory = (
                        raw_joint_trajectory
                    )

                    optimized, optimize_elapsed = (
                        _optimize_sync(
                            planning_node,
                            moveit_trajectory,
                            vel_scale,
                            acc_scale,
                            optimizer_name=(
                                selected_optimizer
                            ),
                        )
                    )

                    final_joint_trajectory = (
                        optimized.joint_trajectory
                    )

                    if not getattr(
                            final_joint_trajectory,
                            "points",
                            None,
                    ):
                        raise RuntimeError(
                            f"Ordered PTP segment "
                            f"{label!r} optimizer "
                            "returned an empty trajectory"
                        )

                plan_elapsed = (
                        perf_counter() -
                        plan_started
                )

                mark_motion_timing(
                    self.node,
                    "ordered_segment_plan_done",
                    index=index + 1,
                    label=label,
                    segment_type=segment_type,
                    duration_s=plan_elapsed,
                    points=len(
                        final_joint_trajectory.points
                    ),
                    native_ptp_ms=float(
                        response.total_time_ms
                    ),
                    ik_ms=float(
                        response.ik_time_ms
                    ),
                    validation_ms=float(
                        response.validation_time_ms
                    ),
                    optimize_s=float(
                        optimize_elapsed
                    ),
                    blendR=blend_r,
                    deferred=bool(
                        defer_optimization
                    ),
                )

                planning_node.get_logger().info(
                    f"[OrderedChain][PTP] "
                    f"Planned '{label}' "
                    f"points="
                    f"{len(final_joint_trajectory.points)} "
                    f"native="
                    f"{response.total_time_ms:.2f}ms "
                    f"IK="
                    f"{response.ik_time_ms:.2f}ms "
                    f"validation="
                    f"{response.validation_time_ms:.2f}ms "
                    f"optimize="
                    f"{optimize_elapsed:.3f}s "
                    f"deferred="
                    f"{bool(defer_optimization)} "
                    f"blendR="
                    f"{blend_r:.3f}mm "
                    f"total="
                    f"{plan_elapsed:.3f}s"
                )

                return {
                    "type": segment_type,
                    "label": label,

                    "start_position": list(current_cartesian[:6]),

                    "target_position": list(
                        target_base[:6]
                    ),

                    #
                    # Even an untimed trajectory has the correct
                    # final joint positions, so this remains valid
                    # as the seed for planning the next move.
                    #
                    "final_state":
                        _robot_state_from_trajectory_end(
                            final_joint_trajectory
                        ),

                    "trajectory":
                        final_joint_trajectory,

                    "plan_elapsed_s":
                        plan_elapsed,

                    "optimize_elapsed_s":
                        optimize_elapsed,

                    "protected": bool(
                        segment.get(
                            "protected",
                            False,
                        )
                    ),

                    "blendR": blend_r,
                    "vel_scale": vel_scale,
                    "acc_scale": acc_scale,

                    "optimization_deferred": bool(
                        defer_optimization
                    ),
                }

            # ============================================================
            # PATH
            # ============================================================
            if segment_type == "path":
                #
                # Version 1:
                #
                # Path blending is intentionally NOT supported.
                #
                # The existing path builder currently plans AND
                # time-parameterizes internally, so leave it unchanged.
                #
                if blend_r > 0.0:
                    raise RuntimeError(
                        f"Ordered path segment "
                        f"{label!r} has blendR="
                        f"{blend_r:.3f}, but path "
                        "blending is not supported yet"
                    )

                path_base = []

                for waypoint in (
                        segment.get("path") or []
                ):
                    if len(waypoint) >= 6:
                        wp_full = list(
                            waypoint[:6]
                        )

                    else:
                        wp_full = [
                            waypoint[0],
                            waypoint[1],
                            waypoint[2],
                            current_cartesian[3],
                            current_cartesian[4],
                            current_cartesian[5],
                        ]

                    path_base.append(
                        list(
                            self.apply_workobject(
                                wp_full,
                                user_id=user,
                            )[:6]
                        )
                    )

                if not path_base:
                    raise RuntimeError(
                        f"Ordered-chain path "
                        f"segment {label!r} is empty"
                    )

                planning_path = [
                    list(current_cartesian[:6])
                ]

                planning_path.extend(
                    path_base
                )

                start_gap_mm = math.sqrt(
                    (
                            float(path_base[0][0]) -
                            float(current_cartesian[0])
                    ) ** 2
                    +
                    (
                            float(path_base[0][1]) -
                            float(current_cartesian[1])
                    ) ** 2
                    +
                    (
                            float(path_base[0][2]) -
                            float(current_cartesian[2])
                    ) ** 2
                )

                planning_node.get_logger().info(
                    f"[OrderedChain] Planning "
                    f"path segment '{label}' "
                    f"from previous target: "
                    f"start_gap_mm="
                    f"{start_gap_mm:.3f} "
                    f"path_waypoints="
                    f"{len(path_base)} "
                    f"planning_waypoints="
                    f"{len(planning_path)}"
                )

                joint_trajectory = (
                    _build_follow_path_trajectory(
                        planning_node,
                        command_path=planning_path,
                        start_state=current_state,
                        tool_transform=tool_transform,
                        vel_scaling=vel_scale,
                        acc_scaling=acc_scale,
                        trajectory_optimizer_name=(
                            selected_optimizer
                        ),
                    )
                )

                plan_elapsed = (
                        perf_counter() -
                        plan_started
                )

                mark_motion_timing(
                    self.node,
                    "ordered_segment_plan_done",
                    index=index + 1,
                    label=label,
                    segment_type=segment_type,
                    duration_s=plan_elapsed,
                    points=len(
                        getattr(
                            joint_trajectory,
                            "points",
                            [],
                        )
                        or []
                    ),
                    waypoints=len(
                        planning_path
                    ),
                )

                return {
                    "type": segment_type,
                    "label": label,

                    "target_position": list(
                        path_base[-1][:6]
                    ),

                    "final_state":
                        _robot_state_from_trajectory_end(
                            joint_trajectory
                        ),

                    "trajectory":
                        joint_trajectory,

                    "plan_elapsed_s":
                        plan_elapsed,

                    "optimize_elapsed_s": 0.0,

                    "protected": bool(
                        segment.get(
                            "protected",
                            False,
                        )
                    ),

                    "blendR": 0.0,
                    "vel_scale": vel_scale,
                    "acc_scale": acc_scale,

                    "optimization_deferred":
                        False,
                }

            # ============================================================
            # UNWIND J6
            # ============================================================
            if segment_type == "unwind_joint6":
                #
                # Unwind must never participate in blending.
                #
                if blend_r > 0.0:
                    raise RuntimeError(
                        f"Ordered unwind segment "
                        f"{label!r} cannot use blendR"
                    )

                joint_names = list(
                    getattr(
                        config,
                        "JOINT_NAMES",
                        [],
                    )
                    or []
                )

                joint_name = str(
                    getattr(
                        config,
                        "EXECUTOR_POST_UNWIND_JOINT_NAME",
                        "Joint_6",
                    )
                ).strip()

                if joint_name not in joint_names:
                    raise RuntimeError(
                        f"Joint {joint_name!r} "
                        "is not configured"
                    )

                if (
                        index == len(segments) - 1
                        and bool(
                    getattr(
                        config,
                        "EXECUTOR_ORDERED_FINAL_UNWIND_LIVE_EXECUTION",
                        True,
                    )
                )
                ):
                    planning_node.get_logger().info(
                        "[UNWIND_J6] Ordered final "
                        "unwind will be planned live "
                        "during execution"
                    )

                    return {
                        "type": segment_type,
                        "label": label,

                        "target_position": list(
                            current_cartesian[:6]
                        ),

                        "final_state":
                            current_state,

                        "runtime_unwind": True,

                        "vel": segment.get(
                            "vel",
                            config.DEFAULT_VEL_PERCENT,
                        ),

                        "acc": segment.get(
                            "acc",
                            config.DEFAULT_ACC_PERCENT,
                        ),

                        "trajectories": [],
                        "trajectory_checks": [],
                        "check": None,

                        "plan_elapsed_s":
                            perf_counter() -
                            plan_started,

                        "protected": bool(
                            segment.get(
                                "protected",
                                False,
                            )
                        ),

                        "blendR": 0.0,
                    }

                axis_index = int(
                    getattr(
                        config,
                        "EXECUTOR_POST_UNWIND_ROTATION_AXIS_INDEX",
                        5,
                    )
                )

                joint_index = (
                    joint_names.index(
                        joint_name
                    )
                )

                by_name = (
                    _joint_positions_by_name(
                        current_state
                    )
                )

                current_value = float(
                    by_name[joint_name]
                )

                final_target = (
                    self.node
                    .trajectory_executor
                    ._canonical_angle(
                        current_value
                    )
                )

                min_delta = float(
                    getattr(
                        config,
                        "EXECUTOR_POST_UNWIND_MIN_DELTA_RAD",
                        0.5,
                    )
                )

                remaining = (
                        final_target -
                        current_value
                )

                if abs(remaining) < min_delta:
                    planning_node.get_logger().info(
                        "[UNWIND_J6] "
                        "Ordered-chain unwind skipped "
                        "- no unwind needed"
                    )

                    return {
                        "type": segment_type,
                        "label": label,

                        "target_position": list(
                            current_cartesian[:6]
                        ),

                        "final_state":
                            current_state,

                        "trajectories": [],
                        "trajectory_checks": [],
                        "check": None,

                        "plan_elapsed_s":
                            perf_counter() -
                            plan_started,

                        "protected": bool(
                            segment.get(
                                "protected",
                                False,
                            )
                        ),

                        "blendR": 0.0,
                    }

                vel_percent_unwind = (
                    self.node
                    .trajectory_executor
                    ._clamp_percentage(
                        segment.get(
                            "vel",
                            config.DEFAULT_VEL_PERCENT,
                        )
                    )
                )

                acc_percent_unwind = (
                    self.node
                    .trajectory_executor
                    ._clamp_percentage(
                        segment.get(
                            "acc",
                            config.DEFAULT_ACC_PERCENT,
                        )
                    )
                )

                vel_scale_unwind = (
                        vel_percent_unwind /
                        100.0
                )

                acc_scale_unwind = (
                        acc_percent_unwind /
                        100.0
                )

                sign = float(
                    getattr(
                        config,
                        "EXECUTOR_POST_UNWIND_ROTATION_AXIS_SIGN",
                        1.0,
                    )
                )

                if abs(sign) < 1e-9:
                    sign = 1.0

                max_step_deg = max(
                    1.0,
                    abs(
                        float(
                            getattr(
                                config,
                                "EXECUTOR_POST_UNWIND_ROTATIONAL_SEGMENT_DEG",
                                180.0,
                            )
                        )
                    ),
                )

                total_delta_deg = (
                        math.degrees(
                            remaining
                        )
                        * sign
                )

                segment_count = max(
                    1,
                    int(
                        math.ceil(
                            abs(
                                total_delta_deg
                            )
                            / max_step_deg
                        )
                    ),
                )

                planning_node.get_logger().info(
                    "[UNWIND_J6] Planning "
                    "ordered-chain rotational unwind: "
                    f"{joint_name} "
                    f"{current_value:.3f} -> "
                    f"{final_target:.3f} rad "
                    f"delta={remaining:.3f} rad "
                    f"cart_axis={axis_index} "
                    f"cart_delta="
                    f"{total_delta_deg:.3f}deg "
                    f"segments={segment_count} "
                    f"max_segment="
                    f"{max_step_deg:.1f}deg "
                    f"vel="
                    f"{vel_percent_unwind:.1f}% "
                    f"acc="
                    f"{acc_percent_unwind:.1f}%"
                )

                trajectories = []
                trajectory_checks = []

                planning_state = (
                    current_state
                )

                planning_cartesian = list(
                    current_cartesian[:6]
                )

                planning_value = (
                    current_value
                )

                for unwind_index in range(
                        1,
                        segment_count + 1,
                ):
                    remaining = (
                            final_target -
                            planning_value
                    )

                    remaining_deg = (
                            math.degrees(
                                remaining
                            )
                            * sign
                    )

                    if abs(remaining) < min_delta:
                        break

                    segment_delta_deg = (
                        math.copysign(
                            min(
                                abs(
                                    remaining_deg
                                ),
                                max_step_deg,
                            ),
                            remaining_deg,
                        )
                    )

                    segment_joint_target = (
                            planning_value
                            +
                            math.radians(
                                segment_delta_deg
                            )
                            / sign
                    )

                    target_cartesian = list(
                        planning_cartesian[:6]
                    )

                    target_cartesian[
                        axis_index
                    ] = (
                            float(
                                target_cartesian[
                                    axis_index
                                ]
                            )
                            + segment_delta_deg
                    )

                    planning_node.get_logger().info(
                        f"[UNWIND_J6] Planning "
                        f"ordered unwind segment "
                        f"{unwind_index}/"
                        f"{segment_count}: "
                        f"{planning_value:.3f} -> "
                        f"{segment_joint_target:.3f} "
                        f"rad "
                        f"(final="
                        f"{final_target:.3f}), "
                        f"cart_delta="
                        f"{segment_delta_deg:.3f}deg"
                    )

                    joint_trajectory = (
                        _plan_unwind_direct_ik_trajectory(
                            planning_cartesian,
                            target_cartesian,
                            axis_index,
                            vel_scale_unwind,
                            acc_scale_unwind,
                            planning_state,
                            joint_name=joint_name,
                            joint_start=planning_value,
                            joint_target=(
                                segment_joint_target
                            ),
                        )
                    )

                    trajectories.append(
                        joint_trajectory
                    )

                    trajectory_checks.append({
                        "joint_names":
                            joint_names,

                        "joint_name":
                            joint_name,

                        "joint_index":
                            joint_index,

                        "target_value":
                            segment_joint_target,
                    })

                    planning_state = (
                        _robot_state_from_trajectory_end(
                            joint_trajectory
                        )
                    )

                    planning_cartesian = (
                        target_cartesian
                    )

                    planning_value = float(
                        _joint_positions_by_name(
                            planning_state
                        ).get(
                            joint_name,
                            planning_value,
                        )
                    )

                return {
                    "type": segment_type,
                    "label": label,

                    "target_position": list(
                        planning_cartesian[:6]
                    ),

                    "final_state":
                        planning_state,

                    "trajectories":
                        trajectories,

                    "trajectory_checks":
                        trajectory_checks,

                    "check": {
                        "joint_names":
                            joint_names,

                        "joint_name":
                            joint_name,

                        "joint_index":
                            joint_index,

                        "target_value":
                            final_target,
                    },

                    "plan_elapsed_s":
                        perf_counter() -
                        plan_started,

                    "protected": bool(
                        segment.get(
                            "protected",
                            False,
                        )
                    ),

                    "blendR": 0.0,
                }

            raise RuntimeError(
                f"Unsupported ordered-chain "
                f"segment type: "
                f"{segment_type!r}"
            )

        def _xyz_distance_mm(a, b):
            return math.sqrt(
                (float(b[0]) - float(a[0])) ** 2
                + (float(b[1]) - float(a[1])) ** 2
                + (float(b[2]) - float(a[2])) ** 2
            )

        def _joint_path_distances(trajectory):
            """
            Return cumulative Euclidean joint-space distance for a JointTrajectory.

            Result:
                [0.0, d1, d2, ..., total]
            """
            points = list(
                getattr(
                    trajectory,
                    "points",
                    [],
                )
                or []
            )

            if not points:
                return []

            distances = [0.0]

            for index in range(1, len(points)):
                previous = points[index - 1].positions
                current = points[index].positions

                step = math.sqrt(
                    sum(
                        (
                                float(current[joint])
                                - float(previous[joint])
                        ) ** 2
                        for joint in range(
                            min(
                                len(previous),
                                len(current),
                            )
                        )
                    )
                )

                distances.append(
                    distances[-1] + step
                )

            return distances

        def _blend_trim_fraction(
                radius_mm,
                cartesian_length_mm,
        ):
            """
            Convert blendR to a trim fraction of the actual joint path.

            Translational segments keep approximate Cartesian-mm semantics.
            Rotation-dominant segments use a conservative joint-path fraction
            because a Cartesian radius is not meaningful when XYZ barely moves.
            """
            radius_mm = max(
                0.0,
                float(radius_mm),
            )

            cartesian_length_mm = max(
                0.0,
                float(cartesian_length_mm),
            )

            #
            # A segment can move a millimetre or two in XYZ while being
            # predominantly rotational.  Treat those micro-translations as
            # rotation-dominant; otherwise a large blendR (for example 20 mm)
            # gets clamped to a 45% trim on BOTH ends of the short segment and
            # consumes the whole middle trajectory.
            #
            rotation_dominant_xyz_mm = max(
                1.0,
                float(
                    getattr(
                        config,
                        "ORDERED_BLEND_ROTATION_DOMINANT_XYZ_MM",
                        5.0,
                    )
                ),
            )

            if cartesian_length_mm > rotation_dominant_xyz_mm:
                return min(
                    0.45,
                    radius_mm / cartesian_length_mm,
                )

            return 0.25


        def _wait_state_validity(joint_names, joint_positions, timeout_s=2.0):
            from moveit_msgs.srv import GetStateValidity
            from sensor_msgs.msg import JointState

            client = planning_node.get_state_validity_client()
            if client is None or not client.wait_for_service(timeout_sec=1.0):
                raise TimeoutError("MoveIt state-validity service unavailable")

            request = GetStateValidity.Request()
            joint_state = JointState()
            joint_state.name = list(joint_names)
            joint_state.position = [float(value) for value in joint_positions]
            request.robot_state.joint_state = joint_state
            request.robot_state.is_diff = True
            request.group_name = str(config.PLANNING_GROUP)

            future = client.call_async(request)
            deadline = time.monotonic() + float(timeout_s)
            while time.monotonic() < deadline:
                if future.done():
                    return future.result()
                time.sleep(0.001)

            raise TimeoutError("MoveIt state-validity request timed out")

        def _build_blended_group(planned_segments):
            """
            Build one raw JointTrajectory from a contiguous LIN/PTP blend group.

            Example:

                A blendR=20
                B blendR=30
                C blendR=15
                D blendR=0

            becomes one physical trajectory:

                A -> blend -> B -> blend -> C -> blend -> D

            Middle segments can be trimmed at both ends. Effective blend radii
            are reduced if adjacent blend regions would overlap.
            """
            from trajectory_msgs.msg import (
                JointTrajectory,
                JointTrajectoryPoint,
            )

            if len(planned_segments) < 2:
                raise RuntimeError(
                    "Blended group requires at least two segments"
                )

            segment_count = len(planned_segments)

            trajectories = []
            joint_distances = []
            joint_lengths = []
            cartesian_lengths_mm = []

            # ----------------------------------------------------------
            # Validate all members.
            # ----------------------------------------------------------
            for index, planned in enumerate(planned_segments):
                trajectory = planned.get("trajectory")

                if trajectory is None:
                    raise RuntimeError(
                        f"Cannot blend empty/no-op segment "
                        f"{planned.get('label')!r}"
                    )

                points = list(
                    getattr(
                        trajectory,
                        "points",
                        [],
                    )
                    or []
                )

                if len(points) < 2:
                    raise RuntimeError(
                        f"Blend segment {planned.get('label')!r} "
                        f"requires at least 2 trajectory points; "
                        f"got {len(points)}"
                    )

                #
                # Very short raw trajectories do not contain enough interior
                # samples for a middle segment to be trimmed independently at
                # both ends.  Resample them to seven evenly spaced joint-space
                # points before computing blend entry/exit indices.
                #
                # Seven points gives:
                #
                #   0%, 16.7%, 33.3%, 50%, 66.7%, 83.3%, 100%
                #
                # so a conservative 25% trim leaves a clear middle region.
                #
                if len(points) < 7:
                    from trajectory_msgs.msg import JointTrajectoryPoint

                    first_positions = [
                        float(value)
                        for value in points[0].positions
                    ]

                    last_positions = [
                        float(value)
                        for value in points[-1].positions
                    ]

                    if len(first_positions) != len(last_positions):
                        raise RuntimeError(
                            f"Blend segment {planned.get('label')!r} "
                            "has mismatched joint dimensions"
                        )

                    original_count = len(points)
                    densified_points = []

                    for sample_index in range(7):
                        fraction = (
                            float(sample_index)
                            / 6.0
                        )

                        point = JointTrajectoryPoint()
                        point.positions = [
                            start_value
                            + (
                                end_value
                                - start_value
                            )
                            * fraction
                            for start_value, end_value
                            in zip(
                                first_positions,
                                last_positions,
                            )
                        ]
                        point.velocities = []
                        point.accelerations = []
                        point.effort = []
                        point.time_from_start.sec = 0
                        point.time_from_start.nanosec = 0

                        densified_points.append(point)

                    trajectory.points = densified_points
                    points = list(trajectory.points)

                    planning_node.get_logger().info(
                        "[OrderedBlend] Densified short segment "
                        f"{planned.get('label')!r} "
                        f"from {original_count} to 7 joint-space points"
                    )

                trajectories.append(trajectory)

                cartesian_length_mm = _xyz_distance_mm(
                    planned["start_position"],
                    planned["target_position"],
                )

                distances = _joint_path_distances(
                    trajectory
                )

                joint_length = (
                    distances[-1]
                    if distances
                    else 0.0
                )

                if joint_length <= 1e-9:
                    raise RuntimeError(
                        f"Cannot blend true no-op segment "
                        f"{planned.get('label')!r}"
                    )

                cartesian_lengths_mm.append(
                    cartesian_length_mm
                )

                joint_distances.append(
                    distances
                )

                joint_lengths.append(
                    joint_length
                )

            joint_names = list(
                trajectories[0].joint_names
            )

            if not joint_names:
                raise RuntimeError(
                    "Blended group has no joint names"
                )

            for index, trajectory in enumerate(
                    trajectories[1:],
                    start=1,
            ):
                if list(trajectory.joint_names) != joint_names:
                    raise RuntimeError(
                        f"Joint-name/order mismatch at "
                        f"blend segment {index + 1}"
                    )

            # ----------------------------------------------------------
            # Validate exact planned junctions.
            # ----------------------------------------------------------
            junction_tolerance = float(
                getattr(
                    config,
                    "ORDERED_BLEND_JUNCTION_TOL_RAD",
                    0.02,
                )
            )

            for junction in range(segment_count - 1):
                left_end = list(
                    trajectories[junction]
                    .points[-1]
                    .positions
                )

                right_start = list(
                    trajectories[junction + 1]
                    .points[0]
                    .positions
                )

                max_error = max(
                    abs(float(a) - float(b))
                    for a, b in zip(
                        left_end,
                        right_start,
                    )
                )

                if max_error > junction_tolerance:
                    raise RuntimeError(
                        f"Blend junction "
                        f"{junction + 1}/{segment_count - 1} "
                        f"mismatch: {max_error:.6f}rad > "
                        f"{junction_tolerance:.6f}rad"
                    )

            # ----------------------------------------------------------
            # Initial effective radius for every junction.
            #
            # Radius belongs to the LEFT segment.
            # ----------------------------------------------------------
            requested_radii = []
            effective_radii = []

            for junction in range(segment_count - 1):
                requested = max(
                    0.0,
                    float(
                        planned_segments[junction]
                        .get("blendR", 0.0)
                        or 0.0
                    ),
                )

                if requested <= 0.0:
                    raise RuntimeError(
                        f"Internal blend-group segment "
                        f"{planned_segments[junction].get('label')!r} "
                        "has blendR=0"
                    )

                left_cartesian_mm = (
                    cartesian_lengths_mm[
                        junction
                    ]
                )

                right_cartesian_mm = (
                    cartesian_lengths_mm[
                        junction + 1
                    ]
                )

                effective = requested

                if left_cartesian_mm > 1.0:
                    effective = min(
                        effective,
                        0.45 * left_cartesian_mm,
                    )

                if right_cartesian_mm > 1.0:
                    effective = min(
                        effective,
                        0.45 * right_cartesian_mm,
                    )

                requested_radii.append(requested)
                effective_radii.append(effective)

            # ----------------------------------------------------------
            # Middle-segment overlap is checked from the actual selected
            # joint-trajectory indices below.  This intentionally avoids
            # XYZ-only scaling for rotation-dominant segments.
            # ----------------------------------------------------------

            min_radius = float(
                getattr(
                    config,
                    "ORDERED_BLEND_MIN_RADIUS_MM",
                    0.5,
                )
            )

            for junction, effective in enumerate(
                    effective_radii
            ):
                if effective < min_radius:
                    raise RuntimeError(
                        f"Effective blend radius at junction "
                        f"{junction + 1} is too small: "
                        f"{effective:.3f}mm"
                    )

            # ----------------------------------------------------------
            # Find trajectory indices corresponding approximately
            # to each Cartesian blend radius.
            # ----------------------------------------------------------
            entry_indices = [None] * (
                    segment_count - 1
            )

            exit_indices = [None] * (
                    segment_count - 1
            )

            for junction in range(
                    segment_count - 1
            ):
                left_points = list(
                    trajectories[junction].points
                )

                right_points = list(
                    trajectories[junction + 1].points
                )

                radius = effective_radii[junction]

                left_trim_fraction = (
                    _blend_trim_fraction(
                        radius,
                        cartesian_lengths_mm[
                            junction
                        ],
                    )
                )

                right_trim_fraction = (
                    _blend_trim_fraction(
                        radius,
                        cartesian_lengths_mm[
                            junction + 1
                        ],
                    )
                )

                left_target_distance = (
                    joint_lengths[junction]
                    * (
                        1.0
                        - left_trim_fraction
                    )
                )

                right_target_distance = (
                    joint_lengths[
                        junction + 1
                    ]
                    * right_trim_fraction
                )

                left_distances = (
                    joint_distances[
                        junction
                    ]
                )

                right_distances = (
                    joint_distances[
                        junction + 1
                    ]
                )

                entry_index = min(
                    range(
                        1,
                        len(left_points) - 1,
                    ),
                    key=lambda point_index: abs(
                        left_distances[
                            point_index
                        ]
                        - left_target_distance
                    ),
                )

                exit_index = min(
                    range(
                        1,
                        len(right_points) - 1,
                    ),
                    key=lambda point_index: abs(
                        right_distances[
                            point_index
                        ]
                        - right_target_distance
                    ),
                )

                entry_indices[junction] = (
                    entry_index
                )

                exit_indices[junction] = (
                    exit_index
                )

            # ----------------------------------------------------------
            # Index-level overlap verification.
            #
            # A middle trajectory must still contain at least one
            # original point between incoming and outgoing blends.
            # ----------------------------------------------------------
            for segment_index in range(
                    1,
                    segment_count - 1,
            ):
                start_index = (
                    exit_indices[
                        segment_index - 1
                        ]
                )

                end_index = (
                    entry_indices[
                        segment_index
                    ]
                )

                if start_index >= end_index:
                    points = list(
                        trajectories[
                            segment_index
                        ].points
                    )

                    #
                    # The requested radii consume too much of this middle
                    # segment.  Shrink both blend regions symmetrically in
                    # index space rather than rejecting the whole chain.
                    #
                    fallback_start = max(
                        1,
                        int(
                            round(
                                0.25
                                * (len(points) - 1)
                            )
                        ),
                    )

                    fallback_end = min(
                        len(points) - 2,
                        int(
                            round(
                                0.75
                                * (len(points) - 1)
                            )
                        ),
                    )

                    if fallback_start >= fallback_end:
                        raise RuntimeError(
                            f"Blend regions overlap in segment "
                            f"{segment_index + 1} "
                            f"{planned_segments[segment_index].get('label')!r}: "
                            f"start={start_index} end={end_index}; "
                            f"cannot create safe fallback with "
                            f"{len(points)} points"
                        )

                    planning_node.get_logger().warning(
                        "[OrderedBlend] Shrinking overlapping blend regions "
                        f"for segment {segment_index + 1} "
                        f"{planned_segments[segment_index].get('label')!r}: "
                        f"requested start={start_index} end={end_index} "
                        f"-> fallback start={fallback_start} "
                        f"end={fallback_end}"
                    )

                    #
                    # Incoming junction exits into this segment.
                    #
                    exit_indices[
                        segment_index - 1
                    ] = fallback_start

                    #
                    # Outgoing junction starts blending before this segment
                    # reaches its exact endpoint.
                    #
                    entry_indices[
                        segment_index
                    ] = fallback_end

            # ----------------------------------------------------------
            # Generate a joint-space quadratic Bezier at each junction.
            # ----------------------------------------------------------
            sample_count = max(
                6,
                int(
                    getattr(
                        config,
                        "ORDERED_BLEND_SAMPLES",
                        12,
                    )
                ),
            )

            blends = []

            for junction in range(
                    segment_count - 1
            ):
                left = trajectories[junction]
                right = trajectories[junction + 1]

                q0 = [
                    float(v)
                    for v in left.points[
                        entry_indices[junction]
                    ].positions
                ]

                q1 = [
                    float(v)
                    for v in left.points[-1].positions
                ]

                q2 = [
                    float(v)
                    for v in right.points[
                        exit_indices[junction]
                    ].positions
                ]

                blend_positions = []

                for sample in range(
                        sample_count + 1
                ):
                    u = (
                            float(sample)
                            / float(sample_count)
                    )

                    a = (1.0 - u) ** 2
                    b = 2.0 * (1.0 - u) * u
                    c = u ** 2

                    q = [
                        (
                                a * q0[joint]
                                + b * q1[joint]
                                + c * q2[joint]
                        )
                        for joint in range(
                            len(q0)
                        )
                    ]

                    blend_positions.append(q)

                blends.append(blend_positions)

            # ----------------------------------------------------------
            # Validate ONLY newly generated blend samples.
            #
            # Original A/B/C/... trajectories have already been
            # validated by their planners.
            # ----------------------------------------------------------
            validation_started = perf_counter()

            for junction, blend_positions in enumerate(
                    blends
            ):
                for sample_index, q in enumerate(
                        blend_positions
                ):
                    validity = _wait_state_validity(
                        joint_names,
                        q,
                    )

                    if not bool(
                            getattr(
                                validity,
                                "valid",
                                False,
                            )
                    ):
                        contacts = []

                        for contact in list(
                                getattr(
                                    validity,
                                    "contacts",
                                    [],
                                )
                                or []
                        ):
                            body_1 = str(
                                getattr(
                                    contact,
                                    "contact_body_1",
                                    "",
                                )
                                or getattr(
                                    contact,
                                    "body_name_1",
                                    "",
                                )
                            )

                            body_2 = str(
                                getattr(
                                    contact,
                                    "contact_body_2",
                                    "",
                                )
                                or getattr(
                                    contact,
                                    "body_name_2",
                                    "",
                                )
                            )

                            if body_1 or body_2:
                                contacts.append(
                                    (
                                        f"{body_1}<->{body_2}"
                                        if body_1 and body_2
                                        else body_1 or body_2
                                    )
                                )

                        detail = (
                            f" contacts={contacts}"
                            if contacts
                            else ""
                        )

                        raise RuntimeError(
                            f"Blend junction "
                            f"{junction + 1} sample "
                            f"{sample_index}/{sample_count} "
                            f"is invalid{detail}"
                        )

            # ----------------------------------------------------------
            # Build one raw trajectory:
            #
            # segment0 -> blend0 -> segment1 middle
            #          -> blend1 -> segment2 middle ...
            # ----------------------------------------------------------
            merged = JointTrajectory()
            merged.joint_names = list(
                joint_names
            )

            def append_positions(positions):
                point = JointTrajectoryPoint()

                point.positions = [
                    float(v)
                    for v in positions
                ]

                merged.points.append(point)

            for segment_index in range(
                    segment_count
            ):
                points = list(
                    trajectories[
                        segment_index
                    ].points
                )

                if segment_index == 0:
                    start_index = 0
                else:
                    start_index = exit_indices[
                        segment_index - 1
                        ]

                if segment_index == (
                        segment_count - 1
                ):
                    end_index = (
                            len(points) - 1
                    )
                else:
                    end_index = entry_indices[
                        segment_index
                    ]

                for point_index in range(
                        start_index,
                        end_index + 1,
                ):
                    #
                    # Don't duplicate exact same point when
                    # joining slices.
                    #
                    positions = points[
                        point_index
                    ].positions

                    if merged.points:
                        previous = (
                            merged.points[-1]
                            .positions
                        )

                        if all(
                                abs(
                                    float(a)
                                    - float(b)
                                ) <= 1e-12
                                for a, b in zip(
                                    previous,
                                    positions,
                                )
                        ):
                            continue

                    append_positions(
                        positions
                    )

                if segment_index < (
                        segment_count - 1
                ):
                    #
                    # q0 is already the final point of the
                    # left segment slice.
                    #
                    # q2 will be the first point of the
                    # next segment slice.
                    #
                    for q in blends[
                        segment_index
                    ][1:-1]:
                        append_positions(q)

            planning_node.get_logger().info(
                "[OrderedBlend] Built raw group "
                f"segments={segment_count} "
                f"labels="
                f"{[s.get('label') for s in planned_segments]} "
                f"requested_radii="
                f"{[round(v, 3) for v in requested_radii]} "
                f"effective_radii="
                f"{[round(v, 3) for v in effective_radii]} "
                f"cartesian_lengths_mm="
                f"{[round(v, 3) for v in cartesian_lengths_mm]} "
                f"joint_lengths="
                f"{[round(v, 6) for v in joint_lengths]} "
                f"entry_indices={entry_indices} "
                f"exit_indices={exit_indices} "
                f"merged_points="
                f"{len(merged.points)} "
                f"validation_s="
                f"{perf_counter() - validation_started:.3f}"
            )

            return (
                merged,
                effective_radii,
            )

        def _execute_planned_segment(index, total, planned, preplanned_ready_count=0):
            exec_started = perf_counter()
            segment_type = planned["type"]
            mark_motion_timing(
                self.node,
                "ordered_segment_execute_start",
                index=index,
                label=planned.get("label"),
                segment_type=segment_type,
                preplanned_ready_count=int(preplanned_ready_count),
            )
            self._set_ordered_motion_chain_status(
                active=True,
                phase="executing",
                total_segments=total,
                current_segment_index=index - 1,
                current_segment_number=index,
                current_segment_label=planned.get("label"),
                current_segment_type=segment_type,
                current_segment_protected=bool(planned.get("protected", False)),
                executed_segments_count=max(0, index - 1),
                preplanned_ready_count=int(preplanned_ready_count),
                result=None,
            )
            if segment_type in {"linear", "ptp", "path", "blended"}:

                if bool(planned.get("noop", False)):
                    planning_node.get_logger().info(
                        f"[OrderedChain] Skipping no-op "
                        f"segment {index}/{total} "
                        f"label='{planned['label']}' "
                        f"type={segment_type}"
                    )

                    result = 0

                else:

                    duration = planned["trajectory"].points[-1].time_from_start
                    duration_s = float(duration.sec) + float(duration.nanosec) / 1e9
                    timeout_s = max(
                        float(getattr(config, "EXECUTOR_TIME_MIN_S", 5.0)),
                        duration_s * float(getattr(config, "EXECUTOR_TIME_MULTIPLIER", 2.0)),
                    )
                    execution_timeout_s = duration_s + timeout_s + 2.0
                    planning_node.get_logger().info(
                        f"[OrderedChain] Sending planned segment {index}/{total} label='{planned['label']}' "
                        f"type={segment_type} points={len(planned['trajectory'].points)} duration_s={duration_s:.3f} "
                        f"controller_goal_tolerance_s={timeout_s:.3f} wait_timeout_s={execution_timeout_s:.3f} "
                        f"plan_s={planned['plan_elapsed_s']:.3f}"
                    )
                    if not _wait_ordered_trajectory_point_match(
                            planned["label"],
                            planned["trajectory"],
                            planned["trajectory"].points[0],
                            "start",
                    ):
                        result = config.MOTION_ERROR_CONTROLLER_EXECUTION_FAILED
                    else:
                        mark_motion_timing(
                            self.node,
                            "ordered_controller_handoff_start",
                            index=index,
                            label=planned.get("label"),
                            points=len(getattr(planned["trajectory"], "points", []) or []),
                        )
                        _send_trajectory_to_controller(self.node, planned["trajectory"])

                        mark_motion_timing(
                            self.node,
                            "ordered_wait_execution_start",
                            index=index,
                            label=planned.get("label"),
                            timeout_s=execution_timeout_s,
                        )
                        result = _wait_execution_complete(self.node, timeout_s=execution_timeout_s)
                        mark_motion_timing(
                            self.node,
                            "ordered_wait_execution_done",
                            index=index,
                            label=planned.get("label"),
                            result=int(result) if isinstance(result, int) else result,
                            duration_s=perf_counter() - exec_started,
                        )
                        if result == 0:
                            planning_node.get_logger().info(
                                f"[OrderedChain] Controller completed segment {index}/{total} "
                                f"label='{planned['label']}', verifying live end state before next segment"
                            )
                            if not _wait_ordered_trajectory_point_match(
                                    planned["label"],
                                    planned["trajectory"],
                                    planned["trajectory"].points[-1],
                                    "end",
                            ):
                                result = config.MOTION_ERROR_CONTROLLER_EXECUTION_FAILED
            elif segment_type == "blend_consumed":
                planning_node.get_logger().info(
                    f"[OrderedChain] Segment {index}/{total} "
                    f"label='{planned['label']}' was executed inside the "
                    "previous blended controller trajectory"
                )
                result = 0

            elif segment_type == "unwind_joint6":
                if bool(planned.get("runtime_unwind", False)):
                    planning_node.get_logger().info(
                        f"[OrderedChain] Executing live final unwind label='{planned['label']}' "
                        f"plan_s={planned['plan_elapsed_s']:.3f}"
                    )
                    result = self._unwind_joint6_with_rotational_path(
                        vel=planned.get("vel", config.DEFAULT_VEL_PERCENT),
                        acc=planned.get("acc", config.DEFAULT_ACC_PERCENT),
                        queue_if_busy=False,
                    )
                else:
                    result = 0
                trajectory_checks = list(planned.get("trajectory_checks") or [])
                for unwind_index, joint_trajectory in enumerate(planned["trajectories"], start=1):
                    duration = joint_trajectory.points[-1].time_from_start
                    duration_s = float(duration.sec) + float(duration.nanosec) / 1e9
                    timeout_s = max(
                        float(getattr(config, "EXECUTOR_TIME_MIN_S", 5.0)),
                        duration_s * float(getattr(config, "EXECUTOR_TIME_MULTIPLIER", 2.0)),
                    )
                    execution_timeout_s = duration_s + timeout_s + 2.0
                    planning_node.get_logger().info(
                        f"[OrderedChain] Sending planned unwind {unwind_index}/{len(planned['trajectories'])} "
                        f"points={len(joint_trajectory.points)} duration_s={duration_s:.3f} "
                        f"controller_goal_tolerance_s={timeout_s:.3f} wait_timeout_s={execution_timeout_s:.3f}"
                    )
                    _send_trajectory_to_controller(
                        self.node,
                        joint_trajectory,
                        preserve_explicit_wrap=True,
                        unwind_check=trajectory_checks[unwind_index - 1]
                        if unwind_index - 1 < len(trajectory_checks)
                        else planned.get("check"),
                    )
                    result = _wait_execution_complete(self.node, timeout_s=execution_timeout_s)
                    if result != 0:
                        break
                if result == 0 and planned.get("check") is not None:
                    result = 0 if self.node.trajectory_executor._verify_explicit_unwind_complete(
                        planned["check"]) else -6
                if result != 0:
                    setattr(self.node, "_last_ordered_unwind_failure_time", time.time())
                    setattr(self.node, "_last_ordered_unwind_failure_result", int(result))
            else:
                result = -1
            self._set_ordered_motion_chain_status(
                active=True,
                phase="segment_completed" if result == 0 else "segment_failed",
                current_segment_index=index - 1,
                current_segment_number=index,
                current_segment_label=planned.get("label"),
                current_segment_type=segment_type,
                current_segment_protected=bool(planned.get("protected", False)),
                executed_segments_count=index if result == 0 else max(0, index - 1),
                result=int(result) if isinstance(result, int) else result,
            )
            planning_node.get_logger().info(
                f"[TIMING] ordered_motion_chain_segment index={index} label='{planned['label']}' "
                f"type={segment_type} result={result} elapsed_s={perf_counter() - exec_started:.3f}"
            )
            mark_motion_timing(
                self.node,
                "ordered_segment_execute_done",
                index=index,
                label=planned.get("label"),
                segment_type=segment_type,
                result=int(result) if isinstance(result, int) else result,
                duration_s=perf_counter() - exec_started,
            )
            return result

        executor = ThreadPoolExecutor(max_workers=1)
        planned_queue = Queue()
        stop_planning = Event()
        plan_timeout_s = float(getattr(config, "CUSTOM_SEQUENCE_PLAN_TIMEOUT_S", 10.0)) + 30.0
        planning_lock = Lock()
        planned_by_index = {}

        def _preplanned_snapshot(current_index=0):
            with planning_lock:
                ready_indexes = sorted(i for i in planned_by_index if i >= current_index)
                last_index = max(planned_by_index) if planned_by_index else None
                last_planned = planned_by_index.get(last_index) if last_index is not None else None
                next_index = ready_indexes[0] if ready_indexes else None
                next_planned = planned_by_index.get(next_index) if next_index is not None else None
                return {
                    "planned_segments_count": len(planned_by_index),
                    "preplanned_ready_count": len(ready_indexes),
                    "next_preplanned_segment_index": next_index,
                    "next_preplanned_segment_number": next_index + 1 if next_index is not None else None,
                    "next_preplanned_segment_label": next_planned.get("label") if next_planned else None,
                    "next_preplanned_segment_type": next_planned.get("type") if next_planned else None,
                    "last_planned_segment_index": last_index,
                    "last_planned_segment_number": last_index + 1 if last_index is not None else None,
                    "last_planned_segment_label": last_planned.get("label") if last_planned else None,
                    "last_planned_segment_type": last_planned.get("type") if last_planned else None,
                }

        def _mark_planned(index, planned_segment):
            with planning_lock:
                planned_by_index[index] = {
                    "label": planned_segment.get("label"),
                    "type": planned_segment.get("type"),
                }
            self._set_ordered_motion_chain_status(**_preplanned_snapshot(current_index=0))

        def _mark_consumed(index):
            with planning_lock:
                planned_by_index.pop(index, None)

        def _planning_worker():
            worker_started = perf_counter()
            mark_motion_timing(
                self.node,
                "ordered_planning_worker_start",
                segments=len(segments),
            )

            previous_target = start_cartesian
            previous_state = start_state

            try:
                index = 0

                while index < len(segments):
                    if stop_planning.is_set():
                        mark_motion_timing(
                            self.node,
                            "ordered_planning_worker_stopped",
                            index=index + 1,
                            duration_s=perf_counter() - worker_started,
                        )
                        break

                    segment = segments[index]
                    segment_type = str(segment.get("type") or "").strip().lower()
                    blend_r = max(
                        0.0,
                        float(segment.get("blendR", 0.0) or 0.0),
                    )

                    wants_blend = blend_r > 0.0

                    if wants_blend:
                        if segment_type not in {"linear", "ptp"}:
                            raise RuntimeError(
                                f"blendR is currently supported only for LIN/PTP; "
                                f"segment {index + 1} is {segment_type!r}"
                            )

                        if index + 1 >= len(segments):
                            raise RuntimeError(
                                f"Segment {index + 1} requests blendR={blend_r:.3f} "
                                "but there is no next segment"
                            )

                        #
                        # Detect the complete contiguous blend group.
                        #
                        # Example:
                        #
                        #   A blendR=20
                        #   B blendR=30
                        #   C blendR=15
                        #   D blendR=0
                        #
                        # becomes one physical controller trajectory [A, B, C, D].
                        #
                        group_end = index

                        while group_end < len(segments) - 1:
                            current_segment = segments[group_end]
                            current_type = str(
                                current_segment.get("type") or ""
                            ).strip().lower()

                            current_blend_r = max(
                                0.0,
                                float(
                                    current_segment.get("blendR", 0.0)
                                    or 0.0
                                ),
                            )

                            if current_blend_r <= 0.0:
                                break

                            if current_type not in {"linear", "ptp"}:
                                raise RuntimeError(
                                    f"blendR is currently supported only for LIN/PTP; "
                                    f"segment {group_end + 1} is {current_type!r}"
                                )

                            next_segment = segments[group_end + 1]
                            next_type = str(
                                next_segment.get("type") or ""
                            ).strip().lower()

                            if next_type not in {"linear", "ptp"}:
                                raise RuntimeError(
                                    f"Segment {group_end + 1} requests "
                                    f"blendR={current_blend_r:.3f}, but the next "
                                    f"segment {group_end + 2} is {next_type!r}. "
                                    "Blend groups currently support only LIN/PTP -> LIN/PTP."
                                )

                            group_end += 1

                        if group_end <= index:
                            raise RuntimeError(
                                f"Could not form blend group starting at segment {index + 1}"
                            )

                        #
                        # Plan every logical segment in the group as raw geometry.
                        # Each following segment uses the predicted final state of
                        # the previous segment, preserving the existing PTP/LIN
                        # branch selection and preplanning behaviour.
                        #
                        planned_group = []
                        group_target = previous_target
                        group_state = previous_state

                        for group_index in range(index, group_end + 1):
                            planned_member = _plan_ordered_segment(
                                group_index,
                                segments[group_index],
                                group_target,
                                group_state,
                                defer_optimization=True,
                            )

                            if bool(planned_member.get("noop", False)):
                                raise RuntimeError(
                                    f"Cannot blend no-op segment "
                                    f"{planned_member['label']!r}"
                                )

                            planned_group.append(planned_member)
                            group_target = planned_member["target_position"]
                            group_state = planned_member["final_state"]

                        #
                        # Replace all exact internal waypoints with validated
                        # joint-space blend curves and obtain one raw trajectory.
                        #
                        raw_blended, effective_radii = _build_blended_group(
                            planned_group
                        )

                        moveit_trajectory = RobotTrajectory()
                        moveit_trajectory.joint_trajectory = raw_blended

                        #
                        # Conservative first implementation:
                        # the complete group uses the slowest requested velocity
                        # and acceleration of any logical segment.
                        #
                        group_vel_scale = min(
                            float(member.get("vel_scale", 1.0))
                            for member in planned_group
                        )

                        group_acc_scale = min(
                            float(member.get("acc_scale", 1.0))
                            for member in planned_group
                        )

                        optimized, optimize_elapsed = _optimize_sync(
                            planning_node,
                            moveit_trajectory,
                            group_vel_scale,
                            group_acc_scale,
                            optimizer_name=selected_optimizer,
                        )

                        optimized_joint_trajectory = optimized.joint_trajectory

                        if not getattr(
                            optimized_joint_trajectory,
                            "points",
                            None,
                        ):
                            raise RuntimeError(
                                "Optimizer returned empty blended-group trajectory"
                            )

                        first = planned_group[0]
                        last = planned_group[-1]

                        combined = {
                            "type": "blended",
                            "label": " -> ".join(
                                str(member.get("label") or "")
                                for member in planned_group
                            ),
                            "start_position": list(first["start_position"]),
                            "target_position": list(last["target_position"]),
                            "final_state": _robot_state_from_trajectory_end(
                                optimized_joint_trajectory
                            ),
                            "trajectory": optimized_joint_trajectory,
                            "plan_elapsed_s": (
                                sum(
                                    float(
                                        member.get("plan_elapsed_s", 0.0)
                                        or 0.0
                                    )
                                    for member in planned_group
                                )
                                + float(optimize_elapsed)
                            ),
                            "optimize_elapsed_s": float(optimize_elapsed),
                            "protected": any(
                                bool(member.get("protected", False))
                                for member in planned_group
                            ),
                            "blendR": float(
                                first.get("blendR", 0.0) or 0.0
                            ),
                            "effective_blend_radii": list(effective_radii),
                            "vel_scale": group_vel_scale,
                            "acc_scale": group_acc_scale,
                            "logical_segment_count": len(planned_group),
                        }

                        #
                        # Queue one physical trajectory at the first logical
                        # segment index.
                        #
                        planned_queue.put((index, combined, None))

                        mark_motion_timing(
                            self.node,
                            "ordered_segment_queued",
                            index=index + 1,
                            label=combined.get("label"),
                            segment_type="blended",
                            blend_group_size=len(planned_group),
                            effective_blend_radii=[
                                float(value)
                                for value in effective_radii
                            ],
                            duration_s=perf_counter() - worker_started,
                        )

                        _mark_planned(index, combined)

                        #
                        # Preserve the existing one-queue-entry-per-logical-
                        # segment consumer/status contract.  Every later member
                        # of the group is physically executed inside `combined`.
                        #
                        for consumed_offset in range(1, len(planned_group)):
                            logical_index = index + consumed_offset
                            member = planned_group[consumed_offset]

                            consumed = {
                                "type": "blend_consumed",
                                "label": str(
                                    member.get("label")
                                    or f"segment_{logical_index + 1}"
                                ),
                                "start_position": list(
                                    member.get("start_position") or []
                                ),
                                "target_position": list(
                                    member["target_position"]
                                ),
                                "final_state": combined["final_state"],
                                "trajectory": None,
                                "plan_elapsed_s": 0.0,
                                "optimize_elapsed_s": 0.0,
                                "protected": bool(
                                    member.get("protected", False)
                                ),
                                "blendR": 0.0,
                            }

                            planned_queue.put(
                                (logical_index, consumed, None)
                            )
                            _mark_planned(
                                logical_index,
                                consumed,
                            )

                        previous_target = list(last["target_position"])
                        previous_state = combined["final_state"]

                        index = group_end + 1
                        continue

                    planned_segment = _plan_ordered_segment(
                        index,
                        segment,
                        previous_target,
                        previous_state,
                    )

                    planned_queue.put(
                        (index, planned_segment, None)
                    )

                    mark_motion_timing(
                        self.node,
                        "ordered_segment_queued",
                        index=index + 1,
                        label=planned_segment.get("label"),
                        duration_s=perf_counter() - worker_started,
                    )

                    _mark_planned(
                        index,
                        planned_segment,
                    )

                    previous_target = (
                        planned_segment["target_position"]
                    )

                    previous_state = (
                        planned_segment["final_state"]
                    )

                    index += 1

                planned_queue.put(
                    (None, None, None)
                )

                mark_motion_timing(
                    self.node,
                    "ordered_planning_worker_done",
                    duration_s=perf_counter() - worker_started,
                )

            except Exception as exc:
                mark_motion_timing(
                    self.node,
                    "ordered_planning_worker_error",
                    duration_s=perf_counter() - worker_started,
                    error=str(exc),
                )

                planned_queue.put(
                    (None, None, exc)
                )

        mark_motion_timing(self.node, "ordered_planner_submit_start")
        planner_future = executor.submit(_planning_worker)
        mark_motion_timing(self.node, "ordered_planner_submit_done")
        try:
            for expected_index in range(len(segments)):
                wait_started = perf_counter()
                mark_motion_timing(
                    self.node,
                    "ordered_plan_wait_start",
                    index=expected_index + 1,
                    timeout_s=plan_timeout_s,
                )
                try:
                    planned_index, planned, exc = planned_queue.get(timeout=plan_timeout_s)
                except Empty as exc:
                    raise TimeoutError(
                        f"Timed out waiting for ordered-chain plan index={expected_index + 1}"
                    ) from exc
                if exc is not None:
                    raise RuntimeError(f"ordered-chain planning failed: {exc}") from exc
                if planned_index is None or planned is None:
                    raise RuntimeError(
                        f"ordered-chain planner ended before segment {expected_index + 1}"
                    )
                if planned_index != expected_index:
                    raise RuntimeError(
                        f"ordered-chain planner returned segment {planned_index + 1}, "
                        f"expected {expected_index + 1}"
                    )
                planning_node.get_logger().info(
                    f"[TIMING] ordered_motion_chain_plan_ready index={expected_index + 1} "
                    f"wait_before_execute_s={perf_counter() - wait_started:.3f}"
                )
                mark_motion_timing(
                    self.node,
                    "ordered_plan_ready",
                    index=expected_index + 1,
                    label=planned.get("label"),
                    wait_before_execute_s=perf_counter() - wait_started,
                    plan_s=float(planned.get("plan_elapsed_s", 0.0) or 0.0),
                )
                _mark_consumed(expected_index)
                preplanned_snapshot = _preplanned_snapshot(current_index=expected_index + 1)
                self._set_ordered_motion_chain_status(**preplanned_snapshot)

                if bool(getattr(self.node, "_ordered_motion_chain_stop_requested", False)):
                    stop_planning.set()
                    self._set_ordered_motion_chain_status(
                        active=False,
                        phase="stopped",
                        current_segment_index=expected_index,
                        current_segment_number=expected_index + 1,
                        current_segment_label=planned.get("label"),
                        current_segment_type=planned.get("type"),
                        current_segment_protected=bool(planned.get("protected", False)),
                        result=-14,
                    )
                    return -14

                setattr(self.node, "_suppress_post_success_unwind", expected_index + 1 < len(segments))
                result = _execute_planned_segment(
                    expected_index + 1,
                    len(segments),
                    planned,
                    preplanned_ready_count=preplanned_snapshot["preplanned_ready_count"],
                )
                if result != 0:
                    stop_planning.set()
                    return result
            planner_future.result(timeout=1.0)
            return 0
        finally:
            setattr(self.node, "_suppress_post_success_unwind", previous_execution_suppress)
            stop_planning.set()
            executor.shutdown(wait=True)

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
            self.node.get_logger().info('[UNWIND_J6] Rotational-path unwind skipped - no unwind needed')
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
            current_pos_wobj = self.get_current_position()
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
    ):
        started_at = perf_counter()
        direct_ik_step_deg = max(0.1, float(getattr(config, 'EXECUTOR_POST_UNWIND_DIRECT_IK_STEP_DEG', 4.0)))
        waypoints_base = self._rotational_path_waypoints_base(
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            max_step_override_deg=direct_ik_step_deg,
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
                waypoint = self.apply_workobject(waypoint)
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

    # ---------------- Status Methods ----------------
    def get_current_position(self):
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

            # Transform from base to workobject frame if a workobject exists
            if self.workobject is not None:
                pose = self.workobject.apply(pose, inverse=True)

            return pose
        except Exception as e:
            print(f"get_current_position error: {e}")
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
    def start_jog(self, axis: RobotAxis, direction: Direction, step, vel, acc):
        drive_error = self._reject_if_drive_not_enabled("JOG")
        if drive_error is not None:
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[JOG] Rejected: %s",
                self.node.get_hardware_fault_reason(),
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

        current_pos_wobj = self.get_current_position()
        if current_pos_wobj is None or len(current_pos_wobj) < 6:
            return -1

        x, y, z, rx, ry, rz = current_pos_wobj
        deltas = [0.0] * 6
        deltas[axis_val - 1] = step * dir_val
        target_pos_wobj = [
            x + deltas[0], y + deltas[1], z + deltas[2],
            rx + deltas[3], ry + deltas[4], rz + deltas[5]
        ]

        new_pos_base = self.apply_workobject(target_pos_wobj)

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
            deadline = time.time() + float(getattr(config, 'JOG_BLOCKING_TIMEOUT_S', 5.0))
            while time.time() < deadline:
                if (
                        not self.node.is_executing
                        and not self.node.is_motion_active()
                        and not self.node.has_pending_motion()
                ):
                    return self.node.last_move_result
                time.sleep(0.01)

            self.node.get_logger().error(
                '[JOG] Timed out waiting for jog motion to complete after %.2fs',
                float(getattr(config, 'JOG_BLOCKING_TIMEOUT_S', 5.0)),
            )
            return -1
        except Exception as e:
            self.node.get_logger().error(f"Jog error: {e}")
            return -1

    @staticmethod
    def _rotational_jog_max_step_deg():
        return max(0.1, float(getattr(config, 'JOG_MAX_ORIENTATION_STEP_DEG', 5.0)))

    def _should_interpolate_rotational_jog(self, angular_delta_deg):
        return abs(float(angular_delta_deg)) > self._rotational_jog_max_step_deg()

    def _send_rotational_jog_path(self, current_pos_wobj, target_pos_wobj, rotation_index, vel_scale, acc_scale):
        angular_delta = float(target_pos_wobj[rotation_index]) - float(current_pos_wobj[rotation_index])
        max_step = self._rotational_jog_max_step_deg()
        step_count = max(2, int(math.ceil(abs(angular_delta) / max_step)))
        waypoints_base = self._rotational_path_waypoints_base(
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
        )
        target_base = waypoints_base[-1]
        self.node.get_logger().info(
            f'[JOG] Rotational jog path: axis_index={rotation_index} '
            f'delta={angular_delta:.3f}deg waypoints={len(waypoints_base)} '
            f'max_step={max_step:.3f}deg'
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