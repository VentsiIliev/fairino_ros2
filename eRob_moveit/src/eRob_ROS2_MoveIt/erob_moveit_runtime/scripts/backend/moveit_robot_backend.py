#!/usr/bin/env python3
from enums import RobotAxis, Direction
from time import perf_counter
from threading import Event
import math
import time
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
                dx = waypoints_pose[i+1][0] - waypoints_pose[i][0]
                dy = waypoints_pose[i+1][1] - waypoints_pose[i][1]
                dz = waypoints_pose[i+1][2] - waypoints_pose[i][2]
                total_dist += (dx**2 + dy**2 + dz**2) ** 0.5
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

    def execute_custom_sequence(self, segments, tool=0, user=0, blocking=True):
        started_at = perf_counter()
        if self.node is None:
            return -1
        if not segments:
            return -1
        drive_error = self._reject_if_drive_not_enabled("EXECUTE_CUSTOM_SEQUENCE")
        if drive_error is not None:
            return drive_error
        if self.node is not None and not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[EXECUTE_CUSTOM_SEQUENCE] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        try:
            tool_transform = self.node.get_tool_transform(tool)
            custom_segments = []
            for segment in segments:
                position_base = self.apply_workobject(segment["position"], user_id=user)
                custom_segments.append({
                    "label": str(segment.get("label", "")),
                    "position": position_base,
                    "vel": float(segment["vel"]),
                    "acc": float(segment["acc"]),
                    "motion_type": str(segment.get("motion_type", "linear")),
                })

            from motion.planning.custom_sequence import execute_custom_sequence
            result = execute_custom_sequence(
                self.node,
                custom_segments,
                tool_transform=tool_transform,
            )
            self.node.get_logger().info(
                f"[TIMING] backend_execute_custom_sequence blocking={bool(blocking)} "
                f"result={result} segments={len(custom_segments)} elapsed_s={perf_counter() - started_at:.3f}"
            )
            return result
        except Exception as e:
            print(f"execute_custom_sequence error: {e}")
            return -1

    def execute_staged_path(
        self,
        stage_position,
        path,
        tool=0,
        user=0,
        stage_vel=0.6,
        stage_acc=0.4,
        path_vel=0.6,
        path_acc=0.4,
        blocking=True,
        trajectory_optimizer=None,
    ):
        started_at = perf_counter()
        if self.node is None or not stage_position or not path:
            return -1
        drive_error = self._reject_if_drive_not_enabled("EXECUTE_STAGED_PATH")
        if drive_error is not None:
            return drive_error
        if not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[EXECUTE_STAGED_PATH] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY
        try:
            tool_transform = self.node.get_tool_transform(tool)
            stage_position_base = self.apply_workobject(list(stage_position[:6]), user_id=user)
            path_base = []
            for waypoint in path:
                if len(waypoint) >= 6:
                    wp_full = list(waypoint[:6])
                else:
                    current = self.get_current_position() or [0, 0, 0, *config.DEFAULT_ORIENTATION]
                    wp_full = [waypoint[0], waypoint[1], waypoint[2], current[3], current[4], current[5]]
                path_base.append(list(self.apply_workobject(wp_full, user_id=user)[:6]))

            selected_optimizer = trajectory_optimizer
            if not selected_optimizer:
                selected_optimizer = str(
                    getattr(config, "PATH_TRAJECTORY_OPTIMIZER", "") or ""
                ).strip().upper() or None

            from motion.planning.staged_path import execute_staged_path
            result = execute_staged_path(
                self.node,
                stage_position=list(stage_position_base[:6]),
                command_path=path_base,
                stage_vel=float(stage_vel),
                stage_acc=float(stage_acc),
                path_vel=float(path_vel),
                path_acc=float(path_acc),
                tool_transform=tool_transform,
                trajectory_optimizer_name=selected_optimizer,
            )
            self.node.get_logger().info(
                f"[TIMING] backend_execute_staged_path blocking={bool(blocking)} "
                f"result={result} waypoints={len(path_base)} elapsed_s={perf_counter() - started_at:.3f}"
            )
            return result
        except Exception as e:
            print(f"execute_staged_path error: {e}")
            return -1

    def unwind_joint6(self, blocking=True, queue_if_busy=True, vel=None, acc=None):
        if self.node is None:
            return -1
        drive_error = self._reject_if_drive_not_enabled("UNWIND_J6")
        if drive_error is not None:
            return drive_error
        if not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[UNWIND_J6] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY

        result = self.node.trajectory_executor.request_explicit_unwind(
            queue_if_busy=queue_if_busy,
            vel=vel,
            acc=acc,
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
            import time
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
        waypoints_base = []
        for step_index in range(1, step_count + 1):
            alpha = step_index / float(step_count)
            waypoint = list(current_pos_wobj[:6])
            waypoint[3] = float(current_pos_wobj[3]) + (float(target_pos_wobj[3]) - float(current_pos_wobj[3])) * alpha
            waypoint[4] = float(current_pos_wobj[4]) + (float(target_pos_wobj[4]) - float(current_pos_wobj[4])) * alpha
            waypoint[5] = float(current_pos_wobj[5]) + (float(target_pos_wobj[5]) - float(current_pos_wobj[5])) * alpha
            waypoints_base.append(list(self.apply_workobject(waypoint)[:6]))

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
