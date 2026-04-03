#!/usr/bin/env python3
from enums import RobotAxis, Direction
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

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0, blocking=True, trajectory_optimizer=None):
        if len(position) != 6:
            return -1
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
                deadline = time.time() + config.BLOCKING_MOVE_TIMEOUT_S
                while self.node.is_executing and time.time() < deadline:
                    time.sleep(0.05)
                if self.node.is_executing:
                    self.node.get_logger().error(
                        f"[MOVE_LINER] Timed out waiting for move to complete after {config.BLOCKING_MOVE_TIMEOUT_S}s")
                    return -1
                return self.node.last_move_result

            return 0  # non-blocking: fire-and-forget
        except Exception as e:
            print(f"move_liner error: {e}")
            return -1

    def execute_path(self, path, rx=None, ry=None, rz=None, vel=0.6, acc=0.4, blocking=True):
        """Execute path with automatic selection of best execution strategy.

        Strategy selection based on path density:
        - Dense paths (avg spacing < 2mm): Use compute_cartesian_path (continuous contour)
        - Sparse paths (avg spacing >= 2mm): Always use compute_cartesian_path for consistency

        This ensures smooth motion without stops for all path types.
        """
        if not path or self.node is None:
            return -1
        if not self.node.is_hardware_ready_for_motion():
            self.node.get_logger().error(
                "[EXECUTE_PATH] Rejected: %s",
                self.node.get_hardware_fault_reason(),
            )
            return config.MOTION_ERROR_HARDWARE_NOT_READY

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
                        rx, ry, rz = config.DEFAULT_ORIENTATION
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

        # A single waypoint is not a real path. Route it through the single-target
        # pipeline so adaptive interpolation / micro-move Jacobian fallback are applied
        # before MoveIt + TOTG are involved.
        if len(waypoints_xyz) == 1:
            self.node.get_logger().info(
                "[EXECUTE_PATH] Single waypoint detected — delegating to single-target planner"
            )
            from motion.strategies import SingleTargetStrategy

            target = waypoints_xyz[0]
            result = self.node.execute(
                SingleTargetStrategy(target[0], target[1], target[2], rx, ry, rz, vel, acc)
            )
        else:
            # ✅ ALWAYS use compute_cartesian_path for consistency
            # Controller's spline interpolation + TOTG provides smooth continuous motion
            self.node.get_logger().info("[EXECUTE_PATH] Using MoveIt compute_cartesian_path (continuous trajectory)")
            from motion.strategies import PathStrategy
            result = self.node.execute(PathStrategy(waypoints_xyz, rx, ry, rz, vel, acc))

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
            deadline = time.time() + config.BLOCKING_MOVE_TIMEOUT_S
            while self.node.is_executing and time.time() < deadline:
                time.sleep(0.05)
            if self.node.is_executing:
                self.node.get_logger().error(
                    f"[EXECUTE_PATH] Timed out waiting for move to complete after {config.BLOCKING_MOVE_TIMEOUT_S}s")
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

        new_pos_base = self.apply_workobject([
            x + deltas[0], y + deltas[1], z + deltas[2],
            rx + deltas[3], ry + deltas[4], rz + deltas[5]
        ])

        vel_scale = max(0.0, min(1.0, vel / 100.0))
        acc_scale = max(0.0, min(1.0, acc / 100.0))

        try:
            x_b, y_b, z_b, rx_b, ry_b, rz_b = new_pos_base
            avoid_collisions = bool(getattr(config, 'JOG_AVOID_COLLISIONS', True))
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

        Note: Not implemented in the ROS2 version - requires hardware interface
        """
        print(f"setDigitalOutput: port {portId} -> {value} (not implemented in ROS2)")
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
