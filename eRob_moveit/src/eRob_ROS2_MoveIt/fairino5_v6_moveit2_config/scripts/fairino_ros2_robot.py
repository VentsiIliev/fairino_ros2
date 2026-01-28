#!/usr/bin/env python3
from enums import RobotAxis, Direction


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
            result = self.node.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale=vel_scale, acc_scale=acc_scale, planner_id='LIN', tool_transform=tool_transform)

            # Return error code if planning/submission failed
            if result != 0:
                return result

            if blocking:
                self.node.get_logger().info(f"[MOVE_LINER] Blocking until position reached: {position_base}")
                return self.wait_for_position(position_base, threshold=1.0, timeout=60.0)

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
        result = self.node.send_path_cartesian(
            waypoints_mm=waypoints_xyz,
            rx=rx,
            ry=ry,
            rz=rz,
            vel_scaling=vel,
            acc_scaling=acc
        )

        # Return error code if planning/submission failed
        if result < 0:
            # Error codes: -2, -3, -5 etc
            return result

        if result > 0:
            # Positive = queued position (don't block on queued commands)
            self.node.get_logger().info(f"[EXECUTE_PATH] Command queued at position {result}")
            return result

        # result == 0: executing immediately
        if blocking:
            last_waypoint = waypoints_xyz[-1] + [rx, ry, rz]
            self.node.get_logger().info(f"[EXECUTE_PATH] Blocking until final position reached")
            return self.wait_for_position(last_waypoint, threshold=1.0, timeout=60.0)

        return 0

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

    def wait_for_position(self, target_position, threshold=1.0, timeout=30.0, check_interval=0.01):
        import time, math
        start_time = time.time()
        if len(target_position) >= 3:
            target_xyz = target_position[:3]
        else:
            return False

        while True:
            if time.time() - start_time > timeout:
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
                    return True
            except:
                pass

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
            result = self.node.send_cartesian_goal(
                x_base, y_base, z_base,
                rx_base, ry_base, rz_base,
                vel_scale=vel_scale, acc_scale=acc_scale,
                planner_id='LIN'
            )
            return result
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

