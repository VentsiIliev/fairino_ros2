from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from utils.transformation_utils import TransformationUtils
from fairino5_v6_moveit2_config.srv import ApplyIPP
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


def send_path_cartesian(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
    """Cartesian Path: Uses MoveIt's compute_cartesian_path service with adaptive step sizing.

    Args:
        waypoints_mm: List of TCP waypoints [x_mm, y_mm, z_mm] in millimeters
        rx, ry, rz: TCP orientation in degrees (same for all waypoints)
    """
    robot_controller.safety_manager.force_update()
    num_waypoints = len(waypoints_mm)
    robot_controller.get_logger().info(f'[Cartesian Path] Computing smooth path through {num_waypoints} waypoints')
    robot_controller.get_logger().info(f'[Cartesian Path] vel= {vel_scaling}, acc= {acc_scaling}')

    # Debug: Log first TCP waypoint
    if num_waypoints > 0:
        robot_controller.get_logger().info(f'[Cartesian Path] First TCP waypoint: X={waypoints_mm[0][0]:.1f}mm Y={waypoints_mm[0][1]:.1f}mm Z={waypoints_mm[0][2]:.1f}mm')
        robot_controller.get_logger().info(f'[Cartesian Path] TCP orientation: RX={rx}° RY={ry}° RZ={rz}°')
        robot_controller.get_logger().info(f'[Cartesian Path] T_tool offset from ee_link: {robot_controller.T_tool[2,3] * 1000:.1f}mm in Z')

    if not robot_controller.cart_path_client.wait_for_service(timeout_sec=1.0):
        robot_controller.get_logger().error('[Cartesian Path] compute_cartesian_path service not available')
        return

    # Create waypoint poses for ee_link (applying inverse of tool transform only)
    # Since MoveIt now plans for ee_link, we only need to remove the tool offset
    waypoints = []
    for i, wp in enumerate(waypoints_mm):
        # Build the desired TCP pose
        tcp_pose = [wp[0], wp[1], wp[2], rx, ry, rz]
        T_tcp_desired = TransformationUtils.pose_to_transform(tcp_pose)

        # Apply inverse of tool transform (ee_link → TCP)
        # to get ee_link pose, which MoveIt will plan for
        # Input: TCP coordinates → Output: ee_link coordinates
        T_ee_link = TransformationUtils.remove_tcp_offset(T_tcp_desired, robot_controller.T_tool)

        # Extract ee_link pose
        ee_position = T_ee_link[:3, 3]
        ee_quat = TransformationUtils.matrix_to_quaternion(T_ee_link[:3, :3])

        # Pre-validate waypoint safety
        is_safe, msg = robot_controller.safety_manager.check_position_safety(
            ee_position[0], ee_position[1], ee_position[2]
        )
        if not is_safe:
            robot_controller.get_logger().error(f'[SAFETY] Waypoint {i + 1} rejected: {msg}')
            return  # Reject entire path if any waypoint unsafe
        if "Warning" in msg and i == 0:  # Only warn once
            robot_controller.get_logger().warning(f'[SAFETY] {msg}')

        # Debug: Log first waypoint transformation
        if i == 0:
            robot_controller.get_logger().info(f'[Cartesian Path] After inv(T_tool): ee_link Z={ee_position[2] * 1000:.1f}mm')

        pose = Pose()
        pose.position.x = ee_position[0]
        pose.position.y = ee_position[1]
        pose.position.z = ee_position[2]
        pose.orientation.x = ee_quat[0]
        pose.orientation.y = ee_quat[1]
        pose.orientation.z = ee_quat[2]
        pose.orientation.w = ee_quat[3]
        waypoints.append(pose)

    # Adaptive step size based on path complexity
    # Finer steps for larger paths ensure smoother trajectories
    if num_waypoints > 10:
        max_step = 0.0015  # 1.5 mm
    elif num_waypoints > 5:
        max_step = 0.001  # 1 mm
    else:
        max_step = 0.0008  # 0.8 mm

    # Create a service request
    request = GetCartesianPath.Request()
    request.header.frame_id = 'base_link'
    request.group_name = 'fairino5_v6_group'
    request.link_name = 'ee_link'  # Explicitly set link_name to match SRDF tip_link
    request.waypoints = waypoints
    request.max_step = max_step
    request.jump_threshold = 0.0  # Disabled - no jump checking
    request.avoid_collisions = True
    # Set velocity and acceleration scaling factors for TOTG
    request.max_velocity_scaling_factor = vel_scaling
    request.max_acceleration_scaling_factor = acc_scaling

    # ✅ CRITICAL FIX: Set start state to robot's CURRENT position
    # Without this, MoveIt uses an empty/default state causing trajectory mismatch
    if hasattr(robot_controller, 'current_joint_state') and robot_controller.current_joint_state is not None:
        request.start_state.joint_state = robot_controller.current_joint_state
        request.start_state.is_diff = False  # Use as absolute state, not differential
    else:
        robot_controller.get_logger().warning('[Cartesian Path] No current joint state available - trajectory may not execute correctly')

    robot_controller.get_logger().info(f'[Cartesian Path] Using max_step={max_step * 1000:.1f}mm for {num_waypoints} waypoints')

    robot_controller.get_logger().info('[Cartesian Path] Requesting cartesian path computation...')
    future = robot_controller.cart_path_client.call_async(request)
    future.add_done_callback(lambda f: _cartesian_path_response(robot_controller,f, vel_scaling, acc_scaling))

def apply_ipp_totg(robot_controller, trajectory, vel_scaling=0.6, acc_scaling=0.4, callback=None):
    """Call the IPP service to apply TOTG (ASYNC).

    Args:
        trajectory: RobotTrajectory to time-parameterize
        vel_scaling: Velocity scaling factor [0.0-1.0]
        acc_scaling: Acceleration scaling factor [0.0-1.0]
        callback: Function to call with (trajectory_or_None) when done
    """
    robot_controller.get_logger().info('[TOTG] Checking if IPP service is available...')

    if not robot_controller.ipp_client.wait_for_service(timeout_sec=5.0):
        robot_controller.get_logger().error('[TOTG] ✗ IPP service /apply_ipp NOT available after 5s!')
        robot_controller.get_logger().error('[TOTG]    Is ipp_helper node running? Check: ros2 node list | grep ipp')
        if callback:
            callback(None)
        return

    robot_controller.get_logger().info('[TOTG] ✓ IPP service is available')

    request = ApplyIPP.Request()
    request.trajectory = trajectory.joint_trajectory
    request.max_velocity_scaling = float(vel_scaling)
    request.max_acceleration_scaling = float(acc_scaling)

    robot_controller.get_logger().info(
        f'[TOTG] Requesting time-optimal parameterization (vel={vel_scaling}, acc={acc_scaling})')

    # Call ASYNC to avoid blocking the executor
    future = robot_controller.ipp_client.call_async(request)

    # Handle response when ready
    def handle_ipp_response(fut):
        try:
            response = fut.result()

            if response is None:
                robot_controller.get_logger().error('[TOTG] ✗ Response is None')
                if callback:
                    callback(None)
                return

            # ✅ FIX: Extract JointTrajectory from RobotTrajectory response
            if not hasattr(response, 'trajectory'):
                robot_controller.get_logger().error('[TOTG] ✗ Response has no trajectory attribute')
                if callback:
                    callback(None)
                return

            # Response is RobotTrajectory, extract joint_trajectory
            if not hasattr(response.trajectory, 'joint_trajectory'):
                robot_controller.get_logger().error('[TOTG] ✗ Response.trajectory has no joint_trajectory attribute')
                if callback:
                    callback(None)
                return

            joint_traj = response.trajectory.joint_trajectory
            num_points = len(joint_traj.points)

            if num_points == 0:
                robot_controller.get_logger().error('[TOTG] ✗ Empty trajectory - TOTG failed')
                if callback:
                    callback(None)
                return

            robot_controller.get_logger().info(
                f'[TOTG] ✓ Generated {num_points} time-parameterized points')

            # Validate that timestamps are present
            has_timestamps = False
            for pt in joint_traj.points:
                if pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0:
                    has_timestamps = True
                    break

            if not has_timestamps:
                robot_controller.get_logger().error('[TOTG] ✗ Response has no timestamps - INVALID trajectory')
                if callback:
                    callback(None)
                return

            # ✅ Success: Update trajectory with time-parameterized result
            trajectory.joint_trajectory = joint_traj
            if callback:
                callback(trajectory)

        except Exception as e:
            robot_controller.get_logger().error(f'[TOTG] ✗ Service call failed: {e}')
            import traceback
            robot_controller.get_logger().error(f'[TOTG] Traceback: {traceback.format_exc()}')
            if callback:
                callback(None)

    future.add_done_callback(handle_ipp_response)

def _cartesian_path_response(robot_controller, future, vel_scaling, acc_scaling):
        robot_controller.safety_manager.force_update()
        try:
            response = future.result()
            fraction = response.fraction
            robot_controller.get_logger().info(f'[Cartesian Path] Path computed: {fraction * 100:.1f}% successful')

            if fraction < 0.9:
                robot_controller.get_logger().warning(f'[Cartesian Path] Only {fraction * 100:.1f}% of path could be computed')
                return

            # Get the computed trajectory
            trajectory = response.solution

            robot_controller.get_logger().info(
                f'[Cartesian Path] Computed trajectory has {len(trajectory.joint_trajectory.points)} points')

            # ✅ CRITICAL: Apply TOTG for time-optimal velocity profile (ASYNC)
            # This ensures smooth continuous motion without stops at waypoints
            def on_totg_done(result_trajectory):
                if result_trajectory is None:
                    robot_controller.get_logger().error('[Cartesian Path] ✗ TOTG failed - aborting execution')
                    robot_controller.get_logger().error('[Cartesian Path] Cannot execute trajectory without time parameterization')
                    return

                # Send trajectory directly to the controller
                _send_trajectory_to_controller(robot_controller,result_trajectory.joint_trajectory)

            apply_ipp_totg(robot_controller,trajectory, vel_scaling, acc_scaling, callback=on_totg_done)

        except Exception as e:
            robot_controller.get_logger().error(f'[Cartesian Path] Service call failed: {e}')

def _send_trajectory_to_controller(robot_controller, joint_trajectory):
        """Send trajectory DIRECTLY to the low-level controller for maximum smoothness.

        Bypasses MoveIt's ExecuteTrajectory action which can enforce unwanted path constraints.
        The controller's spline interpolation will smoothly blend through all waypoints.
        """
        if not robot_controller.execution_lock.acquire(blocking=False):
            robot_controller.get_logger().warning('[Controller] Trajectory already executing, ignoring')
            return

        if robot_controller.is_executing:
            robot_controller.get_logger().warning('[Controller] Previous trajectory still active')
            robot_controller.execution_lock.release()
            return

        if not robot_controller.controller_client.wait_for_server(timeout_sec=1.0):
            robot_controller.get_logger().error('[Controller] fairino5_controller not available')
            robot_controller.execution_lock.release()
            return

        # ✅ CRITICAL VALIDATION: Check for timestamps
        if len(joint_trajectory.points) == 0:
            robot_controller.get_logger().error('[Controller] ✗ Empty trajectory - aborting')
            robot_controller.execution_lock.release()
            return

        has_valid_timestamps = False
        for pt in joint_trajectory.points:
            if pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0:
                has_valid_timestamps = True
                break

        if not has_valid_timestamps:
            robot_controller.get_logger().error('[Controller] ✗ Trajectory has NO timestamps - aborting')
            robot_controller.get_logger().error('[Controller] TOTG must have failed. Cannot execute.')
            robot_controller.execution_lock.release()
            return

        robot_controller.is_executing = True

        # Create controller goal with NO path tolerances
        # Only enforce goal position (final point)

        goal_tolerance = []
        for name in joint_trajectory.joint_names:
            tol = JointTolerance()
            tol.name = name
            tol.position = 0.01  # 0.01 rad final position tolerance
            tol.velocity = 0.0  # Must stop at end
            tol.acceleration = 0.0
            goal_tolerance.append(tol)

        controller_goal = FollowJointTrajectory.Goal()
        controller_goal.trajectory = joint_trajectory
        controller_goal.path_tolerance = []  # EMPTY = smooth blending, no stops
        controller_goal.goal_tolerance = goal_tolerance

        # Calculate trajectory duration and set VERY generous time tolerance
        # This prevents spurious aborts when controller is slightly behind schedule
        if len(joint_trajectory.points) > 0:
            last_point = joint_trajectory.points[-1]
            traj_duration_sec = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            # Tolerance = 2x trajectory duration or 5s minimum
            # This gives controller plenty of room for execution delays
            time_tolerance_sec = max(5.0, traj_duration_sec * 2.0)

            # DEBUG: Log trajectory details
            robot_controller.get_logger().info(
                f'[Controller] Trajectory duration: {traj_duration_sec:.2f}s, timeout: {time_tolerance_sec:.1f}s')
            robot_controller.get_logger().info(
                f'[Controller] First point positions: {[round(p, 3) for p in joint_trajectory.points[0].positions]}')
            robot_controller.get_logger().info(f'[Controller] Last point positions: {[round(p, 3) for p in last_point.positions]}')
            robot_controller.get_logger().info(
                f'[Controller] First point time: {joint_trajectory.points[0].time_from_start.sec + joint_trajectory.points[0].time_from_start.nanosec / 1e9:.3f}s')
            robot_controller.get_logger().info(f'[Controller] Last point time: {traj_duration_sec:.3f}s')
        else:
            time_tolerance_sec = 5.0

        controller_goal.goal_time_tolerance.sec = int(time_tolerance_sec)
        controller_goal.goal_time_tolerance.nanosec = int((time_tolerance_sec % 1.0) * 1e9)

        robot_controller.get_logger().info(
            f'[Controller] Sending {len(joint_trajectory.points)} points directly to controller '
            f'(spline interpolation, no path tolerance, goal_time_tolerance={time_tolerance_sec:.1f}s)')

        future = robot_controller.controller_client.send_goal_async(controller_goal)
        robot_controller.active_execute_send_future = future
        future.add_done_callback(lambda f: _controller_goal_response(robot_controller, f))


def send_cartesian_goal(robot_controller, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, planner_id='LIN',
                        tool_transform=None):
    """
    Send a single-point Cartesian goal using MoveIt's compute_cartesian_path service.
    This provides smooth motion through Pilz planner without MoveGroup action.

    Args:
        x_mm, y_mm, z_mm: TCP target position in mm
        rx, ry, rz: TCP target orientation in degrees
        vel_scale: Velocity scaling (0.0-1.0)
        acc_scale: Acceleration scaling (0.0-1.0)
        planner_id: Ignored (kept for API compatibility, always uses cartesian path)
        tool_transform: Tool offset transform (optional)
    """
    robot_controller.safety_manager.force_update()

    if robot_controller.is_executing:
        robot_controller.get_logger().warning('[MOVE] Previous trajectory still executing, ignoring new goal')
        return

    # Use a single-waypoint cartesian path (smoother than MoveGroup action)
    send_path_cartesian(
        robot_controller,
        waypoints_mm=[[x_mm, y_mm, z_mm]],
        rx=rx, ry=ry, rz=rz,
        vel_scaling=vel_scale,
        acc_scaling=acc_scale
    )

def _controller_goal_response(robot_controller, future):
    """Handle controller goal acceptance."""
    robot_controller.active_execute_send_future = None
    try:
        goal_handle = future.result()
        if not goal_handle.accepted:
            robot_controller.get_logger().error('[Controller] Trajectory execution rejected by fairino5_controller')
            robot_controller.active_controller_goal = None
            robot_controller.is_executing = False
            robot_controller.execution_lock.release()
            return

        robot_controller.get_logger().info('[Controller] Trajectory accepted by fairino5_controller')
        robot_controller.active_controller_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: _controller_goal_result(robot_controller, f))
    except Exception as e:
        robot_controller.get_logger().error(f'[Controller] Goal response error: {e}')
        robot_controller.is_executing = False
        robot_controller.execution_lock.release()

def _controller_goal_result(robot_controller, future):
    """Handle controller goal completion."""
    try:
        result = future.result().result
        if result.error_code == 0:
            robot_controller.get_logger().info('[Controller] ✓ Trajectory execution succeeded!')
        else:
            robot_controller.get_logger().error(f'[Controller] Trajectory execution failed with error: {result.error_code}')
    except Exception as e:
        robot_controller.get_logger().error(f'[Controller] Result error: {e}')
    finally:
        robot_controller.active_controller_goal = None
        robot_controller.is_executing = False
        if robot_controller.execution_lock.locked():
            robot_controller.execution_lock.release()