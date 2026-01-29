from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import RobotState
from utils.transformation_utils import TransformationUtils
from .trajectory_executor import _send_trajectory_to_controller
from .trajectory_optimization import apply_ipp_totg
import numpy as np


def _diagnose_fk_mismatch(robot_controller, first_waypoint_pose, joint_state):
    """
    Compare the first waypoint (from C++ Cartesian) with MoveIt's FK result.
    This helps identify why compute_cartesian_path returns 0% success.
    """
    logger = robot_controller.get_logger()

    # Create FK service client if not exists
    if not hasattr(robot_controller, '_fk_client'):
        robot_controller._fk_client = robot_controller.create_client(
            GetPositionFK, '/compute_fk'
        )

    if not robot_controller._fk_client.wait_for_service(timeout_sec=1.0):
        logger.warning('[FK Diagnostic] /compute_fk service not available')
        return

    # Build FK request
    from copy import deepcopy
    fk_request = GetPositionFK.Request()
    fk_request.header.frame_id = 'base_link'
    fk_request.fk_link_names = ['ee_link']

    # Set robot state from current joints
    fk_request.robot_state.joint_state = deepcopy(joint_state)
    fk_request.robot_state.is_diff = False

    # Call FK service with timeout polling (safe to call from callbacks)
    try:
        import time
        future = robot_controller._fk_client.call_async(fk_request)

        # Poll for completion (works inside callbacks unlike spin_until_future_complete)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 2.0:
                logger.warning('[FK Diagnostic] FK service call timed out')
                return
            time.sleep(0.01)

        if future.result() is None:
            logger.warning('[FK Diagnostic] FK service call returned None')
            return

        fk_response = future.result()

        if fk_response.error_code.val != 1:  # MoveItErrorCodes.SUCCESS = 1
            logger.warning(f'[FK Diagnostic] FK computation failed with error code: {fk_response.error_code.val}')
            return

        if len(fk_response.pose_stamped) == 0:
            logger.warning('[FK Diagnostic] FK returned no poses')
            return

        # Extract MoveIt's FK result for ee_link
        moveit_pose = fk_response.pose_stamped[0].pose
        moveit_pos = np.array([
            moveit_pose.position.x * 1000,  # Convert to mm
            moveit_pose.position.y * 1000,
            moveit_pose.position.z * 1000
        ])
        moveit_quat = np.array([
            moveit_pose.orientation.x,
            moveit_pose.orientation.y,
            moveit_pose.orientation.z,
            moveit_pose.orientation.w
        ])

        # Extract first waypoint position (already in meters in the Pose)
        waypoint_pos = np.array([
            first_waypoint_pose.position.x * 1000,  # Convert to mm
            first_waypoint_pose.position.y * 1000,
            first_waypoint_pose.position.z * 1000
        ])
        waypoint_quat = np.array([
            first_waypoint_pose.orientation.x,
            first_waypoint_pose.orientation.y,
            first_waypoint_pose.orientation.z,
            first_waypoint_pose.orientation.w
        ])

        # Compute position difference
        pos_diff = waypoint_pos - moveit_pos
        pos_dist = np.linalg.norm(pos_diff)

        # Compute orientation difference (quaternion dot product)
        quat_dot = abs(np.dot(moveit_quat, waypoint_quat))
        angle_diff_deg = np.degrees(2 * np.arccos(np.clip(quat_dot, -1.0, 1.0)))

        # Log comparison
        logger.info('=' * 60)
        logger.info('[FK Diagnostic] Comparing first waypoint vs MoveIt FK:')
        logger.info(f'[FK Diagnostic] MoveIt FK ee_link:  X={moveit_pos[0]:.2f} Y={moveit_pos[1]:.2f} Z={moveit_pos[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] First waypoint:     X={waypoint_pos[0]:.2f} Y={waypoint_pos[1]:.2f} Z={waypoint_pos[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] Position DIFF:      dX={pos_diff[0]:.2f} dY={pos_diff[1]:.2f} dZ={pos_diff[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] Position distance:  {pos_dist:.2f} mm')
        logger.info(f'[FK Diagnostic] Orientation diff:   {angle_diff_deg:.2f} degrees')
        logger.info(f'[FK Diagnostic] MoveIt quat:        [{moveit_quat[0]:.4f}, {moveit_quat[1]:.4f}, {moveit_quat[2]:.4f}, {moveit_quat[3]:.4f}]')
        logger.info(f'[FK Diagnostic] Waypoint quat:      [{waypoint_quat[0]:.4f}, {waypoint_quat[1]:.4f}, {waypoint_quat[2]:.4f}, {waypoint_quat[3]:.4f}]')

        if pos_dist > 1.0:  # More than 1mm difference
            logger.error(f'[FK Diagnostic] ⚠️  POSITION MISMATCH > 1mm! This may cause path planning failure.')
        if angle_diff_deg > 1.0:  # More than 1 degree difference
            logger.error(f'[FK Diagnostic] ⚠️  ORIENTATION MISMATCH > 1°! This may cause path planning failure.')

        logger.info('=' * 60)

    except Exception as e:
        logger.error(f'[FK Diagnostic] Exception during FK comparison: {e}')


def send_path_cartesian(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
    """Cartesian Path: Uses MoveIt's compute_cartesian_path service with adaptive step sizing.

    Args:
        waypoints_mm: List of TCP waypoints [x_mm, y_mm, z_mm] in millimeters
        rx, ry, rz: TCP orientation in degrees (same for all waypoints)

    Returns:
        int: 0 if executing now, >0 if queued (queue position), -2 if service unavailable, -5 if queue full
    """
    robot_controller.safety_manager.force_update()

    # If robot is executing, queue this command
    if robot_controller.is_executing:
        result = robot_controller.motion_queue.submit(
            task_function=_execute_path_internal,
            task_args=[robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling]
        )
        if isinstance(result, tuple):
            task_id, position = result
            robot_controller.get_logger().info(f'[Queue] Motion queued at position {position} (task #{task_id})')
            return position  # Return queue position (positive number)
        else:
            robot_controller.get_logger().error(f'[Queue] Queue is full! Cannot accept new motion')
            return result  # -5 = queue full

    # Robot is idle, execute immediately
    return _execute_path_internal(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling)


def _execute_path_internal(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
    """Internal function to execute path (called directly or from queue)."""
    num_waypoints = len(waypoints_mm)
    robot_controller.get_logger().info(f'[Cartesian Path] Computing smooth path through {num_waypoints} waypoints')
    robot_controller.get_logger().info(f'[Cartesian Path] vel= {vel_scaling}, acc= {acc_scaling}')

    # Debug: Log first TCP waypoint
    if num_waypoints > 0:
        robot_controller.get_logger().info(
            f'[Cartesian Path] First TCP waypoint: X={waypoints_mm[0][0]:.1f}mm Y={waypoints_mm[0][1]:.1f}mm Z={waypoints_mm[0][2]:.1f}mm')
        robot_controller.get_logger().info(f'[Cartesian Path] TCP orientation: RX={rx}° RY={ry}° RZ={rz}°')
        robot_controller.get_logger().info(
            f'[Cartesian Path] T_tool offset from ee_link: {robot_controller.T_tool[2, 3] * 1000:.1f}mm in Z')

    if not robot_controller.cart_path_client.wait_for_service(timeout_sec=1.0):
        robot_controller.get_logger().error('[Cartesian Path] compute_cartesian_path service not available')
        return -2

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
            return -3  # Reject entire path if any waypoint unsafe
        if "Warning" in msg and i == 0:  # Only warn once
            robot_controller.get_logger().warning(f'[SAFETY] {msg}')

        # Debug: Log ALL waypoint positions (not just first)
        robot_controller.get_logger().info(
            f'[Cartesian Path] Waypoint {i+1}/{num_waypoints} ee_link: X={ee_position[0]*1000:.2f} Y={ee_position[1]*1000:.2f} Z={ee_position[2]*1000:.2f} mm')

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
    request.avoid_collisions = True  # Re-enabled - safety walls removed from planning scene
    # Set velocity and acceleration scaling factors for TOTG
    request.max_velocity_scaling_factor = vel_scaling
    request.max_acceleration_scaling_factor = acc_scaling

    # ✅ CRITICAL FIX: Set start state to robot's CURRENT position
    # Without this, MoveIt uses an empty/default state causing trajectory mismatch
    if hasattr(robot_controller, 'current_joint_state') and robot_controller.current_joint_state is not None:
        from sensor_msgs.msg import JointState
        from copy import deepcopy

        current_state = deepcopy(robot_controller.current_joint_state)
        current_state.header.stamp = robot_controller.get_clock().now().to_msg()

        request.start_state.joint_state = current_state
        request.start_state.is_diff = False  # Use as absolute state, not differential
        robot_controller.get_logger().info(
            f'[Cartesian Path] Using current joint state as start: {[round(p, 3) for p in current_state.position[:6]]}')
    else:
        robot_controller.get_logger().warning(
            '[Cartesian Path] No current joint state available - trajectory may not execute correctly')

    robot_controller.get_logger().info(
        f'[Cartesian Path] Using max_step={max_step * 1000:.1f}mm for {num_waypoints} waypoints')

    # Run FK diagnostic to identify any mismatch between C++ Cartesian and MoveIt FK
    if hasattr(robot_controller, 'current_joint_state') and robot_controller.current_joint_state is not None:
        _diagnose_fk_mismatch(robot_controller, waypoints[0], robot_controller.current_joint_state)

    robot_controller.get_logger().info('[Cartesian Path] Requesting cartesian path computation...')
    future = robot_controller.cart_path_client.call_async(request)
    future.add_done_callback(lambda f: _cartesian_path_response(robot_controller, f, vel_scaling, acc_scaling))
    return 0  # Request submitted successfully


def _cartesian_path_response(robot_controller, future, vel_scaling, acc_scaling):
    robot_controller.safety_manager.force_update()
    try:
        response = future.result()
        fraction = response.fraction
        robot_controller.get_logger().info(f'[Cartesian Path] Path computed: {fraction * 100:.1f}% successful')

        if fraction < 0.9:
            robot_controller.get_logger().error(
                f'[Cartesian Path] ✗ Only {fraction * 100:.1f}% of path could be computed')
            robot_controller.get_logger().error(f'[Cartesian Path] Possible reasons:')
            robot_controller.get_logger().error(f'[Cartesian Path]   1. Target unreachable from current position')
            robot_controller.get_logger().error(f'[Cartesian Path]   2. Path goes through collision/obstacles')
            robot_controller.get_logger().error(f'[Cartesian Path]   3. Joint limits would be exceeded')

            # Log current position for debugging
            if hasattr(robot_controller, 'prev_cartesian') and robot_controller.prev_cartesian is not None:
                curr = robot_controller.prev_cartesian
                robot_controller.get_logger().error(
                    f'[Cartesian Path] Current: X={curr[0]:.1f} Y={curr[1]:.1f} Z={curr[2]:.1f} RX={curr[3]:.1f} RY={curr[4]:.1f} RZ={curr[5]:.1f}')

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
                robot_controller.get_logger().error(
                    '[Cartesian Path] Cannot execute trajectory without time parameterization')
                return

            # Send trajectory directly to the controller
            _send_trajectory_to_controller(robot_controller, result_trajectory.joint_trajectory)

        apply_ipp_totg(robot_controller, trajectory, vel_scaling, acc_scaling, callback=on_totg_done)

    except Exception as e:
        robot_controller.get_logger().error(f'[Cartesian Path] Service call failed: {e}')


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

    Returns:
        int: 0 on success, -1 if robot is busy, -2 if service unavailable, -3 if safety rejected
    """
    robot_controller.safety_manager.force_update()

    if robot_controller.is_executing:
        robot_controller.get_logger().warning('[MOVE] Previous trajectory still executing, ignoring new goal')
        return -1

    # Get current cartesian position as starting point
    current_cart = robot_controller.prev_cartesian
    if current_cart is None or len(current_cart) < 3:
        robot_controller.get_logger().error('[MOVE] No current position available, cannot plan path')
        return -4

    # Create 2-waypoint path: current position → target position
    # This allows MoveIt to plan a smooth linear interpolation
    waypoints_mm = [
        [current_cart[0], current_cart[1], current_cart[2]],  # Start from current position
        [x_mm, y_mm, z_mm]  # Move to target
    ]

    robot_controller.get_logger().info(
        f'[MOVE] Planning from [{current_cart[0]:.1f}, {current_cart[1]:.1f}, {current_cart[2]:.1f}] to [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}]')

    return send_path_cartesian(
        robot_controller,
        waypoints_mm=waypoints_mm,
        rx=rx, ry=ry, rz=rz,
        vel_scaling=vel_scale,
        acc_scaling=acc_scale
    )
