"""
Multi-Waypoint Cartesian Path
==============================
Public entry:  send_path_cartesian
Internal path: _execute_path

Accepts a list of TCP waypoints [x_mm, y_mm, z_mm] with a single fixed
orientation (rx, ry, rz) applied to all waypoints. Queues the motion if
the robot is already executing. Adaptive IK step size scales with waypoint count.
"""

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from utils.transformation_utils import TransformationUtils
from .trajectory_planner import _cartesian_path_response, _diagnose_fk_mismatch
import numpy as np


# ------------------------------------------------------------------------------
# Public entry point
# ------------------------------------------------------------------------------

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
            task_function=_execute_path,
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
    return _execute_path(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling)


# ------------------------------------------------------------------------------
# Internal execution
# ------------------------------------------------------------------------------

def _execute_path(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
    """Internal function to execute path (called directly or from queue)."""
    robot_controller._last_requested_delta_mm = 0.0  # reset: multi-waypoint path, not a precision move
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
    request.avoid_collisions = True  # Re-enabled - safety walls removed from a planning scene

    request.max_velocity_scaling_factor = vel_scaling
    request.max_acceleration_scaling_factor = acc_scaling

    # CRITICAL: Set start state to the robot's CURRENT position
    # Without this, MoveIt uses an empty/default state causing trajectory mismatch
    if hasattr(robot_controller, 'current_joint_state') and robot_controller.current_joint_state is not None:
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
