"""
Single-Point Cartesian Move
===========================
Public entry:  send_cartesian_goal
Internal path: _execute_single_point
Helper:        _generate_adaptive_waypoints

Generates enough intermediate waypoints between the current pose and the target
so that TOTG always receives ≥3 trajectory points, even for sub-mm moves.
Safety check is performed only on the target pose (start = current robot pos,
intermediates are linearly interpolated between two safe endpoints).
"""

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from utils.transformation_utils import TransformationUtils
from .trajectory_planner import _cartesian_path_response, _jacobian_fallback_move
import numpy as np


# ------------------------------------------------------------------------------
# Helper
# ------------------------------------------------------------------------------

def _generate_adaptive_waypoints(start_mm, target_mm):
    """
    Generate a small number of intermediate key-frame waypoints between start and target
    for MoveIt's compute_cartesian_path request. MoveIt interpolates path density
    internally via max_step; these waypoints are just structural anchors.
    Sub-1mm moves bypass this function entirely (handled by Jacobian).
    start_mm / target_mm: [x, y, z, rx, ry, rz] with xyz in mm.
    Returns list of intermediate + final waypoints (same format), NOT including start.
    """
    dx = target_mm[0] - start_mm[0]
    dy = target_mm[1] - start_mm[1]
    dz = target_mm[2] - start_mm[2]
    distance_mm = (dx * dx + dy * dy + dz * dz) ** 0.5

    # MoveIt's max_step parameter handles path density internally; request waypoints
    # are just key frames. Keep count low: 1 intermediate per 2mm, hard-capped at 4.
    # Sub-1mm moves go through Jacobian directly so dense waypoints are never needed.
    #
    # distance  segments  intermediates  total waypoints sent to MoveIt
    # --------  --------  -------------  ------------------------------
    #  < 1mm    Jacobian path — this function's output is never used
    #   1mm         2           1          3  (start + 1 mid + end)
    #   2mm         2           1          3
    #   5mm         3           2          4
    #  10mm         4           3          5
    #  50mm         4           3          5  (hard cap)
    segments = max(2, min(4, int(distance_mm / 2.0) + 2))

    waypoints = []
    for i in range(1, segments):
        t = i / segments
        waypoints.append([
            start_mm[0] + t * dx,
            start_mm[1] + t * dy,
            start_mm[2] + t * dz,
            target_mm[3],
            target_mm[4],
            target_mm[5],
        ])
    waypoints.append(list(target_mm))
    return waypoints


# ------------------------------------------------------------------------------
# Internal execution
# ------------------------------------------------------------------------------

def _execute_single_point(robot_controller, start_wp, target_wp, vel_scaling, acc_scaling):
    """
    Execute a single-point Cartesian move (move_cartesian / move_liner).

    Generates adaptive intermediate waypoints between start_wp and target_wp so
    that TOTG always receives enough trajectory points, even for sub-mm moves.
    Safety check is performed only on the target (start = current robot position).

    start_wp / target_wp: [x_mm, y_mm, z_mm, rx, ry, rz]
    """
    adaptive = _generate_adaptive_waypoints(start_wp, target_wp)
    waypoints_full = [start_wp] + adaptive
    num_waypoints = len(waypoints_full)

    dx = target_wp[0] - start_wp[0]
    dy = target_wp[1] - start_wp[1]
    dz = target_wp[2] - start_wp[2]
    distance_mm = (dx * dx + dy * dy + dz * dz) ** 0.5
    robot_controller.get_logger().info(
        f'[Single Point] {num_waypoints} waypoints (Δ={distance_mm:.3f}mm, '
        f'{len(adaptive)} intermediates)')

    if not robot_controller.cart_path_client.wait_for_service(timeout_sec=1.0):
        robot_controller.get_logger().error('[Single Point] compute_cartesian_path not available')
        return -2

    waypoints = []
    for i, (x_mm, y_mm, z_mm, rx, ry, rz) in enumerate(waypoints_full):
        T_tcp = TransformationUtils.pose_to_transform([x_mm, y_mm, z_mm, rx, ry, rz])
        T_ee = TransformationUtils.remove_tcp_offset(T_tcp, robot_controller.T_tool)
        ee_pos = T_ee[:3, 3]
        ee_quat = TransformationUtils.matrix_to_quaternion(T_ee[:3, :3])

        # Safety check only on the target (waypoint 0 = current position, intermediates
        # are linearly interpolated between start and a safe target)
        if i == num_waypoints - 1:
            is_safe, msg = robot_controller.safety_manager.check_position_safety(
                ee_pos[0], ee_pos[1], ee_pos[2])
            if not is_safe:
                robot_controller.get_logger().error(f'[SAFETY] Single-point target rejected: {msg}')
                return -3
            if 'Warning' in msg:
                robot_controller.get_logger().warning(f'[SAFETY] {msg}')

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = ee_pos[0], ee_pos[1], ee_pos[2]
        pose.orientation.x, pose.orientation.y = ee_quat[0], ee_quat[1]
        pose.orientation.z, pose.orientation.w = ee_quat[2], ee_quat[3]
        waypoints.append(pose)

    p0, p1 = waypoints[0].position, waypoints[-1].position
    delta_m = np.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2)
    robot_controller._last_requested_delta_mm = delta_m * 1000.0
    robot_controller._last_full_waypoints = waypoints

    # Sub-1mm: IK solver cannot distinguish poses at the step sizes needed (all solutions
    # collapse to one joint config → 1-point trajectory). Use Jacobian directly instead.
    if delta_m < 0.001:
        robot_controller.get_logger().info(
            f'[Single Point] Sub-1mm ({delta_m * 1000:.3f}mm): Jacobian direct (skip IK)')
        with robot_controller.lock:
            robot_controller.is_executing = True
            robot_controller.plan_generation += 1
            gen = robot_controller.plan_generation
        ok = _jacobian_fallback_move(robot_controller, waypoints, vel_scaling, acc_scaling, gen)
        if not ok:
            with robot_controller.lock:
                robot_controller.is_executing = False
                robot_controller.last_move_result = -8
            return -8
        return 0

    if delta_m < 0.002:
        max_step = 0.0005  # sub-2mm
    elif delta_m < 0.01:
        max_step = 0.0008  # sub-10mm
    else:
        max_step = 0.005  # >10mm moves, 5mm per step → fewer IK calls

    request = GetCartesianPath.Request()
    request.header.frame_id = 'base_link'
    request.group_name = 'fairino5_v6_group'
    request.link_name = 'ee_link'
    request.waypoints = waypoints
    request.max_step = max_step
    request.jump_threshold = 0.0
    request.avoid_collisions = False
    request.max_velocity_scaling_factor = vel_scaling
    request.max_acceleration_scaling_factor = acc_scaling

    if hasattr(robot_controller, 'current_joint_state') and robot_controller.current_joint_state is not None:
        from copy import deepcopy
        current_state = deepcopy(robot_controller.current_joint_state)
        current_state.header.stamp = robot_controller.get_clock().now().to_msg()
        request.start_state.joint_state = current_state
        request.start_state.is_diff = False
        robot_controller.get_logger().info(
            f'[Single Point] start joints: {[round(p, 3) for p in current_state.position[:6]]}')
    else:
        robot_controller.get_logger().warning('[Single Point] No current joint state — trajectory may mismatch')

    # REPLACE WITH (single call, lock BEFORE send):
    with robot_controller.lock:
        robot_controller.is_executing = True
        robot_controller.plan_generation += 1
        gen = robot_controller.plan_generation

    future = robot_controller.cart_path_client.call_async(request)
    future.add_done_callback(lambda f: _cartesian_path_response(robot_controller, f, vel_scaling, acc_scaling, gen))
    return 0


# ------------------------------------------------------------------------------
# Public entry point
# ------------------------------------------------------------------------------

def send_cartesian_goal(robot_controller, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, planner_id='LIN',
                        tool_transform=None, allow_preempt=False):
    """
    Send a single-point Cartesian goal. If the robot is already executing,
    the new goal is silently ignored — no preemption, no queuing.
    """
    robot_controller.safety_manager.force_update()

    if robot_controller.is_executing:
        robot_controller.get_logger().warning('[MOVE] Already executing — ignoring new goal')
        return -1

    current_cart = robot_controller.prev_cartesian
    if current_cart is None or len(current_cart) < 6:
        robot_controller.get_logger().error('[MOVE] No current position available, cannot plan path')
        return -4

    start_wp  = [current_cart[0], current_cart[1], current_cart[2],
                 current_cart[3], current_cart[4], current_cart[5]]
    target_wp = [x_mm, y_mm, z_mm, rx, ry, rz]

    robot_controller.get_logger().info(
        f'[MOVE] [{start_wp[0]:.1f}, {start_wp[1]:.1f}, {start_wp[2]:.1f}, RZ={start_wp[5]:.1f}°]'
        f' → [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}, RZ={rz:.1f}°]')

    return _execute_single_point(robot_controller, start_wp, target_wp, vel_scale, acc_scale)

