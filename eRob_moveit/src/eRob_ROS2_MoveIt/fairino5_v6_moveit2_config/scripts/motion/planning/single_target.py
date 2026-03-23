"""
Single-Point Cartesian Move
===========================
Public entry:  send_cartesian_goal
Internal path: _execute_single_point → _to_pose_list → _dispatch_moveit
Request build: _build_cartesian_request  (shared, lives in trajectory_planner)

Generates adaptive intermediate waypoints between current pose and target so
that TOTG always receives ≥3 trajectory points. Safety check is performed only
on the final target (start = current robot pos; intermediates are linearly
interpolated between two already-validated endpoints).
"""

from .trajectory_planner import (
    _cartesian_path_response,
    _jacobian_fallback_move,
    _build_cartesian_request,
    _begin_execution,
)
from .planner_utils import _set_result, _require_cart_path_service, _to_pose_list
import numpy as np
import config


_ORIENTATION_DIRECT_JACOBIAN_TOL_DEG = 0.5
_ORIENTATION_ONLY_MOVE_STEP_M = 0.01
_SHORT_MOVE_ORIENTATION_INTERP_MM = 80.0


def _wrapped_angle_delta_deg(start_deg, target_deg):
    return abs(((target_deg - start_deg + 180.0) % 360.0) - 180.0)


def _compute_max_step(delta_m, orientation_delta_deg=0.0):
    """
    Compute adaptive max_step for MoveIt's compute_cartesian_path.

    max_step controls internal interpolation resolution:
      • smaller step → denser sampling, more IK calls & collision checks
      • larger step → faster planning but risk undersampling

    Goal:
      • guarantee ≥3 trajectory points for TOTG
      • keep MoveIt planning time predictable
      • avoid excessive points for short linear moves
      • avoid pathological planning time for orientation-dominant moves

    Constraints:
      min_step        = 0.5 mm   (0.0005 m)
      max_step_limit  = 25 mm    (0.025 m)
      min_segments    = 2        (ensures ≥3 points)
      target_segments = 6–8      (for typical single-point moves)

    Returns
    -------
    float
        max_step in meters for compute_cartesian_path
    """
    min_step = 0.0005       # 0.5 mm
    max_step_limit = 0.025  # 25 mm
    min_segments = 2        # ensures ≥3 points
    target_segments = 6     # reasonable for short linear moves

    # Pure / near-pure orientation changes should not be planned with a tiny
    # Cartesian sampling step derived from near-zero XYZ translation. That
    # creates hundreds of unnecessary internal points and multi-second planning
    # delays for simple wrist rotations.
    if delta_m < 0.005 and orientation_delta_deg > _ORIENTATION_DIRECT_JACOBIAN_TOL_DEG:
        return _ORIENTATION_ONLY_MOVE_STEP_M

    # estimate segments to keep ~6 points max for single-point moves
    segments = max(min_segments, min(target_segments, int(np.ceil(delta_m / 0.01))))

    # compute step size
    step = delta_m / segments

    # clamp to allowed bounds
    return float(np.clip(step, min_step, max_step_limit))


def _generate_adaptive_waypoints(start_mm, target_mm):
    """
    Generate a minimal set of structural waypoints for MoveIt.

    Notes:
      • Only generates intermediates to guarantee ≥3 trajectory points.
      • MoveIt internal max_step handles further interpolation.
      • Sub-1mm moves are handled by Jacobian fallback (no waypoints needed).

    distance    intermediates  total waypoints sent to MoveIt
    --------    -------------  ------------------------------
      < 1 mm        0                  2  (start + target)
      1–10 mm      1                  3
     10–50 mm      2                  4
    > 50 mm        3                  5  (hard cap)

    start_mm / target_mm: [x, y, z, rx, ry, rz] in mm
    Returns list of intermediate + final waypoints (not including start)
    """
    dx = target_mm[0] - start_mm[0]
    dy = target_mm[1] - start_mm[1]
    dz = target_mm[2] - start_mm[2]
    distance_mm = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    # Determine number of intermediates
    if distance_mm < 1.0:
        segments = 1  # 2 points total
    elif distance_mm < 10.0:
        segments = 2  # 3 points total
    elif distance_mm < 50.0:
        segments = 3  # 4 points total
    else:
        segments = 4  # 5 points total (hard cap)

    interpolate_orientation = distance_mm <= _SHORT_MOVE_ORIENTATION_INTERP_MM
    drx = target_mm[3] - start_mm[3]
    dry = target_mm[4] - start_mm[4]
    drz = ((target_mm[5] - start_mm[5] + 180.0) % 360.0) - 180.0

    waypoints = []
    for i in range(1, segments):
        t = i / segments
        if interpolate_orientation:
            rx = start_mm[3] + t * drx
            ry = start_mm[4] + t * dry
            rz = start_mm[5] + t * drz
        else:
            rx, ry, rz = target_mm[3], target_mm[4], target_mm[5]
        waypoints.append([
            start_mm[0] + t * dx,
            start_mm[1] + t * dy,
            start_mm[2] + t * dz,
            rx,
            ry,
            rz,
        ])
    waypoints.append(list(target_mm))
    return waypoints



def _execute_jacobian_move(robot_controller, poses, delta_m, vel_scaling, acc_scaling):
    # Sub-5mm: compute_cartesian_path costs ~250ms fixed overhead regardless of step size.
    # Jacobian is accurate to <0.01mm for 5mm steps (~2ms + ~5ms collision check).
    # Each call reads fresh joint state so linearisation errors do not accumulate.
    robot_controller.get_logger().info(
        f'[Single Point] Sub-5mm ({delta_m * 1000:.3f}mm): Jacobian direct (skip MoveIt)')
    gen = _begin_execution(robot_controller)
    ok = _jacobian_fallback_move(robot_controller, poses, vel_scaling, acc_scaling, gen)
    if not ok:
        _set_result(robot_controller, -8)
        return -8
    return 0


def _dispatch_moveit(robot_controller, request, vel_scaling, acc_scaling):
    gen = _begin_execution(robot_controller)
    future = robot_controller.request_cartesian_path(request)
    future.add_done_callback(
        lambda f: _cartesian_path_response(robot_controller, f, vel_scaling, acc_scaling, gen))


def _execute_single_point(robot_controller, start_wp, target_wp, vel_scaling, acc_scaling,
                          tool_transform=None):
    """
    Execute a single-point Cartesian move (move_cartesian / move_liner).

    Generates adaptive intermediate waypoints between start_wp and target_wp so
    that TOTG always receives enough trajectory points, even for sub-mm moves.
    Safety check is performed only on the target (start = current robot position).

    start_wp / target_wp: [x_mm, y_mm, z_mm, rx, ry, rz]
    """
    adaptive     = _generate_adaptive_waypoints(start_wp, target_wp)
    waypoints_mm = [start_wp] + adaptive

    dx  = target_wp[0] - start_wp[0]
    dy  = target_wp[1] - start_wp[1]
    dz  = target_wp[2] - start_wp[2]
    distance_mm = (dx * dx + dy * dy + dz * dz) ** 0.5
    robot_controller.get_logger().info(
        f'[Single Point] {len(waypoints_mm)} waypoints (Δ={distance_mm:.3f}mm, '
        f'{len(adaptive)} intermediates)')

    if not _require_cart_path_service(robot_controller, 'Single Point'):
        return -2

    T_tool = tool_transform if tool_transform is not None else robot_controller.T_tool
    poses, err = _to_pose_list(robot_controller, waypoints_mm, T_tool)
    if err:
        return err

    p0, p1  = poses[0].position, poses[-1].position
    delta_m = np.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2)
    orientation_delta_deg = max(
        _wrapped_angle_delta_deg(start_wp[3], target_wp[3]),
        _wrapped_angle_delta_deg(start_wp[4], target_wp[4]),
        _wrapped_angle_delta_deg(start_wp[5], target_wp[5]),
    )
    robot_controller.set_last_requested_delta_mm(delta_m * 1000.0)
    robot_controller.set_last_full_waypoints(poses)

    if delta_m < 0.005 and orientation_delta_deg <= _ORIENTATION_DIRECT_JACOBIAN_TOL_DEG:
        return _execute_jacobian_move(robot_controller, poses, delta_m, vel_scaling, acc_scaling)

    if delta_m < 0.005:
        robot_controller.get_logger().info(
            f'[Single Point] Sub-5mm translation but orientation delta={orientation_delta_deg:.3f}° '
            '— bypassing Jacobian direct and forcing MoveIt path')

    max_step = _compute_max_step(delta_m, orientation_delta_deg)
    if delta_m < 0.005 and orientation_delta_deg > _ORIENTATION_DIRECT_JACOBIAN_TOL_DEG:
        robot_controller.get_logger().info(
            f'[Single Point] Orientation-dominant move: using coarse MoveIt eef_step={max_step:.4f}m '
            f'for orientation delta={orientation_delta_deg:.3f}°')

    _dispatch_moveit(
        robot_controller,
        _build_cartesian_request(robot_controller, poses, max_step,
                                  vel_scaling, acc_scaling),
        vel_scaling, acc_scaling,
    )
    return 0


def send_cartesian_goal(robot_controller, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                        tool_transform=None):
    """
    Send a single-point Cartesian goal. If the robot is already executing,
    the new goal is silently ignored — no preemption, no queuing.
    """
    robot_controller.force_safety_update()

    current_cart = robot_controller.prev_cartesian
    if current_cart is None or len(current_cart) < 6:
        robot_controller.get_logger().error('[MOVE] No current position available, cannot plan path')
        return -4

    start_wp  = list(current_cart[:6])
    target_wp = [x_mm, y_mm, z_mm, rx, ry, rz]

    robot_controller.get_logger().info(
        f'[MOVE] [{start_wp[0]:.1f}, {start_wp[1]:.1f}, {start_wp[2]:.1f}, RZ={start_wp[5]:.1f}°]'
        f' → [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}, RZ={rz:.1f}°]')

    return _execute_single_point(robot_controller, start_wp, target_wp, vel_scale, acc_scale,
                                 tool_transform=tool_transform)
