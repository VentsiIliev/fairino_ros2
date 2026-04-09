"""
Trajectory Planner
==================
Shared request builder and MoveIt /compute_cartesian_path response handler.

Public utilities:
  _build_cartesian_request()  — builds a GetCartesianPath.Request with all
                                 common fields; callers pass only the differences
                                 (poses, max_step, vel/acc scaling, optional
                                 start_state, optional avoid_collisions flag).

Data flow for a normal Cartesian move:
  caller (single_target / trajectory)
      → _build_cartesian_request()           ← assembles GetCartesianPath.Request
      → cart_path_client.call_async(request)
      → _cartesian_path_response()           ← triggered by ROS2 service response
          → apply_ipp_totg / apply_ruckig    (time parameterization, async)
              → on_time_param_done()          ← triggered by /apply_ipp response
                  → _send_trajectory_to_controller()

Data flow for sub-5mm Jacobian fallback:
  _cartesian_path_response() detects ≤1 trajectory point
      → _jacobian_fallback_move()
          → _jacobian_check_and_execute()
              → /check_state_validity (mid)  ← async, _cb_mid triggered on response
              → /check_state_validity (end)  ← async, _cb_end triggered on response
                  → _on_both_done()          ← called when BOTH results arrive
                      → _send_trajectory_to_controller()
"""
from copy import deepcopy
import math

from .planner_utils import _set_result, _is_stale, _begin_execution
from .planner_diagnostics import _diagnose_fk_mismatch, _diagnose_start_collision
from .jacobian_move import _jacobian_fallback_move
from ..execution.trajectory_executor import _send_trajectory_to_controller
from ..execution.trajectory_optimizer import resolve_trajectory_optimizer
from moveit_msgs.srv import GetCartesianPath
import config


# ─── Main path-planning response handler ─────────────────────────────────────

def _nearest_equivalent_angle(reference: float, value: float) -> float:
    """Shift `value` by ±2π so it stays closest to `reference`."""
    adjusted = float(value)
    ref = float(reference)
    two_pi = 2.0 * math.pi
    while adjusted - ref > math.pi:
        adjusted -= two_pi
    while adjusted - ref < -math.pi:
        adjusted += two_pi
    return adjusted


def _wrap_angle_into_limits(reference: float, value: float, lower: float, upper: float) -> float:
    """Shift `value` by ±2π to stay inside [lower, upper] while staying near `reference`."""
    two_pi = 2.0 * math.pi
    candidates = []
    for shift in range(-3, 4):
        candidate = float(value) + shift * two_pi
        if lower - 1e-9 <= candidate <= upper + 1e-9:
            candidates.append(candidate)
    if not candidates:
        return float(value)
    return min(candidates, key=lambda candidate: abs(candidate - reference))


def _unwrap_joint_trajectory_positions(trajectory, reference_positions=None) -> tuple[object, float]:
    """Keep revolute joint positions on a continuous branch across the path.

    MoveIt can return equivalent joint states that differ by ±2π for the same
    Cartesian pose. That is valid kinematically but disastrous for downstream
    time parameterization and controller start alignment. Normalize each point
    to stay closest to the previous point, optionally seeding from the live
    joint state.
    """
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None or not joint_trajectory.points:
        return trajectory, 0.0

    previous = list(reference_positions) if reference_positions is not None else None
    max_adjustment = 0.0

    for point in joint_trajectory.points:
        positions = list(point.positions)
        if previous is None:
            previous = list(positions)
            continue

        unwrapped = []
        for ref, value in zip(previous, positions):
            adjusted = _nearest_equivalent_angle(ref, value)
            max_adjustment = max(max_adjustment, abs(adjusted - value))
            unwrapped.append(adjusted)
        point.positions = unwrapped
        previous = list(unwrapped)

    return trajectory, max_adjustment


def _project_joint6_to_reference_branch(trajectory, reference_positions=None) -> tuple[object, float]:
    """Project Joint_6 onto the nearest equivalent branch of the reference state.

    This intentionally does not preserve accumulated multi-turn wrapping for the
    wrist. For execution we want the equivalent branch nearest the live start
    state, otherwise MoveIt can hand back a valid Cartesian path whose Joint_6
    endpoint differs by one or more full turns and the controller will chase
    that numeric target instead of the nearby equivalent.
    """
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None or not joint_trajectory.points:
        return trajectory, 0.0
    if reference_positions is None:
        return trajectory, 0.0

    reference = list(reference_positions)
    if len(reference) != len(joint_trajectory.joint_names):
        return trajectory, 0.0

    max_adjustment = 0.0
    for point in joint_trajectory.points:
        positions = list(point.positions)
        adjusted_any = False
        for joint_index, joint_name in enumerate(joint_trajectory.joint_names):
            name = str(joint_name or '').strip().lower()
            if name not in {'joint_6', 'j6', 'axis_6'} and not name.endswith('_6'):
                continue
            original = float(positions[joint_index])
            adjusted = _nearest_equivalent_angle(reference[joint_index], original)
            if abs(adjusted - original) > 1e-9:
                positions[joint_index] = adjusted
                max_adjustment = max(max_adjustment, abs(adjusted - original))
                adjusted_any = True
        if adjusted_any:
            point.positions = positions

    return trajectory, max_adjustment


def _limit_safe_joint_wrapping(trajectory, reference_positions=None) -> tuple[object, float]:
    """Rebase wrapped joints only when the current branch violates hard limits.

    Keep the planner's chosen branch whenever it is already within the hardware
    window. Only shift by ±2π when a point falls outside the controller-safe
    limit range. This avoids "helpful" untangling that reverses the intended
    wrist rotation on otherwise valid paths.
    """
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None or not joint_trajectory.points:
        return trajectory, 0.0

    previous = list(reference_positions) if reference_positions is not None else None
    if previous is None and joint_trajectory.points:
        previous = list(joint_trajectory.points[0].positions)

    max_adjustment = 0.0
    lower_limit = -12.5664
    upper_limit = 12.5664

    for point_index, point in enumerate(joint_trajectory.points):
        positions = list(point.positions)
        if previous is None:
            previous = list(positions)
            continue

        adjusted_positions = list(positions)
        for joint_index, joint_name in enumerate(joint_trajectory.joint_names):
            name = str(joint_name or "").strip().lower()
            if name not in {"joint_6", "j6", "axis_6"} and not name.endswith("_6"):
                continue
            current_value = positions[joint_index]
            if lower_limit - 1e-9 <= current_value <= upper_limit + 1e-9:
                continue
            reference = previous[joint_index]
            adjusted = _wrap_angle_into_limits(reference, current_value, lower_limit, upper_limit)
            max_adjustment = max(max_adjustment, abs(adjusted - current_value))
            adjusted_positions[joint_index] = adjusted
        point.positions = adjusted_positions
        previous = list(adjusted_positions)

    return trajectory, max_adjustment

def _sanitize_optimizer_start(rc, trajectory, log_prefix):
    """Align the optimizer input trajectory to the latest live joint state."""
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None or not joint_trajectory.points:
        return trajectory

    current_joint_state = getattr(rc, 'current_joint_state', None)
    if current_joint_state is None:
        return trajectory

    state_names = list(getattr(current_joint_state, 'name', []) or [])
    state_positions = list(getattr(current_joint_state, 'position', []) or [])
    if not state_names or len(state_names) != len(state_positions):
        return trajectory

    position_by_name = {
        name: position
        for name, position in zip(state_names, state_positions)
    }

    ordered_positions = []
    for joint_name in joint_trajectory.joint_names:
        if joint_name not in position_by_name:
            return trajectory
        ordered_positions.append(position_by_name[joint_name])

    align_tol = float(getattr(config, 'OPTIMIZER_START_ALIGN_TOL_RAD', 0.0))
    merge_tol = float(getattr(config, 'OPTIMIZER_START_MERGE_TOL_RAD', align_tol))

    sanitized = deepcopy(trajectory)
    sanitized, max_wrap_adjustment = _unwrap_joint_trajectory_positions(
        sanitized,
        reference_positions=ordered_positions,
    )
    if max_wrap_adjustment > 1e-6:
        rc.get_logger().info(
            f'{log_prefix} Unwrapped joint trajectory continuity '
            f'(max wrap adjustment {max_wrap_adjustment:.4f} rad)'
        )
    sanitized, max_branch_projection = _project_joint6_to_reference_branch(
        sanitized,
        reference_positions=ordered_positions,
    )
    if max_branch_projection > 1e-6:
        rc.get_logger().info(
            f'{log_prefix} Projected Joint_6 to live branch before optimization '
            f'(max wrap adjustment {max_branch_projection:.4f} rad)'
        )
    points = sanitized.joint_trajectory.points
    first_point = points[0]

    deltas = [abs(a - b) for a, b in zip(first_point.positions, ordered_positions)]
    max_delta = max(deltas, default=0.0)

    if max_delta > 0.0:
        first_point.positions = ordered_positions
        if hasattr(first_point, 'velocities') and first_point.velocities:
            first_point.velocities = [0.0] * len(ordered_positions)
        if hasattr(first_point, 'accelerations') and first_point.accelerations:
            first_point.accelerations = [0.0] * len(ordered_positions)
        if hasattr(first_point, 'effort') and first_point.effort:
            first_point.effort = []

        if max_delta >= align_tol > 0.0:
            rc.get_logger().info(
                f'{log_prefix} Aligned optimizer start to live joint state '
                f'(max joint delta {max_delta:.4f} rad)'
            )

    if len(points) >= 2 and merge_tol > 0.0:
        second_point = points[1]
        second_delta = max(
            (abs(a - b) for a, b in zip(second_point.positions, first_point.positions)),
            default=0.0,
        )
        if second_delta <= merge_tol:
            points.pop(1)
            rc.get_logger().info(
                f'{log_prefix} Dropped near-duplicate first segment before optimization '
                f'(max joint delta {second_delta:.4f} rad)'
            )

    return sanitized

def _apply_time_param(rc, trajectory, vel_scaling, acc_scaling, gen, log_prefix='[Plan]', trajectory_optimizer_name=None):
    """
    Apply TOTG or Ruckig time parameterization and dispatch to the hardware controller.

    Eliminates the duplicated on_time_param_done + TOTG/Ruckig dispatch pattern
    that previously appeared in both _cartesian_path_response and
    _execute_pending_trajectory.

    Args:
        rc:          RobotController node
        trajectory:  MoveIt RobotTrajectory (untimed joint-space waypoints)
        vel_scaling: velocity scaling factor (0–1)
        acc_scaling: acceleration scaling factor (0–1)
        gen:         plan_generation token for staleness detection
        log_prefix:  log tag, e.g. '[Cartesian Path]' or '[EXECUTE_PATH]'
    """
    prepared_trajectory = _sanitize_optimizer_start(rc, trajectory, log_prefix)

    def on_done(result):
        if _is_stale(rc, gen):
            rc.get_logger().info(f'{log_prefix} Stale TOTG response discarded')
            return
        if result is None:
            rc.get_logger().error(f'{log_prefix} Time parameterization failed')
            _set_result(rc, -7)
            return
        with rc.lock:
            rc.last_move_result = 0
        _send_trajectory_to_controller(rc, result.joint_trajectory)

    optimizer = resolve_trajectory_optimizer(trajectory_optimizer_name, node=rc, default_optimizer=rc.trajectory_optimizer)
    optimizer.optimize(
        rc,
        prepared_trajectory,
        vel_scaling,
        acc_scaling,
        on_done,
    )


def _build_cartesian_request(rc, poses, max_step, vel_scaling, acc_scaling,
                              start_state=None, avoid_collisions=True):
    req = GetCartesianPath.Request()
    req.header.frame_id               = config.BASE_LINK
    req.group_name                    = config.PLANNING_GROUP
    req.link_name                     = config.EE_LINK
    req.waypoints                     = poses
    req.max_step                      = max_step
    req.jump_threshold                = 0.0
    req.avoid_collisions              = avoid_collisions
    req.max_velocity_scaling_factor   = vel_scaling
    req.max_acceleration_scaling_factor = acc_scaling

    if start_state is not None:
        req.start_state = start_state
    elif rc.current_joint_state is not None:
        state = deepcopy(rc.current_joint_state)
        state.header.stamp = rc.get_clock().now().to_msg()
        req.start_state.joint_state = state
        req.start_state.is_diff = False
    else:
        rc.get_logger().warning('[Plan] No current joint state — trajectory may mismatch')
    return req


def _cartesian_path_response(robot_controller, future, vel_scaling, acc_scaling, generation=None, trajectory_optimizer_name=None):
    """
    Callback triggered when MoveIt's /compute_cartesian_path service responds.

    This is the central decision point after path planning completes. It handles
    all outcomes: success, partial failure, zero-fraction failure, and the
    special ≤1-point case where MoveIt collapsed all waypoints to a single
    joint config (robot already at target, or sub-mm move needing Jacobian).

    Triggered by: future.add_done_callback() set in single_target.py or
                  trajectory.py immediately after call_async(GetCartesianPath).

    Success path:
        fraction ≥ CARTESIAN_MIN_FRACTION → applies TOTG or Ruckig time
        parameterization (async), then on_time_param_done() dispatches the
        trajectory to the hardware controller.

    Failure paths:
        fraction < CARTESIAN_MIN_FRACTION → logs error, fires collision diagnostic,
                                            sets result=-11, returns.
        ≤1 trajectory point + fraction≈0  → planning failed entirely, result=-6.
        ≤1 trajectory point + fraction≈1
          + large delta                   → Jacobian fallback (_jacobian_fallback_move).
          + tiny delta                    → robot already at target, result=0.

    Args:
        robot_controller: RobotController node
        future:           rclpy Future wrapping GetCartesianPath.Response
        vel_scaling:      velocity scaling factor (0–1) forwarded to time parameterization
        acc_scaling:      acceleration scaling factor (0–1) forwarded to time parameterization
        generation:       plan_generation value captured at submission; used for staleness check
    """
    # Re-publish safety walls + ACM so MoveIt's scene is current for any
    # subsequent validity checks (e.g. _jacobian_check_and_execute)
    robot_controller.force_safety_update()

    if _is_stale(robot_controller, generation):
        robot_controller.get_logger().info('[Cartesian Path] Stale response discarded (preempted)')
        return

    try:
        response = future.result()
        fraction = response.fraction
        robot_controller.get_logger().info(f'[Cartesian Path] Path computed: {fraction * 100:.1f}% successful')

        # ── Partial / zero path ──────────────────────────────────────────────
        if fraction < config.CARTESIAN_MIN_FRACTION:
            requested_delta_mm = robot_controller.get_last_requested_delta_mm()
            stored_wps = robot_controller.get_last_full_waypoints() or []
            planned_trajectory = getattr(response, 'solution', None)
            planned_points = len(getattr(getattr(planned_trajectory, 'joint_trajectory', None), 'points', []) or [])

            if 0.0 <= requested_delta_mm <= config.JACOBIAN_FALLBACK_MIN_DELTA_MM and fraction > 0.0:
                robot_controller.get_logger().info(
                    '[Cartesian Path] Partial fraction on sub-threshold micro-move '
                    f'(fraction={fraction * 100:.1f}%, delta={requested_delta_mm:.3f}mm, '
                    f'points={planned_points}) — treating as already satisfied'
                )
                _set_result(robot_controller, 0)
                return

            if (
                0.0 < requested_delta_mm <= config.SHORT_CARTESIAN_JACOBIAN_FALLBACK_MAX_DELTA_MM
                and 2 <= len(stored_wps) <= 5
                and fraction > 0.0
            ):
                robot_controller.get_logger().warning(
                    '[Cartesian Path] Partial fraction on short single-target move '
                    f'(fraction={fraction * 100:.1f}%, delta={requested_delta_mm:.3f}mm) — '
                    'trying Jacobian fallback'
                )
                ok = _jacobian_fallback_move(
                    robot_controller,
                    stored_wps,
                    vel_scaling,
                    acc_scaling,
                    generation,
                )
                if ok:
                    return

            robot_controller.get_logger().error(
                f'[Cartesian Path] Only {fraction * 100:.1f}% of path could be computed')
            robot_controller.get_logger().error(f'[Cartesian Path] Possible reasons:')
            robot_controller.get_logger().error(f'[Cartesian Path]   1. Target unreachable from current position')
            robot_controller.get_logger().error(f'[Cartesian Path]   2. Path goes through collision/obstacles')
            robot_controller.get_logger().error(f'[Cartesian Path]   3. Joint limits would be exceeded')

            if robot_controller.prev_cartesian is not None:
                curr = robot_controller.prev_cartesian
                robot_controller.get_logger().error(
                    f'[Cartesian Path] Current: X={curr[0]:.1f} Y={curr[1]:.1f} Z={curr[2]:.1f} '
                    f'RX={curr[3]:.1f} RY={curr[4]:.1f} RZ={curr[5]:.1f}')

            # Fire async collision diagnostic — does not affect result code
            _diagnose_start_collision(robot_controller)

            _set_result(robot_controller, -11)
            return

        # ── Trajectory retrieved ─────────────────────────────────────────────
        trajectory = response.solution
        num_pts = len(trajectory.joint_trajectory.points)
        robot_controller.get_logger().info(f'[Cartesian Path] Computed trajectory has {num_pts} points')

        # ── ≤1 point: MoveIt collapsed all waypoints to a single config ──────
        # This happens when:
        #   a) The robot is already at the target within IK precision (delta < threshold)
        #   b) A sub-mm move was requested — MoveIt snaps it to the nearest IK solution
        if num_pts <= 1:
            if response.fraction < config.JACOBIAN_FALLBACK_MIN_FRACTION:
                # Fraction too low even with ≤1 point — genuine planning failure
                robot_controller.get_logger().warning(
                    f'[Cartesian Path] ≤1 point, fraction={response.fraction * 100:.0f}% — planning failed')
                _set_result(robot_controller, -6)
                return

            requested_delta_mm = robot_controller.get_last_requested_delta_mm()

            if requested_delta_mm <= config.JACOBIAN_FALLBACK_MIN_DELTA_MM:
                # Robot is already at the target — no motion needed
                robot_controller.get_logger().info(
                    '[Cartesian Path] ≤1 point (100%) — robot already at target within IK precision')
                _set_result(robot_controller, 0)
                return

            # Large delta but ≤1 point: Jacobian pseudoinverse fallback
            # (MoveIt couldn't produce a multi-point trajectory for a short move)
            robot_controller.get_logger().warning(
                f'[Cartesian Path] ≤1 point but delta={requested_delta_mm:.3f}mm — trying Jacobian fallback')
            stored_wps = robot_controller.get_last_full_waypoints()
            if stored_wps:
                ok = _jacobian_fallback_move(
                    robot_controller, stored_wps, vel_scaling, acc_scaling, generation)
                if ok:
                    # Jacobian path owns is_executing and last_move_result from here
                    return

            robot_controller.get_logger().error(
                '[Cartesian Path] Jacobian fallback unavailable — returning -8')
            _set_result(robot_controller, -8)
            return

        # ── Normal multi-point trajectory: apply time parameterization ────────
        # TOTG or Ruckig adds velocity/acceleration/jerk profiles to the raw
        # joint-space waypoints returned by MoveIt (which have no timing).
        _apply_time_param(robot_controller, trajectory, vel_scaling, acc_scaling,
                          generation, log_prefix='[Cartesian Path]',
                          trajectory_optimizer_name=trajectory_optimizer_name)

    except Exception as e:
        robot_controller.get_logger().error(f'[Cartesian Path] Service call failed: {e}')
        _set_result(robot_controller, -2)


# ─── Re-exports so callers (single_target.py, trajectory.py) need no changes ─
# These names were previously defined directly in this file; they now live in
# their respective submodules but are re-exported here for backwards compatibility.
__all__ = [
    '_apply_time_param',
    '_build_cartesian_request',
    '_begin_execution',
    '_set_result',
    '_is_stale',
    '_diagnose_fk_mismatch',
    '_diagnose_start_collision',
    '_jacobian_fallback_move',
    '_cartesian_path_response',
]
