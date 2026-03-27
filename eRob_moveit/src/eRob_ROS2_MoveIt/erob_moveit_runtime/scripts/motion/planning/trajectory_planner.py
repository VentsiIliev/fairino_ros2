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

from .planner_utils import _set_result, _is_stale, _begin_execution
from .planner_diagnostics import _diagnose_fk_mismatch, _diagnose_start_collision
from .jacobian_move import _jacobian_fallback_move
from ..execution.trajectory_executor import _send_trajectory_to_controller
from moveit_msgs.srv import GetCartesianPath
import config


# ─── Main path-planning response handler ─────────────────────────────────────

def _apply_time_param(rc, trajectory, vel_scaling, acc_scaling, gen, log_prefix='[Plan]'):
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

    rc.trajectory_optimizer.optimize(
        rc,
        trajectory,
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


def _cartesian_path_response(robot_controller, future, vel_scaling, acc_scaling, generation=None):
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
                          generation, log_prefix='[Cartesian Path]')

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
