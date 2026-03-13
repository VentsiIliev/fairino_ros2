"""
Trajectory Planner
==================
MoveIt /compute_cartesian_path response handler and re-exports for callers.

Data flow for a normal Cartesian move:
  caller (single_target / trajectory)
      → sends GetCartesianPath request (async)
      → _cartesian_path_response()          ← triggered by ROS2 service response
          → apply_ipp_totg / apply_ruckig   (time parameterization, async)
              → on_time_param_done()         ← triggered by /apply_ipp response
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

from .planner_utils import TIME_PARAMETERIZATION, _set_result, _is_stale
from .planner_diagnostics import _diagnose_fk_mismatch, _diagnose_start_collision
from .jacobian_move import _jacobian_fallback_move
from ..execution.trajectory_optimization import apply_ipp_totg, apply_ruckig_service
from ..execution.trajectory_executor import _send_trajectory_to_controller
import config


# ─── Main path-planning response handler ─────────────────────────────────────

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
                                            sets result=-3, returns.
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
    robot_controller.safety_manager.force_update()

    if _is_stale(robot_controller, generation):
        robot_controller.get_logger().info('[Cartesian Path] Stale response discarded (preempted)')
        return

    try:
        response = future.result()
        fraction = response.fraction
        robot_controller.get_logger().info(f'[Cartesian Path] Path computed: {fraction * 100:.1f}% successful')

        # ── Partial / zero path ──────────────────────────────────────────────
        if fraction < config.CARTESIAN_MIN_FRACTION:
            robot_controller.get_logger().error(
                f'[Cartesian Path] Only {fraction * 100:.1f}% of path could be computed')
            robot_controller.get_logger().error(f'[Cartesian Path] Possible reasons:')
            robot_controller.get_logger().error(f'[Cartesian Path]   1. Target unreachable from current position')
            robot_controller.get_logger().error(f'[Cartesian Path]   2. Path goes through collision/obstacles')
            robot_controller.get_logger().error(f'[Cartesian Path]   3. Joint limits would be exceeded')

            if hasattr(robot_controller, 'prev_cartesian') and robot_controller.prev_cartesian is not None:
                curr = robot_controller.prev_cartesian
                robot_controller.get_logger().error(
                    f'[Cartesian Path] Current: X={curr[0]:.1f} Y={curr[1]:.1f} Z={curr[2]:.1f} '
                    f'RX={curr[3]:.1f} RY={curr[4]:.1f} RZ={curr[5]:.1f}')

            # Fire async collision diagnostic — does not affect result code
            _diagnose_start_collision(robot_controller)

            _set_result(robot_controller, -3)
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

            requested_delta_mm = getattr(robot_controller, '_last_requested_delta_mm', 0.0)

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
            stored_wps = getattr(robot_controller, '_last_full_waypoints', None)
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
        # Both services are async; on_time_param_done is their shared callback.
        def on_time_param_done(result_trajectory):
            """
            Callback triggered when the /apply_ipp (TOTG) or Ruckig service responds
            with a fully time-parameterized trajectory.

            Triggered by: apply_ipp_totg() or apply_ruckig_service() internal callback
                          after the C++ ipp_helper / ruckig_helper node responds.

            On success: dispatches the trajectory to the hardware controller.
            On failure (result_trajectory is None): sets result=-7 and aborts.
            """
            if _is_stale(robot_controller, generation):
                robot_controller.get_logger().info('[Cartesian Path] Stale TOTG response discarded (preempted)')
                return

            if result_trajectory is None:
                robot_controller.get_logger().error(
                    '[Cartesian Path] Time parameterization failed - aborting execution')
                _set_result(robot_controller, -7)
                return

            # Record success before handing off to executor (which will clear
            # is_executing only when the hardware controller confirms completion)
            with robot_controller.lock:
                robot_controller.last_move_result = 0
            _send_trajectory_to_controller(robot_controller, result_trajectory.joint_trajectory)

        if TIME_PARAMETERIZATION == "RUCKIG":
            apply_ruckig_service(robot_controller, trajectory, vel_scaling, acc_scaling,
                                 callback=on_time_param_done)
        else:  # "TOTG" (default)
            apply_ipp_totg(robot_controller, trajectory, vel_scaling, acc_scaling,
                           callback=on_time_param_done)

    except Exception as e:
        robot_controller.get_logger().error(f'[Cartesian Path] Service call failed: {e}')
        _set_result(robot_controller, -2)


# ─── Re-exports so callers (single_target.py, trajectory.py) need no changes ─
# These names were previously defined directly in this file; they now live in
# their respective submodules but are re-exported here for backwards compatibility.
__all__ = [
    'TIME_PARAMETERIZATION',
    '_set_result',
    '_is_stale',
    '_diagnose_fk_mismatch',
    '_diagnose_start_collision',
    '_jacobian_fallback_move',
    '_cartesian_path_response',
]
