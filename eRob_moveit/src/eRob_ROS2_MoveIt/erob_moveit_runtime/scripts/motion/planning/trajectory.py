"""
Multi-Waypoint Cartesian Path
...
"""

from moveit_msgs.msg import RobotState
from .trajectory_planner import _cartesian_path_response, _build_cartesian_request, _apply_time_param
from .trajectory_planner import _nearest_equivalent_angle, _unwrap_joint_trajectory_positions, _limit_safe_joint_wrapping
from .planner_diagnostics import _diagnose_start_collision
from .planner_utils import _set_result, _is_stale, _begin_execution, _require_cart_path_service, _to_pose_list
from .single_target import _execute_single_point
import numpy as np
import config


def _canonical_principal_angle(value: float) -> float:
    adjusted = float(value)
    two_pi = 2.0 * np.pi
    while adjusted > np.pi:
        adjusted -= two_pi
    while adjusted <= -np.pi:
        adjusted += two_pi
    return adjusted


def _preferred_branch_reference(joint_name: str, live_reference: float) -> float:
    name = str(joint_name or "").strip().lower()
    if name in {"joint_6", "j6", "axis_6"} or name.endswith("_6"):
        return _canonical_principal_angle(live_reference)
    return float(live_reference)


def send_path_cartesian(
    robot_controller,
    waypoints_mm,
    rx,
    ry,
    rz,
    vel_scaling,
    acc_scaling,
    trajectory_optimizer_name=None,
    orientation_mode="constant",
):
    robot_controller.force_safety_update()
    result = _execute_path(
        robot_controller,
        waypoints_mm,
        rx,
        ry,
        rz,
        vel_scaling,
        acc_scaling,
        trajectory_optimizer_name=trajectory_optimizer_name,
        orientation_mode=orientation_mode,
    )
    if result != 0:
        robot_controller.mark_current_motion_complete(result)
    return result


def _execute_path(
    robot_controller,
    waypoints_mm,
    rx,
    ry,
    rz,
    vel_scaling,
    acc_scaling,
    trajectory_optimizer_name=None,
    orientation_mode="constant",
):
    robot_controller.set_last_requested_delta_mm(0.0)
    num_waypoints = len(waypoints_mm)
    robot_controller.get_logger().info(f'[EXECUTE_PATH] Received path with {num_waypoints} waypoints')
    robot_controller.get_logger().info(f'[EXECUTE_PATH] Transforming waypoints from work object to base frame')

    if not _require_cart_path_service(robot_controller, 'EXECUTE_PATH'):
        return -2

    # Compute average spacing
    total_dist_mm = sum(
        np.linalg.norm(np.array(waypoints_mm[i][:3]) - np.array(waypoints_mm[i-1][:3]))
        for i in range(1, num_waypoints)
    ) if num_waypoints > 1 else 0.0

    avg_spacing_mm = total_dist_mm / max(num_waypoints - 1, 1)
    robot_controller.get_logger().info(f'[EXECUTE_PATH] {num_waypoints} waypoints, avg spacing: {avg_spacing_mm:.2f}mm')
    robot_controller.get_logger().info(f'[EXECUTE_PATH] First waypoint: {waypoints_mm[0][:3]}')

    # Derive MoveIt interpolation step from the path's own waypoint density.
    # Keep all user waypoints as geometric anchors; only avoid oversampling
    # between already-dense anchors.
    #
    # Examples:
    #   avg spacing 3.0 mm  -> 4.05 mm -> clamped to 5.0 mm
    #   avg spacing 5.0 mm  -> 6.75 mm
    #   avg spacing 6.0 mm  -> 8.10 mm
    #   avg spacing 8.0 mm  -> 10.8 mm
    #   avg spacing 10.0 mm -> 13.5 mm
    #   avg spacing 12.0 mm -> 16.2 mm -> clamped to 15.0 mm
    #   avg spacing 20.0 mm -> 27.0 mm -> clamped to 15.0 mm
    avg_spacing_m = avg_spacing_mm / 1000.0
    max_step = float(np.clip(avg_spacing_m * 1.35, 0.005, 0.015))

    # Build EE poses + safety check every waypoint
    orientation_mode = str(orientation_mode or "constant").strip().lower()
    if orientation_mode == "per_waypoint":
        waypoints_6d = []
        for wp in waypoints_mm:
            if len(wp) >= 6:
                waypoints_6d.append(list(wp[:6]))
            else:
                waypoints_6d.append([wp[0], wp[1], wp[2], rx, ry, rz])
    else:
        waypoints_6d = [[wp[0], wp[1], wp[2], rx, ry, rz] for wp in waypoints_mm]
    waypoints, err = _to_pose_list(robot_controller, waypoints_6d, robot_controller.T_tool,
                                   check_last_only=False)
    if err:
        return err

    # Check if robot is far from first waypoint
    current_cart = robot_controller.prev_cartesian
    if current_cart is not None and len(current_cart) >= 3:
        approach_dist = np.linalg.norm(np.array(waypoints_mm[0][:3]) - np.array(current_cart[:3]))
        robot_controller.get_logger().info(f'[EXECUTE_PATH] Distance to first waypoint: {approach_dist:.1f}mm')
        if approach_dist > config.PATH_APPROACH_THRESHOLD_MM:
            robot_controller.get_logger().info(
                f'[EXECUTE_PATH] Gap {approach_dist:.1f}mm > {config.PATH_APPROACH_THRESHOLD_MM}mm threshold — '
                f'planning first, then approaching')
            _plan_then_approach(robot_controller, waypoints, waypoints_mm[0], rx, ry, rz,
                                vel_scaling, acc_scaling, max_step, trajectory_optimizer_name)
            return 0

    # Normal flow: robot is close to first waypoint, plan from current state
    request = _build_cartesian_request(robot_controller, waypoints, max_step, vel_scaling, acc_scaling)
    robot_controller.get_logger().info(f'[Cartesian Path] max_step={max_step*1000:.1f}mm')
    gen = _begin_execution(robot_controller)
    future = robot_controller.request_cartesian_path(request)
    future.add_done_callback(
        lambda f: _cartesian_path_response(
            robot_controller,
            f,
            vel_scaling,
            acc_scaling,
            gen,
            trajectory_optimizer_name=trajectory_optimizer_name,
        )
    )
    return 0


def _plan_then_approach(robot_controller, waypoints, first_wp_mm, rx, ry, rz,
                        vel_scaling, acc_scaling, max_step, trajectory_optimizer_name=None):
    """
    Three-phase execution for when the robot is far from the path start:

    Phase 1 — IK query: compute_cartesian_path([wp[0]], max_step=0.1, no collision)
              → get joint state at wp[0] without moving
    Phase 2 — Plan:     compute_cartesian_path(all waypoints, start=IK@wp[0])
              → if fails: abort, robot stays put
              → if succeeds: store trajectory, queue _execute_pending_trajectory
    Phase 3 — Approach: _execute_single_point(current → wp[0])
              → on completion, motion queue fires _execute_pending_trajectory
    """

    # ── Phase 1: Quick IK for first waypoint ──────────────────────────────────
    ik_request = _build_cartesian_request(robot_controller, [waypoints[0]], 0.1, 1.0, 1.0,
                                          avoid_collisions=False)

    robot_controller.get_logger().info('[EXECUTE_PATH] Phase 1: IK query for first waypoint...')
    gen = _begin_execution(robot_controller)
    ik_future = robot_controller.request_cartesian_path(ik_request)

    def on_ik_done(f):
        if _is_stale(robot_controller, gen):
            return
        try:
            ik_resp = f.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[EXECUTE_PATH] IK service error: {e}')
            _set_result(robot_controller, -2)
            return

        traj = ik_resp.solution.joint_trajectory
        if ik_resp.fraction < 0.99 or not traj.points:
            robot_controller.get_logger().error(
                f'[EXECUTE_PATH] First waypoint unreachable (IK fraction={ik_resp.fraction:.2f}) — aborting, robot stays put')
            _set_result(robot_controller, -11)
            return

        ik_joints = traj.points[-1]
        live_joint_state = getattr(robot_controller, 'current_joint_state', None)
        normalized_positions = list(ik_joints.positions)
        if live_joint_state is not None:
            state_names = list(getattr(live_joint_state, 'name', []) or [])
            state_positions = list(getattr(live_joint_state, 'position', []) or [])
            if state_names and len(state_names) == len(state_positions):
                live_by_name = {
                    name: position
                    for name, position in zip(state_names, state_positions)
                }
                adjusted = []
                max_branch_adjustment = 0.0
                can_normalize = True
                for joint_name, value in zip(traj.joint_names, normalized_positions):
                    reference = live_by_name.get(joint_name)
                    if reference is None:
                        can_normalize = False
                        break
                    preferred_reference = _preferred_branch_reference(joint_name, reference)
                    normalized = _nearest_equivalent_angle(preferred_reference, value)
                    max_branch_adjustment = max(max_branch_adjustment, abs(normalized - value))
                    adjusted.append(normalized)
                if can_normalize:
                    normalized_positions = adjusted
                    if max_branch_adjustment > 1e-6:
                        robot_controller.get_logger().info(
                            '[EXECUTE_PATH] Phase 1 IK branch-normalized to live joint state '
                            f'(max wrap adjustment {max_branch_adjustment:.4f} rad)'
                        )
        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] IK ok: {[round(p, 3) for p in normalized_positions[:6]]}')

        # Build virtual start state at first waypoint
        virtual_start = RobotState()
        virtual_start.joint_state.name = list(traj.joint_names)
        virtual_start.joint_state.position = list(normalized_positions)
        virtual_start.is_diff = False

        # ── Phase 2: Plan full path from virtual start ─────────────────────
        plan_request = _build_cartesian_request(robot_controller, waypoints, max_step,
                                                vel_scaling, acc_scaling,
                                                start_state=virtual_start)

        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] Phase 2: Planning {len(waypoints)}-waypoint path from wp[0] '
            f'(max_step={max_step*1000:.1f}mm)...')
        plan_future = robot_controller.request_cartesian_path(plan_request)
        plan_future.add_done_callback(
            lambda future: on_plan_done(future, list(normalized_positions))
        )

    def on_plan_done(f, phase1_reference_positions):
        if _is_stale(robot_controller, gen):
            return
        try:
            resp = f.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[EXECUTE_PATH] Plan service error: {e}')
            _set_result(robot_controller, -2)
            return

        fraction = resp.fraction
        robot_controller.get_logger().info(f'[EXECUTE_PATH] Path planned: {fraction*100:.1f}%')

        if fraction < config.CARTESIAN_MIN_FRACTION:
            robot_controller.get_logger().error(
                f'[EXECUTE_PATH] Planning failed ({fraction*100:.1f}%) — NOT moving to first waypoint')
            _diagnose_start_collision(robot_controller)
            _set_result(robot_controller, -11)
            return  # ← robot has not moved at all

        num_pts = len(resp.solution.joint_trajectory.points)
        resp.solution, max_wrap_adjustment = _unwrap_joint_trajectory_positions(
            resp.solution,
            reference_positions=phase1_reference_positions,
        )
        if max_wrap_adjustment > 1e-6:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Phase 2 path branch-normalized to Phase 1 IK '
                f'(max wrap adjustment {max_wrap_adjustment:.4f} rad)'
            )
        resp.solution, max_limit_wrap_adjustment = _limit_safe_joint_wrapping(
            resp.solution,
            reference_positions=phase1_reference_positions,
        )
        if max_limit_wrap_adjustment > 1e-6:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Phase 2 path rebased to limit-safe branches '
                f'(max wrap adjustment {max_limit_wrap_adjustment:.4f} rad)'
            )
        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] ✓ Plan succeeded ({num_pts} pts). '
            f'Queuing execution, approaching wp[0]: '
            f'[{first_wp_mm[0]:.1f}, {first_wp_mm[1]:.1f}, {first_wp_mm[2]:.1f}]')

        if num_pts <= 1:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Planned path collapsed to a single point at wp[0] — '
                'skipping deferred Phase 3 and executing only the approach move')

            current_cart = robot_controller.prev_cartesian
            if current_cart is None or len(current_cart) < 6:
                robot_controller.get_logger().error(
                    '[EXECUTE_PATH] Lost current position before approach — aborting')
                _set_result(robot_controller, -4)
                return

            _execute_single_point(
                robot_controller,
                start_wp=list(current_cart[:6]),
                target_wp=[first_wp_mm[0], first_wp_mm[1], first_wp_mm[2], rx, ry, rz],
                vel_scaling=vel_scaling,
                acc_scaling=acc_scaling,
            )
            return

        # Store trajectory for deferred execution
        robot_controller.stage_pending_path(
            resp.solution,
            vel_scaling,
            acc_scaling,
            trajectory_optimizer_name=trajectory_optimizer_name,
        )

        # Queue trajectory execution — fires after approach completes
        robot_controller.submit_motion_task(_execute_pending_trajectory, [robot_controller])

        # ── Phase 3: Approach first waypoint ───────────────────────────────
        current_cart = robot_controller.prev_cartesian
        if current_cart is None or len(current_cart) < 6:
            robot_controller.get_logger().error('[EXECUTE_PATH] Lost current position before approach — aborting')
            robot_controller.clear_motion_queue()
            _set_result(robot_controller, -4)
            return

        # Bypass send_cartesian_goal's is_executing check — we own is_executing here
        # _execute_single_point re-acquires the lock and increments plan_generation
        _execute_single_point(
            robot_controller,
            start_wp=list(current_cart[:6]),
            target_wp=[first_wp_mm[0], first_wp_mm[1], first_wp_mm[2], rx, ry, rz],
            vel_scaling=vel_scaling,
            acc_scaling=acc_scaling,
        )

    ik_future.add_done_callback(on_ik_done)


def _execute_pending_trajectory(robot_controller):
    """
    Execute the trajectory stored by _plan_then_approach.
    Called from the motion queue after the approach move completes.
    Aborts silently if the approach move failed.
    """
    if robot_controller.last_move_result != 0:
        robot_controller.get_logger().error(
            f'[EXECUTE_PATH] Approach failed (code={robot_controller.last_move_result}) — '
            f'discarding queued path execution')
        robot_controller.clear_pending_path()
        return -1

    trajectory, vel_scaling, acc_scaling, trajectory_optimizer_name = robot_controller.consume_pending_path()
    if vel_scaling is None:
        vel_scaling = config.DEFAULT_VEL_SCALING
    if acc_scaling is None:
        acc_scaling = config.DEFAULT_ACC_SCALING

    if trajectory is None:
        robot_controller.get_logger().error('[EXECUTE_PATH] No pending trajectory!')
        return -1

    num_pts = len(trajectory.joint_trajectory.points)
    robot_controller.get_logger().info(
        f'[EXECUTE_PATH] Phase 3: Executing pre-planned trajectory ({num_pts} pts)')

    if num_pts <= 1:
        robot_controller.get_logger().info(
            '[EXECUTE_PATH] Pending trajectory has <=1 point — nothing left to execute after approach')
        _set_result(robot_controller, 0)
        return 0

    gen = _begin_execution(robot_controller)
    _apply_time_param(
        robot_controller,
        trajectory,
        vel_scaling,
        acc_scaling,
        gen,
        log_prefix='[EXECUTE_PATH]',
        trajectory_optimizer_name=trajectory_optimizer_name,
    )
    return 0
