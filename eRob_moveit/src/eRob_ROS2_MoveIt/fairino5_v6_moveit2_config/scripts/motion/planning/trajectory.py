"""
Multi-Waypoint Cartesian Path
...
"""

from moveit_msgs.msg import RobotState
from .trajectory_planner import _cartesian_path_response, _build_cartesian_request, _apply_time_param
from .planner_diagnostics import _diagnose_start_collision
from .planner_utils import _set_result, _is_stale, _begin_execution, _require_cart_path_service, _to_pose_list
from .single_target import _execute_single_point
import numpy as np
import config


def send_path_cartesian(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
    robot_controller.force_safety_update()
    result = _execute_path(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling)
    if result != 0:
        robot_controller.mark_current_motion_complete(result)
    return result


def _execute_path(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
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

    # Build EE poses + safety check every waypoint
    waypoints_6d = [[wp[0], wp[1], wp[2], rx, ry, rz] for wp in waypoints_mm]
    waypoints, err = _to_pose_list(robot_controller, waypoints_6d, robot_controller.T_tool,
                                   check_last_only=False)
    if err:
        return err

    avg_spacing_mm = total_dist_mm / max(num_waypoints - 1, 1)
    robot_controller.get_logger().info(f'[EXECUTE_PATH] {num_waypoints} waypoints, avg spacing: {avg_spacing_mm:.2f}mm')
    robot_controller.get_logger().info(f'[EXECUTE_PATH] First waypoint: {waypoints_mm[0][:3]}')

    # Step size based on avg spacing — MORE waypoints = path already dense = LARGER steps
    if avg_spacing_mm < 5:
        max_step = 0.003
    elif avg_spacing_mm < 15:
        max_step = 0.005
    else:
        max_step = 0.010

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
                                vel_scaling, acc_scaling, max_step)
            return 0

    # Normal flow: robot is close to first waypoint, plan from current state
    request = _build_cartesian_request(robot_controller, waypoints, max_step, vel_scaling, acc_scaling)
    robot_controller.get_logger().info(f'[Cartesian Path] max_step={max_step*1000:.1f}mm')
    gen = _begin_execution(robot_controller)
    future = robot_controller.request_cartesian_path(request)
    future.add_done_callback(lambda f: _cartesian_path_response(robot_controller, f, vel_scaling, acc_scaling, gen))
    return 0


def _plan_then_approach(robot_controller, waypoints, first_wp_mm, rx, ry, rz,
                        vel_scaling, acc_scaling, max_step):
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
    ik_request = _build_cartesian_request(robot_controller, [waypoints[0]], 0.1, 0.0, 0.0,
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
            _set_result(robot_controller, -3)
            return

        ik_joints = traj.points[-1]
        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] IK ok: {[round(p, 3) for p in ik_joints.positions[:6]]}')

        # Build virtual start state at first waypoint
        virtual_start = RobotState()
        virtual_start.joint_state.name = list(traj.joint_names)
        virtual_start.joint_state.position = list(ik_joints.positions)
        virtual_start.is_diff = False

        # ── Phase 2: Plan full path from virtual start ─────────────────────
        plan_request = _build_cartesian_request(robot_controller, waypoints, max_step,
                                                vel_scaling, acc_scaling,
                                                start_state=virtual_start)

        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] Phase 2: Planning {len(waypoints)}-waypoint path from wp[0] '
            f'(max_step={max_step*1000:.1f}mm)...')
        plan_future = robot_controller.request_cartesian_path(plan_request)
        plan_future.add_done_callback(on_plan_done)

    def on_plan_done(f):
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
            _set_result(robot_controller, -3)
            return  # ← robot has not moved at all

        num_pts = len(resp.solution.joint_trajectory.points)
        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] ✓ Plan succeeded ({num_pts} pts). '
            f'Queuing execution, approaching wp[0]: '
            f'[{first_wp_mm[0]:.1f}, {first_wp_mm[1]:.1f}, {first_wp_mm[2]:.1f}]')

        # Store trajectory for deferred execution
        robot_controller.stage_pending_path(resp.solution, vel_scaling, acc_scaling)

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

    trajectory, vel_scaling, acc_scaling = robot_controller.consume_pending_path()
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

    gen = _begin_execution(robot_controller)
    _apply_time_param(robot_controller, trajectory, vel_scaling, acc_scaling,
                      gen, log_prefix='[EXECUTE_PATH]')
    return 0
