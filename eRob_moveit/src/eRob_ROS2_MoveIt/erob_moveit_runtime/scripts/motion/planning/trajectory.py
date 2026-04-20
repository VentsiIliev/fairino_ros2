"""
Multi-Waypoint Cartesian Path
...
"""

from copy import deepcopy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotState
from .trajectory_planner import _cartesian_path_response, _build_cartesian_request, _apply_time_param
from .trajectory_planner import (
    _joint6_path_stats,
    _nearest_equivalent_angle,
    _project_joint6_to_reference_branch,
    _regularize_joint6_branch_sequence,
    _stabilize_joint6_path_shape,
    _unwrap_joint_trajectory_positions,
)
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
    skip_approach_check=False,
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
        skip_approach_check=skip_approach_check,
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
    skip_approach_check=False,
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
    if not skip_approach_check and current_cart is not None and len(current_cart) >= 3:
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
              → if succeeds: build one combined approach+contour trajectory
    Phase 3 — Execute:  time-parameterize and execute the combined trajectory
    """

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

        approach_solution = deepcopy(ik_resp.solution)

        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] IK ok: {[round(p, 3) for p in normalized_positions[:6]]}')

        live_joint_state = getattr(robot_controller, 'current_joint_state', None)
        if live_joint_state is not None:
            state_names = list(getattr(live_joint_state, 'name', []) or [])
            state_positions = list(getattr(live_joint_state, 'position', []) or [])
            if state_names and len(state_names) == len(state_positions):
                live_by_name = {
                    name: position
                    for name, position in zip(state_names, state_positions)
                }
                ordered_live_positions = []
                can_normalize_approach = True
                for joint_name in approach_solution.joint_trajectory.joint_names:
                    if joint_name not in live_by_name:
                        can_normalize_approach = False
                        break
                    ordered_live_positions.append(live_by_name[joint_name])
                if can_normalize_approach:
                    approach_solution, _ = _unwrap_joint_trajectory_positions(
                        approach_solution,
                        reference_positions=ordered_live_positions,
                    )
                    approach_solution, _ = _project_joint6_to_reference_branch(
                        approach_solution,
                        reference_positions=ordered_live_positions,
                    )

        if approach_solution.joint_trajectory.points:
            approach_solution.joint_trajectory.points[-1].positions = list(normalized_positions)

        virtual_start = RobotState()
        virtual_start.joint_state.name = list(traj.joint_names)
        virtual_start.joint_state.position = list(normalized_positions)
        virtual_start.is_diff = False

        plan_request = _build_cartesian_request(robot_controller, waypoints, max_step,
                                                vel_scaling, acc_scaling,
                                                start_state=virtual_start)

        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] Phase 2: Planning {len(waypoints)}-waypoint path from wp[0] '
            f'(max_step={max_step*1000:.1f}mm)...')
        plan_future = robot_controller.request_cartesian_path(plan_request)
        plan_future.add_done_callback(
            lambda future: on_plan_done(
                future,
                list(normalized_positions),
                deepcopy(approach_solution),
            )
        )

    def on_plan_done(f, phase1_reference_positions, approach_solution):
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
            return

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
        resp.solution, max_branch_projection = _project_joint6_to_reference_branch(
            resp.solution,
            reference_positions=phase1_reference_positions,
        )
        if max_branch_projection > 1e-6:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Phase 2 Joint_6 projected to Phase 1 branch '
                f'(max wrap adjustment {max_branch_projection:.4f} rad)'
            )
        resp.solution, max_path_stabilization = _stabilize_joint6_path_shape(resp.solution)
        if max_path_stabilization > 1e-6:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Phase 2 Joint_6 path stabilized to start/end branch '
                f'(max wrap adjustment {max_path_stabilization:.4f} rad)'
            )
        resp.solution, max_sequence_regularization = _regularize_joint6_branch_sequence(resp.solution)
        if max_sequence_regularization > 1e-6:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Phase 2 Joint_6 branch sequence regularized '
                f'(max wrap adjustment {max_sequence_regularization:.4f} rad)'
            )
        joint6_stats = _joint6_path_stats(resp.solution)
        if joint6_stats is not None:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Phase 2 Joint_6 stats: '
                f"start={joint6_stats['start']:.4f}, end={joint6_stats['end']:.4f}, "
                f"min={joint6_stats['min']:.4f}, max={joint6_stats['max']:.4f}, "
                f"span={joint6_stats['span']:.4f}, "
                f"endpoint_delta={joint6_stats['endpoint_delta']:.4f}, "
                f"max_step={joint6_stats['max_step']:.4f}, "
                f"points={joint6_stats['num_points']}"
            )

        if num_pts <= 1:
            robot_controller.get_logger().info(
                '[EXECUTE_PATH] Planned path collapsed to a single point at wp[0] — '
                'executing only the Phase 1 approach move'
            )
            _apply_time_param(
                robot_controller,
                deepcopy(approach_solution),
                vel_scaling,
                acc_scaling,
                gen,
                log_prefix='[EXECUTE_PATH]',
                trajectory_optimizer_name=trajectory_optimizer_name,
            )
            return

        contour_points = len(resp.solution.joint_trajectory.points)
        robot_controller.get_logger().info(
            '[EXECUTE_PATH] ✓ Plan succeeded. Building single combined trajectory '
            f'via wp[0] ({contour_points} raw contour points)'
        )

        combined_trajectory = _combine_joint_trajectories(
            deepcopy(approach_solution),
            deepcopy(resp.solution),
        )
        _apply_time_param(
            robot_controller,
            combined_trajectory,
            vel_scaling,
            acc_scaling,
            gen,
            log_prefix='[EXECUTE_PATH]',
            trajectory_optimizer_name=trajectory_optimizer_name,
        )

    ik_future.add_done_callback(on_ik_done)


def _combine_joint_trajectories(approach_trajectory, contour_trajectory):
    """
    Concatenate the untimed approach and contour trajectories into one path.
    The contour's first point is the same wp[0] state already reached by the
    approach, so drop that duplicated point before time parameterization.
    """
    approach_joint_trajectory = approach_trajectory.joint_trajectory
    contour_joint_trajectory = contour_trajectory.joint_trajectory

    if not contour_joint_trajectory.points:
        return approach_trajectory
    if not approach_joint_trajectory.points:
        return contour_trajectory

    if list(approach_joint_trajectory.joint_names) != list(contour_joint_trajectory.joint_names):
        raise ValueError('Approach and contour trajectories use different joint orders')

    combined = deepcopy(approach_trajectory)
    combined.joint_trajectory.points.extend(deepcopy(contour_joint_trajectory.points[1:]))
    return combined
