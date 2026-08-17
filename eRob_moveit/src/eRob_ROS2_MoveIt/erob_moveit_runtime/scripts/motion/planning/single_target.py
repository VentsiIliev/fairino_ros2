"""
Single-Point Cartesian Move
===========================
Public entry:  send_cartesian_goal
Internal path: _execute_single_point → _to_pose_list → _dispatch_moveit
Request build: _build_cartesian_request  (shared, lives in trajectory_planner)

Generates adaptive intermediate waypoints between current pose and target so
that TOTG always receives >=3 trajectory points. Safety check is performed only
on the final target (start = current robot pos; intermediates are linearly
interpolated between two already-validated endpoints).
"""

from .trajectory_planner import (
    _cartesian_path_response,
    _jacobian_fallback_move,
    _build_cartesian_request,
    _begin_execution,
)
from moveit_msgs.msg import RobotState
from .planner_utils import _set_result, _require_cart_path_service, _to_pose_list
import numpy as np
from time import perf_counter
import config

_ORIENTATION_DIRECT_JACOBIAN_TOL_DEG = 0.5
_ORIENTATION_ONLY_MOVE_STEP_M = 0.01
_SHORT_MOVE_ORIENTATION_INTERP_MM = 80.0
_SHORT_MOVE_ORIENTATION_INTERP_MIN_DELTA_DEG = 1.0


def _wrapped_angle_delta_deg(start_deg, target_deg):
    return abs(((target_deg - start_deg + 180.0) % 360.0) - 180.0)


def _compute_max_step(delta_m, orientation_delta_deg=0.0):
    min_step = 0.0005
    max_step_limit = 0.025
    min_segments = 2
    target_segments = 6

    if delta_m < 0.005 and orientation_delta_deg > _ORIENTATION_DIRECT_JACOBIAN_TOL_DEG:
        return _ORIENTATION_ONLY_MOVE_STEP_M

    segments = max(min_segments, min(target_segments, int(np.ceil(delta_m / 0.01))))
    step = delta_m / segments
    return float(np.clip(step, min_step, max_step_limit))


def _generate_adaptive_waypoints(start_mm, target_mm):
    dx = target_mm[0] - start_mm[0]
    dy = target_mm[1] - start_mm[1]
    dz = target_mm[2] - start_mm[2]
    distance_mm = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    if distance_mm < 1.0:
        segments = 1
    elif distance_mm < 10.0:
        segments = 2
    elif distance_mm < 50.0:
        segments = 3
    else:
        segments = 4

    drx = target_mm[3] - start_mm[3]
    dry = target_mm[4] - start_mm[4]
    drz = ((target_mm[5] - start_mm[5] + 180.0) % 360.0) - 180.0
    orientation_delta_deg = max(abs(drx), abs(dry), abs(drz))
    interpolate_orientation = (
            distance_mm <= _SHORT_MOVE_ORIENTATION_INTERP_MM
            and orientation_delta_deg >= _SHORT_MOVE_ORIENTATION_INTERP_MIN_DELTA_DEG
    )

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


def _execute_jacobian_move(
        robot_controller,
        poses,
        delta_m,
        vel_scaling,
        acc_scaling,
        *,
        avoid_collisions=True,
):
    robot_controller.get_logger().info(
        f'[Single Point] Sub-5mm ({delta_m * 1000:.3f}mm): Jacobian direct (skip MoveIt)')
    gen = _begin_execution(robot_controller)
    ok = _jacobian_fallback_move(
        robot_controller,
        poses,
        vel_scaling,
        acc_scaling,
        gen,
        avoid_collisions=avoid_collisions,
    )
    if not ok:
        _set_result(robot_controller, -8)
        return -8
    return 0


def _dispatch_moveit(robot_controller, request, vel_scaling, acc_scaling, trajectory_optimizer_name=None):
    dispatch_started_at = perf_counter()
    robot_controller._last_cartesian_request_started_at = dispatch_started_at
    robot_controller._last_cartesian_request_kind = 'single_target'
    robot_controller._last_cartesian_request_waypoints = len(getattr(request, 'waypoints', []) or [])
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(
            robot_controller,
            "planning_start",
            strategy="cartesian_path",
            waypoints=robot_controller._last_cartesian_request_waypoints,
        )
    except Exception:
        pass
    gen = _begin_execution(robot_controller)
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(robot_controller, "cartesian_service_dispatch",
                                waypoints=robot_controller._last_cartesian_request_waypoints)
    except Exception:
        pass
    service_call_started_at = perf_counter()
    future = robot_controller.request_cartesian_path(request)
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(
            robot_controller,
            "cartesian_service_dispatched",
            duration_s=perf_counter() - service_call_started_at,
        )
    except Exception:
        pass
    future.add_done_callback(
        lambda f: _cartesian_path_response(
            robot_controller, f, vel_scaling, acc_scaling, gen,
            trajectory_optimizer_name=trajectory_optimizer_name))


def _resolve_start_state(robot_controller, start_pose):
    """Resolve a joint-space start state that matches the current Cartesian start pose."""
    from copy import deepcopy
    from moveit_msgs.srv import GetPositionIK
    import time

    robot_context = getattr(robot_controller, "robot_context", None)

    planning_group = str(
        getattr(robot_context, "planning_group", "")
        or getattr(config, "PLANNING_GROUP", "manipulator")
    )

    ee_link = str(
        getattr(robot_context, "ee_link", "")
        or getattr(config, "EE_LINK", "ee_link")
    )

    base_link = str(
        getattr(robot_context, "base_link", "")
        or getattr(config, "BASE_LINK", "base_link")
    )

    ik_client = robot_controller.get_ik_client()
    if ik_client is None or not ik_client.wait_for_service(timeout_sec=0.5):
        robot_controller.get_logger().warning(
            '[Single Point] IK start-state sync unavailable — using live joint state')
        return None

    req = GetPositionIK.Request()
    req.ik_request.group_name = planning_group
    req.ik_request.ik_link_name = ee_link
    req.ik_request.pose_stamped.header.frame_id = base_link
    req.ik_request.pose_stamped.header.stamp = robot_controller.get_clock().now().to_msg()
    req.ik_request.pose_stamped.pose = start_pose
    req.ik_request.avoid_collisions = True
    req.ik_request.timeout.sec = 1
    req.ik_request.timeout.nanosec = 0

    if robot_controller.current_joint_state is not None:
        live_state = robot_controller.current_joint_state

        state_names = list(
            getattr(live_state, "name", []) or []
        )
        state_positions = list(
            getattr(live_state, "position", []) or []
        )

        if state_names and len(state_names) == len(state_positions):
            position_by_name = dict(
                zip(state_names, state_positions)
            )

            joint_names = list(
                getattr(robot_context, "joint_names", ())
                or getattr(config, "JOINT_NAMES", [])
                or []
            )

            if joint_names and all(
                    name in position_by_name
                    for name in joint_names
            ):
                seed = deepcopy(live_state)
                seed.header.stamp = (
                    robot_controller.get_clock().now().to_msg()
                )
                seed.name = list(joint_names)
                seed.position = [
                    float(position_by_name[name])
                    for name in joint_names
                ]

                # Do not accidentally leave 12-element derivative arrays
                # attached to a 6-joint seed.
                seed.velocity = []
                seed.effort = []

                req.ik_request.robot_state.joint_state = seed
                req.ik_request.robot_state.is_diff = False

    future = ik_client.call_async(req)
    deadline = time.time() + 1.5
    while time.time() < deadline:
        if future.done():
            break
        time.sleep(0.01)

    if not future.done():
        robot_controller.get_logger().warning(
            '[Single Point] IK start-state sync timed out — using live joint state')
        return None

    try:
        resp = future.result()
    except Exception as exc:
        robot_controller.get_logger().warning(
            f'[Single Point] IK start-state sync failed: {exc} — using live joint state')
        return None

    error_code = getattr(
        getattr(resp, "error_code", None),
        "val",
        None,
    )

    solution = getattr(resp, "solution", None)
    joint_state = getattr(solution, "joint_state", None)

    solution_names = list(
        getattr(joint_state, "name", []) or []
    )

    solution_positions = list(
        getattr(joint_state, "position", []) or []
    )

    joint_names = list(
        getattr(robot_context, "joint_names", ())
        or getattr(config, "JOINT_NAMES", [])
        or []
    )

    usable_solution = (
            bool(joint_names)
            and bool(solution_names)
            and len(solution_names) == len(solution_positions)
    )

    if usable_solution:
        position_by_name = dict(
            zip(solution_names, solution_positions)
        )

        usable_solution = all(
            name in position_by_name
            for name in joint_names
        )

    if not usable_solution:
        live_joint_state = robot_controller.current_joint_state

        live_names = (
            list(getattr(live_joint_state, "name", []) or [])
            if live_joint_state is not None
            else []
        )

        live_positions = (
            list(getattr(live_joint_state, "position", []) or [])
            if live_joint_state is not None
            else []
        )

        robot_controller.get_logger().warning(
            '[Single Point] IK start-state sync returned no usable joint solution '
            f'(error_code={error_code}, '
            f'names={solution_names}, '
            f'positions={len(solution_positions)}) '
            '— using live joint state'
        )

        if live_names and live_positions:
            robot_controller.get_logger().warning(
                '[Single Point] Live joint state fallback: '
                f'{[(name, round(pos, 6)) for name, pos in zip(live_names, live_positions)]}'
            )

            robot_controller.get_logger().warning(
                '[Single Point] Requested Cartesian start pose: '
                f'X={start_pose.position.x * 1000:.2f} '
                f'Y={start_pose.position.y * 1000:.2f} '
                f'Z={start_pose.position.z * 1000:.2f} '
                f'Q=[{start_pose.orientation.x:.4f}, '
                f'{start_pose.orientation.y:.4f}, '
                f'{start_pose.orientation.z:.4f}, '
                f'{start_pose.orientation.w:.4f}]'
            )

        return None

    ordered_positions = [
        float(position_by_name[name])
        for name in joint_names
    ]

    start_state = RobotState()
    start_state.joint_state.name = list(joint_names)
    start_state.joint_state.position = ordered_positions
    start_state.is_diff = False

    robot_controller.get_logger().info(
        '[Single Point] Using IK-synchronized start state '
        f'for {planning_group}'
    )

    return start_state


def _execute_single_point(robot_controller, start_wp, target_wp, vel_scaling, acc_scaling,
                          tool_transform=None, avoid_collisions=None, trajectory_optimizer_name=None):
    from config import resolve_avoid_collisions
    execute_started_at = perf_counter()
    avoid_collisions = resolve_avoid_collisions(avoid_collisions)
    dx = target_wp[0] - start_wp[0]
    dy = target_wp[1] - start_wp[1]
    dz = target_wp[2] - start_wp[2]
    distance_mm = (dx * dx + dy * dy + dz * dz) ** 0.5
    adaptive = _generate_adaptive_waypoints(start_wp, target_wp)
    # Exact intermediate Cartesian anchors can over-constrain MoveIt's nonlinear
    # IK continuation on short single-target moves. For normal linear moves we
    # therefore pass only start/end and let compute_cartesian_path interpolate
    # internally. Structural waypoints remain only for true micro moves.
    use_structural_waypoints = distance_mm < 5.0
    waypoints_mm = [start_wp] + adaptive if use_structural_waypoints else [start_wp, target_wp]
    robot_controller.get_logger().info(
        f'[Single Point] {len(waypoints_mm)} waypoints (Δ={distance_mm:.3f}mm, '
        f'{len(waypoints_mm) - 1} segments, structural_waypoints={use_structural_waypoints})')
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(
            robot_controller,
            "cartesian_waypoints_ready",
            duration_s=perf_counter() - execute_started_at,
            waypoints=len(waypoints_mm),
            distance_mm=float(distance_mm),
        )
    except Exception:
        pass

    service_wait_started_at = perf_counter()
    if not _require_cart_path_service(robot_controller, 'Single Point'):
        return -2
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(robot_controller, "cartesian_service_ready",
                                duration_s=perf_counter() - service_wait_started_at)
    except Exception:
        pass

    T_tool = tool_transform if tool_transform is not None else robot_controller.T_tool
    pose_build_started_at = perf_counter()
    poses, err = _to_pose_list(robot_controller, waypoints_mm, T_tool)
    if err:
        return err
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(robot_controller, "cartesian_poses_built",
                                duration_s=perf_counter() - pose_build_started_at, poses=len(poses))
    except Exception:
        pass

    p0, p1 = poses[0].position, poses[-1].position
    delta_m = np.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2)
    orientation_delta_deg = max(
        _wrapped_angle_delta_deg(start_wp[3], target_wp[3]),
        _wrapped_angle_delta_deg(start_wp[4], target_wp[4]),
        _wrapped_angle_delta_deg(start_wp[5], target_wp[5]),
    )
    robot_controller.set_last_requested_delta_mm(delta_m * 1000.0)
    robot_controller.set_last_full_waypoints(poses)

    if delta_m < 0.005 and orientation_delta_deg <= _ORIENTATION_DIRECT_JACOBIAN_TOL_DEG:
        return _execute_jacobian_move(
            robot_controller,
            poses,
            delta_m,
            vel_scaling,
            acc_scaling,
            avoid_collisions=avoid_collisions,
        )

    if delta_m < 0.005:
        robot_controller.get_logger().info(
            f'[Single Point] Sub-5mm translation but orientation delta={orientation_delta_deg:.3f}° '
            '— bypassing Jacobian direct and forcing MoveIt path')

    max_step = _compute_max_step(delta_m, orientation_delta_deg)
    if delta_m < 0.005 and orientation_delta_deg > _ORIENTATION_DIRECT_JACOBIAN_TOL_DEG:
        robot_controller.get_logger().info(
            f'[Single Point] Orientation-dominant move: using coarse MoveIt eef_step={max_step:.4f}m '
            f'for orientation delta={orientation_delta_deg:.3f}°')

    stack_check_started_at = perf_counter()
    if not robot_controller.is_motion_stack_ready():
        robot_controller.get_logger().error(
            f'[Single Point] Motion stack not ready: {robot_controller.get_motion_stack_fault_reason()}')
        return config.MOTION_ERROR_HARDWARE_NOT_READY
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(robot_controller, "motion_stack_ready",
                                duration_s=perf_counter() - stack_check_started_at)
    except Exception:
        pass

    if bool(getattr(config, "SINGLE_TARGET_USE_IK_SYNC_START_STATE", False)):
        start_state = _resolve_start_state(robot_controller, poses[0])
    else:
        start_state = None
        robot_controller.get_logger().info(
            '[Single Point] Skipping IK-synchronized start state for Cartesian planning'
        )
    request_build_started_at = perf_counter()
    request = _build_cartesian_request(
        robot_controller,
        poses,
        max_step,
        vel_scaling,
        acc_scaling,
        start_state=start_state,
        avoid_collisions=avoid_collisions,
    )
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(
            robot_controller,
            "cartesian_request_built",
            duration_s=perf_counter() - request_build_started_at,
            max_step_m=float(max_step),
            avoid_collisions=bool(avoid_collisions),
        )
    except Exception:
        pass
    _dispatch_moveit(
        robot_controller,
        request,
        vel_scaling,
        acc_scaling,
        trajectory_optimizer_name=trajectory_optimizer_name,
    )
    return 0


def send_cartesian_goal(robot_controller, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                        tool_transform=None, avoid_collisions=None, trajectory_optimizer=None):
    from config import resolve_avoid_collisions
    safety_started_at = perf_counter()
    robot_controller.force_safety_update()
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(robot_controller, "preplan_safety_update",
                                duration_s=perf_counter() - safety_started_at)
    except Exception:
        pass
    avoid_collisions = resolve_avoid_collisions(avoid_collisions)

    current_cart = robot_controller.prev_cartesian
    if current_cart is None or len(current_cart) < 6:
        robot_controller.get_logger().error('[MOVE] No current position available, cannot plan path')
        return -4

    start_wp = list(current_cart[:6])
    target_wp = [x_mm, y_mm, z_mm, rx, ry, rz]

    robot_controller.get_logger().info(
        f'[MOVE] [{start_wp[0]:.1f}, {start_wp[1]:.1f}, {start_wp[2]:.1f}, RZ={start_wp[5]:.1f}°]'
        f' → [{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}, RZ={rz:.1f}°]')
    if not avoid_collisions:
        robot_controller.get_logger().info('[MOVE] MoveIt collision checking disabled for this request')

    return _execute_single_point(robot_controller, start_wp, target_wp, vel_scale, acc_scale,
                                 tool_transform=tool_transform,
                                 avoid_collisions=avoid_collisions,
                                 trajectory_optimizer_name=trajectory_optimizer)
