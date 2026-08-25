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
from time import perf_counter

from .planner_utils import _set_result, _is_stale, _begin_execution
from .planner_diagnostics import _diagnose_fk_mismatch, _diagnose_start_collision
from .jacobian_move import _jacobian_fallback_move
from ..execution.trajectory_executor import _send_trajectory_to_controller
from ..execution.trajectory_optimizer import resolve_trajectory_optimizer
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath, GetStateValidity
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation, Slerp
import config


# ─── Main path-planning response handler ─────────────────────────────────────

def _validate_cartesian_trajectory_state_validity_async(
    robot_controller,
    trajectory,
    generation,
    on_success,
    *,
    avoid_collisions=True,
) -> bool:
    if not bool(getattr(config, "CARTESIAN_STATE_VALIDITY_ENABLED", True)):
        on_success()
        return True
    if not avoid_collisions or not config.resolve_avoid_collisions(True):
        robot_controller.get_logger().info(
            '[Cartesian Path] Collision validation skipped for this request'
        )
        on_success()
        return True

    joint_trajectory = getattr(trajectory, "joint_trajectory", None)
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    points = list(getattr(joint_trajectory, "points", []) or [])
    if not joint_names or not points:
        on_success()
        return True

    client = robot_controller.get_state_validity_client()
    if client is None or not client.wait_for_service(timeout_sec=1.0):
        robot_controller.get_logger().error(
            "[Cartesian Path] /check_state_validity unavailable — aborting collision-checked trajectory"
        )
        _set_result(robot_controller, -10)
        return False

    stride = max(1, int(getattr(config, "CARTESIAN_STATE_VALIDITY_STRIDE", 1)))
    indexes = set(range(0, len(points), stride))
    indexes.add(0)
    indexes.add(len(points) - 1)
    indexes = sorted(indexes)

    validation = {
        "indexes": indexes,
        "next_offset": 0,
        "responses": {},
        "future": None,
        "done": False,
        "timer": None,
    }
    robot_controller._pending_cartesian_state_validation = validation

    def _clear_validation():
        validation["done"] = True
        timer = validation.get("timer")
        if timer is not None:
            try:
                timer.cancel()
                robot_controller.destroy_timer(timer)
            except Exception:
                pass
        if getattr(robot_controller, "_pending_cartesian_state_validation", None) is validation:
            robot_controller._pending_cartesian_state_validation = None

    def _fail(message):
        if validation["done"]:
            return
        _clear_validation()
        robot_controller.get_logger().error(message)
        _set_result(robot_controller, -10)

    def _finish_if_ready():
        if validation["done"] or len(validation["responses"]) != len(indexes):
            return
        _clear_validation()
        if _is_stale(robot_controller, generation):
            robot_controller.get_logger().info(
                "[Cartesian Path] Stale collision validation response discarded"
            )
            return
        if _evaluate_cartesian_validity_responses(robot_controller, indexes, validation["responses"]):
            on_success()

    def _send_next_request():
        if validation["done"]:
            return
        if validation["next_offset"] >= len(indexes):
            _finish_if_ready()
            return
        index = indexes[validation["next_offset"]]
        validation["next_offset"] += 1
        req = _make_state_validity_request(joint_names, points[index].positions)
        future = client.call_async(req)
        validation["future"] = future

        def _on_done(done_future):
            if validation["done"]:
                return
            try:
                validation["responses"][index] = done_future.result()
            except Exception as exc:
                _fail(f"[Cartesian Path] State validity request failed at sample {index}: {exc}")
                return
            _send_next_request()

        future.add_done_callback(_on_done)

    def _on_timeout():
        if validation["done"]:
            return
        missing = indexes[validation["next_offset"]:]
        current = indexes[max(0, validation["next_offset"] - 1)] if indexes else None
        _fail(
            "[Cartesian Path] State validity validation timed out "
            f"after {float(getattr(config, 'CARTESIAN_STATE_VALIDITY_TIMEOUT_S', 5.0)):.1f}s "
            f"(current_sample={current}, remaining={missing})"
        )

    robot_controller.get_logger().info(
        f"[Cartesian Path] Collision validation requested ({len(indexes)} state checks)"
    )
    timeout_s = max(0.5, float(getattr(config, "CARTESIAN_STATE_VALIDITY_TIMEOUT_S", 5.0)))
    validation["timer"] = robot_controller.create_timer(timeout_s, _on_timeout)
    _send_next_request()
    return True


def _make_state_validity_request(joint_names, positions):
    js = JointState()
    js.name = list(joint_names)
    js.position = [float(value) for value in positions]
    state = RobotState()
    state.joint_state = js
    state.is_diff = True

    req = GetStateValidity.Request()
    req.robot_state = state
    req.group_name = config.PLANNING_GROUP
    return req


def _evaluate_cartesian_validity_responses(robot_controller, indexes, responses) -> bool:
    allow_escape = bool(getattr(config, "CARTESIAN_ALLOW_START_COLLISION_ESCAPE", True))
    escape_depth_tol_m = float(getattr(config, "CARTESIAN_START_COLLISION_ESCAPE_DEPTH_TOL_M", 0.001))
    initial_contact_pairs = None
    initial_max_depth = 0.0
    saw_valid_after_start_collision = False
    allowed_escape_samples = 0

    for index in indexes:
        response = responses[index]
        valid = bool(getattr(response, "valid", False))
        contacts = list(getattr(response, "contacts", []) or [])
        contact_pairs = {
            tuple(sorted((str(c.contact_body_1), str(c.contact_body_2))))
            for c in contacts
        }
        max_depth = max((abs(float(getattr(c, "depth", 0.0))) for c in contacts), default=0.0)

        if valid:
            if initial_contact_pairs is not None:
                saw_valid_after_start_collision = True
            continue

        if allow_escape and index == 0 and contact_pairs:
            initial_contact_pairs = set(contact_pairs)
            initial_max_depth = max_depth
            allowed_escape_samples += 1
            robot_controller.get_logger().warning(
                "[Cartesian Path] Start state is already in collision; allowing escape only "
                f"for existing contacts={_format_contact_pairs(initial_contact_pairs)} "
                f"initial_penetration={initial_max_depth * 1000.0:.2f}mm"
            )
            continue

        if allow_escape and initial_contact_pairs is not None and contact_pairs:
            has_only_initial_contacts = contact_pairs.issubset(initial_contact_pairs)
            depth_ok = max_depth <= initial_max_depth + escape_depth_tol_m
            if has_only_initial_contacts and depth_ok:
                allowed_escape_samples += 1
                robot_controller.get_logger().warning(
                    f"[Cartesian Path] Allowing collision-escape sample {index}: "
                    f"contacts={_format_contact_pairs(contact_pairs)} "
                    f"penetration={max_depth * 1000.0:.2f}mm"
                )
                continue

        if contacts:
            pairs = sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in contacts})
            robot_controller.get_logger().error(
                f"[Cartesian Path] Collision validation failed at sample {index}: {pairs}"
            )
        else:
            robot_controller.get_logger().error(
                f"[Cartesian Path] Collision validation failed at sample {index}: invalid state"
            )
        _set_result(robot_controller, -10)
        return False

    if initial_contact_pairs is not None:
        if saw_valid_after_start_collision:
            robot_controller.get_logger().warning(
                f"[Cartesian Path] Collision escape accepted; cleared after "
                f"{allowed_escape_samples} colliding sample(s)"
            )
        else:
            robot_controller.get_logger().warning(
                f"[Cartesian Path] Collision escape accepted but final sampled state is still in "
                f"the original contact set={_format_contact_pairs(initial_contact_pairs)}"
            )

    robot_controller.get_logger().info(
        f"[Cartesian Path] Collision validation passed ({len(indexes)} state checks)"
    )
    return True


def _format_contact_pairs(contact_pairs) -> list[str]:
    return [
        f"{a}<->{b}"
        for a, b in sorted(contact_pairs)
    ]

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


def _duration_to_float(duration) -> float:
    return float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) * 1e-9


def _set_duration_from_float(duration, seconds: float) -> None:
    seconds = max(0.0, float(seconds))
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    duration.sec = sec
    duration.nanosec = nanosec


def _scale_optional_sequence(values, scale: float):
    if not values:
        return values
    return [float(value) * scale for value in values]


def _stretch_single_target_joint_rate_if_needed(rc, joint_trajectory):
    """Stretch single-target timing when a configured joint rate is too high."""
    if getattr(rc, "_last_cartesian_request_kind", None) != "single_target":
        return joint_trajectory

    points = list(getattr(joint_trajectory, "points", []) or [])
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    if len(points) < 2 or not joint_names:
        return joint_trajectory

    limits = getattr(config, "SINGLE_TARGET_JOINT_RATE_LIMITS_RAD_S", {}) or {}
    if not isinstance(limits, dict) or not limits:
        return joint_trajectory

    required_scale = 1.0
    worst_name = ""
    worst_rate = 0.0
    worst_limit = 0.0
    for joint_name, raw_limit in limits.items():
        if joint_name not in joint_names:
            continue
        try:
            limit = float(raw_limit)
        except (TypeError, ValueError):
            continue
        if limit <= 0.0:
            continue

        joint_index = joint_names.index(joint_name)
        for prev_point, point in zip(points, points[1:]):
            prev_t = _duration_to_float(prev_point.time_from_start)
            point_t = _duration_to_float(point.time_from_start)
            dt = point_t - prev_t
            if dt <= 1e-9:
                continue
            delta = abs(float(point.positions[joint_index]) - float(prev_point.positions[joint_index]))
            rate = delta / dt
            scale = rate / limit
            if scale > required_scale:
                required_scale = scale
                worst_name = str(joint_name)
                worst_rate = rate
                worst_limit = limit

    if required_scale <= 1.001:
        return joint_trajectory

    for point in points:
        t = _duration_to_float(point.time_from_start)
        _set_duration_from_float(point.time_from_start, t * required_scale)
        point.velocities = _scale_optional_sequence(point.velocities, 1.0 / required_scale)
        point.accelerations = _scale_optional_sequence(
            point.accelerations,
            1.0 / (required_scale * required_scale),
        )

    rc.get_logger().warning(
        f'[Single Point] Stretched trajectory timing by {required_scale:.2f}x '
        f'to respect {worst_name} rate limit '
        f'(peak {worst_rate:.3f} rad/s > {worst_limit:.3f} rad/s); '
        f'new duration {_duration_to_float(points[-1].time_from_start):.3f}s'
    )
    return joint_trajectory


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


def _stabilize_joint6_path_shape(trajectory) -> tuple[object, float]:
    """Bias Joint_6 to the branch implied by its start/end path shape.

    Even after continuity unwrapping, MoveIt can leave intermediate Joint_6
    points on equivalent branches that preserve the same endpoint but create a
    large internal excursion. That later shows up as unexpected wrist unwind.
    Re-project each intermediate point to the equivalent angle nearest the
    linear interpolation between the already-normalized start and end values.
    """
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None or len(joint_trajectory.points) < 3:
        return trajectory, 0.0

    joint_index = None
    for index, joint_name in enumerate(joint_trajectory.joint_names):
        name = str(joint_name or '').strip().lower()
        if name in {'joint_6', 'j6', 'axis_6'} or name.endswith('_6'):
            joint_index = index
            break
    if joint_index is None:
        return trajectory, 0.0

    start_value = float(joint_trajectory.points[0].positions[joint_index])
    end_value = float(joint_trajectory.points[-1].positions[joint_index])
    max_adjustment = 0.0
    num_points = len(joint_trajectory.points)

    for point_index, point in enumerate(joint_trajectory.points[1:-1], start=1):
        positions = list(point.positions)
        original = float(positions[joint_index])
        alpha = point_index / float(num_points - 1)
        interpolated = start_value + alpha * (end_value - start_value)
        adjusted = _nearest_equivalent_angle(interpolated, original)
        if abs(adjusted - original) > 1e-9:
            positions[joint_index] = adjusted
            point.positions = positions
            max_adjustment = max(max_adjustment, abs(adjusted - original))

    return trajectory, max_adjustment


def _joint6_path_stats(trajectory):
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None or not joint_trajectory.points:
        return None

    joint_index = None
    for index, joint_name in enumerate(joint_trajectory.joint_names):
        name = str(joint_name or '').strip().lower()
        if name in {'joint_6', 'j6', 'axis_6'} or name.endswith('_6'):
            joint_index = index
            break
    if joint_index is None:
        return None

    values = [float(point.positions[joint_index]) for point in joint_trajectory.points]
    if not values:
        return None

    max_step = 0.0
    for previous, current in zip(values, values[1:]):
        max_step = max(max_step, abs(current - previous))

    return {
        'start': values[0],
        'end': values[-1],
        'min': min(values),
        'max': max(values),
        'span': max(values) - min(values),
        'endpoint_delta': values[-1] - values[0],
        'max_step': max_step,
        'num_points': len(values),
    }


def _regularize_joint6_branch_sequence(trajectory) -> tuple[object, float]:
    """Choose a single smooth Joint_6 branch sequence across the whole path.

    For each point, search equivalent ±2π branches and pick the one that best
    matches both the previous adjusted point and the global start→end trend.
    This is stronger than local unwrapping when MoveIt leaves a wrist path on a
    mixed set of equivalent branches.
    """
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None or len(joint_trajectory.points) < 2:
        return trajectory, 0.0

    joint_index = None
    for index, joint_name in enumerate(joint_trajectory.joint_names):
        name = str(joint_name or '').strip().lower()
        if name in {'joint_6', 'j6', 'axis_6'} or name.endswith('_6'):
            joint_index = index
            break
    if joint_index is None:
        return trajectory, 0.0

    start_value = float(joint_trajectory.points[0].positions[joint_index])
    end_value = float(joint_trajectory.points[-1].positions[joint_index])
    previous_adjusted = start_value
    max_adjustment = 0.0
    two_pi = 2.0 * math.pi
    num_points = len(joint_trajectory.points)

    for point_index, point in enumerate(joint_trajectory.points[1:], start=1):
        positions = list(point.positions)
        original = float(positions[joint_index])
        alpha = point_index / float(num_points - 1)
        trend_target = start_value + alpha * (end_value - start_value)

        best_value = original
        best_cost = None
        for shift in range(-3, 4):
            candidate = original + shift * two_pi
            # Favor staying close to both the previous point and the global
            # start/end trend, with slightly higher weight on continuity.
            continuity_cost = abs(candidate - previous_adjusted)
            trend_cost = abs(candidate - trend_target)
            cost = continuity_cost * 1.5 + trend_cost
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_value = candidate

        positions[joint_index] = best_value
        point.positions = positions
        max_adjustment = max(max_adjustment, abs(best_value - original))
        previous_adjusted = best_value

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


def _format_pose_for_log(pose):
    return (
        f"pos=({pose.position.x:.5f}, {pose.position.y:.5f}, {pose.position.z:.5f})m "
        f"quat=({pose.orientation.x:.5f}, {pose.orientation.y:.5f}, "
        f"{pose.orientation.z:.5f}, {pose.orientation.w:.5f})"
    )


def _interpolate_pose(start_pose, target_pose, t: float):
    alpha = max(0.0, min(1.0, float(t)))
    pose = Pose()
    pose.position.x = start_pose.position.x + (target_pose.position.x - start_pose.position.x) * alpha
    pose.position.y = start_pose.position.y + (target_pose.position.y - start_pose.position.y) * alpha
    pose.position.z = start_pose.position.z + (target_pose.position.z - start_pose.position.z) * alpha

    start_quat = [
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z,
        start_pose.orientation.w,
    ]
    target_quat = [
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w,
    ]
    rotations = Rotation.from_quat([start_quat, target_quat])
    quat = Slerp([0.0, 1.0], rotations)([alpha]).as_quat()[0]
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def _request_cartesian_diag_ik_async(robot_controller, pose, seed_joint_state, avoid_collisions: bool):
    from moveit_msgs.srv import GetPositionIK

    ik_client = robot_controller.get_ik_client()
    if ik_client is None or not ik_client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("MoveIt IK service unavailable")

    req = GetPositionIK.Request()
    req.ik_request.group_name = config.PLANNING_GROUP
    req.ik_request.ik_link_name = config.EE_LINK
    req.ik_request.pose_stamped.header.frame_id = config.BASE_LINK
    req.ik_request.pose_stamped.header.stamp = robot_controller.get_clock().now().to_msg()
    req.ik_request.pose_stamped.pose = pose
    req.ik_request.avoid_collisions = bool(avoid_collisions)
    req.ik_request.timeout.sec = 2
    req.ik_request.timeout.nanosec = 0
    req.ik_request.robot_state.joint_state = deepcopy(seed_joint_state)
    req.ik_request.robot_state.is_diff = False

    return ik_client.call_async(req)


def _request_cartesian_diag_validity_async(robot_controller, joint_state):
    from moveit_msgs.srv import GetStateValidity

    client = robot_controller.get_state_validity_client()
    if client is None or not client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("MoveIt state validity service unavailable")

    req = GetStateValidity.Request()
    req.robot_state.joint_state = deepcopy(joint_state)
    req.robot_state.is_diff = False
    req.group_name = config.PLANNING_GROUP
    return client.call_async(req)


def _extract_ik_solution_joint_state(response):
    solution = getattr(response, "solution", None)
    joint_state = getattr(solution, "joint_state", None)
    names = list(getattr(joint_state, "name", []) or [])
    positions = list(getattr(joint_state, "position", []) or [])
    if not names or not positions:
        return None, names, positions
    return joint_state, names, positions


def _joint_state_from_names_positions(robot_controller, joint_names, joint_positions):
    from sensor_msgs.msg import JointState

    joint_state = JointState()
    joint_state.name = list(joint_names)
    joint_state.position = [float(position) for position in joint_positions]
    joint_state.header.stamp = robot_controller.get_clock().now().to_msg()
    return joint_state


def _trajectory_endpoint_seed(response):
    trajectory = getattr(response, "solution", None)
    joint_trajectory = getattr(trajectory, "joint_trajectory", None)
    points = list(getattr(joint_trajectory, "points", []) or [])
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    if not points or not joint_names:
        return None
    last_positions = list(getattr(points[-1], "positions", []) or [])
    if len(last_positions) != len(joint_names):
        return None
    return joint_names, last_positions


def _build_branch_probe_seeds(robot_controller, base_seed_joint_state, cartesian_response=None):
    seeds = [("current_seed", deepcopy(base_seed_joint_state))]

    endpoint_seed = _trajectory_endpoint_seed(cartesian_response)
    if endpoint_seed is not None:
        names, positions = endpoint_seed
        seeds.append((
            "last_successful_cartesian_point",
            _joint_state_from_names_positions(robot_controller, names, positions),
        ))

    names = list(getattr(base_seed_joint_state, "name", []) or [])
    positions = list(getattr(base_seed_joint_state, "position", []) or [])
    if names and len(names) == len(positions):
        zero_seed = _joint_state_from_names_positions(
            robot_controller,
            names,
            [0.0] * len(positions),
        )
        seeds.append(("zero_seed", zero_seed))

        canonical_positions = []
        two_pi = 2.0 * math.pi
        for position in positions:
            adjusted = float(position)
            while adjusted > math.pi:
                adjusted -= two_pi
            while adjusted <= -math.pi:
                adjusted += two_pi
            canonical_positions.append(adjusted)
        seeds.append((
            "canonical_current_seed",
            _joint_state_from_names_positions(robot_controller, names, canonical_positions),
        ))

        for joint_index, joint_name in enumerate(names):
            name = str(joint_name or "").strip().lower()
            if name not in {"joint_6", "j6", "axis_6"} and not name.endswith("_6"):
                continue
            for shift in (-two_pi, two_pi):
                shifted = list(positions)
                shifted[joint_index] = float(shifted[joint_index]) + shift
                seeds.append((
                    f"{joint_name}_{'minus' if shift < 0.0 else 'plus'}_2pi_seed",
                    _joint_state_from_names_positions(robot_controller, names, shifted),
                ))
            break

    unique = []
    seen = set()
    for label, seed in seeds:
        key = (
            tuple(getattr(seed, "name", []) or []),
            tuple(round(float(value), 6) for value in (getattr(seed, "position", []) or [])),
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append((label, seed))
    return unique


def _start_branch_probe(robot_controller, label: str, pose, seed_label: str, seed_joint_state):
    logger = robot_controller.get_logger()
    try:
        future = _request_cartesian_diag_ik_async(
            robot_controller,
            pose,
            seed_joint_state,
            avoid_collisions=False,
        )
    except Exception as exc:
        logger.error(f"[CartesianDiag] {label} branch_probe {seed_label} request failed: {exc}")
        return

    def _on_done(future):
        try:
            response = future.result()
            error_code = int(getattr(getattr(response, "error_code", None), "val", 0))
            joint_state, names, positions = _extract_ik_solution_joint_state(response)
            logger.error(
                "[CartesianDiag] "
                f"{label} branch_probe {seed_label} "
                f"avoid_collisions=False error_code={error_code} joints={len(positions)}"
            )
            if joint_state is not None:
                logger.error(
                    "[CartesianDiag] "
                    f"{label} branch_probe {seed_label} solution="
                    f"{[(name, round(float(pos), 6)) for name, pos in zip(names, positions)]}"
                )
        except Exception as exc:
            logger.error(f"[CartesianDiag] {label} branch_probe {seed_label} failed: {exc}")

    future.add_done_callback(_on_done)


def _start_branch_probe_set(robot_controller, label: str, pose, base_seed_joint_state, cartesian_response=None):
    seeds = _build_branch_probe_seeds(
        robot_controller,
        base_seed_joint_state,
        cartesian_response=cartesian_response,
    )
    for seed_label, seed_joint_state in seeds:
        if seed_label == "current_seed":
            continue
        _start_branch_probe(robot_controller, label, pose, seed_label, seed_joint_state)


def _make_orientation_variant_pose(pose, euler_delta_deg):
    base_quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    base_rotation = Rotation.from_quat(base_quat)
    delta_rotation = Rotation.from_euler("xyz", euler_delta_deg, degrees=True)
    variant_quat = (delta_rotation * base_rotation).as_quat()

    variant = Pose()
    variant.position.x = pose.position.x
    variant.position.y = pose.position.y
    variant.position.z = pose.position.z
    variant.orientation.x = float(variant_quat[0])
    variant.orientation.y = float(variant_quat[1])
    variant.orientation.z = float(variant_quat[2])
    variant.orientation.w = float(variant_quat[3])
    return variant


def _make_position_variant_pose(pose, delta_xyz_m):
    variant = Pose()
    variant.position.x = pose.position.x + float(delta_xyz_m[0])
    variant.position.y = pose.position.y + float(delta_xyz_m[1])
    variant.position.z = pose.position.z + float(delta_xyz_m[2])
    variant.orientation.x = pose.orientation.x
    variant.orientation.y = pose.orientation.y
    variant.orientation.z = pose.orientation.z
    variant.orientation.w = pose.orientation.w
    return variant


def _start_single_reach_probe(robot_controller, label: str, pose, seed_joint_state):
    logger = robot_controller.get_logger()
    try:
        future = _request_cartesian_diag_ik_async(
            robot_controller,
            pose,
            seed_joint_state,
            avoid_collisions=False,
        )
    except Exception as exc:
        logger.error(f"[CartesianDiag] reach_probe {label} request failed: {exc}")
        return

    def _on_done(future):
        try:
            response = future.result()
            error_code = int(getattr(getattr(response, "error_code", None), "val", 0))
            _joint_state, _names, positions = _extract_ik_solution_joint_state(response)
            logger.error(
                "[CartesianDiag] "
                f"reach_probe {label} avoid_collisions=False "
                f"error_code={error_code} joints={len(positions)}"
            )
        except Exception as exc:
            logger.error(f"[CartesianDiag] reach_probe {label} failed: {exc}")

    future.add_done_callback(_on_done)


def _start_endpoint_reachability_probes(robot_controller, pose, seed_joint_state):
    # Diagnostic only. These probes answer whether the endpoint fails because of
    # strict TCP orientation or because the XYZ is outside the IK workspace.
    orientation_variants = [
        ("orient_rx_plus_10deg", (10.0, 0.0, 0.0)),
        ("orient_rx_minus_10deg", (-10.0, 0.0, 0.0)),
        ("orient_ry_plus_10deg", (0.0, 10.0, 0.0)),
        ("orient_ry_minus_10deg", (0.0, -10.0, 0.0)),
        ("orient_rz_plus_10deg", (0.0, 0.0, 10.0)),
        ("orient_rz_minus_10deg", (0.0, 0.0, -10.0)),
    ]
    for label, euler_delta_deg in orientation_variants:
        _start_single_reach_probe(
            robot_controller,
            label,
            _make_orientation_variant_pose(pose, euler_delta_deg),
            seed_joint_state,
        )

    position_variants = [
        ("pos_x_minus_50mm", (-0.050, 0.0, 0.0)),
        ("pos_y_minus_50mm", (0.0, -0.050, 0.0)),
        ("pos_z_plus_50mm", (0.0, 0.0, 0.050)),
    ]
    for label, delta_xyz_m in position_variants:
        _start_single_reach_probe(
            robot_controller,
            label,
            _make_position_variant_pose(pose, delta_xyz_m),
            seed_joint_state,
        )


def _log_cartesian_pose_diagnostics(robot_controller, label: str, pose, seed_joint_state):
    logger = robot_controller.get_logger()
    logger.error(f"[CartesianDiag] {label}: {_format_pose_for_log(pose)}")
    logger.error(
        f"[CartesianDiag] {label}: avoid_collisions=False is diagnostic only; "
        "it is never used for execution"
    )

    results = {False: None, True: None}

    def _try_finish_classification():
        no_collision_result = results.get(False)
        collision_result = results.get(True)
        if no_collision_result is None or collision_result is None:
            return

        no_collision_error, no_collision_state = no_collision_result
        collision_error, collision_state = collision_result

        if no_collision_state is None:
            logger.error(
                f"[CartesianDiag] {label} classification: IK/reachability/joint-limits "
                f"(collision-disabled IK failed, error_code={no_collision_error})"
            )
            return

        try:
            validity_future = _request_cartesian_diag_validity_async(
                robot_controller,
                no_collision_state,
            )
        except Exception as exc:
            logger.error(f"[CartesianDiag] {label} state validity request failed: {exc}")
            return

        def _on_validity_done(future):
            try:
                validity = future.result()
                valid = bool(getattr(validity, "valid", False))
                logger.error(f"[CartesianDiag] {label} collision-disabled IK state_valid={valid}")
                contacts = list(getattr(validity, "contacts", []) or [])
                if contacts:
                    pairs = sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in contacts})
                    logger.error(f"[CartesianDiag] {label} collision-disabled IK contacts={pairs}")
            except Exception as exc:
                logger.error(f"[CartesianDiag] {label} state validity check failed: {exc}")

            if collision_state is None:
                logger.error(
                    f"[CartesianDiag] {label} classification: collision/scene-constraint filtering "
                    f"(collision-disabled IK succeeds, collision-enabled IK failed, error_code={collision_error})"
                )
            else:
                logger.error(
                    f"[CartesianDiag] {label} classification: IK exists with collision filtering; "
                    "partial Cartesian failure is likely interpolation jump, joint-limit continuity, "
                    "or Cartesian solver sampling before/after this probe point"
                )

        validity_future.add_done_callback(_on_validity_done)

    def _make_ik_done_callback(avoid_collisions):
        def _on_ik_done(future):
            try:
                response = future.result()
                error_code = int(getattr(getattr(response, "error_code", None), "val", 0))
                joint_state, names, positions = _extract_ik_solution_joint_state(response)
                results[avoid_collisions] = (error_code, joint_state)
                logger.error(
                    "[CartesianDiag] "
                    f"{label} IK avoid_collisions={avoid_collisions} "
                    f"error_code={error_code} joints={len(positions)}"
                )
                if joint_state is not None:
                    logger.error(
                        "[CartesianDiag] "
                        f"{label} IK solution="
                        f"{[(name, round(float(pos), 6)) for name, pos in zip(names, positions)]}"
                    )
            except Exception as exc:
                results[avoid_collisions] = (None, None)
                logger.error(
                    "[CartesianDiag] "
                    f"{label} IK avoid_collisions={avoid_collisions} failed: {exc}"
                )
            _try_finish_classification()
        return _on_ik_done

    from config import resolve_avoid_collisions
    collision_enabled = resolve_avoid_collisions(True)  # Check if global collision checking is on
    avoid_collision_values = (False, True) if collision_enabled else (False,)

    for avoid_collisions in avoid_collision_values:
        try:
            future = _request_cartesian_diag_ik_async(
                robot_controller,
                pose,
                seed_joint_state,
                avoid_collisions=avoid_collisions,
            )
            future.add_done_callback(_make_ik_done_callback(avoid_collisions))
        except Exception as exc:
            results[avoid_collisions] = (None, None)
            logger.error(
                "[CartesianDiag] "
                f"{label} IK avoid_collisions={avoid_collisions} request failed: {exc}"
            )
            _try_finish_classification()


def _diagnose_partial_cartesian_failure(robot_controller, fraction: float, cartesian_response=None):
    logger = robot_controller.get_logger()
    waypoints = robot_controller.get_last_full_waypoints() or []
    if len(waypoints) < 2:
        logger.warning("[CartesianDiag] No stored waypoints available for partial-path diagnostics")
        return

    seed_joint_state = getattr(robot_controller, "current_joint_state", None)
    if seed_joint_state is None:
        logger.warning("[CartesianDiag] No current joint state available for partial-path diagnostics")
        return

    requested_delta_mm = float(robot_controller.get_last_requested_delta_mm() or 0.0)
    if requested_delta_mm <= 0.0:
        logger.warning("[CartesianDiag] Requested delta unavailable for partial-path diagnostics")
        return

    max_step_m = float(getattr(config, "CARTESIAN_MAX_STEP", 0.025))
    sample_step_fraction = max_step_m * 1000.0 / requested_delta_mm
    first_failed_t = min(1.0, max(0.0, float(fraction)) + max(sample_step_fraction, 1e-6))

    logger.error(
        "[CartesianDiag] Partial path probe: "
        f"fraction={float(fraction) * 100:.1f}% requested_delta={requested_delta_mm:.3f}mm "
        f"max_step={max_step_m:.4f}m first_failed_t~{first_failed_t:.4f}"
    )
    logger.error(
        "[CartesianDiag] seed_joint_state="
        f"{[(name, round(float(pos), 6)) for name, pos in zip(seed_joint_state.name, seed_joint_state.position)]}"
    )

    start_pose = waypoints[0]
    target_pose = waypoints[-1]
    failed_probe = _interpolate_pose(start_pose, target_pose, first_failed_t)
    _log_cartesian_pose_diagnostics(robot_controller, "first_failed_sample", failed_probe, seed_joint_state)
    _start_branch_probe_set(
        robot_controller,
        "first_failed_sample",
        failed_probe,
        seed_joint_state,
        cartesian_response=cartesian_response,
    )

    if first_failed_t < 0.999:
        _log_cartesian_pose_diagnostics(robot_controller, "endpoint", target_pose, seed_joint_state)
        _start_branch_probe_set(
            robot_controller,
            "endpoint",
            target_pose,
            seed_joint_state,
            cartesian_response=cartesian_response,
        )
        _start_endpoint_reachability_probes(robot_controller, target_pose, seed_joint_state)

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
    sanitized, max_path_stabilization = _stabilize_joint6_path_shape(sanitized)
    if max_path_stabilization > 1e-6:
        rc.get_logger().info(
            f'{log_prefix} Stabilized Joint_6 path shape before optimization '
            f'(max wrap adjustment {max_path_stabilization:.4f} rad)'
        )
    sanitized, max_sequence_regularization = _regularize_joint6_branch_sequence(sanitized)
    if max_sequence_regularization > 1e-6:
        rc.get_logger().info(
            f'{log_prefix} Regularized Joint_6 branch sequence before optimization '
            f'(max wrap adjustment {max_sequence_regularization:.4f} rad)'
        )
    joint6_stats = _joint6_path_stats(sanitized)
    if joint6_stats is not None:
        rc.get_logger().info(
            f"{log_prefix} Joint_6 stats before optimization: "
            f"start={joint6_stats['start']:.4f}, end={joint6_stats['end']:.4f}, "
            f"min={joint6_stats['min']:.4f}, max={joint6_stats['max']:.4f}, "
            f"span={joint6_stats['span']:.4f}, "
            f"endpoint_delta={joint6_stats['endpoint_delta']:.4f}, "
            f"max_step={joint6_stats['max_step']:.4f}, "
            f"points={joint6_stats['num_points']}"
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
    optimizer_prepare_started_at = perf_counter()
    prepared_trajectory = _sanitize_optimizer_start(rc, trajectory, log_prefix)
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(rc, "optimizer_prepare_done", duration_s=perf_counter() - optimizer_prepare_started_at)
    except Exception:
        pass
    optimizer_started_at = perf_counter()

    def on_done(result):
        if _is_stale(rc, gen):
            rc.get_logger().info(f'{log_prefix} Stale TOTG response discarded')
            return
        if result is None:
            rc.get_logger().info(
                f'[TIMING] optimizer kind={getattr(rc, "_last_cartesian_request_kind", "unknown")} '
                f'optimizer={trajectory_optimizer_name or getattr(getattr(rc, "trajectory_optimizer", None), "__class__", type("", (), {})).__name__} '
                f'success=false elapsed_s={perf_counter() - optimizer_started_at:.3f}'
            )
            rc.get_logger().error(f'{log_prefix} Time parameterization failed')
            _set_result(rc, -7)
            return
        total_from_request = None
        request_started_at = getattr(rc, '_last_cartesian_request_started_at', None)
        if request_started_at is not None:
            total_from_request = perf_counter() - float(request_started_at)
        optimizer_elapsed_s = perf_counter() - optimizer_started_at
        rc.get_logger().info(
            f'[TIMING] optimizer kind={getattr(rc, "_last_cartesian_request_kind", "unknown")} '
            f'optimizer={trajectory_optimizer_name or getattr(getattr(rc, "trajectory_optimizer", None), "__class__", type("", (), {})).__name__} '
            f'success=true traj_points={len(getattr(getattr(result, "joint_trajectory", None), "points", []) or [])} '
            f'elapsed_s={optimizer_elapsed_s:.3f}'
            + (f' total_from_request_s={total_from_request:.3f}' if total_from_request is not None else '')
        )
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(
                rc,
                "optimizer_done",
                duration_s=optimizer_elapsed_s,
                traj_points=len(getattr(getattr(result, "joint_trajectory", None), "points", []) or []),
            )
        except Exception:
            pass
        with rc.lock:
            rc.last_move_result = 0
        joint_trajectory = _stretch_single_target_joint_rate_if_needed(rc, result.joint_trajectory)
        _send_trajectory_to_controller(rc, joint_trajectory)

    optimizer = resolve_trajectory_optimizer(trajectory_optimizer_name, node=rc, default_optimizer=rc.trajectory_optimizer)
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(rc, "optimizer_request_start", optimizer=optimizer.__class__.__name__)
    except Exception:
        pass
    optimizer.optimize(
        rc,
        prepared_trajectory,
        vel_scaling,
        acc_scaling,
        on_done,
    )


def _build_cartesian_request(rc, poses, max_step, vel_scaling, acc_scaling,
                              start_state=None, avoid_collisions=False):
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
    response_callback_started_at = perf_counter()

    if _is_stale(robot_controller, generation):
        robot_controller.get_logger().info('[Cartesian Path] Stale response discarded (preempted)')
        return

    try:
        callback_started = perf_counter()
        response_read_started_at = perf_counter()
        response = future.result()
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(
                robot_controller,
                "cartesian_response_read",
                duration_s=perf_counter() - response_read_started_at,
                callback_delay_s=callback_started - response_callback_started_at,
            )
        except Exception:
            pass
        fraction = response.fraction
        request_started_at = getattr(robot_controller, '_last_cartesian_request_started_at', None)
        planning_elapsed_s = None
        if request_started_at is not None:
            planning_elapsed_s = callback_started - float(request_started_at)
        solution = getattr(response, 'solution', None)
        joint_trajectory = getattr(solution, 'joint_trajectory', None) if solution is not None else None
        num_pts = len(getattr(joint_trajectory, 'points', []) or [])
        robot_controller.get_logger().info(
            f'[TIMING] cartesian_plan kind={getattr(robot_controller, "_last_cartesian_request_kind", "unknown")} '
            f'waypoints={getattr(robot_controller, "_last_cartesian_request_waypoints", "unknown")} '
            f'fraction={fraction:.4f} traj_points={num_pts} '
            + (f'elapsed_s={planning_elapsed_s:.3f}' if planning_elapsed_s is not None else 'elapsed_s=unknown')
        )
        if getattr(robot_controller, "_last_cartesian_request_kind", None) == "single_target":
            try:
                from motion.move_linear_timing import mark as mark_move_linear_timing
                mark_move_linear_timing(
                    robot_controller,
                    "planning_done",
                    strategy="cartesian_path",
                    fraction=float(fraction),
                    traj_points=num_pts,
                    plan_elapsed_s=planning_elapsed_s if planning_elapsed_s is not None else -1.0,
                )
            except Exception:
                pass
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
                    avoid_collisions=avoid_collisions,
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

            if bool(getattr(config, "CARTESIAN_FAILURE_DIAGNOSTICS_ENABLED", False)):
                # Diagnostic only: probes failure cause, never relaxes execution collision checks.
                _diagnose_partial_cartesian_failure(
                    robot_controller,
                    fraction,
                    cartesian_response=response,
                )

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
        def _on_validated():
            try:
                from motion.move_linear_timing import mark as mark_move_linear_timing
                mark_move_linear_timing(robot_controller, "collision_validation_done", points=num_pts)
            except Exception:
                pass
            _apply_time_param(robot_controller, trajectory, vel_scaling, acc_scaling,
                              generation, log_prefix='[Cartesian Path]',
                              trajectory_optimizer_name=trajectory_optimizer_name)

        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(robot_controller, "collision_validation_start", points=num_pts)
        except Exception:
            pass
        _validate_cartesian_trajectory_state_validity_async(
            robot_controller,
            trajectory,
            generation,
            _on_validated,
            avoid_collisions=avoid_collisions,
        )

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
