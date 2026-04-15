"""
Joint-space PTP planning with orientation-policy validation.

Behavior:
- Keep the runtime API unchanged: callers still pass a 6D target pose.
- Always aim for the requested final TCP pose.
- If requested TCP orientation is effectively the current orientation, suppress
  unnecessary reorientation along the path and minimize wrist motion.
- Otherwise allow orientation change, but reject trajectories that deviate too
  far from the requested orientation progression.
- Validate sampled joint states through MoveIt state validity before execution.
"""

from __future__ import annotations

from copy import deepcopy
from itertools import product
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import config
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .planner_utils import _begin_execution, _set_result, _to_pose_list
from .trajectory_planner import _apply_time_param


def _wait_future(future, timeout_s: float, description: str):
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise TimeoutError(f"{description} timed out after {timeout_s:.1f}s")


def _quaternion_angle_deg(quat_a, quat_b) -> float:
    quat_a = np.asarray(quat_a, dtype=float)
    quat_b = np.asarray(quat_b, dtype=float)
    dot = abs(float(np.dot(quat_a, quat_b)))
    dot = np.clip(dot, -1.0, 1.0)
    return float(np.degrees(2.0 * np.arccos(dot)))


def _nearest_equivalent_angle(reference, value):
    adjusted = float(value)
    ref = float(reference)
    two_pi = 2.0 * np.pi
    while adjusted - ref > np.pi:
        adjusted -= two_pi
    while adjusted - ref < -np.pi:
        adjusted += two_pi
    return adjusted


def _normalize_joint_branch(current_positions, target_positions):
    return [
        _nearest_equivalent_angle(current, target)
        for current, target in zip(current_positions, target_positions)
    ]


def _weighted_joint_cost(current_positions, target_positions):
    deltas = [abs(target - current) for current, target in zip(current_positions, target_positions)]
    cost = sum(delta ** 2 for delta in deltas)
    wrist_penalty_start_rad = np.radians(float(getattr(config, "PTP_WRIST_PENALTY_START_DEG", 45.0)))
    if len(deltas) >= 3:
        for delta in deltas[-3:]:
            if delta > wrist_penalty_start_rad:
                overflow = delta - wrist_penalty_start_rad
                cost += 4.0 * (overflow ** 2)
    return cost


def _choose_min_cost_equivalent_joint_positions(current_positions, target_positions):
    normalized = _normalize_joint_branch(current_positions, target_positions)
    if len(normalized) < 3:
        return normalized

    wrist_start = len(normalized) - 3
    best_positions = list(normalized)
    best_cost = _weighted_joint_cost(current_positions, best_positions)
    two_pi = 2.0 * np.pi

    for wraps in product((-1, 0, 1), repeat=3):
        candidate = list(normalized)
        for offset, wrap_count in enumerate(wraps):
            index = wrist_start + offset
            candidate[index] += wrap_count * two_pi
        cost = _weighted_joint_cost(current_positions, candidate)
        if cost < best_cost:
            best_positions = candidate
            best_cost = cost

    return best_positions


def _get_live_joint_state(robot_controller):
    joint_state = getattr(robot_controller, "current_joint_state", None)
    if joint_state is None:
        return None, None
    names = list(getattr(joint_state, "name", []) or [])
    positions = list(getattr(joint_state, "position", []) or [])
    if not names or len(names) != len(positions):
        return None, None
    return names, positions


def _request_ik(robot_controller, target_pose, seed_joint_state, *, avoid_collisions=False, timeout_s=1.0):
    from moveit_msgs.srv import GetPositionIK

    ik_client = robot_controller.get_ik_client()
    if ik_client is None or not ik_client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("MoveIt IK service unavailable")

    req = GetPositionIK.Request()
    req.ik_request.group_name = config.PLANNING_GROUP
    req.ik_request.ik_link_name = config.EE_LINK
    req.ik_request.pose_stamped.header.frame_id = config.BASE_LINK
    req.ik_request.pose_stamped.header.stamp = robot_controller.get_clock().now().to_msg()
    req.ik_request.pose_stamped.pose = target_pose
    req.ik_request.avoid_collisions = bool(avoid_collisions)
    req.ik_request.timeout.sec = int(timeout_s)
    req.ik_request.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1_000_000_000)
    req.ik_request.robot_state.joint_state = seed_joint_state
    req.ik_request.robot_state.is_diff = False
    future = ik_client.call_async(req)
    return _wait_future(future, timeout_s=timeout_s + 1.0, description="MoveIt IK request")


def _request_fk(robot_controller, joint_names, joint_positions):
    from moveit_msgs.srv import GetPositionFK

    fk_client = robot_controller.get_fk_client()
    if fk_client is None or not fk_client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("MoveIt FK service unavailable")

    req = GetPositionFK.Request()
    req.header.frame_id = config.BASE_LINK
    req.fk_link_names = [config.EE_LINK]
    req.robot_state.joint_state.name = list(joint_names)
    req.robot_state.joint_state.position = list(joint_positions)
    req.robot_state.is_diff = False
    future = fk_client.call_async(req)
    return _wait_future(future, timeout_s=2.0, description="MoveIt FK request")


def _check_state_validity(robot_controller, joint_names, joint_positions):
    from moveit_msgs.srv import GetStateValidity

    client = robot_controller.get_state_validity_client()
    if client is None or not client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("MoveIt state validity service unavailable")

    req = GetStateValidity.Request()
    req.robot_state.joint_state = JointState(
        name=list(joint_names),
        position=list(joint_positions),
    )
    req.group_name = config.PLANNING_GROUP
    future = client.call_async(req)
    return _wait_future(future, timeout_s=2.0, description="MoveIt state validity request")


def _build_seed_joint_state(robot_controller):
    joint_names, current_positions = _get_live_joint_state(robot_controller)
    if joint_names is None or current_positions is None:
        raise RuntimeError("current joint state not available")

    seed = JointState()
    seed.name = list(joint_names)
    seed.position = list(current_positions)
    seed.header.stamp = robot_controller.get_clock().now().to_msg()
    return seed, joint_names, current_positions


def _format_pose(pose):
    return (
        f"pos=({pose.position.x:.5f}, {pose.position.y:.5f}, {pose.position.z:.5f})m "
        f"quat=({pose.orientation.x:.5f}, {pose.orientation.y:.5f}, "
        f"{pose.orientation.z:.5f}, {pose.orientation.w:.5f})"
    )


def _log_ik_failure_diagnostics(
    robot_controller,
    target_pose,
    current_tcp_pose,
    target_tcp_pose,
    seed_joint_state,
    first_error_code,
):
    logger = robot_controller.get_logger()
    logger.error("[PTP][IK_DIAG] Target IK failed")
    logger.error(f"[PTP][IK_DIAG] first_error_code={first_error_code}")
    logger.error(
        "[PTP][IK_DIAG] current_tcp_pose_mm_deg="
        f"{[round(float(v), 4) for v in current_tcp_pose]}"
    )
    logger.error(
        "[PTP][IK_DIAG] target_tcp_pose_mm_deg="
        f"{[round(float(v), 4) for v in target_tcp_pose]}"
    )
    logger.error(f"[PTP][IK_DIAG] target_ee_pose={_format_pose(target_pose)}")
    logger.error(
        "[PTP][IK_DIAG] seed_joint_state="
        f"{[(name, round(float(pos), 6)) for name, pos in zip(seed_joint_state.name, seed_joint_state.position)]}"
    )

    for avoid_collisions, timeout_s in ((False, 3.0), (True, 3.0)):
        try:
            retry = _request_ik(
                robot_controller,
                target_pose,
                seed_joint_state,
                avoid_collisions=avoid_collisions,
                timeout_s=timeout_s,
            )
            retry_error = int(getattr(getattr(retry, "error_code", None), "val", 0))
            solution = getattr(retry, "solution", None)
            joint_state = getattr(solution, "joint_state", None)
            names = list(getattr(joint_state, "name", []) or [])
            positions = list(getattr(joint_state, "position", []) or [])
            logger.error(
                "[PTP][IK_DIAG] retry "
                f"avoid_collisions={avoid_collisions} timeout={timeout_s:.1f}s "
                f"error_code={retry_error} joints={len(positions)}"
            )
            if names and positions:
                logger.error(
                    "[PTP][IK_DIAG] retry_solution="
                    f"{[(name, round(float(pos), 6)) for name, pos in zip(names, positions)]}"
                )
        except Exception as exc:
            logger.error(
                "[PTP][IK_DIAG] retry "
                f"avoid_collisions={avoid_collisions} timeout={timeout_s:.1f}s failed: {exc}"
            )

    try:
        validity = _check_state_validity(
            robot_controller,
            seed_joint_state.name,
            seed_joint_state.position,
        )
        logger.error(
            "[PTP][IK_DIAG] current_seed_state_valid="
            f"{bool(getattr(validity, 'valid', False))}"
        )
        contacts = list(getattr(validity, "contacts", []) or [])
        if contacts:
            pairs = sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in contacts})
            logger.error(f"[PTP][IK_DIAG] current_seed_contacts={pairs}")
    except Exception as exc:
        logger.error(f"[PTP][IK_DIAG] current seed validity check failed: {exc}")


def _resolve_target_joint_positions(
    robot_controller,
    target_pose,
    current_joint_names,
    current_positions,
    current_tcp_pose,
    target_tcp_pose,
):
    seed_joint_state = JointState()
    seed_joint_state.name = list(current_joint_names)
    seed_joint_state.position = list(current_positions)
    seed_joint_state.header.stamp = robot_controller.get_clock().now().to_msg()

    ik_response = _request_ik(robot_controller, target_pose, seed_joint_state)
    error_code = int(getattr(getattr(ik_response, "error_code", None), "val", 0))
    solution = getattr(ik_response, "solution", None)
    joint_state = getattr(solution, "joint_state", None)
    target_names = list(getattr(joint_state, "name", []) or [])
    target_positions = list(getattr(joint_state, "position", []) or [])
    if error_code != 1 or not target_names or not target_positions:
        _log_ik_failure_diagnostics(
            robot_controller,
            target_pose,
            current_tcp_pose,
            target_tcp_pose,
            seed_joint_state,
            error_code,
        )
        raise RuntimeError("target IK failed")

    target_by_name = {name: pos for name, pos in zip(target_names, target_positions)}
    ordered = []
    for joint_name, current in zip(current_joint_names, current_positions):
        if joint_name not in target_by_name:
            raise RuntimeError(f"IK solution missing joint {joint_name}")
        ordered.append(target_by_name[joint_name])
    return _choose_min_cost_equivalent_joint_positions(current_positions, ordered)


def _make_joint_trajectory(joint_names, current_positions, target_positions):
    max_joint_delta = max(abs(target - current) for current, target in zip(current_positions, target_positions))
    step = float(getattr(config, "PTP_JOINT_INTERPOLATION_STEP_RAD", 0.08))
    min_segments = int(getattr(config, "PTP_MIN_INTERPOLATION_SEGMENTS", 8))
    max_segments = int(getattr(config, "PTP_MAX_INTERPOLATION_SEGMENTS", 80))
    segments = int(np.clip(np.ceil(max_joint_delta / max(step, 1e-4)), min_segments, max_segments))

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    traj.points = []
    for index in range(segments + 1):
        t = index / max(segments, 1)
        positions = [
            current + (target - current) * t
            for current, target in zip(current_positions, target_positions)
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        traj.points.append(point)
    return traj


def _reject_same_orientation_wrist_flip(current_positions, target_positions):
    max_wrist_delta_deg = float(getattr(config, "PTP_LOCKED_MAX_WRIST_DELTA_DEG", 120.0))
    wrist_start = max(len(current_positions) - 3, 0)
    wrist_deltas_deg = [
        float(np.degrees(abs(target - current)))
        for current, target in zip(current_positions[wrist_start:], target_positions[wrist_start:])
    ]
    if any(delta > max_wrist_delta_deg for delta in wrist_deltas_deg):
        return (
            -11,
            "same-orientation PTP chose a far wrist branch "
            f"(wrist deltas deg={','.join(f'{delta:.1f}' for delta in wrist_deltas_deg)})",
        )
    return 0, None


def _reject_excessive_wrist_rotation(current_positions, target_positions, orientation_locked):
    max_wrist_delta_deg = float(
        getattr(
            config,
            "PTP_LOCKED_MAX_WRIST_DELTA_DEG" if orientation_locked else "PTP_MAX_WRIST_DELTA_DEG",
            120.0 if orientation_locked else 160.0,
        )
    )
    wrist_start = max(len(current_positions) - 3, 0)
    wrist_deltas_deg = [
        float(np.degrees(abs(target - current)))
        for current, target in zip(current_positions[wrist_start:], target_positions[wrist_start:])
    ]
    if any(delta > max_wrist_delta_deg for delta in wrist_deltas_deg):
        return (
            -11,
            "PTP rejected due to excessive wrist rotation "
            f"(wrist deltas deg={','.join(f'{delta:.1f}' for delta in wrist_deltas_deg)}, "
            f"limit={max_wrist_delta_deg:.1f})",
        )
    return 0, None


def _pose_quaternion(pose):
    return np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ], dtype=float)


def _validate_interpolated_states(
    robot_controller,
    joint_trajectory,
    start_pose,
    target_pose,
    orientation_locked,
):
    start_quat = _pose_quaternion(start_pose)
    target_quat = _pose_quaternion(target_pose)
    if orientation_locked:
        max_allowed_deg = float(getattr(config, "PTP_LOCKED_PATH_MAX_DRIFT_DEG", 2.0))
        slerp = None
    else:
        max_allowed_deg = float(getattr(config, "PTP_ORIENTED_PATH_MAX_DEVIATION_DEG", 5.0))
        rotations = Rotation.from_quat(np.vstack([start_quat, target_quat]))
        slerp = Slerp([0.0, 1.0], rotations)

    total_points = len(joint_trajectory.points)
    for index, point in enumerate(joint_trajectory.points):
        validity = _check_state_validity(robot_controller, joint_trajectory.joint_names, point.positions)
        if not bool(getattr(validity, "valid", False)):
            return -10, f"sample {index} is in collision or invalid"

        fk_response = _request_fk(robot_controller, joint_trajectory.joint_names, point.positions)
        fk_error = int(getattr(getattr(fk_response, "error_code", None), "val", 0))
        fk_poses = list(getattr(fk_response, "pose_stamped", []) or [])
        if fk_error != 1 or not fk_poses:
            return -11, f"FK failed at sample {index}"

        pose = fk_poses[0].pose
        quat = _pose_quaternion(pose)
        if orientation_locked:
            deviation_deg = _quaternion_angle_deg(quat, start_quat)
        else:
            t = index / max(total_points - 1, 1)
            reference_quat = slerp([t]).as_quat()[0]
            deviation_deg = _quaternion_angle_deg(quat, reference_quat)

        if deviation_deg > max_allowed_deg:
            return -11, (
                f"orientation deviation {deviation_deg:.2f}° exceeds "
                f"{max_allowed_deg:.2f}° at sample {index}"
            )

    return 0, None


def send_ptp_goal(
    robot_controller,
    x_mm,
    y_mm,
    z_mm,
    rx,
    ry,
    rz,
    vel_scale,
    acc_scale,
    tool_transform=None,
    trajectory_optimizer_name=None,
):
    robot_controller.force_safety_update()

    current_cart = robot_controller.prev_cartesian
    if current_cart is None or len(current_cart) < 6:
        robot_controller.get_logger().error("[PTP] No current Cartesian pose available")
        return -4

    current_joint_names = None
    current_positions = None
    try:
        _seed, current_joint_names, current_positions = _build_seed_joint_state(robot_controller)
        current_tcp_pose = list(current_cart[:6])
        requested_tcp_pose = [x_mm, y_mm, z_mm, rx, ry, rz]

        current_rot = Rotation.from_euler("xyz", current_tcp_pose[3:], degrees=True)
        requested_rot = Rotation.from_euler("xyz", requested_tcp_pose[3:], degrees=True)
        orientation_delta_deg = _quaternion_angle_deg(current_rot.as_quat(), requested_rot.as_quat())
        lock_tol_deg = float(getattr(config, "PTP_LOCK_ORIENTATION_TOL_DEG", 2.0))
        orientation_locked = orientation_delta_deg <= lock_tol_deg

        target_tcp_pose = list(requested_tcp_pose)

        T_tool = tool_transform if tool_transform is not None else robot_controller.T_tool
        target_poses, err = _to_pose_list(robot_controller, [target_tcp_pose], T_tool, check_last_only=True)
        if err:
            return err
        target_pose = target_poses[0]

        start_pose = _to_pose_list(robot_controller, [current_tcp_pose], T_tool, check_last_only=True)[0][0]
        target_positions = _resolve_target_joint_positions(
            robot_controller,
            target_pose,
            current_joint_names,
            current_positions,
            current_tcp_pose,
            target_tcp_pose,
        )
        max_joint_delta = max(
            abs(target - current)
            for current, target in zip(current_positions, target_positions)
        )
        noop_delta = float(getattr(config, "PTP_NOOP_JOINT_DELTA_RAD", 0.001))
        if max_joint_delta <= noop_delta:
            robot_controller.get_logger().info(
                "[PTP] Target already reached in joint space "
                f"(max_joint_delta={max_joint_delta:.6f} rad <= {noop_delta:.6f} rad)"
            )
            _set_result(robot_controller, 0)
            return 0
        if orientation_locked:
            wrist_code, wrist_reason = _reject_same_orientation_wrist_flip(
                current_positions,
                target_positions,
            )
            if wrist_code != 0:
                robot_controller.get_logger().error(f"[PTP] {wrist_reason}")
                return wrist_code
        wrist_code, wrist_reason = _reject_excessive_wrist_rotation(
            current_positions,
            target_positions,
            orientation_locked=orientation_locked,
        )
        if wrist_code != 0:
            robot_controller.get_logger().error(f"[PTP] {wrist_reason}")
            return wrist_code
        joint_trajectory = _make_joint_trajectory(current_joint_names, current_positions, target_positions)
        validation_code, validation_reason = _validate_interpolated_states(
            robot_controller,
            joint_trajectory,
            start_pose,
            target_pose,
            orientation_locked=orientation_locked,
        )
        if validation_code != 0:
            robot_controller.get_logger().error(f"[PTP] Validation failed: {validation_reason}")
            return validation_code
    except TimeoutError as exc:
        robot_controller.get_logger().error(f"[PTP] Planner service unavailable: {exc}")
        return -2
    except RuntimeError as exc:
        robot_controller.get_logger().error(f"[PTP] {exc}")
        return -11
    except Exception as exc:
        robot_controller.get_logger().error(f"[PTP] Unexpected planning failure: {exc}")
        return -1

    robot_controller.get_logger().info(
        "[PTP] Planning joint-space PTP to "
        f"[{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}] "
        f"(orientation_locked={orientation_locked}, orientation_delta={orientation_delta_deg:.2f}°)"
    )

    moveit_trajectory = RobotTrajectory()
    moveit_trajectory.joint_trajectory = joint_trajectory
    generation = _begin_execution(robot_controller)
    _apply_time_param(
        robot_controller,
        moveit_trajectory,
        vel_scale,
        acc_scale,
        generation,
        log_prefix="[PTP]",
        trajectory_optimizer_name=trajectory_optimizer_name,
    )
    return 0
