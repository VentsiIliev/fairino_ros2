"""Direct tracking-IK prototype for dense Cartesian contour execution."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from time import perf_counter, time, sleep
from typing import Optional

import numpy as np

import config
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .trajectory_planner import _apply_time_param
from .planner_utils import _begin_execution


@dataclass
class ContourValidationReport:
    points: int = 0
    fast_failures: int = 0
    retries: int = 0
    max_fk_position_error_mm: float = 0.0
    max_fk_orientation_error_deg: float = 0.0
    max_joint_step_rad: float = 0.0
    max_joint_span_rad: float = 0.0
    max_endpoint_delta_rad: float = 0.0
    state_validity_checks: int = 0
    first_failed_index: Optional[int] = None
    failure_reason: Optional[str] = None
    details: str = ""
    timings: dict[str, float] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.failure_reason is None


@dataclass
class ContourIkExecution:
    trajectory: RobotTrajectory
    report: ContourValidationReport


def maybe_execute_direct_contour_ik(
    robot_controller,
    waypoints_6d,
    poses,
    total_dist_mm: float,
    vel_scaling: float,
    acc_scaling: float,
    trajectory_optimizer_name=None,
):
    """Attempt direct contour IK, returning True on dispatched execution.

    Returns:
        True: direct IK produced and dispatched a trajectory.
        False: direct IK was disabled, skipped, or rejected; caller should use
               the existing MoveIt Cartesian path fallback.
    """
    started_at = perf_counter()
    if not _direct_ik_should_run(robot_controller, waypoints_6d, total_dist_mm):
        return False

    try:
        result = _build_direct_contour_trajectory(robot_controller, poses)
    except Exception as exc:
        robot_controller.get_logger().warning(
            f"[CONTOUR_IK] fallback reason=exception details={exc}"
        )
        return False

    result.report.timings["total_before_optimizer_s"] = perf_counter() - started_at
    _log_report(robot_controller, result.report)
    if not result.report.ok:
        return False

    generation = _begin_execution(robot_controller)
    robot_controller._last_cartesian_request_kind = "direct_contour_ik"
    robot_controller._last_cartesian_request_waypoints = len(poses)
    robot_controller._last_cartesian_request_started_at = started_at
    _apply_time_param(
        robot_controller,
        result.trajectory,
        vel_scaling,
        acc_scaling,
        generation,
        log_prefix="[CONTOUR_IK]",
        trajectory_optimizer_name=trajectory_optimizer_name,
    )
    return True


def _direct_ik_should_run(robot_controller, waypoints_6d, total_dist_mm: float) -> bool:
    if not bool(getattr(config, "CONTOUR_DIRECT_IK_ENABLED", False)):
        return False
    if len(waypoints_6d) < int(getattr(config, "CONTOUR_DIRECT_IK_MIN_POINTS", 50)):
        return False
    if float(total_dist_mm) < float(getattr(config, "CONTOUR_DIRECT_IK_MIN_TOTAL_LENGTH_MM", 20.0)):
        return False
    if getattr(robot_controller, "current_joint_state", None) is None:
        robot_controller.get_logger().warning(
            "[CONTOUR_IK] fallback reason=no_current_joint_state"
        )
        return False
    return True


def _build_direct_contour_trajectory(robot_controller, poses) -> ContourIkExecution:
    if bool(getattr(config, "CONTOUR_BATCH_IK_ENABLED", True)):
        batched = _build_batched_contour_trajectory(robot_controller, poses)
        if batched is not None:
            return batched

    report = ContourValidationReport(points=len(poses))
    solve_started_at = perf_counter()
    joint_names, seed_positions = _current_positions_in_config_order(robot_controller)
    initial_positions = list(seed_positions)
    seed_state = _joint_state(robot_controller, joint_names, seed_positions)

    solved_points = []
    for index, pose in enumerate(poses):
        response, fast_failed, retry_count = _solve_pose_with_retries(
            robot_controller,
            pose,
            seed_state,
        )
        if fast_failed:
            report.fast_failures += 1
        report.retries += retry_count

        if not _moveit_success(getattr(response, "error_code", None)):
            report.failure_reason = "ik_failed"
            report.first_failed_index = index
            report.details = f"error_code={getattr(getattr(response, 'error_code', None), 'val', None)}"
            return ContourIkExecution(_empty_trajectory(joint_names), report)

        solution_positions = _positions_from_solution(response.solution.joint_state, joint_names)
        if solution_positions is None:
            report.failure_reason = "ik_solution_joint_mismatch"
            report.first_failed_index = index
            report.details = "IK response did not contain all configured joints"
            return ContourIkExecution(_empty_trajectory(joint_names), report)

        normalized = _normalize_joint_branch(seed_positions, solution_positions)
        solved_points.append(normalized)
        seed_positions = normalized
        seed_state = _joint_state(robot_controller, joint_names, seed_positions)

    report.timings["solve_s"] = perf_counter() - solve_started_at

    validate_started_at = perf_counter()
    _validate_joint_continuity(solved_points, report, initial_positions=initial_positions)
    if report.ok:
        _validate_fk_accuracy(robot_controller, poses, joint_names, solved_points, report)
    if report.ok:
        _validate_sampled_state_validity(robot_controller, joint_names, solved_points, report)
    report.timings["validate_s"] = perf_counter() - validate_started_at

    trajectory = _trajectory_from_points(joint_names, solved_points)
    return ContourIkExecution(trajectory, report)


def _build_batched_contour_trajectory(robot_controller, poses):
    try:
        from erob_moveit_runtime.srv import ComputeContourIK
    except Exception as exc:
        robot_controller.get_logger().warning(
            f"[CONTOUR_IK] batch helper unavailable: failed to import service: {exc}"
        )
        return None

    client = robot_controller.get_contour_ik_client()
    if client is None or not client.wait_for_service(timeout_sec=0.2):
        robot_controller.get_logger().warning(
            "[CONTOUR_IK] batch helper service unavailable; using per-point IK prototype"
        )
        return None

    joint_names, seed_positions = _current_positions_in_config_order(robot_controller)
    request = ComputeContourIK.Request()
    request.seed_state = _joint_state(robot_controller, joint_names, seed_positions)
    request.poses = list(poses)
    request.group_name = config.PLANNING_GROUP
    request.link_name = config.EE_LINK
    request.timeout_s = float(getattr(config, "CONTOUR_IK_TIMEOUT_S", 0.003))
    request.retry_timeout_s = float(getattr(config, "CONTOUR_IK_RETRY_TIMEOUT_S", 0.02))
    request.fk_position_tolerance_mm = float(getattr(config, "CONTOUR_IK_FK_POSITION_TOL_MM", 0.15))
    request.fk_orientation_tolerance_deg = float(getattr(config, "CONTOUR_IK_FK_ORIENTATION_TOL_DEG", 0.25))
    request.max_joint_step_rad = float(getattr(config, "CONTOUR_IK_MAX_JOINT_STEP_RAD", 0.08))
    request.max_joint_span_rad = float(getattr(config, "CONTOUR_IK_MAX_JOINT_SPAN_RAD", np.pi))
    request.max_endpoint_delta_rad = float(getattr(config, "CONTOUR_IK_MAX_ENDPOINT_DELTA_RAD", np.pi))
    request.full_turn_joint_names = [
        str(name)
        for name in getattr(config, "CONTOUR_IK_FULL_TURN_JOINT_NAMES", []) or []
    ]
    request.full_turn_max_joint_span_rad = float(
        getattr(config, "CONTOUR_IK_FULL_TURN_MAX_JOINT_SPAN_RAD", 6.6)
    )
    request.full_turn_max_endpoint_delta_rad = float(
        getattr(config, "CONTOUR_IK_FULL_TURN_MAX_ENDPOINT_DELTA_RAD", 6.6)
    )
    request.smoothing_enabled = bool(getattr(config, "CONTOUR_IK_SMOOTHING_ENABLED", True))
    request.smoothing_iterations = max(
        0,
        int(getattr(config, "CONTOUR_IK_SMOOTHING_ITERATIONS", 2)),
    )
    request.smoothing_alpha = float(getattr(config, "CONTOUR_IK_SMOOTHING_ALPHA", 0.35))
    request.smoothing_fk_position_tolerance_mm = float(
        getattr(config, "CONTOUR_IK_SMOOTHING_FK_POSITION_TOL_MM", 0.05)
    )
    request.smoothing_fk_orientation_tolerance_deg = float(
        getattr(config, "CONTOUR_IK_SMOOTHING_FK_ORIENTATION_TOL_DEG", 0.10)
    )

    timeout_s = float(getattr(config, "CONTOUR_BATCH_IK_SERVICE_TIMEOUT_S", 10.0))
    response = _wait_future(
        client.call_async(request),
        timeout_s=timeout_s,
        description="batch contour IK request",
    )

    report = ContourValidationReport(
        points=len(poses),
        fast_failures=int(getattr(response, "fast_failures", 0)),
        retries=int(getattr(response, "retries", 0)),
        max_fk_position_error_mm=float(getattr(response, "max_fk_position_error_mm", 0.0)),
        max_fk_orientation_error_deg=float(getattr(response, "max_fk_orientation_error_deg", 0.0)),
        max_joint_step_rad=float(getattr(response, "max_joint_step_rad", 0.0)),
        max_joint_span_rad=float(getattr(response, "max_joint_span_rad", 0.0)),
        max_endpoint_delta_rad=float(getattr(response, "max_endpoint_delta_rad", 0.0)),
    )
    report.timings["batch_solve_s"] = float(getattr(response, "solve_time_s", 0.0))
    if not bool(getattr(response, "success", False)):
        report.failure_reason = "batch_ik_failed"
        failed_index = int(getattr(response, "failed_index", 0))
        if failed_index >= 0 and failed_index < len(poses):
            report.first_failed_index = failed_index
        report.details = (
            f"error_code={getattr(response, 'error_code', None)} "
            f"message={getattr(response, 'message', '')}"
        )
        return ContourIkExecution(response.trajectory, report)

    if report.ok:
        _validate_sampled_state_validity(
            robot_controller,
            list(response.trajectory.joint_trajectory.joint_names),
            [list(point.positions) for point in response.trajectory.joint_trajectory.points],
            report,
        )
    return ContourIkExecution(response.trajectory, report)


def _solve_pose_with_retries(robot_controller, pose, seed_state):
    fast_timeout = float(getattr(config, "CONTOUR_IK_TIMEOUT_S", 0.003))
    fast_attempts = max(1, int(getattr(config, "CONTOUR_IK_ATTEMPTS", 1)))
    retry_timeout = float(getattr(config, "CONTOUR_IK_RETRY_TIMEOUT_S", 0.02))
    retry_attempts = max(0, int(getattr(config, "CONTOUR_IK_RETRY_ATTEMPTS", 3)))

    last_response = None
    for _ in range(fast_attempts):
        last_response = _request_ik(robot_controller, pose, seed_state, fast_timeout)
        if _moveit_success(getattr(last_response, "error_code", None)):
            return last_response, False, 0

    retries = 0
    for _ in range(retry_attempts):
        retries += 1
        last_response = _request_ik(robot_controller, pose, seed_state, retry_timeout)
        if _moveit_success(getattr(last_response, "error_code", None)):
            return last_response, True, retries

    return last_response, True, retries


def _request_ik(robot_controller, pose, seed_state, timeout_s: float):
    client = robot_controller.get_ik_client()
    if client is None or not client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("MoveIt IK service unavailable")

    req = GetPositionIK.Request()
    req.ik_request.group_name = config.PLANNING_GROUP
    req.ik_request.ik_link_name = config.EE_LINK
    req.ik_request.pose_stamped.header.frame_id = config.BASE_LINK
    req.ik_request.pose_stamped.header.stamp = robot_controller.get_clock().now().to_msg()
    req.ik_request.pose_stamped.pose = pose
    req.ik_request.avoid_collisions = False
    req.ik_request.timeout.sec = int(timeout_s)
    req.ik_request.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1_000_000_000)
    req.ik_request.robot_state.joint_state = deepcopy(seed_state)
    req.ik_request.robot_state.is_diff = False
    return _wait_future(
        client.call_async(req),
        timeout_s=max(0.25, timeout_s + 0.2),
        description="MoveIt contour IK request",
    )


def _validate_fk_accuracy(robot_controller, poses, joint_names, solved_points, report):
    position_tol_mm = float(getattr(config, "CONTOUR_IK_FK_POSITION_TOL_MM", 0.15))
    orientation_tol_deg = float(getattr(config, "CONTOUR_IK_FK_ORIENTATION_TOL_DEG", 0.25))
    client = robot_controller.get_fk_client()
    if client is None or not client.wait_for_service(timeout_sec=1.0):
        report.failure_reason = "fk_service_unavailable"
        return

    for index, (pose, positions) in enumerate(zip(poses, solved_points)):
        req = GetPositionFK.Request()
        req.header.frame_id = config.BASE_LINK
        req.fk_link_names = [config.EE_LINK]
        req.robot_state.joint_state = _joint_state(robot_controller, joint_names, positions)
        req.robot_state.is_diff = False
        response = _wait_future(
            client.call_async(req),
            timeout_s=2.0,
            description="MoveIt contour FK request",
        )
        if not _moveit_success(getattr(response, "error_code", None)) or not response.pose_stamped:
            report.failure_reason = "fk_failed"
            report.first_failed_index = index
            report.details = f"error_code={getattr(getattr(response, 'error_code', None), 'val', None)}"
            return

        fk_pose = response.pose_stamped[0].pose
        position_error_mm = _position_error_mm(pose, fk_pose)
        orientation_error_deg = _orientation_error_deg(pose, fk_pose)
        report.max_fk_position_error_mm = max(report.max_fk_position_error_mm, position_error_mm)
        report.max_fk_orientation_error_deg = max(report.max_fk_orientation_error_deg, orientation_error_deg)
        if position_error_mm > position_tol_mm or orientation_error_deg > orientation_tol_deg:
            report.failure_reason = "fk_error"
            report.first_failed_index = index
            report.details = (
                f"position_error_mm={position_error_mm:.4f} "
                f"orientation_error_deg={orientation_error_deg:.4f}"
            )
            return


def _validate_joint_continuity(solved_points, report, initial_positions=None):
    if not solved_points:
        report.failure_reason = "empty_trajectory"
        return
    max_step_allowed = float(getattr(config, "CONTOUR_IK_MAX_JOINT_STEP_RAD", 0.08))
    max_span_allowed = float(getattr(config, "CONTOUR_IK_MAX_JOINT_SPAN_RAD", np.pi))
    max_endpoint_delta_allowed = float(getattr(config, "CONTOUR_IK_MAX_ENDPOINT_DELTA_RAD", np.pi))
    points_array = np.asarray(solved_points, dtype=float)
    joint_spans = np.max(points_array, axis=0) - np.min(points_array, axis=0)
    endpoint_deltas = np.abs(points_array[-1] - points_array[0])
    report.max_joint_span_rad = float(np.max(joint_spans))
    report.max_endpoint_delta_rad = float(np.max(endpoint_deltas))
    if report.max_joint_span_rad > max_span_allowed:
        joint_index = int(np.argmax(joint_spans))
        report.failure_reason = "joint_span_exceeded"
        report.first_failed_index = None
        report.details = (
            f"joint_index={joint_index} span_rad={report.max_joint_span_rad:.4f} "
            f"limit_rad={max_span_allowed:.4f}"
        )
        return
    if report.max_endpoint_delta_rad > max_endpoint_delta_allowed:
        joint_index = int(np.argmax(endpoint_deltas))
        report.failure_reason = "endpoint_delta_exceeded"
        report.first_failed_index = None
        report.details = (
            f"joint_index={joint_index} endpoint_delta_rad={report.max_endpoint_delta_rad:.4f} "
            f"limit_rad={max_endpoint_delta_allowed:.4f}"
        )
        return
    for index, positions in enumerate(solved_points):
        if not np.all(np.isfinite(np.asarray(positions, dtype=float))):
            report.failure_reason = "non_finite_joint_value"
            report.first_failed_index = index
            return
        reference = initial_positions if index == 0 else solved_points[index - 1]
        if reference is None:
            continue
        step = float(np.max(np.abs(np.asarray(positions) - np.asarray(reference))))
        report.max_joint_step_rad = max(report.max_joint_step_rad, step)
        if step > max_step_allowed:
            report.failure_reason = "joint_step_exceeded"
            report.first_failed_index = index
            report.details = f"max_step_rad={step:.4f} limit_rad={max_step_allowed:.4f}"
            return


def _validate_sampled_state_validity(robot_controller, joint_names, solved_points, report):
    if not bool(getattr(config, "CONTOUR_STATE_VALIDITY_ENABLED", False)):
        return
    client = robot_controller.get_state_validity_client()
    if client is None or not client.wait_for_service(timeout_sec=1.0):
        report.failure_reason = "state_validity_service_unavailable"
        return

    stride = max(1, int(getattr(config, "CONTOUR_STATE_VALIDITY_STRIDE", 10)))
    indexes = set(range(0, len(solved_points), stride))
    indexes.add(0)
    indexes.add(len(solved_points) - 1)
    for index in sorted(indexes):
        req = GetStateValidity.Request()
        req.robot_state.joint_state = _joint_state(robot_controller, joint_names, solved_points[index])
        req.robot_state.is_diff = False
        req.group_name = config.PLANNING_GROUP
        response = _wait_future(
            client.call_async(req),
            timeout_s=2.0,
            description="MoveIt contour state-validity request",
        )
        report.state_validity_checks += 1
        if not bool(getattr(response, "valid", False)):
            report.failure_reason = "state_validity_failed"
            report.first_failed_index = index
            return


def _current_positions_in_config_order(robot_controller):
    state = robot_controller.current_joint_state
    names = list(getattr(state, "name", []) or [])
    positions = list(getattr(state, "position", []) or [])
    if not names or len(names) != len(positions):
        raise RuntimeError("current joint state is incomplete")
    by_name = dict(zip(names, positions))
    joint_names = list(config.JOINT_NAMES)
    missing = [name for name in joint_names if name not in by_name]
    if missing:
        raise RuntimeError(f"current joint state missing joints: {missing}")
    return joint_names, [float(by_name[name]) for name in joint_names]


def _positions_from_solution(joint_state, joint_names):
    names = list(getattr(joint_state, "name", []) or [])
    positions = list(getattr(joint_state, "position", []) or [])
    if not names or len(names) != len(positions):
        return None
    by_name = dict(zip(names, positions))
    if any(name not in by_name for name in joint_names):
        return None
    return [float(by_name[name]) for name in joint_names]


def _joint_state(robot_controller, joint_names, positions):
    state = JointState()
    state.header.stamp = robot_controller.get_clock().now().to_msg()
    state.name = list(joint_names)
    state.position = [float(value) for value in positions]
    return state


def _trajectory_from_points(joint_names, solved_points):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory = JointTrajectory()
    trajectory.joint_trajectory.joint_names = list(joint_names)
    trajectory.joint_trajectory.points = []
    for positions in solved_points:
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        trajectory.joint_trajectory.points.append(point)
    return trajectory


def _empty_trajectory(joint_names):
    return _trajectory_from_points(joint_names, [])


def _normalize_joint_branch(reference_positions, target_positions):
    return [
        _nearest_equivalent_angle(reference, target)
        for reference, target in zip(reference_positions, target_positions)
    ]


def _nearest_equivalent_angle(reference, value):
    adjusted = float(value)
    ref = float(reference)
    two_pi = 2.0 * np.pi
    while adjusted - ref > np.pi:
        adjusted -= two_pi
    while adjusted - ref < -np.pi:
        adjusted += two_pi
    return adjusted


def _moveit_success(error_code) -> bool:
    return getattr(error_code, "val", None) == MoveItErrorCodes.SUCCESS


def _position_error_mm(expected, actual) -> float:
    expected_xyz = np.asarray([
        expected.position.x,
        expected.position.y,
        expected.position.z,
    ], dtype=float)
    actual_xyz = np.asarray([
        actual.position.x,
        actual.position.y,
        actual.position.z,
    ], dtype=float)
    return float(np.linalg.norm(expected_xyz - actual_xyz) * 1000.0)


def _orientation_error_deg(expected, actual) -> float:
    expected_quat = np.asarray([
        expected.orientation.x,
        expected.orientation.y,
        expected.orientation.z,
        expected.orientation.w,
    ], dtype=float)
    actual_quat = np.asarray([
        actual.orientation.x,
        actual.orientation.y,
        actual.orientation.z,
        actual.orientation.w,
    ], dtype=float)
    norm_product = np.linalg.norm(expected_quat) * np.linalg.norm(actual_quat)
    if norm_product <= 1e-12:
        return 180.0
    dot = abs(float(np.dot(expected_quat, actual_quat) / norm_product))
    dot = float(np.clip(dot, -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(dot)))


def _wait_future(future, timeout_s: float, description: str):
    deadline = time() + float(timeout_s)
    while time() < deadline:
        if future.done():
            return future.result()
        sleep(0.001)
    raise TimeoutError(f"{description} timed out after {timeout_s:.3f}s")


def _log_report(robot_controller, report: ContourValidationReport):
    if report.ok:
        robot_controller.get_logger().info(
            "[CONTOUR_IK] accepted "
            f"points={report.points} fast_failures={report.fast_failures} retries={report.retries} "
            f"fk_max_mm={report.max_fk_position_error_mm:.4f} "
            f"ori_max_deg={report.max_fk_orientation_error_deg:.4f} "
            f"max_joint_step={report.max_joint_step_rad:.4f} "
            f"max_joint_span={report.max_joint_span_rad:.4f} "
            f"max_endpoint_delta={report.max_endpoint_delta_rad:.4f} "
            f"state_validity_checks={report.state_validity_checks}"
        )
    else:
        robot_controller.get_logger().warning(
            "[CONTOUR_IK] fallback "
            f"reason={report.failure_reason} index={report.first_failed_index} {report.details}"
        )
    timings = " ".join(f"{name}={value:.3f}" for name, value in sorted(report.timings.items()))
    if timings:
        robot_controller.get_logger().info(f"[TIMING] contour_ik {timings}")
