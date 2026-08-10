#!/usr/bin/env python3
"""Python boundary for the C++ linked-LIN planning helper."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from time import perf_counter, sleep, time
from typing import Iterable

import config
from geometry_msgs.msg import Transform
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from utils.transformation_utils import TransformationUtils


@dataclass
class LinkedLinReport:
    requested_pose_count: int = 0
    solved_pose_count: int = 0
    first_failed_index: int | None = None
    failure_reason: str | None = None
    details: str = ""
    max_fk_position_error_mm: float = 0.0
    max_fk_orientation_error_deg: float = 0.0
    max_joint_step_rad: float = 0.0
    max_joint_span_rad: float = 0.0
    max_endpoint_delta_rad: float = 0.0
    removed_micro_reversal_points: int = 0
    removed_near_duplicate_points: int = 0
    timings: dict[str, float] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.failure_reason is None


@dataclass
class LinkedLinPlanningResult:
    trajectory: RobotTrajectory
    report: LinkedLinReport


def request_linked_lin_trajectory(
    robot_controller,
    poses: Iterable,
    *,
    seed_state=None,
    tool_transform=None,
    workobject_transform=None,
    use_workobject_transform: bool = False,
    vel_scaling: float | None = None,
    acc_scaling: float | None = None,
    service_timeout_s: float | None = None,
) -> LinkedLinPlanningResult | None:
    """Request one linked-LIN trajectory from the C++ helper.

    Returns ``None`` only when the feature is disabled. Once enabled, helper
    import, availability, timeout, or rejection problems are explicit failures
    so routing code cannot mask linked-LIN regressions with another planner.
    """

    if not bool(getattr(config, "LINKED_LIN_HELPER_ENABLED", False)):
        return None

    poses = list(poses or [])
    if not poses:
        return LinkedLinPlanningResult(
            RobotTrajectory(),
            LinkedLinReport(failure_reason="empty_pose_list"),
        )

    try:
        from erob_moveit_runtime.srv import ComputeLinkedLin
    except Exception as exc:
        message = f"failed to import service: {exc}"
        robot_controller.get_logger().error(
            f"[LINKED_LIN] helper unavailable: failed to import service: {exc}"
        )
        return LinkedLinPlanningResult(
            RobotTrajectory(),
            LinkedLinReport(
                requested_pose_count=len(poses),
                failure_reason="linked_lin_service_import_failed",
                details=message,
            ),
        )

    client = robot_controller.get_linked_lin_client()
    if client is None or not client.wait_for_service(timeout_sec=0.2):
        message = "helper service unavailable"
        robot_controller.get_logger().error(
            "[LINKED_LIN] helper service unavailable"
        )
        return LinkedLinPlanningResult(
            RobotTrajectory(),
            LinkedLinReport(
                requested_pose_count=len(poses),
                failure_reason="linked_lin_service_unavailable",
                details=message,
            ),
        )

    started_at = perf_counter()
    request = ComputeLinkedLin.Request()
    joint_names, seed_positions = _positions_in_config_order(
        robot_controller,
        seed_state=seed_state,
    )
    request.seed_state = _joint_state(robot_controller, joint_names, seed_positions)
    request.poses = poses
    request.group_name = str(getattr(config, "PLANNING_GROUP", "manipulator"))
    request.link_name = str(getattr(config, "EE_LINK", "ee_link"))
    request.reference_frame = str(getattr(config, "BASE_LINK", "base_link"))
    request.tool_transform = _transform_or_identity(tool_transform)
    request.workobject_transform = _transform_or_identity(workobject_transform)
    request.use_workobject_transform = bool(use_workobject_transform)
    request.max_velocity_scaling = _clamp_scale(
        vel_scaling,
        getattr(config, "DEFAULT_VEL_SCALING", 0.6),
    )
    request.max_acceleration_scaling = _clamp_scale(
        acc_scaling,
        getattr(config, "DEFAULT_ACC_SCALING", 0.4),
    )
    request.cartesian_step_m = float(getattr(config, "LINKED_LIN_CARTESIAN_STEP_M", 0.01))
    request.jump_threshold = float(getattr(config, "LINKED_LIN_JUMP_THRESHOLD", 0.0))
    request.avoid_collisions = bool(config.resolve_avoid_collisions(True))
    request.fk_position_tolerance_mm = float(
        getattr(config, "LINKED_LIN_FK_POSITION_TOL_MM", 0.15)
    )
    request.fk_orientation_tolerance_deg = float(
        getattr(config, "LINKED_LIN_FK_ORIENTATION_TOL_DEG", 0.25)
    )
    request.max_joint_step_rad = float(
        getattr(config, "LINKED_LIN_MAX_JOINT_STEP_RAD", 0.08)
    )
    request.max_joint_span_rad = float(
        getattr(config, "LINKED_LIN_MAX_JOINT_SPAN_RAD", 3.141592653589793)
    )
    request.max_endpoint_delta_rad = float(
        getattr(config, "LINKED_LIN_MAX_ENDPOINT_DELTA_RAD", 3.141592653589793)
    )
    request.full_turn_joint_names = [
        str(name)
        for name in getattr(config, "LINKED_LIN_FULL_TURN_JOINT_NAMES", []) or []
    ]
    request.full_turn_max_joint_span_rad = float(
        getattr(config, "LINKED_LIN_FULL_TURN_MAX_JOINT_SPAN_RAD", 6.6)
    )
    request.full_turn_max_endpoint_delta_rad = float(
        getattr(config, "LINKED_LIN_FULL_TURN_MAX_ENDPOINT_DELTA_RAD", 6.6)
    )
    request.micro_reversal_cleanup_enabled = bool(
        getattr(config, "LINKED_LIN_MICRO_REVERSAL_CLEANUP_ENABLED", True)
    )
    request.micro_reversal_min_angle_deg = float(
        getattr(config, "LINKED_LIN_MICRO_REVERSAL_MIN_ANGLE_DEG", 175.0)
    )
    request.micro_reversal_max_leg_rad = float(
        getattr(config, "LINKED_LIN_MICRO_REVERSAL_MAX_LEG_RAD", 0.002)
    )
    request.micro_reversal_max_endpoint_rad = float(
        getattr(config, "LINKED_LIN_MICRO_REVERSAL_MAX_ENDPOINT_RAD", 0.0001)
    )
    request.near_duplicate_rad = float(
        getattr(config, "LINKED_LIN_NEAR_DUPLICATE_RAD", 0.00001)
    )

    timeout_s = float(
        service_timeout_s
        if service_timeout_s is not None
        else getattr(config, "LINKED_LIN_SERVICE_TIMEOUT_S", 10.0)
    )
    response = _wait_future(
        client.call_async(request),
        timeout_s=timeout_s,
        description="linked LIN helper request",
    )

    # CartesianInterpolator can produce a short *closed* joint-space loop near
    # a singular/ill-conditioned configuration. The C++ cleaner deliberately
    # removes only tiny local A->B->A triplets. Repeated triplet removal can
    # expose the same returning loop at a wider scale (as seen by TOTG), so do
    # one bounded closure-aware pass on the returned joint trajectory.
    #
    # This is intentionally stricter than simply raising max_leg_rad: a loop is
    # removed only when it returns to essentially the same joint state, contains
    # a near-180deg reversal, and its entire excursion is at most 2x the normal
    # microscopic leg limit. The closing endpoint is removed with the loop so a
    # tiny A->A' segment is not left behind for TOTG.
    post_removed = 0
    if bool(getattr(response, "success", False)) and request.micro_reversal_cleanup_enabled:
        post_removed = _canonicalize_returning_micro_loops(
            response.trajectory,
            min_angle_deg=request.micro_reversal_min_angle_deg,
            max_leg_rad=request.micro_reversal_max_leg_rad,
            max_endpoint_rad=request.micro_reversal_max_endpoint_rad,
        )
        if post_removed:
            response.removed_micro_reversal_points = int(
                getattr(response, "removed_micro_reversal_points", 0)
            ) + post_removed
            robot_controller.get_logger().info(
                "[LINKED_LIN] collapsed closed micro-loop(s) after helper "
                f"removed_points={post_removed} "
                f"points={len(response.trajectory.joint_trajectory.points)} "
                f"max_excursion_rad={min(0.01, 2.0 * max(0.0, request.micro_reversal_max_leg_rad)):.6f} "
                f"max_endpoint_rad={max(0.0, request.micro_reversal_max_endpoint_rad):.6f}"
            )

    report = _report_from_response(response)
    report.timings["service_call_s"] = perf_counter() - started_at
    if report.ok:
        robot_controller.get_logger().info(
            "[LINKED_LIN] accepted "
            f"poses={report.requested_pose_count} solved={report.solved_pose_count} "
            f"points={len(response.trajectory.joint_trajectory.points)} "
            f"fk_max_mm={report.max_fk_position_error_mm:.4f} "
            f"ori_max_deg={report.max_fk_orientation_error_deg:.4f} "
            f"max_joint_step={report.max_joint_step_rad:.4f} "
            f"cleanup_rev={report.removed_micro_reversal_points} "
            f"cleanup_dup={report.removed_near_duplicate_points} "
            f"total_s={report.timings.get('helper_total_s', 0.0):.3f}"
        )
    else:
        robot_controller.get_logger().warning(
            "[LINKED_LIN] rejected "
            f"reason={report.failure_reason} index={report.first_failed_index} "
            f"{report.details}"
        )
    return LinkedLinPlanningResult(response.trajectory, report)


def _joint_distance(a, b) -> float:
    return math.sqrt(
        sum((float(y) - float(x)) ** 2 for x, y in zip(a, b))
    )


def _direction_cosine(previous, middle, next_) -> float:
    left = [float(m) - float(p) for p, m in zip(previous, middle)]
    right = [float(n) - float(m) for m, n in zip(middle, next_)]
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    if left_norm <= 1e-12 or right_norm <= 1e-12:
        return 1.0
    return max(
        -1.0,
        min(
            1.0,
            sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm),
        ),
    )


def _canonicalize_returning_micro_loops(
    trajectory,
    *,
    min_angle_deg: float,
    max_leg_rad: float,
    max_endpoint_rad: float,
) -> int:
    """Excise bounded closed joint loops left by CartesianInterpolator.

    Unlike the C++ triplet cleaner this operates on a short window. That lets
    it remove a multi-sample A->...->A' excursion in one operation instead of
    repeatedly peeling the center and exposing a progressively wider 180-degree
    turn to TOTG.
    """

    points = list(getattr(trajectory.joint_trajectory, "points", []) or [])
    if len(points) < 3:
        return 0

    max_leg_rad = max(0.0, float(max_leg_rad))
    max_endpoint_rad = max(0.0, float(max_endpoint_rad))
    if max_leg_rad <= 0.0 or max_endpoint_rad <= 0.0:
        return 0

    # The wider allowance is valid only for a *closed* loop. Keep a hard cap so
    # a bad configuration value can never turn this into generic path pruning.
    max_excursion_rad = min(0.01, 2.0 * max_leg_rad)
    max_cosine = math.cos(
        math.radians(max(90.0, min(180.0, float(min_angle_deg))))
    )
    max_window_span = 7
    removed_points = 0

    changed = True
    while changed and len(points) >= 3:
        changed = False
        for start in range(0, len(points) - 2):
            start_q = list(points[start].positions)
            furthest_end = min(len(points) - 1, start + max_window_span)

            # Prefer the widest closed window so an entire nested loop is
            # removed at once rather than exposing the next outer triplet.
            for end in range(furthest_end, start + 1, -1):
                end_q = list(points[end].positions)
                endpoint_norm = _joint_distance(start_q, end_q)
                if endpoint_norm > max_endpoint_rad:
                    continue

                interior = range(start + 1, end)
                max_excursion = max(
                    (_joint_distance(start_q, points[index].positions) for index in interior),
                    default=0.0,
                )
                if max_excursion <= max_leg_rad or max_excursion > max_excursion_rad:
                    continue

                strongest_reversal_cosine = 1.0
                for middle in range(start + 1, end):
                    strongest_reversal_cosine = min(
                        strongest_reversal_cosine,
                        _direction_cosine(
                            points[middle - 1].positions,
                            points[middle].positions,
                            points[middle + 1].positions,
                        ),
                    )
                if strongest_reversal_cosine > max_cosine:
                    continue

                travelled = sum(
                    _joint_distance(points[index - 1].positions, points[index].positions)
                    for index in range(start + 1, end + 1)
                )
                if travelled < 1.5 * max_excursion:
                    continue

                # Remove the complete returning excursion including its closing
                # A' point. Keeping both A and A' would leave an almost-zero
                # segment whose tangent can itself upset TOTG.
                removed_points += end - start
                del points[start + 1 : end + 1]
                changed = True
                break

            if changed:
                break

    if removed_points:
        trajectory.joint_trajectory.points = points
    return removed_points


def _report_from_response(response) -> LinkedLinReport:
    failed_index = int(getattr(response, "failed_index", 0))
    requested_pose_count = int(getattr(response, "requested_pose_count", 0))
    report = LinkedLinReport(
        requested_pose_count=requested_pose_count,
        solved_pose_count=int(getattr(response, "solved_pose_count", 0)),
        max_fk_position_error_mm=float(getattr(response, "max_fk_position_error_mm", 0.0)),
        max_fk_orientation_error_deg=float(getattr(response, "max_fk_orientation_error_deg", 0.0)),
        max_joint_step_rad=float(getattr(response, "max_joint_step_rad", 0.0)),
        max_joint_span_rad=float(getattr(response, "max_joint_span_rad", 0.0)),
        max_endpoint_delta_rad=float(getattr(response, "max_endpoint_delta_rad", 0.0)),
        removed_micro_reversal_points=int(
            getattr(response, "removed_micro_reversal_points", 0)
        ),
        removed_near_duplicate_points=int(
            getattr(response, "removed_near_duplicate_points", 0)
        ),
    )
    report.timings["helper_planning_s"] = float(getattr(response, "planning_time_s", 0.0))
    report.timings["helper_validation_s"] = float(getattr(response, "validation_time_s", 0.0))
    report.timings["helper_total_s"] = float(getattr(response, "total_time_s", 0.0))
    if not bool(getattr(response, "success", False)):
        report.failure_reason = "linked_lin_failed"
        if 0 <= failed_index < requested_pose_count:
            report.first_failed_index = failed_index
        report.details = (
            f"error_code={getattr(response, 'error_code', None)} "
            f"message={getattr(response, 'message', '')}"
        )
    return report


def _positions_in_config_order(robot_controller, seed_state=None):
    state = getattr(seed_state, "joint_state", seed_state)
    if state is None:
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


def _joint_state(robot_controller, joint_names, positions):
    state = JointState()
    state.header.stamp = robot_controller.get_clock().now().to_msg()
    state.name = list(joint_names)
    state.position = [float(value) for value in positions]
    return state


def _transform_or_identity(transform):
    if isinstance(transform, Transform):
        return transform
    if transform is not None:
        return _matrix_to_transform_msg(transform)
    identity = Transform()
    identity.rotation.w = 1.0
    return identity


def _matrix_to_transform_msg(transform):
    message = Transform()
    message.translation.x = float(transform[0, 3])
    message.translation.y = float(transform[1, 3])
    message.translation.z = float(transform[2, 3])
    quat = TransformationUtils.matrix_to_quaternion(transform[:3, :3])
    message.rotation.x = float(quat[0])
    message.rotation.y = float(quat[1])
    message.rotation.z = float(quat[2])
    message.rotation.w = float(quat[3])
    return message


def _clamp_scale(value, default) -> float:
    if value is None:
        value = default
    return max(0.0, min(1.0, float(value)))


def _wait_future(future, timeout_s: float, description: str):
    deadline = time() + float(timeout_s)
    while time() < deadline:
        if future.done():
            return future.result()
        sleep(0.001)
    raise TimeoutError(f"{description} timed out after {timeout_s:.3f}s")
