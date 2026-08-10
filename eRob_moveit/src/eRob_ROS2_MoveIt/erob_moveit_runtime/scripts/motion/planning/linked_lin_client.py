#!/usr/bin/env python3
"""Python boundary for the C++ linked-LIN planning helper."""

from __future__ import annotations

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
            f"total_s={report.timings.get('helper_total_s', 0.0):.3f}"
        )
    else:
        robot_controller.get_logger().warning(
            "[LINKED_LIN] rejected "
            f"reason={report.failure_reason} index={report.first_failed_index} "
            f"{report.details}"
        )
    return LinkedLinPlanningResult(response.trajectory, report)


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
