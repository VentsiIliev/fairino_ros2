#!/usr/bin/env python3
"""Python boundary for the C++ linked-LIN planning helper.

The helper plans a contiguous LIN run in one C++ request, but it preserves the
logical segment boundaries. Python keeps ownership of blending and trajectory
time parameterization so linked-LIN is a performance optimization rather than a
change in motion semantics.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from time import perf_counter, sleep, time
from typing import Iterable, Sequence

import config
from geometry_msgs.msg import Pose, Transform
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from utils.transformation_utils import TransformationUtils


@dataclass
class LinkedLinReport:
    requested_segment_count: int = 0
    solved_segment_count: int = 0
    first_failed_segment: int | None = None
    failure_reason: str | None = None
    details: str = ""
    segment_boundary_indices: list[int] = field(default_factory=list)
    segment_point_counts: list[int] = field(default_factory=list)
    segment_planning_time_s: list[float] = field(default_factory=list)
    max_fk_position_error_mm: float = 0.0
    max_fk_orientation_error_deg: float = 0.0
    max_joint_step_rad: float = 0.0
    max_joint_span_rad: float = 0.0
    max_endpoint_delta_rad: float = 0.0
    timings: dict[str, float] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.failure_reason is None

    # Compatibility aliases for older logging/callers.
    @property
    def requested_pose_count(self) -> int:
        return self.requested_segment_count

    @property
    def solved_pose_count(self) -> int:
        return self.solved_segment_count

    @property
    def first_failed_index(self) -> int | None:
        return self.first_failed_segment


@dataclass
class LinkedLinPlanningResult:
    trajectory: RobotTrajectory
    segment_trajectories: list[JointTrajectory]
    report: LinkedLinReport


def request_linked_lin_trajectory(
    robot_controller,
    targets: Iterable,
    *,
    labels: Sequence[str] | None = None,
    velocities: Sequence[float] | None = None,
    accelerations: Sequence[float] | None = None,
    blend_radii: Sequence[float] | None = None,
    seed_state=None,
    tool_transform=None,
    workobject_transform=None,
    use_workobject_transform: bool = False,
    service_timeout_s: float | None = None,
) -> LinkedLinPlanningResult | None:
    """Plan several logical LIN segments in one C++ service request.

    ``targets`` contains one TCP target per logical LIN segment. Each target may
    be either a geometry_msgs/Pose or the existing six-value Cartesian format
    [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]. The returned raw trajectory is
    split back into one JointTrajectory per logical segment using the C++
    boundary indices. No blending or cleanup is done here.
    """

    if not bool(getattr(config, "LINKED_LIN_HELPER_ENABLED", False)):
        return None

    targets = list(targets or [])
    count = len(targets)
    if count == 0:
        return LinkedLinPlanningResult(
            RobotTrajectory(),
            [],
            LinkedLinReport(failure_reason="empty_target_list"),
        )

    labels = _normalized_strings(labels, count, "")
    velocities = _normalized_scales(
        velocities,
        count,
        getattr(config, "DEFAULT_VEL_SCALING", 0.6),
    )
    accelerations = _normalized_scales(
        accelerations,
        count,
        getattr(config, "DEFAULT_ACC_SCALING", 0.4),
    )
    blend_radii = _normalized_floats(blend_radii, count, 0.0)
    target_poses = [_target_to_pose(target) for target in targets]

    try:
        from erob_moveit_runtime.srv import ComputeLinkedLin
    except Exception as exc:
        message = f"failed to import service: {exc}"
        robot_controller.get_logger().error(f"[LINKED_LIN] helper unavailable: {message}")
        return LinkedLinPlanningResult(
            RobotTrajectory(),
            [],
            LinkedLinReport(
                requested_segment_count=count,
                failure_reason="linked_lin_service_import_failed",
                details=message,
            ),
        )

    client = robot_controller.get_linked_lin_client()
    if client is None or not client.wait_for_service(timeout_sec=0.2):
        message = "helper service unavailable"
        robot_controller.get_logger().error(f"[LINKED_LIN] {message}")
        return LinkedLinPlanningResult(
            RobotTrajectory(),
            [],
            LinkedLinReport(
                requested_segment_count=count,
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
    request.target_poses = target_poses
    request.labels = list(labels)
    request.velocities = list(velocities)
    request.accelerations = list(accelerations)
    request.blend_radii = list(blend_radii)
    request.group_name = str(getattr(config, "PLANNING_GROUP", "manipulator"))
    request.link_name = str(getattr(config, "EE_LINK", "ee_link"))
    request.reference_frame = str(getattr(config, "BASE_LINK", "base_link"))
    request.tool_transform = _transform_or_identity(tool_transform)
    request.workobject_transform = _transform_or_identity(workobject_transform)
    request.use_workobject_transform = bool(use_workobject_transform)
    request.cartesian_step_m = float(
        getattr(config, "LINKED_LIN_CARTESIAN_STEP_M", 0.01)
    )
    request.jump_threshold = float(
        getattr(config, "LINKED_LIN_JUMP_THRESHOLD", 0.0)
    )
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
    segment_trajectories = (
        _split_segment_trajectories(response.trajectory, report.segment_point_counts)
        if report.ok
        else []
    )

    if report.ok and len(segment_trajectories) != count:
        report.failure_reason = "linked_lin_boundary_mismatch"
        report.details = (
            f"expected {count} segment trajectories, got {len(segment_trajectories)}; "
            f"counts={report.segment_point_counts} "
            f"boundaries={report.segment_boundary_indices}"
        )
        segment_trajectories = []

    if report.ok:
        robot_controller.get_logger().info(
            "[LINKED_LIN] accepted "
            f"segments={report.requested_segment_count} "
            f"points={len(response.trajectory.joint_trajectory.points)} "
            f"segment_points={report.segment_point_counts} "
            f"boundaries={report.segment_boundary_indices} "
            f"max_joint_step={report.max_joint_step_rad:.4f} "
            f"total_s={report.timings.get('helper_total_s', 0.0):.3f}"
        )
    else:
        robot_controller.get_logger().warning(
            "[LINKED_LIN] rejected "
            f"reason={report.failure_reason} "
            f"segment={report.first_failed_segment} {report.details}"
        )

    return LinkedLinPlanningResult(
        response.trajectory,
        segment_trajectories,
        report,
    )


def _split_segment_trajectories(
    trajectory: RobotTrajectory,
    segment_point_counts: Sequence[int],
) -> list[JointTrajectory]:
    source = trajectory.joint_trajectory
    points = list(getattr(source, "points", []) or [])
    result: list[JointTrajectory] = []
    offset = 0

    for raw_count in segment_point_counts:
        count = int(raw_count)
        if count <= 0 or offset + count > len(points):
            return []
        segment = JointTrajectory()
        segment.header = deepcopy(source.header)
        segment.joint_names = list(source.joint_names)
        segment.points = [deepcopy(point) for point in points[offset : offset + count]]
        result.append(segment)
        offset += count

    if offset != len(points):
        return []
    return result


def _report_from_response(response) -> LinkedLinReport:
    requested = int(
        getattr(
            response,
            "requested_segment_count",
            getattr(response, "requested_pose_count", 0),
        )
    )
    solved = int(
        getattr(
            response,
            "solved_segment_count",
            getattr(response, "solved_pose_count", 0),
        )
    )
    failed = int(
        getattr(
            response,
            "failed_segment_index",
            getattr(response, "failed_index", 0),
        )
    )

    report = LinkedLinReport(
        requested_segment_count=requested,
        solved_segment_count=solved,
        segment_boundary_indices=[
            int(value) for value in getattr(response, "segment_boundary_indices", [])
        ],
        segment_point_counts=[
            int(value) for value in getattr(response, "segment_point_counts", [])
        ],
        segment_planning_time_s=[
            float(value) for value in getattr(response, "segment_planning_time_s", [])
        ],
        max_fk_position_error_mm=float(
            getattr(response, "max_fk_position_error_mm", 0.0)
        ),
        max_fk_orientation_error_deg=float(
            getattr(response, "max_fk_orientation_error_deg", 0.0)
        ),
        max_joint_step_rad=float(getattr(response, "max_joint_step_rad", 0.0)),
        max_joint_span_rad=float(getattr(response, "max_joint_span_rad", 0.0)),
        max_endpoint_delta_rad=float(
            getattr(response, "max_endpoint_delta_rad", 0.0)
        ),
    )
    report.timings["helper_planning_s"] = float(
        getattr(response, "planning_time_s", 0.0)
    )
    report.timings["helper_validation_s"] = float(
        getattr(response, "validation_time_s", 0.0)
    )
    report.timings["helper_total_s"] = float(
        getattr(response, "total_time_s", 0.0)
    )

    if not bool(getattr(response, "success", False)):
        report.failure_reason = "linked_lin_failed"
        if 0 <= failed < requested:
            report.first_failed_segment = failed
        report.details = (
            f"error_code={getattr(response, 'error_code', None)} "
            f"message={getattr(response, 'message', '')}"
        )
    return report


def _target_to_pose(target) -> Pose:
    if isinstance(target, Pose):
        return deepcopy(target)
    values = list(target or [])
    if len(values) < 6:
        raise ValueError("linked LIN target must contain six Cartesian values")
    transform = TransformationUtils.pose_to_transform([float(v) for v in values[:6]])
    quat = TransformationUtils.matrix_to_quaternion(transform[:3, :3])
    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def _normalized_strings(values, count: int, default: str) -> list[str]:
    if values is None:
        return [default] * count
    result = [str(value) for value in values]
    if len(result) != count:
        raise ValueError(f"linked LIN metadata size mismatch: expected {count}, got {len(result)}")
    return result


def _normalized_floats(values, count: int, default: float) -> list[float]:
    if values is None:
        return [float(default)] * count
    result = [float(value) for value in values]
    if len(result) != count:
        raise ValueError(f"linked LIN metadata size mismatch: expected {count}, got {len(result)}")
    return result


def _normalized_scales(values, count: int, default: float) -> list[float]:
    return [
        max(0.0, min(1.0, value))
        for value in _normalized_floats(values, count, default)
    ]


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


def _wait_future(future, timeout_s: float, description: str):
    deadline = time() + float(timeout_s)
    while time() < deadline:
        if future.done():
            return future.result()
        sleep(0.001)
    raise TimeoutError(f"{description} timed out after {timeout_s:.3f}s")


__all__ = [
    "LinkedLinPlanningResult",
    "LinkedLinReport",
    "request_linked_lin_trajectory",
]
