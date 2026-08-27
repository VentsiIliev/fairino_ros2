#!/usr/bin/env python3
"""Planning worker for legacy ordered-chain execution."""

from __future__ import annotations

from dataclasses import dataclass
from copy import deepcopy
from time import perf_counter
from typing import Any, Callable, Sequence


@dataclass(frozen=True)
class OrderedPlanningWorkerHooks:
    node: Any
    mark_motion_timing: Callable[..., None]
    publish_planned: Callable[[int, dict[str, Any]], None]
    plan_ordered_segment: Callable[..., dict[str, Any]]
    blend_builder: Any
    optimize_sync: Callable[..., tuple[Any, float]]
    robot_state_from_trajectory_end: Callable[[Any], Any]
    apply_workobject: Callable[..., Any]
    tool_transform: Any
    user: int


def build_ordered_planning_worker_factory(
    *,
    hooks: OrderedPlanningWorkerHooks,
    segments: Sequence[dict[str, Any]],
    start_cartesian: Sequence[float],
    start_state: Any,
    scheduler_bridge: Any,
    selected_optimizer: str | None,
) -> Callable[[Any], Callable[[], None]]:
    """Return a stop-event-aware ordered planning worker factory."""

    def _planning_worker_factory(stop_planning):
        def _planning_worker():
            return execute_ordered_planning_worker(
                hooks=hooks,
                segments=segments,
                start_cartesian=start_cartesian,
                start_state=start_state,
                scheduler_bridge=scheduler_bridge,
                stop_planning=stop_planning,
                selected_optimizer=selected_optimizer,
            )

        return _planning_worker

    return _planning_worker_factory


def execute_ordered_planning_worker(
    *,
    hooks: OrderedPlanningWorkerHooks,
    segments: Sequence[dict[str, Any]],
    start_cartesian: Sequence[float],
    start_state: Any,
    scheduler_bridge: Any,
    stop_planning: Any,
    selected_optimizer: str | None,
) -> None:
    """Plan ordered-chain segments and publish ready planned work."""

    worker_started = perf_counter()
    hooks.mark_motion_timing(
        hooks.node,
        "ordered_planning_worker_start",
        segments=len(segments),
    )

    previous_target = list(start_cartesian)
    previous_state = start_state

    try:
        index = 0

        while index < len(segments):
            if stop_planning.is_set():
                hooks.mark_motion_timing(
                    hooks.node,
                    "ordered_planning_worker_stopped",
                    index=index + 1,
                    duration_s=perf_counter() - worker_started,
                )
                break

            segment = segments[index]
            execution_group = str(segment.get("execution_group") or "").strip()
            execution_policy = str(segment.get("execution_policy") or "").strip().lower()
            if execution_group and execution_policy == "concatenate":
                index, previous_target, previous_state = _plan_concatenated_group(
                    hooks=hooks,
                    segments=segments,
                    start_index=index,
                    previous_target=previous_target,
                    previous_state=previous_state,
                    worker_started=worker_started,
                )
                continue
            blend_r = max(
                0.0,
                float(segment.get("blendR", 0.0) or 0.0),
            )

            if blend_r > 0.0:
                index, previous_target, previous_state = _plan_blend_group(
                    hooks=hooks,
                    segments=segments,
                    start_index=index,
                    previous_target=previous_target,
                    previous_state=previous_state,
                    selected_optimizer=selected_optimizer,
                    worker_started=worker_started,
                )
                continue

            planned_segment = hooks.plan_ordered_segment(
                index,
                segment,
                previous_target,
                previous_state,
            )

            hooks.mark_motion_timing(
                hooks.node,
                "ordered_segment_queued",
                index=index + 1,
                label=planned_segment.get("label"),
                duration_s=perf_counter() - worker_started,
            )

            hooks.publish_planned(index, planned_segment)

            previous_target = planned_segment["target_position"]
            previous_state = planned_segment["final_state"]
            index += 1

        scheduler_bridge.publish_done()

        hooks.mark_motion_timing(
            hooks.node,
            "ordered_planning_worker_done",
            duration_s=perf_counter() - worker_started,
        )

    except Exception as exc:
        hooks.mark_motion_timing(
            hooks.node,
            "ordered_planning_worker_error",
            duration_s=perf_counter() - worker_started,
            error=str(exc),
        )

        scheduler_bridge.publish_error(exc)


def _duration_ns(duration: Any) -> int:
    return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)


def _set_duration_ns(duration: Any, value: int) -> None:
    duration.sec = int(value // 1_000_000_000)
    duration.nanosec = int(value % 1_000_000_000)


def _concatenate_joint_trajectories(planned_group: list[dict[str, Any]]) -> Any:
    """Join timed trajectories without trimming or changing their geometry."""

    first_trajectory = planned_group[0].get("trajectory")
    if first_trajectory is None or not getattr(first_trajectory, "points", None):
        raise RuntimeError("First concatenation member has no trajectory points")
    combined = deepcopy(first_trajectory)
    joint_names = list(combined.joint_names)
    junction_tolerance_rad = 0.02

    for member in planned_group[1:]:
        trajectory = member.get("trajectory")
        points = list(getattr(trajectory, "points", []) or [])
        if not points:
            raise RuntimeError(f"Cannot concatenate empty segment {member.get('label')!r}")
        if list(trajectory.joint_names) != joint_names:
            raise RuntimeError("Concatenated trajectories have different joint order")
        left_positions = list(combined.points[-1].positions)
        right_positions = list(points[0].positions)
        if len(left_positions) != len(right_positions):
            raise RuntimeError("Concatenated trajectory junction has different joint dimensions")
        max_error = max(
            abs(float(left) - float(right))
            for left, right in zip(left_positions, right_positions)
        )
        if max_error > junction_tolerance_rad:
            raise RuntimeError(
                f"Concatenated trajectory junction mismatch: "
                f"{max_error:.6f}rad > {junction_tolerance_rad:.6f}rad"
            )

        offset_ns = _duration_ns(combined.points[-1].time_from_start)
        previous_ns = offset_ns
        # The first point is the shared boundary and is intentionally kept only
        # from the left trajectory. Subsequent timing remains exactly relative
        # to that boundary, so no dwell is introduced.
        for source_point in points[1:]:
            relative_ns = _duration_ns(source_point.time_from_start)
            target_ns = offset_ns + relative_ns
            if target_ns <= previous_ns:
                raise RuntimeError(
                    f"Concatenated segment {member.get('label')!r} has non-increasing timing"
                )
            point = deepcopy(source_point)
            _set_duration_ns(point.time_from_start, target_ns)
            combined.points.append(point)
            previous_ns = target_ns
    return combined


def _plan_concatenated_group(
    *,
    hooks: OrderedPlanningWorkerHooks,
    segments: Sequence[dict[str, Any]],
    start_index: int,
    previous_target: Sequence[float],
    previous_state: Any,
    worker_started: float,
) -> tuple[int, list[float], Any]:
    group_name = str(segments[start_index].get("execution_group") or "").strip()
    group_end = start_index
    while (
        group_end + 1 < len(segments)
        and str(segments[group_end + 1].get("execution_group") or "").strip() == group_name
        and str(segments[group_end + 1].get("execution_policy") or "").strip().lower()
        == "concatenate"
    ):
        group_end += 1
    if group_end == start_index:
        raise RuntimeError(
            f"execution_group {group_name!r} requires at least two contiguous segments"
        )

    planned_group = []
    group_target = list(previous_target)
    group_state = previous_state
    for group_index in range(start_index, group_end + 1):
        planned = hooks.plan_ordered_segment(
            group_index,
            segments[group_index],
            group_target,
            group_state,
        )
        if bool(planned.get("noop", False)):
            raise RuntimeError(
                f"Cannot concatenate no-op segment {planned.get('label')!r}"
            )
        planned_group.append(planned)
        group_target = list(planned["target_position"])
        group_state = planned["final_state"]

    trajectory = _concatenate_joint_trajectories(planned_group)
    first = planned_group[0]
    last = planned_group[-1]
    profile_limits: dict[str, float] = {}
    profile_names: list[str] = []
    for member in planned_group:
        member_limits = member.get("_joint_rate_limits_rad_s") or {}
        if member.get("limit_profile"):
            profile_names.append(str(member["limit_profile"]))
        for joint_name, limit in member_limits.items():
            value = float(limit)
            profile_limits[joint_name] = min(
                profile_limits.get(joint_name, value),
                value,
            )
    combined = {
        "type": "concatenated",
        "label": " + ".join(str(member.get("label") or "") for member in planned_group),
        "start_position": list(first["start_position"]),
        "target_position": list(last["target_position"]),
        "final_state": hooks.robot_state_from_trajectory_end(trajectory),
        "trajectory": trajectory,
        "plan_elapsed_s": sum(
            float(member.get("plan_elapsed_s", 0.0) or 0.0)
            for member in planned_group
        ),
        "optimize_elapsed_s": sum(
            float(member.get("optimize_elapsed_s", 0.0) or 0.0)
            for member in planned_group
        ),
        "protected": any(bool(member.get("protected", False)) for member in planned_group),
        "logical_segment_count": len(planned_group),
    }
    if profile_limits:
        combined["_joint_rate_limits_rad_s"] = profile_limits
        combined["limit_profile"] = ",".join(dict.fromkeys(profile_names))
    hooks.mark_motion_timing(
        hooks.node,
        "ordered_segment_queued",
        index=start_index + 1,
        label=combined["label"],
        segment_type="concatenated",
        execution_group=group_name,
        execution_group_size=len(planned_group),
        duration_s=perf_counter() - worker_started,
    )
    hooks.publish_planned(start_index, combined)
    for offset, member in enumerate(planned_group[1:], start=1):
        logical_index = start_index + offset
        hooks.publish_planned(
            logical_index,
            {
                "type": "concatenate_consumed",
                "label": str(member.get("label") or f"segment_{logical_index + 1}"),
                "start_position": list(member.get("start_position") or []),
                "target_position": list(member["target_position"]),
                "final_state": combined["final_state"],
                "trajectory": None,
                "plan_elapsed_s": 0.0,
                "optimize_elapsed_s": 0.0,
                "protected": bool(member.get("protected", False)),
            },
        )
    return group_end + 1, list(last["target_position"]), combined["final_state"]


def _plan_blend_group(
    *,
    hooks: OrderedPlanningWorkerHooks,
    segments: Sequence[dict[str, Any]],
    start_index: int,
    previous_target: Sequence[float],
    previous_state: Any,
    selected_optimizer: str | None,
    worker_started: float,
) -> tuple[int, list[float], Any]:
    segment_type = str(segments[start_index].get("type") or "").strip().lower()
    blend_r = max(0.0, float(segments[start_index].get("blendR", 0.0) or 0.0))
    if segment_type not in {"linear", "ptp"}:
        raise RuntimeError(
            f"blendR is currently supported only for LIN/PTP; "
            f"segment {start_index + 1} is {segment_type!r}"
        )

    if start_index + 1 >= len(segments):
        raise RuntimeError(
            f"Segment {start_index + 1} requests blendR={blend_r:.3f} "
            "but there is no next segment"
        )

    group_end = start_index
    while group_end < len(segments) - 1:
        current_segment = segments[group_end]
        current_type = str(current_segment.get("type") or "").strip().lower()
        current_blend_r = max(
            0.0,
            float(current_segment.get("blendR", 0.0) or 0.0),
        )

        if current_blend_r <= 0.0:
            break

        if current_type not in {"linear", "ptp"}:
            raise RuntimeError(
                f"blendR is currently supported only for LIN/PTP; "
                f"segment {group_end + 1} is {current_type!r}"
            )

        next_segment = segments[group_end + 1]
        next_type = str(next_segment.get("type") or "").strip().lower()
        if next_type not in {"linear", "ptp"}:
            raise RuntimeError(
                f"Segment {group_end + 1} requests "
                f"blendR={current_blend_r:.3f}, but the next "
                f"segment {group_end + 2} is {next_type!r}. "
                "Blend groups currently support only LIN/PTP -> LIN/PTP."
            )

        group_end += 1

    if group_end <= start_index:
        raise RuntimeError(
            f"Could not form blend group starting at segment {start_index + 1}"
        )

    planned_group = []
    leading_noops = []
    group_target = list(previous_target)
    group_state = previous_state

    for group_index in range(start_index, group_end + 1):
        planned_member = hooks.plan_ordered_segment(
            group_index,
            segments[group_index],
            group_target,
            group_state,
            defer_optimization=True,
        )

        if bool(planned_member.get("noop", False)):
            if not planned_group:
                leading_noops.append((group_index, planned_member))
                group_target = planned_member["target_position"]
                group_state = planned_member["final_state"]
                continue
            raise RuntimeError(
                f"Cannot blend internal no-op segment {planned_member['label']!r}"
            )

        planned_group.append(planned_member)
        group_target = planned_member["target_position"]
        group_state = planned_member["final_state"]

    for logical_index, noop_member in leading_noops:
        hooks.mark_motion_timing(
            hooks.node,
            "ordered_segment_queued",
            index=logical_index + 1,
            label=noop_member.get("label"),
            segment_type=noop_member.get("type"),
            noop=True,
            duration_s=perf_counter() - worker_started,
        )
        hooks.publish_planned(logical_index, noop_member)

    if not planned_group:
        last_noop = leading_noops[-1][1]
        return (
            group_end + 1,
            list(last_noop["target_position"]),
            last_noop["final_state"],
        )

    active_start_index = start_index + len(leading_noops)
    if len(planned_group) == 1:
        # A blend group needs at least two real motions. Re-plan the remaining
        # member normally so its deferred raw trajectory is optimized.
        only_member = hooks.plan_ordered_segment(
            active_start_index,
            segments[active_start_index],
            leading_noops[-1][1]["target_position"],
            leading_noops[-1][1]["final_state"],
        )
        hooks.mark_motion_timing(
            hooks.node,
            "ordered_segment_queued",
            index=active_start_index + 1,
            label=only_member.get("label"),
            duration_s=perf_counter() - worker_started,
        )
        hooks.publish_planned(active_start_index, only_member)
        return (
            group_end + 1,
            list(only_member["target_position"]),
            only_member["final_state"],
        )

    raw_blended, effective_radii = hooks.blend_builder.build(planned_group)

    from moveit_msgs.msg import RobotTrajectory

    moveit_trajectory = RobotTrajectory()
    moveit_trajectory.joint_trajectory = raw_blended

    group_vel_scale = min(
        float(member.get("vel_scale", 1.0)) for member in planned_group
    )
    group_acc_scale = min(
        float(member.get("acc_scale", 1.0)) for member in planned_group
    )

    optimized, optimize_elapsed = hooks.optimize_sync(
        moveit_trajectory,
        group_vel_scale,
        group_acc_scale,
        optimizer_name=selected_optimizer,
    )
    optimized_joint_trajectory = optimized.joint_trajectory

    if not getattr(optimized_joint_trajectory, "points", None):
        raise RuntimeError("Optimizer returned empty blended-group trajectory")

    first = planned_group[0]
    last = planned_group[-1]
    profile_limits = {}
    profile_names = []
    for member in planned_group:
        member_limits = member.get("_joint_rate_limits_rad_s") or {}
        if member.get("limit_profile"):
            profile_names.append(str(member["limit_profile"]))
        for joint_name, limit in member_limits.items():
            value = float(limit)
            profile_limits[joint_name] = min(profile_limits.get(joint_name, value), value)
    combined = {
        "type": "blended",
        "label": " -> ".join(str(member.get("label") or "") for member in planned_group),
        "start_position": list(first["start_position"]),
        "target_position": list(last["target_position"]),
        "final_state": hooks.robot_state_from_trajectory_end(
            optimized_joint_trajectory
        ),
        "trajectory": optimized_joint_trajectory,
        "plan_elapsed_s": (
            sum(
                float(member.get("plan_elapsed_s", 0.0) or 0.0)
                for member in planned_group
            )
            + float(optimize_elapsed)
        ),
        "optimize_elapsed_s": float(optimize_elapsed),
        "protected": any(bool(member.get("protected", False)) for member in planned_group),
        "blendR": float(first.get("blendR", 0.0) or 0.0),
        "effective_blend_radii": list(effective_radii),
        "vel_scale": group_vel_scale,
        "acc_scale": group_acc_scale,
        "logical_segment_count": len(planned_group),
    }
    if profile_limits:
        combined["_joint_rate_limits_rad_s"] = profile_limits
        combined["limit_profile"] = ",".join(dict.fromkeys(profile_names))

    hooks.mark_motion_timing(
        hooks.node,
        "ordered_segment_queued",
        index=active_start_index + 1,
        label=combined.get("label"),
        segment_type="blended",
        blend_group_size=len(planned_group),
        effective_blend_radii=[float(value) for value in effective_radii],
        duration_s=perf_counter() - worker_started,
    )

    hooks.publish_planned(active_start_index, combined)

    for consumed_offset in range(1, len(planned_group)):
        logical_index = active_start_index + consumed_offset
        member = planned_group[consumed_offset]
        consumed = {
            "type": "blend_consumed",
            "label": str(member.get("label") or f"segment_{logical_index + 1}"),
            "start_position": list(member.get("start_position") or []),
            "target_position": list(member["target_position"]),
            "final_state": combined["final_state"],
            "trajectory": None,
            "plan_elapsed_s": 0.0,
            "optimize_elapsed_s": 0.0,
            "protected": bool(member.get("protected", False)),
            "blendR": 0.0,
        }
        hooks.publish_planned(logical_index, consumed)

    return group_end + 1, list(last["target_position"]), combined["final_state"]


__all__ = [
    "OrderedPlanningWorkerHooks",
    "build_ordered_planning_worker_factory",
    "execute_ordered_planning_worker",
]
