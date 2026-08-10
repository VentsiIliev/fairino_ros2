#!/usr/bin/env python3
"""Planning worker for legacy ordered-chain execution."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Any, Callable, Sequence

import config
from motion.planning.linked_lin_client import request_linked_lin_trajectory


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

    if _should_plan_linked_lin_group(segments, start_index, group_end):
        return _plan_linked_lin_group(
            hooks=hooks,
            segments=segments,
            start_index=start_index,
            group_end=group_end,
            previous_target=previous_target,
            previous_state=previous_state,
            selected_optimizer=selected_optimizer,
            worker_started=worker_started,
        )

    planned_group = []
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
            raise RuntimeError(
                f"Cannot blend no-op segment {planned_member['label']!r}"
            )

        planned_group.append(planned_member)
        group_target = planned_member["target_position"]
        group_state = planned_member["final_state"]

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

    hooks.mark_motion_timing(
        hooks.node,
        "ordered_segment_queued",
        index=start_index + 1,
        label=combined.get("label"),
        segment_type="blended",
        blend_group_size=len(planned_group),
        effective_blend_radii=[float(value) for value in effective_radii],
        duration_s=perf_counter() - worker_started,
    )

    hooks.publish_planned(start_index, combined)

    for consumed_offset in range(1, len(planned_group)):
        logical_index = start_index + consumed_offset
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


def _should_plan_linked_lin_group(
    segments: Sequence[dict[str, Any]],
    start_index: int,
    group_end: int,
) -> bool:
    if not bool(getattr(config, "LINKED_LIN_HELPER_ENABLED", False)):
        return False
    if group_end <= start_index:
        return False
    for index in range(start_index, group_end + 1):
        segment_type = str(segments[index].get("type") or "").strip().lower()
        if segment_type != "linear":
            return False
    return True


def _plan_linked_lin_group(
    *,
    hooks: OrderedPlanningWorkerHooks,
    segments: Sequence[dict[str, Any]],
    start_index: int,
    group_end: int,
    previous_target: Sequence[float],
    previous_state: Any,
    selected_optimizer: str | None,
    worker_started: float,
) -> tuple[int, list[float], Any]:
    """Plan LIN members in one C++ request, then reuse the proven BlendBuilder."""

    plan_started = perf_counter()
    planned_members = []

    for group_index in range(start_index, group_end + 1):
        segment = segments[group_index]
        target_base = hooks.apply_workobject(
            list(segment["position"][:6]),
            user_id=hooks.user,
        )
        _check_cartesian_target_safety(hooks.node, target_base, group_index)
        planned_members.append(
            {
                "type": "linear",
                "label": str(segment.get("label") or f"segment_{group_index + 1}"),
                "start_position": (
                    list(previous_target[:6])
                    if group_index == start_index
                    else list(planned_members[-1]["target_position"])
                ),
                "target_position": list(target_base[:6]),
                "protected": bool(segment.get("protected", False)),
                "blendR": max(0.0, float(segment.get("blendR", 0.0) or 0.0)),
                "vel_scale": _scale_from_percent(
                    float(segment.get("vel", getattr(config, "DEFAULT_VEL_PERCENT", 30)))
                ),
                "acc_scale": _scale_from_percent(
                    float(segment.get("acc", getattr(config, "DEFAULT_ACC_PERCENT", 30)))
                ),
            }
        )

    result = request_linked_lin_trajectory(
        hooks.node,
        [member["target_position"] for member in planned_members],
        labels=[member["label"] for member in planned_members],
        velocities=[member["vel_scale"] for member in planned_members],
        accelerations=[member["acc_scale"] for member in planned_members],
        blend_radii=[member["blendR"] for member in planned_members],
        seed_state=previous_state,
        tool_transform=hooks.tool_transform,
    )
    if result is None:
        raise RuntimeError(
            "LINKED_LIN_HELPER_ENABLED is true but linked-LIN client did not run"
        )
    if not result.report.ok:
        raise RuntimeError(
            f"linked-LIN helper rejected group {start_index + 1}-{group_end + 1}: "
            f"{result.report.failure_reason} {result.report.details}"
        )
    if len(result.segment_trajectories) != len(planned_members):
        raise RuntimeError(
            "linked-LIN helper returned wrong segment count: "
            f"expected={len(planned_members)} got={len(result.segment_trajectories)}"
        )

    for member, trajectory in zip(planned_members, result.segment_trajectories):
        member["trajectory"] = trajectory

    hooks.node.get_logger().info(
        "[LINKED_LIN] Raw segment trajectories ready "
        f"segments={len(planned_members)} "
        f"segment_points={result.report.segment_point_counts} "
        f"helper_s={result.report.timings.get('helper_total_s', 0.0):.3f}"
    )

    # Preserve the existing, proven LIN/PTP blending semantics. C++ only
    # replaces the expensive repeated LIN planning calls; it does not invent a
    # different Cartesian blend geometry.
    raw_blended, effective_radii = hooks.blend_builder.build(planned_members)

    from moveit_msgs.msg import RobotTrajectory

    moveit_trajectory = RobotTrajectory()
    moveit_trajectory.joint_trajectory = raw_blended

    group_vel_scale = min(
        float(member.get("vel_scale", 1.0)) for member in planned_members
    )
    group_acc_scale = min(
        float(member.get("acc_scale", 1.0)) for member in planned_members
    )

    optimized, optimize_elapsed = hooks.optimize_sync(
        moveit_trajectory,
        group_vel_scale,
        group_acc_scale,
        optimizer_name=selected_optimizer,
    )
    optimized_joint_trajectory = optimized.joint_trajectory
    if not getattr(optimized_joint_trajectory, "points", None):
        raise RuntimeError("Optimizer returned empty linked-LIN blended trajectory")

    first = planned_members[0]
    last = planned_members[-1]
    combined = {
        "type": "linked_lin",
        "label": " -> ".join(str(member.get("label") or "") for member in planned_members),
        "start_position": list(first["start_position"]),
        "target_position": list(last["target_position"]),
        "final_state": hooks.robot_state_from_trajectory_end(
            optimized_joint_trajectory
        ),
        "trajectory": optimized_joint_trajectory,
        "plan_elapsed_s": perf_counter() - plan_started,
        "optimize_elapsed_s": float(optimize_elapsed),
        "linked_lin_helper_elapsed_s": float(
            result.report.timings.get("helper_total_s", 0.0)
        ),
        "protected": any(bool(member.get("protected", False)) for member in planned_members),
        "blendR": float(first.get("blendR", 0.0) or 0.0),
        "effective_blend_radii": list(effective_radii),
        "vel_scale": group_vel_scale,
        "acc_scale": group_acc_scale,
        "logical_segment_count": len(planned_members),
        "linked_lin_segment_point_counts": list(result.report.segment_point_counts),
        "linked_lin_segment_boundaries": list(result.report.segment_boundary_indices),
        "linked_lin_segment_planning_s": list(result.report.segment_planning_time_s),
    }

    hooks.mark_motion_timing(
        hooks.node,
        "ordered_linked_lin_group_planned",
        index=start_index + 1,
        label=combined.get("label"),
        group_size=len(planned_members),
        raw_points=sum(result.report.segment_point_counts),
        segment_points=list(result.report.segment_point_counts),
        segment_boundaries=list(result.report.segment_boundary_indices),
        effective_blend_radii=[float(value) for value in effective_radii],
        helper_s=float(result.report.timings.get("helper_total_s", 0.0)),
        blend_and_optimize_s=(perf_counter() - plan_started) - float(
            result.report.timings.get("service_call_s", 0.0)
        ),
        optimize_s=float(optimize_elapsed),
        duration_s=perf_counter() - worker_started,
    )
    hooks.mark_motion_timing(
        hooks.node,
        "ordered_segment_queued",
        index=start_index + 1,
        label=combined.get("label"),
        segment_type="linked_lin",
        blend_group_size=len(planned_members),
        effective_blend_radii=[float(value) for value in effective_radii],
        duration_s=perf_counter() - worker_started,
    )

    hooks.publish_planned(start_index, combined)

    for consumed_offset in range(1, len(planned_members)):
        logical_index = start_index + consumed_offset
        member = planned_members[consumed_offset]
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


def _check_cartesian_target_safety(
    node: Any,
    target_position: Sequence[float],
    group_index: int,
) -> None:
    check = getattr(node, "check_position_safety", None)
    if check is None:
        return
    # Existing Cartesian positions are millimetres while safety checks use m.
    x_m = float(target_position[0]) / 1000.0
    y_m = float(target_position[1]) / 1000.0
    z_m = float(target_position[2]) / 1000.0
    is_safe, message = check(x_m, y_m, z_m)
    if not is_safe:
        raise RuntimeError(
            f"linked-LIN target {group_index + 1} rejected by safety walls: {message}"
        )
    if "Warning" in str(message):
        node.get_logger().warning(f"[LINKED_LIN] {message}")


def _scale_from_percent(value: float) -> float:
    return max(0.0, min(1.0, float(value) / 100.0))


__all__ = [
    "OrderedPlanningWorkerHooks",
    "build_ordered_planning_worker_factory",
    "execute_ordered_planning_worker",
]
