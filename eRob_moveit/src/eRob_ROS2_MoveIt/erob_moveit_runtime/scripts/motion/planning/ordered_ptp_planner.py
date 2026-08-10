#!/usr/bin/env python3
"""Legacy ordered-chain PTP segment planner wrapper."""

from __future__ import annotations

from time import perf_counter
from typing import Any, Callable, Mapping

from moveit_msgs.msg import RobotTrajectory

from motion.planning.ordered_segment_parameters import OrderedSegmentParameters


MarkTimingFn = Callable[..., None]


def plan_ordered_ptp_segment(
    planning_node,
    *,
    index: int,
    segment: Mapping[str, Any],
    params: OrderedSegmentParameters,
    current_cartesian,
    current_state,
    tool_transform,
    user: int,
    defer_optimization: bool,
    selected_optimizer: str | None,
    apply_workobject: Callable[..., list[float]],
    mark_motion_timing: MarkTimingFn,
    optimize_sync_fn,
    robot_state_from_trajectory_end_fn,
    plan_started: float | None = None,
) -> dict[str, Any]:
    """Plan one ordered PTP segment and return the current legacy dictionary shape."""

    plan_started = perf_counter() if plan_started is None else plan_started
    target_base = apply_workobject(
        list(segment["position"][:6]),
        user_id=user,
    )

    from motion.planning.ptp_target import plan_ptp_trajectory

    response = plan_ptp_trajectory(
        planning_node,
        target_base,
        current_state.joint_state,
        tool_transform=tool_transform,
    )

    if not bool(response.success):
        raise RuntimeError(f"Ordered PTP segment {params.label!r} rejected: {response.message}")

    if bool(response.noop):
        plan_elapsed = perf_counter() - plan_started
        mark_motion_timing(
            "ordered_segment_plan_done",
            index=index + 1,
            label=params.label,
            segment_type=params.segment_type,
            duration_s=plan_elapsed,
            points=0,
            native_ptp_ms=float(response.total_time_ms),
            ik_ms=float(response.ik_time_ms),
            validation_ms=float(response.validation_time_ms),
            noop=True,
            blendR=params.blend_radius,
        )

        return {
            "type": params.segment_type,
            "label": params.label,
            "start_position": list(current_cartesian[:6]),
            "target_position": list(target_base[:6]),
            "final_state": current_state,
            "trajectory": None,
            "noop": True,
            "plan_elapsed_s": plan_elapsed,
            "optimize_elapsed_s": 0.0,
            "protected": params.protected,
            "blendR": params.blend_radius,
            "vel_scale": params.velocity_scale,
            "acc_scale": params.acceleration_scale,
            "optimization_deferred": bool(defer_optimization),
        }

    raw_joint_trajectory = response.trajectory
    if not getattr(raw_joint_trajectory, "points", None):
        raise RuntimeError(f"Ordered PTP segment {params.label!r} returned an empty trajectory")

    if defer_optimization:
        final_joint_trajectory = raw_joint_trajectory
        optimize_elapsed = 0.0
    else:
        moveit_trajectory = RobotTrajectory()
        moveit_trajectory.joint_trajectory = raw_joint_trajectory
        optimized, optimize_elapsed = optimize_sync_fn(
            planning_node,
            moveit_trajectory,
            params.velocity_scale,
            params.acceleration_scale,
            optimizer_name=selected_optimizer,
        )
        final_joint_trajectory = optimized.joint_trajectory
        if not getattr(final_joint_trajectory, "points", None):
            raise RuntimeError(f"Ordered PTP segment {params.label!r} optimizer returned an empty trajectory")

    plan_elapsed = perf_counter() - plan_started
    mark_motion_timing(
        "ordered_segment_plan_done",
        index=index + 1,
        label=params.label,
        segment_type=params.segment_type,
        duration_s=plan_elapsed,
        points=len(final_joint_trajectory.points),
        native_ptp_ms=float(response.total_time_ms),
        ik_ms=float(response.ik_time_ms),
        validation_ms=float(response.validation_time_ms),
        optimize_s=float(optimize_elapsed),
        blendR=params.blend_radius,
        deferred=bool(defer_optimization),
    )

    planning_node.get_logger().info(
        f"[OrderedChain][PTP] Planned '{params.label}' "
        f"points={len(final_joint_trajectory.points)} "
        f"native={response.total_time_ms:.2f}ms "
        f"IK={response.ik_time_ms:.2f}ms "
        f"validation={response.validation_time_ms:.2f}ms "
        f"optimize={optimize_elapsed:.3f}s "
        f"deferred={bool(defer_optimization)} "
        f"blendR={params.blend_radius:.3f}mm "
        f"total={plan_elapsed:.3f}s"
    )

    return {
        "type": params.segment_type,
        "label": params.label,
        "start_position": list(current_cartesian[:6]),
        "target_position": list(target_base[:6]),
        "final_state": robot_state_from_trajectory_end_fn(final_joint_trajectory),
        "trajectory": final_joint_trajectory,
        "plan_elapsed_s": plan_elapsed,
        "optimize_elapsed_s": optimize_elapsed,
        "protected": params.protected,
        "blendR": params.blend_radius,
        "vel_scale": params.velocity_scale,
        "acc_scale": params.acceleration_scale,
        "optimization_deferred": bool(defer_optimization),
    }
