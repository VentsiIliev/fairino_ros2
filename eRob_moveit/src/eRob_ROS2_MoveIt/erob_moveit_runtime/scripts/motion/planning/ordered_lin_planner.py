#!/usr/bin/env python3
"""Legacy ordered-chain LIN segment planner wrapper."""

from __future__ import annotations

from copy import deepcopy
import inspect
from time import perf_counter
from typing import Any, Callable, Mapping

from motion.planning.ordered_segment_parameters import OrderedSegmentParameters


MarkTimingFn = Callable[..., None]


def plan_ordered_linear_segment(
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
    apply_workobject: Callable[..., list[float]],
    mark_motion_timing: MarkTimingFn,
    plan_segment_fn,
    robot_state_from_trajectory_end_fn,
    plan_started: float | None = None,
) -> dict[str, Any]:
    """Plan one ordered LIN segment and return the current legacy dictionary shape."""

    plan_started = perf_counter() if plan_started is None else plan_started
    target_base = apply_workobject(
        list(segment["position"][:6]),
        user_id=user,
    )

    plan_kwargs = {
        "index": index,
        "segment": {
            "label": params.label,
            "position": list(target_base[:6]),
            "vel": params.velocity_percent,
            "acc": params.acceleration_percent,
            "motion_type": "linear",
        },
        "start_cartesian": list(current_cartesian[:6]),
        "start_state": current_state,
        "tool_transform": tool_transform,
    }

    supports_defer_optimization = "defer_optimization" in inspect.signature(plan_segment_fn).parameters
    if supports_defer_optimization:
        plan_kwargs["defer_optimization"] = bool(defer_optimization)

    planned = plan_segment_fn(
        planning_node,
        **plan_kwargs,
    )

    if defer_optimization and not supports_defer_optimization:
        raw_linear = deepcopy(planned.joint_trajectory)
        for point in raw_linear.points:
            point.velocities = []
            point.accelerations = []
            point.effort = []
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 0
        planned.joint_trajectory = raw_linear
        planned.final_state = robot_state_from_trajectory_end_fn(raw_linear)
        planned.optimize_elapsed_s = 0.0

    plan_elapsed = perf_counter() - plan_started
    mark_motion_timing(
        "ordered_segment_plan_done",
        index=index + 1,
        label=params.label,
        segment_type=params.segment_type,
        duration_s=plan_elapsed,
        points=len(getattr(planned.joint_trajectory, "points", []) or []),
        optimize_s=float(getattr(planned, "optimize_elapsed_s", 0.0) or 0.0),
        blendR=params.blend_radius,
        deferred=bool(defer_optimization),
    )

    return {
        "type": params.segment_type,
        "label": params.label,
        "start_position": list(current_cartesian[:6]),
        "target_position": planned.target_position,
        "final_state": planned.final_state,
        "trajectory": planned.joint_trajectory,
        "plan_elapsed_s": plan_elapsed,
        "optimize_elapsed_s": planned.optimize_elapsed_s,
        "protected": params.protected,
        "blendR": params.blend_radius,
        "vel_scale": params.velocity_scale,
        "acc_scale": params.acceleration_scale,
        "optimization_deferred": bool(defer_optimization),
    }
