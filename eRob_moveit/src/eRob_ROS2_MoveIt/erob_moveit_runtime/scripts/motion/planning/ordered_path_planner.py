#!/usr/bin/env python3
"""Legacy ordered-chain PATH segment planner wrapper."""

from __future__ import annotations

import math
from time import perf_counter
from typing import Any, Callable, Mapping

from motion.planning.ordered_segment_parameters import OrderedSegmentParameters


MarkTimingFn = Callable[..., None]


def plan_ordered_path_segment(
    planning_node,
    *,
    index: int,
    segment: Mapping[str, Any],
    params: OrderedSegmentParameters,
    current_cartesian,
    current_state,
    tool_transform,
    user: int,
    selected_optimizer: str | None,
    apply_workobject: Callable[..., list[float]],
    mark_motion_timing: MarkTimingFn,
    build_follow_path_trajectory_fn,
    robot_state_from_trajectory_end_fn,
    plan_started: float | None = None,
) -> dict[str, Any]:
    """Plan one ordered PATH segment and return the current legacy dictionary shape."""

    if params.blend_radius > 0.0:
        raise RuntimeError(
            f"Ordered path segment {params.label!r} has blendR={params.blend_radius:.3f}, "
            "but path blending is not supported yet"
        )

    plan_started = perf_counter() if plan_started is None else plan_started
    path_base = []
    for waypoint in segment.get("path") or []:
        if len(waypoint) >= 6:
            wp_full = list(waypoint[:6])
        else:
            wp_full = [
                waypoint[0],
                waypoint[1],
                waypoint[2],
                current_cartesian[3],
                current_cartesian[4],
                current_cartesian[5],
            ]
        path_base.append(list(apply_workobject(wp_full, user_id=user)[:6]))

    if not path_base:
        raise RuntimeError(f"Ordered-chain path segment {params.label!r} is empty")

    planning_path = [list(current_cartesian[:6])]
    planning_path.extend(path_base)

    start_gap_mm = math.sqrt(
        (float(path_base[0][0]) - float(current_cartesian[0])) ** 2
        + (float(path_base[0][1]) - float(current_cartesian[1])) ** 2
        + (float(path_base[0][2]) - float(current_cartesian[2])) ** 2
    )

    planning_node.get_logger().info(
        f"[OrderedChain] Planning path segment '{params.label}' from previous target: "
        f"start_gap_mm={start_gap_mm:.3f} "
        f"path_waypoints={len(path_base)} "
        f"planning_waypoints={len(planning_path)}"
    )

    joint_trajectory = build_follow_path_trajectory_fn(
        planning_node,
        command_path=planning_path,
        start_state=current_state,
        tool_transform=tool_transform,
        vel_scaling=params.velocity_scale,
        acc_scaling=params.acceleration_scale,
        trajectory_optimizer_name=selected_optimizer,
    )

    plan_elapsed = perf_counter() - plan_started
    mark_motion_timing(
        "ordered_segment_plan_done",
        index=index + 1,
        label=params.label,
        segment_type=params.segment_type,
        duration_s=plan_elapsed,
        points=len(getattr(joint_trajectory, "points", []) or []),
        waypoints=len(planning_path),
    )

    return {
        "type": params.segment_type,
        "label": params.label,
        "target_position": list(path_base[-1][:6]),
        "final_state": robot_state_from_trajectory_end_fn(joint_trajectory),
        "trajectory": joint_trajectory,
        "plan_elapsed_s": plan_elapsed,
        "optimize_elapsed_s": 0.0,
        "protected": params.protected,
        "blendR": 0.0,
        "vel_scale": params.velocity_scale,
        "acc_scale": params.acceleration_scale,
        "optimization_deferred": False,
    }

