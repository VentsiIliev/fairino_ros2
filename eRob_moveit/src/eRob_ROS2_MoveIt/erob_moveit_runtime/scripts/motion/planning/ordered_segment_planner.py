#!/usr/bin/env python3
"""Dispatcher for legacy ordered-chain segment planning."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Any, Callable

from motion.planning.ordered_lin_planner import plan_ordered_linear_segment
from motion.planning.ordered_path_planner import plan_ordered_path_segment
from motion.planning.ordered_ptp_planner import plan_ordered_ptp_segment
from motion.planning.ordered_segment_parameters import parse_ordered_segment_parameters
from motion.planning.ordered_unwind_planner import plan_ordered_unwind_segment


@dataclass(frozen=True)
class OrderedSegmentPlannerHooks:
    node: Any
    planning_node: Any
    config_obj: Any
    tool_transform: Any
    user: int
    selected_optimizer: str | None
    total_segments: int
    apply_workobject: Callable[..., Any]
    mark_motion_timing: Callable[..., None]
    plan_segment: Callable[..., Any]
    optimize_sync: Callable[..., Any]
    build_follow_path_trajectory: Callable[..., Any]
    robot_state_from_trajectory_end: Callable[..., Any]
    clamp_percentage: Callable[[Any], float]
    canonical_angle: Callable[[float], float]
    plan_unwind_direct_ik_trajectory: Callable[..., Any]


def build_ordered_segment_planner_callback(
    hooks: OrderedSegmentPlannerHooks,
) -> Callable[..., dict[str, Any]]:
    """Return the legacy callback shape used by the ordered planning worker."""

    def _plan_ordered_segment(
        index,
        segment,
        current_cartesian,
        current_state,
        *,
        defer_optimization=False,
    ):
        return plan_ordered_segment(
            hooks=hooks,
            index=index,
            segment=segment,
            current_cartesian=current_cartesian,
            current_state=current_state,
            defer_optimization=defer_optimization,
        )

    return _plan_ordered_segment


def plan_ordered_segment(
    *,
    hooks: OrderedSegmentPlannerHooks,
    index: int,
    segment: dict[str, Any],
    current_cartesian,
    current_state,
    defer_optimization: bool = False,
) -> dict[str, Any]:
    """Plan one ordered-chain segment using backend-injected dependencies."""

    plan_started = perf_counter()
    params = parse_ordered_segment_parameters(
        segment,
        index,
        default_velocity_percent=hooks.config_obj.DEFAULT_VEL_PERCENT,
        default_acceleration_percent=hooks.config_obj.DEFAULT_ACC_PERCENT,
    )
    segment_type = params.segment_type

    hooks.mark_motion_timing(
        hooks.node,
        "ordered_segment_plan_start",
        index=index + 1,
        label=params.label,
        segment_type=segment_type,
        blendR=params.blend_radius,
        defer_optimization=bool(defer_optimization),
    )

    if segment_type == "linear":
        return plan_ordered_linear_segment(
            hooks.planning_node,
            index=index,
            segment=segment,
            params=params,
            current_cartesian=current_cartesian,
            current_state=current_state,
            tool_transform=hooks.tool_transform,
            user=hooks.user,
            defer_optimization=defer_optimization,
            apply_workobject=hooks.apply_workobject,
            mark_motion_timing=lambda event, **kwargs: hooks.mark_motion_timing(
                hooks.node,
                event,
                **kwargs,
            ),
            plan_segment_fn=hooks.plan_segment,
            robot_state_from_trajectory_end_fn=hooks.robot_state_from_trajectory_end,
            plan_started=plan_started,
        )

    if segment_type == "ptp":
        return plan_ordered_ptp_segment(
            hooks.planning_node,
            index=index,
            segment=segment,
            params=params,
            current_cartesian=current_cartesian,
            current_state=current_state,
            tool_transform=hooks.tool_transform,
            user=hooks.user,
            defer_optimization=defer_optimization,
            selected_optimizer=hooks.selected_optimizer,
            apply_workobject=hooks.apply_workobject,
            mark_motion_timing=lambda event, **kwargs: hooks.mark_motion_timing(
                hooks.node,
                event,
                **kwargs,
            ),
            optimize_sync_fn=hooks.optimize_sync,
            robot_state_from_trajectory_end_fn=hooks.robot_state_from_trajectory_end,
            plan_started=plan_started,
        )

    if segment_type == "path":
        return plan_ordered_path_segment(
            hooks.planning_node,
            index=index,
            segment=segment,
            params=params,
            current_cartesian=current_cartesian,
            current_state=current_state,
            tool_transform=hooks.tool_transform,
            user=hooks.user,
            selected_optimizer=hooks.selected_optimizer,
            apply_workobject=hooks.apply_workobject,
            mark_motion_timing=lambda event, **kwargs: hooks.mark_motion_timing(
                hooks.node,
                event,
                **kwargs,
            ),
            build_follow_path_trajectory_fn=hooks.build_follow_path_trajectory,
            robot_state_from_trajectory_end_fn=hooks.robot_state_from_trajectory_end,
            plan_started=plan_started,
        )

    if segment_type == "unwind_joint6":
        return plan_ordered_unwind_segment(
            hooks.planning_node,
            index=index,
            total_segments=hooks.total_segments,
            segment=segment,
            params=params,
            current_cartesian=current_cartesian,
            current_state=current_state,
            config_obj=hooks.config_obj,
            clamp_percentage=hooks.clamp_percentage,
            canonical_angle=hooks.canonical_angle,
            plan_unwind_direct_ik_trajectory_fn=hooks.plan_unwind_direct_ik_trajectory,
            robot_state_from_trajectory_end_fn=hooks.robot_state_from_trajectory_end,
            plan_started=plan_started,
        )

    raise RuntimeError(f"Unsupported ordered-chain segment type: {segment_type!r}")


__all__ = [
    "OrderedSegmentPlannerHooks",
    "build_ordered_segment_planner_callback",
    "plan_ordered_segment",
]
