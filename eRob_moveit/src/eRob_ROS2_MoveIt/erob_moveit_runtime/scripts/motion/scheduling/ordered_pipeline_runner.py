#!/usr/bin/env python3
"""Thread orchestration for legacy ordered-chain planning/execution."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from threading import Event
from typing import Any, Callable

from motion.execution.ordered_execution import (
    OrderedPlannedSequenceHooks,
    execute_ordered_planned_sequence,
)


@dataclass(frozen=True)
class OrderedPipelineRunnerConfig:
    plan_timeout_s: float
    previous_execution_suppress: bool
    stopped_result: int


def _validate_readiness_groups(segments: list[dict[str, Any]]) -> None:
    """Reject a readiness group that reappears after another group/segment."""

    closed: set[str] = set()
    active = ""
    for index, segment in enumerate(segments):
        group = str(segment.get("readiness_group") or "").strip()
        if group == active:
            continue
        if active:
            closed.add(active)
        if group and group in closed:
            raise ValueError(
                f"readiness_group {group!r} must be contiguous; "
                f"it reappears at segment {index + 1}"
            )
        active = group


def run_ordered_planning_and_execution(
    *,
    node: Any,
    mark_motion_timing: Callable[..., None],
    planning_worker_factory: Callable[[Event], Callable[[], Any]],
    sequence_hooks: OrderedPlannedSequenceHooks,
    scheduler_bridge: Any,
    segments: list[dict[str, Any]],
    config: OrderedPipelineRunnerConfig,
    execution_authorized: Event | None = None,
) -> int:
    """Run ordered planning in one worker while consuming planned segments."""

    _validate_readiness_groups(segments)
    executor = ThreadPoolExecutor(max_workers=1)
    stop_planning = Event()
    planning_worker = planning_worker_factory(stop_planning)

    mark_motion_timing(node, "ordered_planner_submit_start")
    planner_future = executor.submit(planning_worker)
    mark_motion_timing(node, "ordered_planner_submit_done")

    try:
        if execution_authorized is not None:
            mark_motion_timing(node, "ordered_execution_waiting_for_authorization")
            while not execution_authorized.wait(timeout=0.1):
                if bool(getattr(node, "_ordered_motion_chain_stop_requested", False)):
                    stop_planning.set()
                    return config.stopped_result
            mark_motion_timing(node, "ordered_execution_authorized")
        return execute_ordered_planned_sequence(
            hooks=sequence_hooks,
            segments=segments,
            scheduler_bridge=scheduler_bridge,
            planner_future=planner_future,
            stop_planning=stop_planning,
            plan_timeout_s=config.plan_timeout_s,
            previous_execution_suppress=config.previous_execution_suppress,
            stopped_result=config.stopped_result,
        )
    finally:
        executor.shutdown(wait=True)


__all__ = [
    "OrderedPipelineRunnerConfig",
    "run_ordered_planning_and_execution",
]
