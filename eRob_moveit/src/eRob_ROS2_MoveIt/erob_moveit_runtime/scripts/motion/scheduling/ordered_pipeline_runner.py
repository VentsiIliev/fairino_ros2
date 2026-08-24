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


def run_ordered_planning_and_execution(
    *,
    node: Any,
    mark_motion_timing: Callable[..., None],
    planning_worker_factory: Callable[[Event], Callable[[], Any]],
    sequence_hooks: OrderedPlannedSequenceHooks,
    scheduler_bridge: Any,
    segments_count: int,
    config: OrderedPipelineRunnerConfig,
    execution_authorized: Event | None = None,
) -> int:
    """Run ordered planning in one worker while consuming planned segments."""

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
            segments_count=segments_count,
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
