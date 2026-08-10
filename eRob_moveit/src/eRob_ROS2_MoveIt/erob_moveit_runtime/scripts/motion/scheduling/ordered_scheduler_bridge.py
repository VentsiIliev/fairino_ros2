#!/usr/bin/env python3
"""Bridge object for migrating legacy ordered-chain scheduling state."""

from __future__ import annotations

from dataclasses import dataclass
from queue import Empty
from typing import Any, Callable, Sequence

from motion.scheduling.ordered_observation import OrderedChainObservation
from motion.scheduling.ordered_planned_queue import OrderedPlannedQueue
from motion.scheduling.status_adapter import (
    ordered_chain_group_state_status,
    ordered_chain_initial_group_states,
    ordered_chain_initial_segment_states_from_mappings,
    ordered_chain_segment_state_status,
    update_ordered_chain_group_states_from_segments,
)


@dataclass(frozen=True)
class OrderedSchedulerRuntime:
    bridge: "OrderedSchedulerBridge"
    publish_scheduler_updates: Callable[[dict[str, Any]], None]
    publish_planned: Callable[[int, dict[str, Any]], None]


class OrderedSchedulerBridge:
    """Owns legacy ordered-chain scheduling observation and planned queue state."""

    def __init__(
        self,
        *,
        scheduler_group_status: tuple[dict[str, Any], ...] = (),
        scheduler_segment_states: dict[int, dict[str, Any]] | None = None,
    ):
        self.queue = OrderedPlannedQueue()
        self.observation = OrderedChainObservation()
        self.segment_states = dict(scheduler_segment_states or {})
        self.group_states = ordered_chain_initial_group_states(
            tuple(scheduler_group_status or ())
        )

    def mark_planned(self, index: int, planned_segment: dict[str, Any]) -> dict[str, Any]:
        return self.observation.mark_planned(index, planned_segment)

    def publish_planned(self, index: int, planned_segment: dict[str, Any]) -> dict[str, Any]:
        self.queue.put_planned(index, planned_segment)
        return self.mark_planned(index, planned_segment)

    def publish_ready_segment(
        self,
        index: int,
        planned_segment: dict[str, Any],
    ) -> dict[str, Any]:
        updates = self.set_segment_state(
            index,
            "READY",
            planned=planned_segment,
            result=None,
        )
        updates.update(self.publish_planned(index, planned_segment))
        return updates

    def publish_done(self) -> None:
        self.queue.put_done()

    def publish_error(self, exc: BaseException) -> None:
        self.queue.put_error(exc)

    def mark_consumed(self, index: int) -> None:
        self.observation.mark_consumed(index)

    def consume_planned(self, index: int, *, current_index: int = 0) -> dict[str, Any]:
        self.mark_consumed(index)
        return self.preplanned_snapshot(current_index=current_index)

    def preplanned_snapshot(self, current_index: int = 0) -> dict[str, Any]:
        return self.observation.preplanned_snapshot(current_index=current_index)

    def mark_executing(
        self,
        index: int,
        planned_segment: dict[str, Any],
    ) -> dict[str, Any]:
        return self.set_segment_state(
            index,
            "EXECUTING",
            planned=planned_segment,
            result=None,
        )

    def mark_finished(
        self,
        index: int,
        planned_segment: dict[str, Any],
        result: Any,
    ) -> dict[str, Any]:
        normalized_result = int(result) if isinstance(result, int) else result
        return self.set_segment_state(
            index,
            "DONE" if result == 0 else "FAILED",
            planned=planned_segment,
            result=normalized_result,
        )

    def set_segment_state(
        self,
        index: int,
        state: str,
        *,
        planned: dict[str, Any] | None = None,
        result: Any | None = None,
    ) -> dict[str, Any]:
        status = self.segment_states.get(int(index))
        if status is None:
            return {}
        if planned is not None:
            status["label"] = str(
                planned.get("label") or status.get("label") or f"segment_{int(index) + 1}"
            )
            status["type"] = str(planned.get("type") or status.get("type") or "")
        status["state"] = str(state)
        status["result"] = result
        update_ordered_chain_group_states_from_segments(
            self.group_states,
            self.segment_states,
        )
        return {
            "scheduler_segment_states": list(
                ordered_chain_segment_state_status(self.segment_states)
            ),
            "scheduler_groups": list(ordered_chain_group_state_status(self.group_states)),
        }

    def wait_for_planned(self, expected_index: int, timeout: float) -> dict[str, Any]:
        try:
            planned_index, planned, exc = self.queue.get(timeout=timeout)
        except Empty as exc:
            raise TimeoutError(
                f"Timed out waiting for ordered-chain plan index={expected_index + 1}"
            ) from exc
        if exc is not None:
            raise RuntimeError(f"ordered-chain planning failed: {exc}") from exc
        if planned_index is None or planned is None:
            raise RuntimeError(
                f"ordered-chain planner ended before segment {expected_index + 1}"
            )
        if int(planned_index) != int(expected_index):
            raise RuntimeError(
                f"ordered-chain planner returned segment {int(planned_index) + 1}, "
                f"expected {expected_index + 1}"
            )
        return planned


def build_ordered_scheduler_runtime(
    *,
    segments: Sequence[dict[str, Any]],
    scheduler_group_status: Sequence[dict[str, Any]] = (),
    scheduler_segment_states: dict[int, dict[str, Any]] | None = None,
    set_ordered_motion_chain_status: Callable[..., None],
) -> OrderedSchedulerRuntime:
    """Build ordered scheduler bridge plus status publication callbacks."""

    segment_states = dict(scheduler_segment_states or {})
    if not segment_states:
        segment_states = ordered_chain_initial_segment_states_from_mappings(segments)

    bridge = OrderedSchedulerBridge(
        scheduler_group_status=tuple(scheduler_group_status or ()),
        scheduler_segment_states=segment_states,
    )

    def _publish_scheduler_updates(updates):
        if updates:
            set_ordered_motion_chain_status(**updates)

    def _publish_planned(index, planned_segment):
        _publish_scheduler_updates(
            bridge.publish_ready_segment(index, planned_segment)
        )

    return OrderedSchedulerRuntime(
        bridge=bridge,
        publish_scheduler_updates=_publish_scheduler_updates,
        publish_planned=_publish_planned,
    )


__all__ = [
    "OrderedSchedulerBridge",
    "OrderedSchedulerRuntime",
    "build_ordered_scheduler_runtime",
]
