#!/usr/bin/env python3
"""Compatibility status helpers for ordered-chain scheduler migration."""

from __future__ import annotations

from typing import Any

from motion.planning.planning_types import MotionSegment
from motion.scheduling.scheduling_types import MotionGroup


INACTIVE_ORDERED_CHAIN_FIELDS = {
    "current_segment_index": None,
    "current_segment_number": None,
    "current_segment_label": None,
    "current_segment_type": None,
    "current_segment_protected": False,
    "preplanned_ready_count": 0,
    "next_preplanned_segment_index": None,
    "next_preplanned_segment_number": None,
    "next_preplanned_segment_label": None,
    "next_preplanned_segment_type": None,
}


def ordered_chain_group_status(groups: tuple[MotionGroup, ...]) -> tuple[dict[str, Any], ...]:
    """Return JSON-safe scheduler group metadata for legacy status payloads."""

    group_status: list[dict[str, Any]] = []
    for group_index, group in enumerate(groups):
        end_index = group.start_index + len(group.segments) - 1
        group_status.append(
            {
                "group_index": group_index,
                "group_number": group_index + 1,
                "start_segment_index": group.start_index,
                "start_segment_number": group.start_index + 1,
                "end_segment_index": end_index,
                "end_segment_number": end_index + 1,
                "planner_name": group.planner_name,
                "segment_count": len(group.segments),
                "hard_stop_after": bool(group.hard_stop_after),
            }
        )
    return tuple(group_status)


def ordered_chain_initial_group_states(
    group_status: tuple[dict[str, Any], ...],
) -> dict[int, dict[str, Any]]:
    """Return initial scheduler-visible group state from group metadata."""

    return {
        int(group["group_index"]): {
            **dict(group),
            "state": "PENDING",
            "result": None,
        }
        for group in group_status
    }


def _group_state_from_segment_states(member_states: list[str]) -> str:
    if not member_states:
        return "PENDING"
    if any(state == "FAILED" for state in member_states):
        return "FAILED"
    if any(state == "EXECUTING" for state in member_states):
        return "EXECUTING"
    if all(state == "DONE" for state in member_states):
        return "DONE"
    if all(state in {"READY", "DONE"} for state in member_states):
        return "READY"
    return "PENDING"


def update_ordered_chain_group_states_from_segments(
    group_states: dict[int, dict[str, Any]],
    segment_states: dict[int, dict[str, Any]],
) -> None:
    """Update group state in-place from member segment states."""

    for group in group_states.values():
        start = int(group["start_segment_index"])
        end = int(group["end_segment_index"])
        members = [
            str(segment_states[index].get("state") or "PENDING")
            for index in range(start, end + 1)
            if index in segment_states
        ]
        state = _group_state_from_segment_states(members)
        group["state"] = state
        if state == "FAILED":
            failed_results = [
                segment_states[index].get("result")
                for index in range(start, end + 1)
                if index in segment_states and segment_states[index].get("state") == "FAILED"
            ]
            group["result"] = failed_results[0] if failed_results else -1
        elif state == "DONE":
            group["result"] = 0
        elif state in {"PENDING", "READY", "EXECUTING"}:
            group["result"] = None


def ordered_chain_group_state_status(
    group_states: dict[int, dict[str, Any]],
) -> tuple[dict[str, Any], ...]:
    """Return JSON-safe scheduler group state metadata."""

    return tuple(dict(group_states[index]) for index in sorted(group_states))


def ordered_chain_initial_segment_states(
    segments: tuple[MotionSegment, ...],
) -> dict[int, dict[str, Any]]:
    """Return initial scheduler-visible segment state for a motion batch."""

    return {
        index: {
            "segment_index": index,
            "segment_number": index + 1,
            "label": segment.label,
            "type": segment.metadata.get("raw_type") or segment.__class__.__name__,
            "state": "PENDING",
            "result": None,
        }
        for index, segment in enumerate(segments)
    }


def ordered_chain_initial_segment_states_from_mappings(
    segments,
) -> dict[int, dict[str, Any]]:
    """Return initial scheduler-visible segment state from legacy mappings."""

    return {
        index: {
            "segment_index": index,
            "segment_number": index + 1,
            "label": str(segment.get("label") or f"segment_{index + 1}"),
            "type": str(segment.get("type") or ""),
            "state": "PENDING",
            "result": None,
        }
        for index, segment in enumerate(segments)
    }


def ordered_chain_segment_state_status(
    segment_states: dict[int, dict[str, Any]],
) -> tuple[dict[str, Any], ...]:
    """Return JSON-safe scheduler segment state metadata."""

    return tuple(dict(segment_states[index]) for index in sorted(segment_states))


def normalize_ordered_chain_status(
    previous: dict[str, Any] | None,
    updates: dict[str, Any],
    *,
    updated_at: float,
) -> dict[str, Any]:
    """Merge ordered-chain status updates and apply compatibility resets."""

    status = dict(previous or {})
    status.update(updates)
    if status.get("active") is False:
        status.update(INACTIVE_ORDERED_CHAIN_FIELDS)
    status["updated_at"] = float(updated_at)
    return status


def ordered_chain_starting_status(
    *,
    total_segments: int,
    scheduler_group_status: tuple[dict[str, Any], ...],
    scheduler_group_states: dict[int, dict[str, Any]],
    scheduler_segment_states: dict[int, dict[str, Any]],
) -> dict[str, Any]:
    """Return legacy status fields for a newly accepted ordered chain."""

    return {
        "active": True,
        "phase": "starting",
        "total_segments": int(total_segments),
        "current_segment_index": None,
        "current_segment_number": None,
        "current_segment_label": None,
        "current_segment_type": None,
        "current_segment_protected": False,
        "scheduler_groups": list(scheduler_group_states.values()),
        "scheduler_group_count": len(scheduler_group_status),
        "scheduler_segment_states": list(scheduler_segment_states.values()),
        "planned_segments_count": 0,
        "executed_segments_count": 0,
        "preplanned_ready_count": 0,
        "next_preplanned_segment_index": None,
        "next_preplanned_segment_number": None,
        "next_preplanned_segment_label": None,
        "next_preplanned_segment_type": None,
        "last_planned_segment_index": None,
        "last_planned_segment_number": None,
        "last_planned_segment_label": None,
        "last_planned_segment_type": None,
        "result": None,
    }


def ordered_chain_executing_status(
    *,
    index: int,
    total: int,
    planned_segment: dict[str, Any],
    preplanned_ready_count: int = 0,
) -> dict[str, Any]:
    """Return legacy status fields for a segment entering execution."""

    return {
        "active": True,
        "phase": "executing",
        "total_segments": int(total),
        "current_segment_index": int(index) - 1,
        "current_segment_number": int(index),
        "current_segment_label": planned_segment.get("label"),
        "current_segment_type": planned_segment.get("type"),
        "current_segment_protected": bool(planned_segment.get("protected", False)),
        "executed_segments_count": max(0, int(index) - 1),
        "preplanned_ready_count": int(preplanned_ready_count),
        "result": None,
    }


def ordered_chain_segment_finished_status(
    *,
    index: int,
    planned_segment: dict[str, Any],
    result: Any,
) -> dict[str, Any]:
    """Return legacy status fields for a completed or failed segment."""

    normalized_result = int(result) if isinstance(result, int) else result
    return {
        "active": True,
        "phase": "segment_completed" if result == 0 else "segment_failed",
        "current_segment_index": int(index) - 1,
        "current_segment_number": int(index),
        "current_segment_label": planned_segment.get("label"),
        "current_segment_type": planned_segment.get("type"),
        "current_segment_protected": bool(planned_segment.get("protected", False)),
        "executed_segments_count": int(index) if result == 0 else max(0, int(index) - 1),
        "result": normalized_result,
    }


def ordered_chain_stopped_status(
    *,
    index: int,
    planned_segment: dict[str, Any],
    result: Any = -14,
) -> dict[str, Any]:
    """Return legacy status fields for an ordered-chain stop before execution."""

    normalized_result = int(result) if isinstance(result, int) else result
    return {
        "active": False,
        "phase": "stopped",
        "current_segment_index": int(index) - 1,
        "current_segment_number": int(index),
        "current_segment_label": planned_segment.get("label"),
        "current_segment_type": planned_segment.get("type"),
        "current_segment_protected": bool(planned_segment.get("protected", False)),
        "result": normalized_result,
    }


def ordered_chain_terminal_status(
    *,
    phase: str,
    result: Any,
    error: str | None = None,
) -> dict[str, Any]:
    """Return legacy status fields for terminal ordered-chain phases."""

    normalized_result = int(result) if isinstance(result, int) else result
    status = {
        "active": False,
        "phase": str(phase),
        "result": normalized_result,
    }
    if error is not None:
        status["error"] = str(error)
    return status


def ordered_chain_preplanned_snapshot(
    planned_by_index: dict[int, dict[str, Any]],
    *,
    current_index: int = 0,
) -> dict[str, Any]:
    """Return legacy preplanned segment fields from scheduler planning state."""

    ready_indexes = sorted(index for index in planned_by_index if index >= current_index)
    last_index = max(planned_by_index) if planned_by_index else None
    last_planned = planned_by_index.get(last_index) if last_index is not None else None
    next_index = ready_indexes[0] if ready_indexes else None
    next_planned = planned_by_index.get(next_index) if next_index is not None else None
    return {
        "planned_segments_count": len(planned_by_index),
        "preplanned_ready_count": len(ready_indexes),
        "next_preplanned_segment_index": next_index,
        "next_preplanned_segment_number": next_index + 1 if next_index is not None else None,
        "next_preplanned_segment_label": next_planned.get("label") if next_planned else None,
        "next_preplanned_segment_type": next_planned.get("type") if next_planned else None,
        "last_planned_segment_index": last_index,
        "last_planned_segment_number": last_index + 1 if last_index is not None else None,
        "last_planned_segment_label": last_planned.get("label") if last_planned else None,
        "last_planned_segment_type": last_planned.get("type") if last_planned else None,
    }


__all__ = [
    "INACTIVE_ORDERED_CHAIN_FIELDS",
    "normalize_ordered_chain_status",
    "ordered_chain_executing_status",
    "ordered_chain_group_state_status",
    "ordered_chain_group_status",
    "ordered_chain_initial_group_states",
    "ordered_chain_initial_segment_states",
    "ordered_chain_preplanned_snapshot",
    "ordered_chain_segment_finished_status",
    "ordered_chain_segment_state_status",
    "ordered_chain_stopped_status",
    "ordered_chain_starting_status",
    "ordered_chain_terminal_status",
    "update_ordered_chain_group_states_from_segments",
]
