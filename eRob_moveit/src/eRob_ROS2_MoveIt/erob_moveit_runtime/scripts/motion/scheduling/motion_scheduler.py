#!/usr/bin/env python3
"""Motion batch grouping helpers.

This module is intentionally planning-free for now. It partitions typed motion
batches into contiguous groups so the existing ordered-chain implementation can
be migrated toward a general scheduler without changing execution behavior in
the same slice.
"""

from __future__ import annotations

from dataclasses import dataclass

from motion.planning.planning_types import (
    LinearSegment,
    MotionSegment,
    PathSegment,
    PtpSegment,
    UnwindSegment,
)
from motion.scheduling.scheduling_types import MotionBatch, MotionGroup


def planner_name_for_segments(segments: tuple[MotionSegment, ...]) -> str:
    """Return the scheduler planner key for a compatible segment group."""

    if not segments:
        raise ValueError("planner_name_for_segments requires at least one segment")

    first = segments[0]
    if isinstance(first, LinearSegment):
        return "linked_lin" if len(segments) > 1 else "lin"
    if isinstance(first, PtpSegment):
        return "ptp"
    if isinstance(first, PathSegment):
        return "path"
    if isinstance(first, UnwindSegment):
        return "unwind"
    return "unknown"


def segments_are_group_compatible(left: MotionSegment, right: MotionSegment) -> bool:
    """Return whether two adjacent segments may be planned in one group."""

    if isinstance(left, (PathSegment, UnwindSegment)):
        return False
    if isinstance(right, (PathSegment, UnwindSegment)):
        return False
    if isinstance(left, LinearSegment) and isinstance(right, LinearSegment):
        return True
    if isinstance(left, PtpSegment) and isinstance(right, PtpSegment):
        return True
    return False


def has_hard_stop_after(segment: MotionSegment) -> bool:
    """Return whether execution/planning should hard-stop after this segment."""

    if isinstance(segment, (PathSegment, UnwindSegment)):
        return True
    return float(segment.blend_radius) <= 0.0


@dataclass(frozen=True)
class MotionScheduler:
    """Pure grouping facade for the first scheduler migration slice."""

    def group_batch(self, batch: MotionBatch) -> tuple[MotionGroup, ...]:
        return group_motion_batch(batch)


def group_motion_batch(batch: MotionBatch) -> tuple[MotionGroup, ...]:
    """Partition a motion batch into contiguous compatible planning groups."""

    segments = tuple(batch.segments)
    groups: list[MotionGroup] = []
    group_start = 0
    current: list[MotionSegment] = []

    def flush(hard_stop_after: bool) -> None:
        nonlocal group_start, current
        if not current:
            return
        group_segments = tuple(current)
        groups.append(
            MotionGroup(
                segments=group_segments,
                start_index=group_start,
                planner_name=planner_name_for_segments(group_segments),
                hard_stop_after=hard_stop_after,
                metadata={
                    "source": "motion_scheduler",
                    "end_index": group_start + len(group_segments) - 1,
                },
            )
        )
        current = []
        group_start = group_start + len(group_segments)

    for index, segment in enumerate(segments):
        if not current:
            group_start = index
            current.append(segment)
        else:
            previous = current[-1]
            if has_hard_stop_after(previous) or not segments_are_group_compatible(previous, segment):
                flush(hard_stop_after=has_hard_stop_after(previous))
                group_start = index
            current.append(segment)

        if has_hard_stop_after(segment):
            flush(hard_stop_after=True)

    if current:
        flush(hard_stop_after=has_hard_stop_after(current[-1]))

    return tuple(groups)


__all__ = [
    "MotionScheduler",
    "group_motion_batch",
    "has_hard_stop_after",
    "planner_name_for_segments",
    "segments_are_group_compatible",
]
