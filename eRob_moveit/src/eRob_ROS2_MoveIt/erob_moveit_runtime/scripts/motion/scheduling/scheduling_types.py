#!/usr/bin/env python3
"""Typed scheduling models for batched motion execution."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from motion.planning.planning_types import MotionSegment, PlannedTrajectory


class MotionGroupState(str, Enum):
    PENDING = "PENDING"
    PLANNING = "PLANNING"
    READY = "READY"
    EXECUTING = "EXECUTING"
    DONE = "DONE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class MotionBatch:
    segments: tuple[MotionSegment, ...]
    blocking: bool = True
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if not self.segments:
            raise ValueError("MotionBatch requires at least one segment")


@dataclass(frozen=True)
class MotionGroup:
    segments: tuple[MotionSegment, ...]
    start_index: int
    planner_name: str
    hard_stop_after: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if not self.segments:
            raise ValueError("MotionGroup requires at least one segment")
        if self.start_index < 0:
            raise ValueError("MotionGroup start_index must be >= 0")


@dataclass
class PlannedMotionGroup:
    group: MotionGroup
    planned: tuple[PlannedTrajectory, ...]
    state: MotionGroupState = MotionGroupState.READY
    result: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

