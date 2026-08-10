#!/usr/bin/env python3
"""Typed planning models shared by motion planners and schedulers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from moveit_msgs.msg import RobotState
from trajectory_msgs.msg import JointTrajectory


@dataclass(frozen=True)
class MotionSegment:
    """Base internal motion segment model."""

    label: str
    velocity: float
    acceleration: float
    blend_radius: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class LinearSegment(MotionSegment):
    target: tuple[float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


@dataclass(frozen=True)
class PtpSegment(MotionSegment):
    target: tuple[float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


@dataclass(frozen=True)
class PathSegment(MotionSegment):
    waypoints: tuple[tuple[float, ...], ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class UnwindSegment(MotionSegment):
    joint_name: str = "Joint_6"
    queue_if_busy: bool = True


@dataclass(frozen=True)
class PlannedTrajectory:
    trajectory: JointTrajectory
    start_state: RobotState | None
    end_state: RobotState | None
    planning_time: float
    metadata: dict[str, Any] = field(default_factory=dict)

