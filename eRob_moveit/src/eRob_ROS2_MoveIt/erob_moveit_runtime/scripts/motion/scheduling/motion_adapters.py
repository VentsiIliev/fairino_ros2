#!/usr/bin/env python3
"""Adapters from public request dictionaries to internal motion models."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, cast

from motion.planning.planning_types import (
    LinearSegment,
    MotionSegment,
    PathSegment,
    PtpSegment,
    UnwindSegment,
)
from motion.scheduling.motion_scheduler import group_motion_batch
from motion.scheduling.scheduling_types import MotionBatch
from motion.scheduling.status_adapter import (
    ordered_chain_group_status,
    ordered_chain_initial_group_states,
    ordered_chain_initial_segment_states,
)

DEFAULT_VELOCITY_PERCENT = 30.0
DEFAULT_ACCELERATION_PERCENT = 30.0


@dataclass(frozen=True)
class OrderedMotionBatchValidation:
    motion_batch: MotionBatch
    scheduler_group_status: tuple[dict[str, Any], ...]
    scheduler_group_states: dict[int, dict[str, Any]]
    scheduler_segment_states: dict[int, dict[str, Any]]
    segment_counts_text: str
    group_summary: str

    def log_message(self) -> str:
        return (
            "[OrderedChain] MotionBatch adapter validated "
            f"segments={len(self.motion_batch.segments)} "
            f"blocking={self.motion_batch.blocking} "
            f"types={self.segment_counts_text} groups={self.group_summary}"
        )

    def timing_fields(self) -> dict[str, Any]:
        return {
            "segments": len(self.motion_batch.segments),
            "types": self.segment_counts_text,
            "groups": self.group_summary,
        }


@dataclass(frozen=True)
class OrderedMotionBatchValidationFailure:
    error: str

    def log_message(self) -> str:
        return (
            "[OrderedChain] MotionBatch adapter validation failed; "
            f"continuing with legacy ordered-chain executor: {self.error}"
        )

    def timing_fields(self) -> dict[str, Any]:
        return {"error": self.error}


def _as_float(value: Any, default: float) -> float:
    if value is None:
        return float(default)
    return float(value)


def _as_pose6(value: Any, label: str) -> tuple[float, float, float, float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 numeric values")
    return cast(tuple[float, float, float, float, float, float], tuple(float(item) for item in value))


def _as_waypoints(value: Any, label: str) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or not value:
        raise ValueError(f"{label} must contain at least one waypoint")

    waypoints: list[tuple[float, ...]] = []
    for index, waypoint in enumerate(value):
        if not isinstance(waypoint, Sequence) or isinstance(waypoint, (str, bytes)):
            raise ValueError(f"{label}[{index}] must be a waypoint sequence")
        if len(waypoint) not in {3, 6}:
            raise ValueError(f"{label}[{index}] must contain 3 or 6 numeric values")
        waypoints.append(tuple(float(item) for item in waypoint))
    return tuple(waypoints)


def ordered_segment_from_mapping(
    raw_segment: Mapping[str, Any],
    index: int,
    *,
    default_velocity: float = DEFAULT_VELOCITY_PERCENT,
    default_acceleration: float = DEFAULT_ACCELERATION_PERCENT,
) -> MotionSegment:
    """Convert one normalized ordered-chain segment dictionary to a typed model."""

    segment_type = str(raw_segment.get("type") or raw_segment.get("kind") or "").strip().lower()
    label = str(raw_segment.get("label") or f"segment_{index + 1}")
    velocity = _as_float(raw_segment.get("vel"), default_velocity)
    acceleration = _as_float(raw_segment.get("acc"), default_acceleration)
    blend_radius = _as_float(raw_segment.get("blendR"), 0.0)
    if blend_radius < 0.0:
        raise ValueError(f"segment {index} blendR must be >= 0")

    metadata = {
        "source": "ordered_motion_chain",
        "source_index": index,
        "raw_type": segment_type,
    }

    if segment_type == "linear":
        return LinearSegment(
            label=label,
            velocity=velocity,
            acceleration=acceleration,
            blend_radius=blend_radius,
            target=_as_pose6(raw_segment.get("position"), f"segment {index} position"),
            metadata=metadata,
        )

    if segment_type == "ptp":
        return PtpSegment(
            label=label,
            velocity=velocity,
            acceleration=acceleration,
            blend_radius=blend_radius,
            target=_as_pose6(raw_segment.get("position"), f"segment {index} position"),
            metadata=metadata,
        )

    if segment_type == "path":
        if blend_radius > 0.0:
            raise ValueError(f"segment {index} path cannot use blendR")
        return PathSegment(
            label=label,
            velocity=velocity,
            acceleration=acceleration,
            blend_radius=0.0,
            waypoints=_as_waypoints(raw_segment.get("path"), f"segment {index} path"),
            metadata=metadata,
        )

    if segment_type == "unwind_joint6":
        if blend_radius > 0.0:
            raise ValueError(f"segment {index} unwind_joint6 cannot use blendR")
        return UnwindSegment(
            label=label,
            velocity=velocity,
            acceleration=acceleration,
            blend_radius=0.0,
            joint_name=str(raw_segment.get("joint_name") or "Joint_6"),
            queue_if_busy=bool(raw_segment.get("queue_if_busy", True)),
            metadata=metadata,
        )

    raise ValueError(f"segment {index} has unsupported type {segment_type!r}")


def ordered_segments_from_mappings(
    raw_segments: Sequence[Mapping[str, Any]],
    *,
    default_velocity: float = DEFAULT_VELOCITY_PERCENT,
    default_acceleration: float = DEFAULT_ACCELERATION_PERCENT,
) -> tuple[MotionSegment, ...]:
    if not raw_segments:
        raise ValueError("ordered motion chain requires at least one segment")
    return tuple(
        ordered_segment_from_mapping(
            raw_segment,
            index,
            default_velocity=default_velocity,
            default_acceleration=default_acceleration,
        )
        for index, raw_segment in enumerate(raw_segments)
    )


def ordered_motion_batch_from_mappings(
    raw_segments: Sequence[Mapping[str, Any]],
    *,
    blocking: bool = True,
    tool: int | None = None,
    user: int | None = None,
    trajectory_optimizer: str | None = None,
    default_velocity: float = DEFAULT_VELOCITY_PERCENT,
    default_acceleration: float = DEFAULT_ACCELERATION_PERCENT,
) -> MotionBatch:
    metadata = {
        "source": "ordered_motion_chain",
        "tool": tool,
        "user": user,
        "trajectory_optimizer": trajectory_optimizer,
    }
    return MotionBatch(
        segments=ordered_segments_from_mappings(
            raw_segments,
            default_velocity=default_velocity,
            default_acceleration=default_acceleration,
        ),
        blocking=bool(blocking),
        metadata=metadata,
    )


def validate_ordered_motion_batch_from_mappings(
    raw_segments: Sequence[Mapping[str, Any]],
    *,
    blocking: bool = True,
    tool: int | None = None,
    user: int | None = None,
    trajectory_optimizer: str | None = None,
    default_velocity: float = DEFAULT_VELOCITY_PERCENT,
    default_acceleration: float = DEFAULT_ACCELERATION_PERCENT,
) -> OrderedMotionBatchValidation:
    """Return typed batch and legacy scheduler status metadata for a request."""

    motion_batch = ordered_motion_batch_from_mappings(
        raw_segments,
        blocking=blocking,
        tool=tool,
        user=user,
        trajectory_optimizer=trajectory_optimizer,
        default_velocity=default_velocity,
        default_acceleration=default_acceleration,
    )
    segment_counts: dict[str, int] = {}
    for motion_segment in motion_batch.segments:
        segment_name = motion_segment.__class__.__name__
        segment_counts[segment_name] = segment_counts.get(segment_name, 0) + 1
    segment_counts_text = ",".join(
        f"{name}:{count}" for name, count in sorted(segment_counts.items())
    )
    motion_groups = group_motion_batch(motion_batch)
    scheduler_group_status = ordered_chain_group_status(motion_groups)
    scheduler_group_states = ordered_chain_initial_group_states(scheduler_group_status)
    scheduler_segment_states = ordered_chain_initial_segment_states(motion_batch.segments)
    group_summary = ",".join(
        (
            f"{group.start_index + 1}:"
            f"{group.planner_name}:"
            f"{len(group.segments)}:"
            f"{'hard' if group.hard_stop_after else 'soft'}"
        )
        for group in motion_groups
    )
    return OrderedMotionBatchValidation(
        motion_batch=motion_batch,
        scheduler_group_status=scheduler_group_status,
        scheduler_group_states=scheduler_group_states,
        scheduler_segment_states=scheduler_segment_states,
        segment_counts_text=segment_counts_text,
        group_summary=group_summary,
    )
