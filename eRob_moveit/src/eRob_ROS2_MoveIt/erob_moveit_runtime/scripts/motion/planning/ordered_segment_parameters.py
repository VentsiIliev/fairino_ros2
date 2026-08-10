#!/usr/bin/env python3
"""Parameter parsing for legacy ordered-chain segment planning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class OrderedSegmentParameters:
    segment_type: str
    label: str
    blend_radius: float
    velocity_percent: float
    acceleration_percent: float
    velocity_scale: float
    acceleration_scale: float
    protected: bool


def _scale_from_percent(value: float) -> float:
    return max(0.0, min(1.0, float(value) / 100.0))


def parse_ordered_segment_parameters(
    segment: Mapping[str, Any],
    index: int,
    *,
    default_velocity_percent: float,
    default_acceleration_percent: float,
) -> OrderedSegmentParameters:
    """Parse common fields from a legacy ordered-chain segment dictionary."""

    segment_type = str(segment.get("type") or "").strip().lower()
    label = str(segment.get("label") or f"segment_{index + 1}")

    # Preserve the legacy planner behavior: negative blendR is clamped here.
    blend_radius = max(0.0, float(segment.get("blendR", 0.0) or 0.0))
    velocity_percent = float(segment.get("vel", default_velocity_percent))
    acceleration_percent = float(segment.get("acc", default_acceleration_percent))

    return OrderedSegmentParameters(
        segment_type=segment_type,
        label=label,
        blend_radius=blend_radius,
        velocity_percent=velocity_percent,
        acceleration_percent=acceleration_percent,
        velocity_scale=_scale_from_percent(velocity_percent),
        acceleration_scale=_scale_from_percent(acceleration_percent),
        protected=bool(segment.get("protected", False)),
    )

