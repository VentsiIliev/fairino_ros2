#!/usr/bin/env python3
"""Absolute Joint-6 velocity/acceleration guard for unwind trajectories."""

from __future__ import annotations

import math
from typing import Any


def _seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _set_seconds(duration: Any, seconds: float) -> None:
    seconds = max(0.0, float(seconds))
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    duration.sec = sec
    duration.nanosec = nanosec


def enforce_unwind_joint_dynamics(
    joint_trajectory: Any,
    logger: Any,
    *,
    joint_name: str,
    velocity_limit_rad_s: float,
    acceleration_limit_rad_s2: float,
) -> float:
    """Stretch trajectory timing to satisfy absolute velocity/acceleration limits."""
    points = list(getattr(joint_trajectory, "points", []) or [])
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    if len(points) < 2 or joint_name not in joint_names:
        return 1.0

    velocity_limit = float(velocity_limit_rad_s)
    acceleration_limit = float(acceleration_limit_rad_s2)
    if velocity_limit <= 0.0 or acceleration_limit <= 0.0:
        raise ValueError("Unwind velocity and acceleration limits must be positive")

    joint_index = joint_names.index(joint_name)
    interval_velocities: list[tuple[float, float]] = []
    peak_velocity = 0.0
    peak_acceleration = 0.0
    for previous, current in zip(points, points[1:]):
        dt = _seconds(current.time_from_start) - _seconds(previous.time_from_start)
        if dt <= 1e-9:
            continue
        velocity = (
            float(current.positions[joint_index])
            - float(previous.positions[joint_index])
        ) / dt
        interval_velocities.append((velocity, dt))
        peak_velocity = max(peak_velocity, abs(velocity))

    # Prefer derivatives emitted by the time parameterizer.  A secant velocity
    # belongs at the midpoint of its interval, not at the interval endpoint;
    # differencing it over only the following interval can hugely overestimate
    # acceleration when TOTG produces uneven sample spacing.
    has_point_velocities = all(
        joint_index < len(list(getattr(point, "velocities", []) or []))
        for point in points
    )
    has_point_accelerations = all(
        joint_index < len(list(getattr(point, "accelerations", []) or []))
        for point in points
    )

    if not has_point_accelerations:
        for (previous_velocity, previous_dt), (current_velocity, current_dt) in zip(
            interval_velocities,
            interval_velocities[1:],
        ):
            midpoint_dt = 0.5 * (previous_dt + current_dt)
            if midpoint_dt <= 1e-9:
                continue
            peak_acceleration = max(
                peak_acceleration,
                abs(current_velocity - previous_velocity) / midpoint_dt,
            )

    for point in points:
        velocities = list(getattr(point, "velocities", []) or [])
        accelerations = list(getattr(point, "accelerations", []) or [])
        if has_point_velocities:
            peak_velocity = max(peak_velocity, abs(float(velocities[joint_index])))
        if has_point_accelerations:
            peak_acceleration = max(
                peak_acceleration,
                abs(float(accelerations[joint_index])),
            )

    velocity_scale = peak_velocity / velocity_limit
    acceleration_scale = math.sqrt(peak_acceleration / acceleration_limit)
    required_scale = max(1.0, velocity_scale, acceleration_scale)
    if required_scale > 1.001:
        for point in points:
            _set_seconds(
                point.time_from_start,
                _seconds(point.time_from_start) * required_scale,
            )
            if getattr(point, "velocities", None):
                point.velocities = [
                    float(value) / required_scale for value in point.velocities
                ]
            if getattr(point, "accelerations", None):
                point.accelerations = [
                    float(value) / (required_scale * required_scale)
                    for value in point.accelerations
                ]

    logger.warning(
        "[UNWIND_J6_LIMIT] "
        f"{joint_name} peak_vel={peak_velocity:.3f}rad/s "
        f"vel_limit={velocity_limit:.3f}rad/s "
        f"peak_acc={peak_acceleration:.3f}rad/s^2 "
        f"acc_limit={acceleration_limit:.3f}rad/s^2 "
        f"time_scale={required_scale:.3f} "
        f"duration={_seconds(points[-1].time_from_start):.3f}s"
    )
    return required_scale


__all__ = ["enforce_unwind_joint_dynamics"]
