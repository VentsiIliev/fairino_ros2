#!/usr/bin/env python3

from types import SimpleNamespace

import pytest

from motion.execution.unwind_dynamics_guard import enforce_unwind_joint_dynamics


class _Logger:
    def warning(self, _message):
        pass


def _duration(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=round((seconds - whole) * 1e9))


def _trajectory(samples):
    return SimpleNamespace(
        joint_names=["Joint_6"],
        points=[
            SimpleNamespace(
                positions=[position],
                velocities=[] if velocity is None else [velocity],
                accelerations=[] if acceleration is None else [acceleration],
                time_from_start=_duration(time_s),
            )
            for time_s, position, velocity, acceleration in samples
        ],
    )


def test_uses_totg_derivatives_instead_of_uneven_secant_acceleration():
    trajectory = _trajectory(
        [
            (0.0, 0.0, 0.0, 4.0),
            (1.0, 1.0, 2.0, 4.0),
            (1.1, 1.2, 2.0, 0.0),
        ]
    )

    scale = enforce_unwind_joint_dynamics(
        trajectory,
        _Logger(),
        joint_name="Joint_6",
        velocity_limit_rad_s=5.0,
        acceleration_limit_rad_s2=6.0,
    )

    assert scale == pytest.approx(1.0)
    assert trajectory.points[-1].time_from_start.sec == 1
    assert trajectory.points[-1].time_from_start.nanosec == 100_000_000


def test_scales_totg_acceleration_derivatives_when_limit_is_exceeded():
    trajectory = _trajectory(
        [
            (0.0, 0.0, 0.0, 24.0),
            (1.0, 1.0, 2.0, 0.0),
        ]
    )

    scale = enforce_unwind_joint_dynamics(
        trajectory,
        _Logger(),
        joint_name="Joint_6",
        velocity_limit_rad_s=5.0,
        acceleration_limit_rad_s2=6.0,
    )

    assert scale == pytest.approx(2.0)
    assert trajectory.points[-1].time_from_start.sec == 2
    assert trajectory.points[0].accelerations[0] == pytest.approx(6.0)


def test_fallback_acceleration_uses_interval_midpoints():
    trajectory = _trajectory(
        [
            (0.0, 0.0, None, None),
            (1.0, 1.0, None, None),
            (1.1, 1.2, None, None),
        ]
    )

    scale = enforce_unwind_joint_dynamics(
        trajectory,
        _Logger(),
        joint_name="Joint_6",
        velocity_limit_rad_s=5.0,
        acceleration_limit_rad_s2=2.0,
    )

    assert scale == pytest.approx(1.0)
