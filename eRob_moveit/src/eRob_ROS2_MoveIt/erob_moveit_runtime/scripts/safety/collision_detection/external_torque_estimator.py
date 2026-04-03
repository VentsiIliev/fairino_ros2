#!/usr/bin/env python3
"""
External torque estimators.

Contains both the original inverse-dynamics residual estimator and a
momentum-observer style estimator that avoids explicit acceleration use.
"""

import time
import numpy as np
from collections import deque
from typing import Optional
import config

from .inverse_dynamics_model import InverseDynamicsModel


class ExternalTorqueEstimator:
    """
    Estimates external torques acting on the robot joints.

    Uses an InverseDynamicsModel to compute expected torques, then
    subtracts from measured torques. Applies a low-pass filter to
    reduce sensor noise.
    """

    def __init__(self, model: InverseDynamicsModel, filter_alpha: float = config.COLLISION_FILTER_ALPHA):
        """
        Args:
            model: Inverse dynamics model for computing expected torques.
            filter_alpha: Low-pass filter coefficient (0-1). Higher = faster response.
        """
        self._model = model
        self._filter_alpha = filter_alpha
        n = model.num_joints

        self._filtered = np.zeros(n)
        self._previous = np.zeros(n)
        self._rate = np.zeros(n)

        self.expected_torque_history = deque(maxlen=config.COLLISION_HISTORY_BUFFER)
        self.external_torque_history = deque(maxlen=config.COLLISION_HISTORY_BUFFER)

    @property
    def num_joints(self) -> int:
        return self._model.num_joints

    @property
    def filtered(self) -> np.ndarray:
        """Current filtered external torque estimate."""
        return self._filtered.copy()

    @property
    def previous(self) -> np.ndarray:
        """Previous filtered external torque (before last update)."""
        return self._previous.copy()

    @property
    def rate(self) -> np.ndarray:
        """Rate of change of filtered external torque (last update)."""
        return self._rate.copy()

    def update(
        self,
        measured: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ):
        """
        Update the external torque estimate with new measurements.

        Computes expected torque, derives external torque, applies
        low-pass filter, and computes rate of change.

        Args:
            measured: Measured joint torques (N*m)
            positions: Joint positions (rad)
            velocities: Joint velocities (rad/s)
            accelerations: Joint accelerations (rad/s^2)
        """
        tau_expected = self._model.compute_expected_torque(positions, velocities, accelerations)
        tau_external = measured - tau_expected

        self.expected_torque_history.append(tau_expected.copy())

        self._previous = self._filtered.copy()
        self._filtered = (
            self._filter_alpha * tau_external +
            (1 - self._filter_alpha) * self._filtered
        )
        self._rate = np.abs(self._filtered - self._previous)

        self.external_torque_history.append(self._filtered.copy())

    def reset(self):
        """Zero the filter state."""
        n = self._model.num_joints
        self._filtered = np.zeros(n)
        self._previous = np.zeros(n)
        self._rate = np.zeros(n)


class MomentumObserverEstimator:
    """
    Momentum-based residual estimator.

    This is a practical approximation of the observer family used in
    sensorless collision papers: it uses joint-space momentum `p=M(q)dq`
    and model bias torque instead of explicit joint acceleration.

    The residual still benefits from low-pass filtering so it can be used
    with the existing collision thresholds and GUI.

    static_torque_bias_nm: per-joint offset (Nm) subtracted from measured
    before integration.  Set each element to the drive_output_torque seen
    at rest for that joint (gravity compensation mismatch between URDF and
    drive firmware).  E.g. J3 ≈ 3.68 Nm → [0, 0, 3.68, 0, 0, 0].
    """

    def __init__(
        self,
        model: InverseDynamicsModel,
        filter_alpha: float = config.COLLISION_FILTER_ALPHA,
        observer_gain: float = 10.0,
        static_torque_bias_nm: Optional[np.ndarray] = None,
        warmup_sec: float = 8.0,
        warmup_vel_threshold_rad_s: float = 0.02,
    ):
        self._model = model
        self._filter_alpha = filter_alpha
        self._observer_gain = observer_gain
        n = model.num_joints
        self._static_bias = (
            np.asarray(static_torque_bias_nm, dtype=float)
            if static_torque_bias_nm is not None
            else np.zeros(n)
        )

        self._filtered = np.zeros(n)
        self._previous = np.zeros(n)
        self._rate = np.zeros(n)
        self._observer_residual = np.zeros(n)
        self._momentum_integral = np.zeros(n)
        self._last_momentum: Optional[np.ndarray] = None
        self._last_timestamp: Optional[float] = None

        self._warmup_sec = warmup_sec
        self._warmup_vel_threshold = warmup_vel_threshold_rad_s
        self._warmup_samples: list = []
        self._warmup_offset = np.zeros(n)
        self._warmup_done = warmup_sec <= 0.0
        self._warmup_start: Optional[float] = None

        self.expected_torque_history = deque(maxlen=config.COLLISION_HISTORY_BUFFER)
        self.external_torque_history = deque(maxlen=config.COLLISION_HISTORY_BUFFER)

    @property
    def num_joints(self) -> int:
        return self._model.num_joints

    @property
    def filtered(self) -> np.ndarray:
        return self._filtered.copy()

    @property
    def previous(self) -> np.ndarray:
        return self._previous.copy()

    @property
    def rate(self) -> np.ndarray:
        return self._rate.copy()

    def update(
        self,
        measured: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ):
        mass_matrix = self._model.compute_mass_matrix(positions)
        momentum = mass_matrix @ velocities
        bias_torque = self._model.compute_bias_torque(positions, velocities)
        now = time.monotonic()

        measured_corrected = measured - self._static_bias

        if not self._warmup_done:
            if self._warmup_start is None:
                self._warmup_start = now
            if np.all(np.abs(velocities) < self._warmup_vel_threshold):
                self._warmup_samples.append(measured_corrected.copy())
            if (now - self._warmup_start) >= self._warmup_sec:
                if len(self._warmup_samples) >= 20:
                    self._warmup_offset = np.mean(self._warmup_samples, axis=0)
                self._warmup_done = True

        measured_final = measured_corrected - self._warmup_offset

        if self._last_timestamp is None or self._last_momentum is None:
            self._momentum_integral = momentum.copy()
            self._observer_residual = np.zeros_like(momentum)
        else:
            effective_dt = max(now - self._last_timestamp, 1e-6)
            self._momentum_integral += (
                measured_final - bias_torque - self._observer_residual
            ) * effective_dt
            self._observer_residual = self._observer_gain * (
                self._momentum_integral - momentum
            )

        tau_expected = bias_torque
        tau_external = self._observer_residual.copy()

        self.expected_torque_history.append(tau_expected.copy())

        self._previous = self._filtered.copy()
        self._filtered = (
            self._filter_alpha * tau_external
            + (1 - self._filter_alpha) * self._filtered
        )
        self._rate = np.abs(self._filtered - self._previous)
        self.external_torque_history.append(self._filtered.copy())

        self._last_momentum = momentum.copy()
        self._last_timestamp = now

    def reset(self):
        n = self._model.num_joints
        self._filtered = np.zeros(n)
        self._previous = np.zeros(n)
        self._rate = np.zeros(n)
        self._observer_residual = np.zeros(n)
        self._momentum_integral = np.zeros(n)
        self._last_momentum = None
        self._last_timestamp = None
