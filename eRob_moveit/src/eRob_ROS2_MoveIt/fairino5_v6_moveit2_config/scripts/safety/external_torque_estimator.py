#!/usr/bin/env python3
"""
External torque estimator.

Computes external (collision) torques by comparing measured torques
against expected torques from an inverse dynamics model, with low-pass filtering.

tau_external = tau_measured - tau_expected
"""

import numpy as np
from collections import deque

from safety.inverse_dynamics_model import InverseDynamicsModel


class ExternalTorqueEstimator:
    """
    Estimates external torques acting on the robot joints.

    Uses an InverseDynamicsModel to compute expected torques, then
    subtracts from measured torques. Applies a low-pass filter to
    reduce sensor noise.
    """

    def __init__(self, model: InverseDynamicsModel, filter_alpha: float = 0.7):
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

        self.expected_torque_history = deque(maxlen=100)
        self.external_torque_history = deque(maxlen=100)

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
