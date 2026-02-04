#!/usr/bin/env python3
"""
Collision detection strategies.

Provides pluggable strategies for deciding whether an external torque
signal constitutes a collision. Each strategy inspects the estimator
state and returns True if a collision is detected.
"""

import numpy as np
from abc import ABC, abstractmethod

from safety.external_torque_estimator import ExternalTorqueEstimator


class CollisionDetectionStrategy(ABC):
    """Abstract interface for a collision detection check."""

    @abstractmethod
    def check(
        self,
        estimator: ExternalTorqueEstimator,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ) -> bool:
        """
        Check whether the current estimator state indicates a collision.

        Args:
            estimator: The external torque estimator (provides filtered torque, rate, etc.)
            velocities: Current joint velocities (rad/s)
            accelerations: Current joint accelerations (rad/s^2)

        Returns:
            True if this strategy detects a collision.
        """

    @abstractmethod
    def get_status(self) -> dict:
        """Return strategy-specific status for debugging."""


class RateThresholdStrategy(CollisionDetectionStrategy):
    """
    Detects collisions via sudden spikes in the rate of change of
    external torque. Thresholds are scaled per-joint by velocity and
    acceleration to tolerate larger model errors during fast motion.
    """

    def __init__(
        self,
        rate_thresholds: np.ndarray,
        vel_scale: float = 2.0,
        acc_scale: float = 0.5
    ):
        self._base_thresholds = np.array(rate_thresholds)
        self._vel_scale = vel_scale
        self._acc_scale = acc_scale
        self._effective_thresholds = self._base_thresholds.copy()
        self._current_rate = np.zeros_like(self._base_thresholds)

    def check(self, estimator, velocities, accelerations) -> bool:
        rate = estimator.rate
        self._current_rate = rate.copy()

        scale = (
            1.0
            + self._vel_scale * np.abs(velocities)
            + self._acc_scale * np.abs(accelerations)
        )
        self._effective_thresholds = self._base_thresholds * scale

        return bool(np.any(rate > self._effective_thresholds))

    def set_thresholds(self, thresholds: np.ndarray):
        """Update base rate thresholds."""
        self._base_thresholds = np.array(thresholds)

    def get_status(self) -> dict:
        return {
            'rate_thresholds': self._base_thresholds.tolist(),
            'effective_rate_thresholds': self._effective_thresholds.tolist(),
            'current_rate': self._current_rate.tolist(),
            'rate_vel_scale': self._vel_scale,
            'rate_acc_scale': self._acc_scale,
        }


class SustainedTorqueStrategy(CollisionDetectionStrategy):
    """
    Detects collisions via sustained external torque exceeding fixed
    thresholds. No velocity/acceleration scaling — uses fixed limits
    to detect slow pushing/pulling.
    """

    def __init__(self, sustained_thresholds: np.ndarray):
        self._thresholds = np.array(sustained_thresholds)

    def check(self, estimator, velocities, accelerations) -> bool:
        return bool(np.any(np.abs(estimator.filtered) > self._thresholds))

    def get_status(self) -> dict:
        return {
            'sustained_thresholds': self._thresholds.tolist(),
        }
