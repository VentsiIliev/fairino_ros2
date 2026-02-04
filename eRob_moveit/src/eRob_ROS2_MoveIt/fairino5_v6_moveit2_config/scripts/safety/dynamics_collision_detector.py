#!/usr/bin/env python3
"""
Dynamics-based collision detection coordinator.

Delegates inverse dynamics, torque estimation, and collision checks
to dedicated classes while preserving the original public API.
"""

import numpy as np
from typing import Optional, Callable, List
from enum import Enum

from safety.external_torque_estimator import ExternalTorqueEstimator
from safety.collision_detection_strategy import (
    CollisionDetectionStrategy,
    RateThresholdStrategy,
    SustainedTorqueStrategy,
)
from safety.inverse_dynamics_model import KDLInverseDynamicsModel


class CollisionState(Enum):
    CLEAR = 0
    DETECTED = 1
    RECOVERING = 2


class DynamicsCollisionDetector:
    """
    Slim coordinator for dynamics-based collision detection.

    Manages the state machine (CLEAR -> DETECTED -> RECOVERING -> CLEAR),
    callbacks, and arm/disarm lifecycle. Delegates torque estimation to
    an ExternalTorqueEstimator and collision checks to a list of
    CollisionDetectionStrategy instances.
    """

    def __init__(
        self,
        estimator: ExternalTorqueEstimator,
        strategies: List[CollisionDetectionStrategy],
        confirmation_samples: int = 1,
        recovery_time: float = 1.0,
        logger=None
    ):
        self._estimator = estimator
        self._strategies = strategies
        self.confirmation_samples = confirmation_samples
        self.recovery_time = recovery_time
        self.logger = logger

        # State machine
        self.state = CollisionState.CLEAR
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.enabled = True
        self.armed = False

        # Callbacks
        self._on_collision_callback: Optional[Callable] = None
        self._on_clear_callback: Optional[Callable] = None

    # -- Backward-compatible properties that delegate to estimator --

    @property
    def expected_torque_history(self):
        return self._estimator.expected_torque_history

    @property
    def external_torque_history(self):
        return self._estimator.external_torque_history

    @property
    def filtered_external_torque(self) -> np.ndarray:
        return self._estimator.filtered

    @property
    def num_joints(self) -> int:
        return self._estimator.num_joints

    # -- Lifecycle --

    def set_on_collision(self, callback: Callable):
        """Set callback when collision detected."""
        self._on_collision_callback = callback

    def set_on_clear(self, callback: Callable):
        """Set callback when collision clears."""
        self._on_clear_callback = callback

    def arm(self):
        """Arm detector (call when motion starts)."""
        self.armed = True
        self.collision_count = 0
        self._estimator.reset()
        if self.logger:
            self.logger.debug('[DynamicsCollisionDetector] Armed')

    def disarm(self):
        """Disarm detector (call when motion ends)."""
        self.armed = False
        if self.logger:
            self.logger.debug('[DynamicsCollisionDetector] Disarmed')

    def enable(self):
        """Enable collision detection."""
        self.enabled = True

    def disable(self):
        """Disable collision detection."""
        self.enabled = False

    # -- Core update --

    def update(
        self,
        measured_efforts: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray,
        timestamp: float
    ) -> bool:
        """
        Update detector with new measurements.

        Returns:
            True if collision detected.
        """
        if not self.enabled:
            return False

        # Update estimator (computes external torque + filters)
        self._estimator.update(measured_efforts, positions, velocities, accelerations)

        # Handle recovery state
        if self.state == CollisionState.RECOVERING:
            if timestamp - self.last_collision_time > self.recovery_time:
                self.state = CollisionState.CLEAR
                self.collision_count = 0
                self.armed = True
                if self._on_clear_callback:
                    self._on_clear_callback()
                if self.logger:
                    self.logger.info('[DynamicsCollisionDetector] Recovered - re-armed')
            return False

        # Only check when armed
        if not self.armed:
            return False

        # Run all strategies
        triggered = any(
            s.check(self._estimator, velocities, accelerations)
            for s in self._strategies
        )

        if triggered:
            self.collision_count += 1
            if self.collision_count >= self.confirmation_samples:
                self.state = CollisionState.DETECTED
                self.last_collision_time = timestamp

                if self.logger:
                    self.logger.warning('[DynamicsCollisionDetector] COLLISION detected!')

                if self._on_collision_callback:
                    self._on_collision_callback()

                self.state = CollisionState.RECOVERING
                self.armed = False
                return True
        else:
            self.collision_count = max(0, self.collision_count - 1)

        return False

    # -- Query methods --

    def get_state(self) -> CollisionState:
        """Get current collision state."""
        return self.state

    def is_collision_detected(self) -> bool:
        """Check if collision currently detected."""
        return self.state in [CollisionState.DETECTED, CollisionState.RECOVERING]

    def get_external_torque(self) -> np.ndarray:
        """Get current filtered external torque estimate."""
        return self._estimator.filtered

    def get_status(self) -> dict:
        """Get detector status for debugging."""
        status = {
            'enabled': self.enabled,
            'armed': self.armed,
            'state': self.state.name,
            'collision_count': self.collision_count,
            'external_torque': self._estimator.filtered.tolist(),
        }
        for strategy in self._strategies:
            status.update(strategy.get_status())
        return status

    def set_thresholds(self, rate_thresholds: np.ndarray):
        """Update collision rate thresholds (delegates to RateThresholdStrategy)."""
        for strategy in self._strategies:
            if isinstance(strategy, RateThresholdStrategy):
                strategy.set_thresholds(rate_thresholds)
                return
        if self.logger:
            self.logger.warning('[DynamicsCollisionDetector] No RateThresholdStrategy found')


def create_dynamics_collision_detector(
    urdf_path: Optional[str] = None,
    urdf_string: Optional[str] = None,
    base_link: str = 'base_link',
    tip_link: str = 'wrist3_link',
    num_joints: int = 6,
    external_torque_rate_thresholds: Optional[np.ndarray] = None,
    external_torque_sustained_thresholds: Optional[np.ndarray] = None,
    enable_sustained_check: bool = False,
    rate_vel_scale: float = 2.0,
    rate_acc_scale: float = 0.5,
    filter_alpha: float = 0.7,
    confirmation_samples: int = 1,
    recovery_time: float = 1.0,
    gravity: np.ndarray = np.array([0, 0, 9.81]),
    include_gravity: bool = False,
    logger=None
) -> DynamicsCollisionDetector:
    """
    Factory function that wires up Model -> Estimator -> Strategies -> Detector.

    Accepts the same parameters as the original DynamicsCollisionDetector
    constructor for easy migration.
    """
    # Defaults
    if external_torque_rate_thresholds is None:
        external_torque_rate_thresholds = np.array([5.0, 5.0, 4.0, 2.7, 2, 1.7])
    if external_torque_sustained_thresholds is None:
        external_torque_sustained_thresholds = np.array([12.0, 12.0, 10.0, 8.0, 6.0, 5.0])

    # 1. Model
    model = KDLInverseDynamicsModel(
        urdf_path=urdf_path,
        urdf_string=urdf_string,
        base_link=base_link,
        tip_link=tip_link,
        num_joints=num_joints,
        gravity=gravity,
        include_gravity=include_gravity,
        logger=logger,
    )

    # 2. Estimator
    estimator = ExternalTorqueEstimator(model, filter_alpha=filter_alpha)

    # 3. Strategies
    strategies: List[CollisionDetectionStrategy] = [
        RateThresholdStrategy(
            rate_thresholds=np.array(external_torque_rate_thresholds),
            vel_scale=rate_vel_scale,
            acc_scale=rate_acc_scale,
        ),
    ]
    if enable_sustained_check:
        strategies.append(
            SustainedTorqueStrategy(np.array(external_torque_sustained_thresholds))
        )

    # 4. Detector
    detector = DynamicsCollisionDetector(
        estimator=estimator,
        strategies=strategies,
        confirmation_samples=confirmation_samples,
        recovery_time=recovery_time,
        logger=logger,
    )

    if logger:
        logger.info('[DynamicsCollisionDetector] Initialized with KDL inverse dynamics')

    return detector
