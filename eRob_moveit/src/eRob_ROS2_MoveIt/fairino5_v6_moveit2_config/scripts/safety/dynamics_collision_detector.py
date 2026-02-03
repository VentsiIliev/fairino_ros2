#!/usr/bin/env python3
"""
Dynamics-based collision detection using inverse dynamics.

Computes expected torques from robot dynamics model and compares with measured torques.
External torques (collisions) = measured - expected

τ_measured = τ_gravity + τ_inertia + τ_coriolis + τ_friction + τ_external
τ_expected = τ_gravity + τ_inertia + τ_coriolis + τ_friction  (from model)
τ_external = τ_measured - τ_expected  (should be ~0 unless collision)
"""

import numpy as np
from collections import deque
from typing import Optional, Callable, Tuple
from enum import Enum

try:
    import PyKDL
    from urdf_parser_py.urdf import URDF
    KDL_AVAILABLE = True
except ImportError:
    KDL_AVAILABLE = False
    print("[DynamicsCollisionDetector] Warning: PyKDL not available, using simplified model")


class CollisionState(Enum):
    CLEAR = 0
    DETECTED = 1
    RECOVERING = 2


class SensitivityPreset:
    """
    Predefined sensitivity levels for collision detection.
    Based on maximum allowable external force (N or kg·m/s²).

    Torque ≈ Force × Distance
    For typical arm length ~0.3m: 1kg force ≈ 3 N·m torque
    """

    # Sensitivity presets: [J1, J2, J3, J4, J5, J6] external torque thresholds (N·m)
    PRESETS = {
        # Ultra-sensitive: ~2kg force → ~6 N·m torque
        'ULTRA_SENSITIVE_2KG': {
            # 'external_torque': np.array([6.0, 6.0, 4.0, 3.0, 2.0, 1.5]),
            # 'rate': np.array([12.0, 12.0, 8.0, 6.0, 4.0, 3.0]),
            # 'description': '~2kg force - Very sensitive, human-safe collaborative'
            'external_torque': np.array([1.5, 1.5, 1.5, 1.0, 0.75, 0.5]),
            'rate': np.array([3.0, 3.0, 2.5, 2.0, 1.5, 1.0]),
            'description': 'Very ultra-sensitive, single-finger detection'
        },

        # High sensitivity: ~4kg force → ~12 N·m torque
        'HIGH_SENSITIVE_4KG': {
            'external_torque': np.array([12.0, 12.0, 8.0, 6.0, 4.0, 3.0]),
            'rate': np.array([20.0, 20.0, 15.0, 10.0, 8.0, 6.0]),
            'description': '~4kg force - High sensitivity, collaborative work'
        },

        # Medium sensitivity: ~5kg force → ~15 N·m torque
        'MEDIUM_SENSITIVE_5KG': {
            'external_torque': np.array([15.0, 15.0, 10.0, 8.0, 5.0, 4.0]),
            'rate': np.array([25.0, 25.0, 18.0, 12.0, 10.0, 8.0]),
            'description': '~5kg force - Balanced sensitivity'
        },

        # Standard sensitivity: ~6kg force → ~18 N·m torque
        'STANDARD_6KG': {
            'external_torque': np.array([18.0, 18.0, 12.0, 10.0, 6.0, 5.0]),
            'rate': np.array([30.0, 30.0, 20.0, 15.0, 12.0, 10.0]),
            'description': '~6kg force - Standard industrial use'
        },

        # Low sensitivity: ~8kg force → ~24 N·m torque
        'LOW_SENSITIVE_8KG': {
            'external_torque': np.array([24.0, 24.0, 16.0, 12.0, 8.0, 6.0]),
            'rate': np.array([40.0, 40.0, 25.0, 18.0, 15.0, 12.0]),
            'description': '~8kg force - Heavy duty operations'
        },

        # Custom/default
        'CUSTOM': {
            'external_torque': np.array([1.5, 1.5, 1.5, 1, 0.75, 0.5]),
            'rate': np.array([15.0, 15.0, 12.0, 8.0, 6.0, 4.0]),
            'description': 'Custom user-defined thresholds'
        }
    }

    @staticmethod
    def get_preset(name: str) -> dict:
        """Get a sensitivity preset by name."""
        return SensitivityPreset.PRESETS.get(name, SensitivityPreset.PRESETS['CUSTOM'])

    @staticmethod
    def list_presets() -> list:
        """List all available preset names."""
        return list(SensitivityPreset.PRESETS.keys())

    @staticmethod
    def get_description(name: str) -> str:
        """Get description of a preset."""
        preset = SensitivityPreset.get_preset(name)
        return preset.get('description', 'Unknown preset')


class DynamicsCollisionDetector:
    """
    Collision detector using inverse dynamics to isolate external torques.

    τ_external = τ_measured - τ_expected(q, dq, ddq)

    Where τ_expected is computed from the robot's dynamic model (URDF).
    """

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        urdf_string: Optional[str] = None,
        base_link: str = 'base_link',
        tip_link: str = 'wrist3_link',
        num_joints: int = 6,
        # External torque thresholds (N·m) - these can be much lower than raw torque thresholds
        external_torque_thresholds: Optional[np.ndarray] = None,
        # Rate threshold for sudden changes
        external_torque_rate_thresholds: Optional[np.ndarray] = None,
        # Filter parameters
        filter_alpha: float = 0.3,  # Low-pass filter for external torque estimate
        confirmation_samples: int = 3,
        recovery_time: float = 1.0,
        # Gravity vector (inverted for correct torque signs in KDL)
        # KDL convention: positive Z = up, so gravity points down = +9.81
        gravity: np.ndarray = np.array([0, 0, 9.81]),
        logger=None
    ):
        self.num_joints = num_joints
        self.logger = logger
        self.gravity = gravity
        self.filter_alpha = filter_alpha
        self.confirmation_samples = confirmation_samples
        self.recovery_time = recovery_time

        # Thresholds for EXTERNAL torque (much lower than raw torque)
        # These represent unexpected forces, not motor effort
        if external_torque_thresholds is None:
            # External torque thresholds - can be much more sensitive
            self.external_torque_thresholds = np.array([8.0, 8.0, 6.0, 4.0, 3.0, 2.0])
        else:
            self.external_torque_thresholds = np.array(external_torque_thresholds)

        if external_torque_rate_thresholds is None:
            self.external_torque_rate_thresholds = np.array([15.0, 15.0, 12.0, 8.0, 6.0, 4.0])
        else:
            self.external_torque_rate_thresholds = np.array(external_torque_rate_thresholds)

        # State
        self.state = CollisionState.CLEAR
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.enabled = True
        self.armed = False

        # Filtered external torque estimate
        self.filtered_external_torque = np.zeros(num_joints)
        self.prev_external_torque = np.zeros(num_joints)

        # Adaptive baseline - instead of single baseline, use moving average
        # This adapts to different robot configurations automatically
        self.use_adaptive_baseline = True
        self.adaptive_baseline = np.zeros(num_joints)
        self.baseline_alpha = 0.01  # Very slow adaptation (1% per sample at 50Hz = ~2 sec time constant)
        self.baseline_initialized = False
        self.baseline_init_samples = 0
        self.baseline_init_required = 30  # Need 30 samples to initialize

        # For stationary baseline (legacy - still used for initial calibration)
        self.baseline_external_torque = None
        self.baseline_samples = deque(maxlen=50)
        self.baseline_calibrated = False

        # History for debugging/visualization
        self.external_torque_history = deque(maxlen=100)
        self.expected_torque_history = deque(maxlen=100)

        # Callbacks
        self._on_collision_callback: Optional[Callable] = None
        self._on_clear_callback: Optional[Callable] = None

        # Initialize dynamics model
        self.kdl_chain = None
        self.id_solver = None
        self.use_kdl = False

        if KDL_AVAILABLE and (urdf_path or urdf_string):
            self._init_kdl_dynamics(urdf_path, urdf_string, base_link, tip_link)

        if not self.use_kdl:
            self._init_simplified_dynamics()

        if self.logger:
            mode = "KDL inverse dynamics" if self.use_kdl else "simplified gravity model"
            self.logger.info(f'[DynamicsCollisionDetector] Initialized with {mode}')

    def _init_kdl_dynamics(self, urdf_path: str, urdf_string: str, base_link: str, tip_link: str):
        """Initialize KDL chain and inverse dynamics solver from URDF."""
        try:
            from kdl_parser_py import urdf as kdl_urdf
            from lxml import etree

            # Load URDF
            if urdf_string:
                robot = URDF.from_xml_string(urdf_string)
            else:
                # Read as bytes and parse with lxml to handle encoding declaration
                with open(urdf_path, 'rb') as f:
                    urdf_bytes = f.read()
                node = etree.fromstring(urdf_bytes)
                robot = URDF.from_xml_string(etree.tostring(node, encoding='unicode'))

            # Build KDL tree
            success, tree = kdl_urdf.treeFromUrdfModel(robot)
            if not success:
                raise RuntimeError("Failed to build KDL tree from URDF")

            # Get chain from base to tip
            self.kdl_chain = tree.getChain(base_link, tip_link)

            if self.kdl_chain.getNrOfJoints() != self.num_joints:
                if self.logger:
                    self.logger.warning(
                        f'[DynamicsCollisionDetector] KDL chain has {self.kdl_chain.getNrOfJoints()} joints, '
                        f'expected {self.num_joints}. Using simplified model.'
                    )
                return

            # Create inverse dynamics solver (RNEA)
            self.id_solver = PyKDL.ChainIdSolver_RNE(self.kdl_chain, PyKDL.Vector(*self.gravity))
            self.use_kdl = True

            if self.logger:
                self.logger.info(f'[DynamicsCollisionDetector] KDL chain loaded: {base_link} -> {tip_link}')

        except Exception as e:
            if self.logger:
                self.logger.warning(f'[DynamicsCollisionDetector] Failed to init KDL: {e}')
            self.use_kdl = False

    def _init_simplified_dynamics(self):
        """
        Initialize simplified dynamics model (gravity compensation only).

        For a 6-DOF robot, we approximate:
        τ_expected ≈ τ_gravity(q) + I_eff * ddq

        This is a reasonable approximation for slow cobot motions.
        """
        # Approximate effective inertias per joint (from URDF data)
        # These are rough estimates - adjust based on your robot
        self.effective_inertia = np.array([
            0.5,   # J1 - base rotation (low inertia for rotation)
            2.0,   # J2 - shoulder (high inertia - big links)
            1.5,   # J3 - elbow (medium inertia)
            0.3,   # J4 - wrist 1
            0.3,   # J5 - wrist 2
            0.1    # J6 - wrist 3 (lowest inertia)
        ])

        # Link masses (from URDF) for gravity compensation
        self.link_masses = np.array([4.377, 14.458, 7.674, 1.627, 1.581, 0.525])

        # Link CoM distances from joint (approximate, in meters)
        self.link_com_distances = np.array([0.146, 0.2125, 0.188, 0.097, 0.098, 0.076])

        # Gravity constant
        self.g = 9.81

    def compute_expected_torque_kdl(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ) -> np.ndarray:
        """
        Compute expected torque using KDL inverse dynamics (RNEA algorithm).

        τ = M(q)·q̈ + C(q,q̇)·q̇ + g(q)
        """
        if not self.use_kdl or self.id_solver is None:
            return self.compute_expected_torque_simplified(positions, velocities, accelerations)

        # Convert to KDL types
        q = PyKDL.JntArray(self.num_joints)
        dq = PyKDL.JntArray(self.num_joints)
        ddq = PyKDL.JntArray(self.num_joints)
        tau = PyKDL.JntArray(self.num_joints)

        # No external wrenches - list of Wrench objects
        f_ext = [PyKDL.Wrench() for _ in range(self.kdl_chain.getNrOfSegments())]

        for i in range(self.num_joints):
            q[i] = positions[i]
            dq[i] = velocities[i]
            ddq[i] = accelerations[i]

        # Compute inverse dynamics
        result = self.id_solver.CartToJnt(q, dq, ddq, f_ext, tau)

        if result < 0:
            if self.logger:
                self.logger.warning('[DynamicsCollisionDetector] KDL inverse dynamics failed')
            return self.compute_expected_torque_simplified(positions, velocities, accelerations)

        return np.array([tau[i] for i in range(self.num_joints)])

    def compute_expected_torque_simplified(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ) -> np.ndarray:
        """
        Simplified expected torque computation.

        τ_expected ≈ τ_gravity(q) + I_eff * ddq

        This ignores Coriolis/centrifugal terms but works well for slow motions.
        """
        tau_expected = np.zeros(self.num_joints)

        # Inertia contribution: τ_inertia = I_eff * ddq
        tau_inertia = self.effective_inertia * accelerations

        # Simplified gravity compensation
        # For a 6-DOF arm, gravity affects joints 2, 3, 4, 5 based on configuration
        # This is a rough approximation - proper gravity comp needs full kinematics

        q = positions

        # J2 (shoulder) - carries most of the arm weight
        # τ_gravity_j2 ≈ (m_upper + m_forearm + m_wrist) * g * L * cos(q2)
        total_mass_after_j2 = sum(self.link_masses[1:])  # ~25 kg
        arm_length_j2 = 0.3  # effective moment arm
        tau_expected[1] = total_mass_after_j2 * self.g * arm_length_j2 * np.cos(q[1])

        # J3 (elbow) - carries forearm and wrist
        total_mass_after_j3 = sum(self.link_masses[2:])  # ~11 kg
        arm_length_j3 = 0.2
        tau_expected[2] = total_mass_after_j3 * self.g * arm_length_j3 * np.cos(q[1] + q[2])

        # J4, J5 - smaller gravity effects
        total_mass_after_j4 = sum(self.link_masses[3:])  # ~3.7 kg
        tau_expected[3] = total_mass_after_j4 * self.g * 0.1 * np.cos(q[1] + q[2] + q[3])

        # Add inertia contribution
        tau_expected += tau_inertia

        return tau_expected

    def compute_external_torque(
        self,
        measured_torque: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ) -> np.ndarray:
        """
        Compute external torque (collision indicator).

        τ_external = τ_measured - τ_expected(q, dq, ddq)
        """
        if self.use_kdl:
            tau_expected = self.compute_expected_torque_kdl(positions, velocities, accelerations)
        else:
            tau_expected = self.compute_expected_torque_simplified(positions, velocities, accelerations)

        tau_external = measured_torque - tau_expected

        # Store for debugging
        self.expected_torque_history.append(tau_expected.copy())

        return tau_external

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
        # Reset filter state
        self.filtered_external_torque = np.zeros(self.num_joints)
        self.prev_external_torque = np.zeros(self.num_joints)
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

    def calibrate_baseline(self):
        """
        Reset baseline calibration.
        Call this when robot is stationary to establish new baseline.
        """
        self.baseline_samples.clear()
        self.baseline_external_torque = None
        self.baseline_calibrated = False
        if self.logger:
            self.logger.info('[DynamicsCollisionDetector] Baseline calibration reset - collecting samples...')

    def set_sensitivity_preset(self, preset_name: str):
        """
        Apply a predefined sensitivity preset.

        Args:
            preset_name: One of the preset names from SensitivityPreset.list_presets()
                        e.g., 'ULTRA_SENSITIVE_2KG', 'HIGH_SENSITIVE_4KG', etc.
        """
        preset = SensitivityPreset.get_preset(preset_name)
        self.external_torque_thresholds = preset['external_torque'].copy()
        self.external_torque_rate_thresholds = preset['rate'].copy()
        self.current_preset = preset_name

        if self.logger:
            self.logger.info(
                f'[DynamicsCollisionDetector] Sensitivity preset: {preset_name} - '
                f'{SensitivityPreset.get_description(preset_name)}'
            )
            self.logger.info(f'  External torque thresholds: {self.external_torque_thresholds}')

    def get_current_preset(self) -> str:
        """Get the name of currently active preset."""
        return getattr(self, 'current_preset', 'CUSTOM')

    def list_available_presets(self) -> dict:
        """Get all available presets with descriptions."""
        return {name: SensitivityPreset.get_description(name)
                for name in SensitivityPreset.list_presets()}

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

        Args:
            measured_efforts: Measured joint torques (N·m)
            positions: Joint positions (rad)
            velocities: Joint velocities (rad/s)
            accelerations: Joint accelerations (rad/s²)
            timestamp: Current time (seconds)

        Returns:
            True if collision detected
        """
        if not self.enabled:
            return False

        # Compute external torque
        tau_external = self.compute_external_torque(
            measured_efforts, positions, velocities, accelerations
        )

        # Low-pass filter to reduce noise
        self.filtered_external_torque = (
            self.filter_alpha * tau_external +
            (1 - self.filter_alpha) * self.filtered_external_torque
        )

        # Store history
        self.external_torque_history.append(self.filtered_external_torque.copy())

        # Adaptive baseline - continuously tracks model errors across all positions
        if self.use_adaptive_baseline:
            velocity_magnitude = np.linalg.norm(velocities)
            accel_magnitude = np.linalg.norm(accelerations)

            # Initialize baseline when robot first becomes stationary
            if not self.baseline_initialized:
                if velocity_magnitude < 0.01 and accel_magnitude < 0.01:
                    self.baseline_init_samples += 1
                    if self.baseline_init_samples >= self.baseline_init_required:
                        self.adaptive_baseline = self.filtered_external_torque.copy()
                        self.baseline_initialized = True
                        if self.logger:
                            self.logger.info(
                                f'[DynamicsCollisionDetector] Adaptive baseline initialized: '
                                f'{np.round(self.adaptive_baseline, 2)}'
                            )
                # Use absolute detection until initialized
                detection_torque = self.filtered_external_torque
            else:
                # Adaptive baseline: slowly track external torque when no collision
                # This adapts to different robot positions automatically

                # Only update baseline when robot is moving slowly (normal operation)
                # Don't update during fast motions or high accelerations
                if velocity_magnitude < 0.5 and accel_magnitude < 1.0:
                    # Check if current external torque is within safe bounds
                    deviation = np.abs(self.filtered_external_torque - self.adaptive_baseline)
                    max_deviation = np.max(deviation / self.external_torque_thresholds)

                    # Only update baseline if deviation is small (no collision)
                    if max_deviation < 0.5:  # Less than 50% of threshold
                        # Slowly adapt baseline toward current value
                        self.adaptive_baseline = (
                            self.baseline_alpha * self.filtered_external_torque +
                            (1 - self.baseline_alpha) * self.adaptive_baseline
                        )

                # Detection uses deviation from adaptive baseline
                detection_torque = self.filtered_external_torque - self.adaptive_baseline
        else:
            # Legacy single-point baseline calibration
            if not self.baseline_calibrated:
                # Collect samples for baseline when robot is not moving
                velocity_magnitude = np.linalg.norm(velocities)
                accel_magnitude = np.linalg.norm(accelerations)

                if velocity_magnitude < 0.01 and accel_magnitude < 0.01:  # Stationary
                    self.baseline_samples.append(self.filtered_external_torque.copy())

                    if len(self.baseline_samples) >= 30:  # Need 30 samples
                        # Compute baseline as median (robust to outliers)
                        self.baseline_external_torque = np.median(list(self.baseline_samples), axis=0)
                        self.baseline_calibrated = True
                        if self.logger:
                            self.logger.info(
                                f'[DynamicsCollisionDetector] Baseline calibrated: '
                                f'{np.round(self.baseline_external_torque, 2)}'
                            )

                # Until calibrated, use absolute thresholds
                detection_torque = self.filtered_external_torque
            else:
                # Use baseline-relative detection (old single-point method)
                detection_torque = self.filtered_external_torque - self.baseline_external_torque

        # Handle recovery state
        if self.state == CollisionState.RECOVERING:
            if timestamp - self.last_collision_time > self.recovery_time:
                self.state = CollisionState.CLEAR
                self.collision_count = 0
                # Re-arm after recovery (for always-armed mode)
                self.armed = True
                if self._on_clear_callback:
                    self._on_clear_callback()
                if self.logger:
                    self.logger.info('[DynamicsCollisionDetector] Recovered - re-armed')
            return False

        # Only check when armed
        if not self.armed:
            self.prev_external_torque = detection_torque.copy() if self.baseline_calibrated else self.filtered_external_torque.copy()
            return False

        # Check thresholds on baseline-relative torque
        abs_external = np.abs(detection_torque)
        threshold_exceeded = np.any(abs_external > self.external_torque_thresholds)

        # Check rate of change
        if self.baseline_calibrated:
            rate = np.abs(detection_torque - self.prev_external_torque)
        else:
            rate = np.abs(self.filtered_external_torque - self.prev_external_torque)
        rate_exceeded = np.any(rate > self.external_torque_rate_thresholds)

        self.prev_external_torque = detection_torque.copy() if self.baseline_calibrated else self.filtered_external_torque.copy()

        collision_detected = threshold_exceeded or rate_exceeded

        if collision_detected:
            self.collision_count += 1

            if self.collision_count >= self.confirmation_samples:
                self.state = CollisionState.DETECTED
                self.last_collision_time = timestamp

                if self.logger:
                    exceeded = []
                    for i in range(self.num_joints):
                        if abs_external[i] > self.external_torque_thresholds[i]:
                            exceeded.append(f'J{i+1}:{abs_external[i]:.2f}N·m')
                    self.logger.warning(f'[DynamicsCollisionDetector] COLLISION! External torque: {exceeded}')

                if self._on_collision_callback:
                    self._on_collision_callback()

                self.state = CollisionState.RECOVERING
                self.armed = False
                return True
        else:
            self.collision_count = max(0, self.collision_count - 1)

        return False

    def get_state(self) -> CollisionState:
        """Get current collision state."""
        return self.state

    def is_collision_detected(self) -> bool:
        """Check if collision currently detected."""
        return self.state in [CollisionState.DETECTED, CollisionState.RECOVERING]

    def get_external_torque(self) -> np.ndarray:
        """Get current filtered external torque estimate."""
        return self.filtered_external_torque.copy()

    def get_status(self) -> dict:
        """Get detector status for debugging."""
        return {
            'enabled': self.enabled,
            'armed': self.armed,
            'state': self.state.name,
            'collision_count': self.collision_count,
            'external_torque': self.filtered_external_torque.tolist(),
            'thresholds': self.external_torque_thresholds.tolist(),
            'use_kdl': self.use_kdl,
            'baseline_initialized': self.baseline_initialized if self.use_adaptive_baseline else self.baseline_calibrated,
            'adaptive_baseline': self.adaptive_baseline.tolist() if self.use_adaptive_baseline else None,
            'use_adaptive_baseline': self.use_adaptive_baseline
        }

    def set_thresholds(self, external_thresholds: np.ndarray, rate_thresholds: Optional[np.ndarray] = None):
        """Update collision thresholds."""
        self.external_torque_thresholds = np.array(external_thresholds)
        if rate_thresholds is not None:
            self.external_torque_rate_thresholds = np.array(rate_thresholds)