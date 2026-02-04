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
from typing import Optional, Callable
from enum import Enum

try:
    import PyKDL
    from urdf_parser_py.urdf import URDF
    KDL_AVAILABLE = True
except ImportError:
    KDL_AVAILABLE = False
    print("[DynamicsCollisionDetector] Warning: PyKDL not available")


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
        # Rate thresholds: max allowable sudden change in external torque per sample (N·m/sample)
        # Collisions cause sudden spikes; model errors change smoothly across configurations
        external_torque_rate_thresholds: Optional[np.ndarray] = None,
        # Velocity/acceleration scaling for rate thresholds
        # effective_threshold[i] = base[i] * (1 + vel_scale * |vel[i]| + acc_scale * |acc[i]|)
        # This relaxes thresholds during fast motion where model errors are larger
        rate_vel_scale: float = 2.0,
        rate_acc_scale: float = 0.5,
        # Filter parameters
        filter_alpha: float = 0.3,  # Low-pass filter for external torque estimate
        confirmation_samples: int = 3,
        recovery_time: float = 1.0,
        # Gravity vector (inverted for correct torque signs in KDL)
        # KDL convention: positive Z = up, so gravity points down = +9.81
        gravity: np.ndarray = np.array([0, 0, 9.81]),
        # Whether to include gravity in expected torque computation.
        # Set to False if the controller already removes gravity from reported efforts.
        include_gravity: bool = False,
        logger=None
    ):
        self.num_joints = num_joints
        self.logger = logger
        self.gravity = gravity
        self.include_gravity = include_gravity
        self.filter_alpha = filter_alpha
        self.confirmation_samples = confirmation_samples
        self.recovery_time = recovery_time

        if external_torque_rate_thresholds is None:
            self.external_torque_rate_thresholds = np.array([15.0, 15.0, 12.0, 8.0, 6.0, 4.0])
        else:
            self.external_torque_rate_thresholds = np.array(external_torque_rate_thresholds)

        self.rate_vel_scale = rate_vel_scale
        self.rate_acc_scale = rate_acc_scale

        # State
        self.state = CollisionState.CLEAR
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.enabled = True
        self.armed = False

        # Filtered external torque estimate
        self.filtered_external_torque = np.zeros(num_joints)
        self.prev_external_torque = np.zeros(num_joints)

        # History for debugging/visualization
        self.external_torque_history = deque(maxlen=100)
        self.expected_torque_history = deque(maxlen=100)

        # Callbacks
        self._on_collision_callback: Optional[Callable] = None
        self._on_clear_callback: Optional[Callable] = None

        # Initialize KDL dynamics model (required)
        self.kdl_chain = None
        self.id_solver = None

        if not KDL_AVAILABLE:
            raise RuntimeError(
                "[DynamicsCollisionDetector] PyKDL is required but not available. "
                "Install with: sudo apt install python3-pykdl"
            )

        if not (urdf_path or urdf_string):
            raise ValueError(
                "[DynamicsCollisionDetector] urdf_path or urdf_string is required"
            )

        self._init_kdl_dynamics(urdf_path, urdf_string, base_link, tip_link)

        if self.id_solver is None:
            raise RuntimeError(
                "[DynamicsCollisionDetector] Failed to initialize KDL dynamics solver"
            )

        if self.logger:
            grav_str = "included" if self.include_gravity else "excluded (controller-compensated)"
            self.logger.info(f'[DynamicsCollisionDetector] Initialized with KDL inverse dynamics, gravity {grav_str}')

    def _init_kdl_dynamics(self, urdf_path: str, urdf_string: str, base_link: str, tip_link: str):
        """Initialize KDL chain and inverse dynamics solver from URDF."""
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
            raise RuntimeError(
                f'[DynamicsCollisionDetector] KDL chain has {self.kdl_chain.getNrOfJoints()} joints, '
                f'expected {self.num_joints}'
            )

        # Create inverse dynamics solver (RNEA)
        # Use zero gravity if controller already compensates for it in reported efforts
        grav = self.gravity if self.include_gravity else np.zeros(3)
        self.id_solver = PyKDL.ChainIdSolver_RNE(self.kdl_chain, PyKDL.Vector(*grav))

        if self.logger:
            self.logger.info(f'[DynamicsCollisionDetector] KDL chain loaded: {base_link} -> {tip_link}')

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
            return np.zeros(self.num_joints)

        return np.array([tau[i] for i in range(self.num_joints)])

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
        tau_expected = self.compute_expected_torque_kdl(positions, velocities, accelerations)

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

    def set_sensitivity_preset(self, preset_name: str):
        """
        Apply a predefined sensitivity preset (rate thresholds only).

        Args:
            preset_name: One of the preset names from SensitivityPreset.list_presets()
                        e.g., 'ULTRA_SENSITIVE_2KG', 'HIGH_SENSITIVE_4KG', etc.
        """
        preset = SensitivityPreset.get_preset(preset_name)
        self.external_torque_rate_thresholds = preset['rate'].copy()
        self.current_preset = preset_name

        if self.logger:
            self.logger.info(
                f'[DynamicsCollisionDetector] Sensitivity preset: {preset_name} - '
                f'{SensitivityPreset.get_description(preset_name)}'
            )
            self.logger.info(f'  Rate thresholds: {self.external_torque_rate_thresholds}')

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
            self.prev_external_torque = self.filtered_external_torque.copy()
            return False

        # Rate-only detection: collisions cause sudden spikes,
        # model errors change smoothly across configurations
        rate = np.abs(self.filtered_external_torque - self.prev_external_torque)
        self.prev_external_torque = self.filtered_external_torque.copy()

        # Scale thresholds per-joint by velocity and acceleration
        # Higher motion speed → larger model errors → need higher thresholds
        scale = (
            1.0
            + self.rate_vel_scale * np.abs(velocities)
            + self.rate_acc_scale * np.abs(accelerations)
        )
        effective_thresholds = self.external_torque_rate_thresholds * scale

        rate_exceeded = np.any(rate > effective_thresholds)

        if rate_exceeded:
            self.collision_count += 1

            if self.collision_count >= self.confirmation_samples:
                self.state = CollisionState.DETECTED
                self.last_collision_time = timestamp

                if self.logger:
                    exceeded = []
                    for i in range(self.num_joints):
                        if rate[i] > effective_thresholds[i]:
                            exceeded.append(
                                f'J{i+1}: rate={rate[i]:.2f} > {effective_thresholds[i]:.2f} N·m/sample'
                            )
                    self.logger.warning(f'[DynamicsCollisionDetector] COLLISION! {exceeded}')

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
            'rate_thresholds': self.external_torque_rate_thresholds.tolist(),
            'rate_vel_scale': self.rate_vel_scale,
            'rate_acc_scale': self.rate_acc_scale,
            'include_gravity': self.include_gravity,
        }

    def set_thresholds(self, rate_thresholds: np.ndarray):
        """Update collision rate thresholds."""
        self.external_torque_rate_thresholds = np.array(rate_thresholds)