#!/usr/bin/env python3
"""
Inverse dynamics model abstraction.

Provides an interface and KDL-based implementation for computing
expected joint torques from the robot's dynamic model (RNEA).
"""

import numpy as np
from abc import ABC, abstractmethod
from typing import Optional

try:
    import PyKDL
    from urdf_parser_py.urdf import URDF
    KDL_AVAILABLE = True
except ImportError:
    KDL_AVAILABLE = False


class InverseDynamicsModel(ABC):
    """Abstract interface for inverse dynamics computation."""

    @abstractmethod
    def compute_expected_torque(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ) -> np.ndarray:
        """
        Compute expected joint torques from the robot's dynamic model.

        tau = M(q)*ddq + C(q,dq)*dq + g(q)

        Args:
            positions: Joint positions (rad)
            velocities: Joint velocities (rad/s)
            accelerations: Joint accelerations (rad/s^2)

        Returns:
            Expected joint torques (N*m)
        """

    @property
    @abstractmethod
    def num_joints(self) -> int:
        """Number of joints in the model."""


class KDLInverseDynamicsModel(InverseDynamicsModel):
    """
    KDL-based inverse dynamics using the RNEA algorithm.

    Parses a URDF to build a KDL chain and solves inverse dynamics
    to compute expected joint torques.
    """

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        urdf_string: Optional[str] = None,
        base_link: str = 'base_link',
        tip_link: str = 'wrist3_link',
        num_joints: int = 6,
        gravity: np.ndarray = np.array([0, 0, 9.81]),
        include_gravity: bool = False,
        logger=None
    ):
        if not KDL_AVAILABLE:
            raise RuntimeError(
                "[KDLInverseDynamicsModel] PyKDL is required but not available. "
                "Install with: sudo apt install python3-pykdl"
            )

        if not (urdf_path or urdf_string):
            raise ValueError(
                "[KDLInverseDynamicsModel] urdf_path or urdf_string is required"
            )

        self._num_joints = num_joints
        self._include_gravity = include_gravity
        self.logger = logger

        self._init_kdl(urdf_path, urdf_string, base_link, tip_link, gravity)

        if self.id_solver is None:
            raise RuntimeError(
                "[KDLInverseDynamicsModel] Failed to initialize KDL dynamics solver"
            )

        if self.logger:
            grav_str = "included" if include_gravity else "excluded (controller-compensated)"
            self.logger.info(f'[KDLInverseDynamicsModel] Initialized, gravity {grav_str}')

    def _init_kdl(self, urdf_path, urdf_string, base_link, tip_link, gravity):
        """Initialize KDL chain and inverse dynamics solver from URDF."""
        from kdl_parser_py import urdf as kdl_urdf
        from lxml import etree

        if urdf_string:
            robot = URDF.from_xml_string(urdf_string)
        else:
            with open(urdf_path, 'rb') as f:
                urdf_bytes = f.read()
            node = etree.fromstring(urdf_bytes)
            robot = URDF.from_xml_string(etree.tostring(node, encoding='unicode'))

        success, tree = kdl_urdf.treeFromUrdfModel(robot)
        if not success:
            raise RuntimeError("Failed to build KDL tree from URDF")

        self.kdl_chain = tree.getChain(base_link, tip_link)

        if self.kdl_chain.getNrOfJoints() != self._num_joints:
            raise RuntimeError(
                f'[KDLInverseDynamicsModel] KDL chain has {self.kdl_chain.getNrOfJoints()} joints, '
                f'expected {self._num_joints}'
            )

        grav = gravity if self._include_gravity else np.zeros(3)
        self.id_solver = PyKDL.ChainIdSolver_RNE(self.kdl_chain, PyKDL.Vector(*grav))

        if self.logger:
            self.logger.info(f'[KDLInverseDynamicsModel] KDL chain loaded: {base_link} -> {tip_link}')

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def include_gravity(self) -> bool:
        return self._include_gravity

    def compute_expected_torque(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ) -> np.ndarray:
        """
        Compute expected torque using KDL inverse dynamics (RNEA algorithm).

        tau = M(q)*ddq + C(q,dq)*dq + g(q)
        """
        q = PyKDL.JntArray(self._num_joints)
        dq = PyKDL.JntArray(self._num_joints)
        ddq = PyKDL.JntArray(self._num_joints)
        tau = PyKDL.JntArray(self._num_joints)

        f_ext = [PyKDL.Wrench() for _ in range(self.kdl_chain.getNrOfSegments())]

        for i in range(self._num_joints):
            q[i] = positions[i]
            dq[i] = velocities[i]
            ddq[i] = accelerations[i]

        result = self.id_solver.CartToJnt(q, dq, ddq, f_ext, tau)

        if result < 0:
            if self.logger:
                self.logger.warning('[KDLInverseDynamicsModel] KDL inverse dynamics failed')
            return np.zeros(self._num_joints)

        return np.array([tau[i] for i in range(self._num_joints)])
