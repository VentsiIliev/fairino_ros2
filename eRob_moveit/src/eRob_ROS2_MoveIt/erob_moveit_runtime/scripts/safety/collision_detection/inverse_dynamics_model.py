#!/usr/bin/env python3
"""
Inverse dynamics model abstraction.

Provides an interface and KDL-based implementation for computing
expected joint torques from the robot's dynamic model (RNEA).
"""

import numpy as np
from abc import ABC, abstractmethod
from typing import Optional
import config

try:
    import PyKDL
    from urdf_parser_py.urdf import URDF
    KDL_AVAILABLE = True
except ImportError:
    KDL_AVAILABLE = False


def _vector(values):
    values = values or [0.0, 0.0, 0.0]
    return PyKDL.Vector(float(values[0]), float(values[1]), float(values[2]))


def _frame(origin):
    if origin is None:
        return PyKDL.Frame.Identity()
    xyz = getattr(origin, "xyz", None) or [0.0, 0.0, 0.0]
    rpy = getattr(origin, "rpy", None) or [0.0, 0.0, 0.0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2])),
        _vector(xyz),
    )


def _inertia(link):
    inertial = getattr(link, "inertial", None)
    if inertial is None:
        return PyKDL.RigidBodyInertia()
    tensor = inertial.inertia
    rotational = PyKDL.RotationalInertia(
        tensor.ixx, tensor.iyy, tensor.izz,
        tensor.ixy, tensor.ixz, tensor.iyz,
    )
    origin = _frame(inertial.origin)
    body = PyKDL.RigidBodyInertia(float(inertial.mass), origin.p, rotational)
    return origin.M * body


def _joint(urdf_joint):
    origin = _frame(urdf_joint.origin)
    if urdf_joint.type in ("revolute", "continuous"):
        return PyKDL.Joint(
            urdf_joint.name, origin.p, origin.M * _vector(urdf_joint.axis),
            PyKDL.Joint.RotAxis,
        )
    if urdf_joint.type == "prismatic":
        return PyKDL.Joint(
            urdf_joint.name, origin.p, origin.M * _vector(urdf_joint.axis),
            PyKDL.Joint.TransAxis,
        )
    return PyKDL.Joint(urdf_joint.name, PyKDL.Joint.Fixed)


def _tree_from_urdf(robot):
    """Small Jazzy-compatible replacement for the unavailable kdl_parser_py."""
    child_links = {joint.child for joint in robot.joints}
    root_names = [link.name for link in robot.links if link.name not in child_links]
    if len(root_names) != 1:
        raise RuntimeError(f"URDF must have one root link, found {root_names}")
    tree = PyKDL.Tree(root_names[0])
    links = {link.name: link for link in robot.links}
    children = {}
    for joint in robot.joints:
        children.setdefault(joint.parent, []).append(joint)

    def add(parent):
        for urdf_joint in children.get(parent, []):
            child = links[urdf_joint.child]
            segment = PyKDL.Segment(
                child.name, _joint(urdf_joint), _frame(urdf_joint.origin), _inertia(child)
            )
            if not tree.addSegment(segment, parent):
                raise RuntimeError(f"Failed to add KDL segment {child.name}")
            add(child.name)

    add(root_names[0])
    return tree


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

    def compute_mass_matrix(self, positions: np.ndarray) -> np.ndarray:
        """Optional joint-space inertia matrix."""
        raise NotImplementedError

    def compute_bias_torque(
        self,
        positions: np.ndarray,
        velocities: np.ndarray
    ) -> np.ndarray:
        """Optional Coriolis/gravity bias torque."""
        raise NotImplementedError


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
        base_link: str = config.BASE_LINK,
        tip_link: str = config.COLLISION_TIP_LINK,
        num_joints: int = config.NUM_JOINTS,
        gravity: np.ndarray = np.array([0, 0, -9.81]),
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
        if urdf_string:
            robot = URDF.from_xml_string(urdf_string)
        else:
            robot = URDF.from_xml_file(urdf_path)

        tree = _tree_from_urdf(robot)

        self.kdl_chain = tree.getChain(base_link, tip_link)

        if self.kdl_chain.getNrOfJoints() != self._num_joints:
            raise RuntimeError(
                f'[KDLInverseDynamicsModel] KDL chain has {self.kdl_chain.getNrOfJoints()} joints, '
                f'expected {self._num_joints}'
            )

        grav = gravity if self._include_gravity else np.zeros(3)
        self.id_solver = PyKDL.ChainIdSolver_RNE(self.kdl_chain, PyKDL.Vector(*grav))
        self.dyn_solver = PyKDL.ChainDynParam(self.kdl_chain, PyKDL.Vector(*grav))

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

    def compute_mass_matrix(self, positions: np.ndarray) -> np.ndarray:
        """Compute the joint-space inertia matrix M(q)."""
        q = PyKDL.JntArray(self._num_joints)
        for i in range(self._num_joints):
            q[i] = positions[i]

        mass = PyKDL.JntSpaceInertiaMatrix(self._num_joints)
        result = self.dyn_solver.JntToMass(q, mass)
        if result < 0:
            if self.logger:
                self.logger.warning('[KDLInverseDynamicsModel] KDL mass matrix solve failed')
            return np.eye(self._num_joints)

        out = np.zeros((self._num_joints, self._num_joints))
        for i in range(self._num_joints):
            for j in range(self._num_joints):
                out[i, j] = mass[i, j]
        return out

    def compute_bias_torque(
        self,
        positions: np.ndarray,
        velocities: np.ndarray
    ) -> np.ndarray:
        """Compute C(q,dq)dq + g(q) using KDL dynamics helpers."""
        q = PyKDL.JntArray(self._num_joints)
        dq = PyKDL.JntArray(self._num_joints)
        coriolis = PyKDL.JntArray(self._num_joints)
        gravity = PyKDL.JntArray(self._num_joints)

        for i in range(self._num_joints):
            q[i] = positions[i]
            dq[i] = velocities[i]

        coriolis_result = self.dyn_solver.JntToCoriolis(q, dq, coriolis)
        gravity_result = self.dyn_solver.JntToGravity(q, gravity)
        if coriolis_result < 0 or gravity_result < 0:
            if self.logger:
                self.logger.warning('[KDLInverseDynamicsModel] KDL bias torque solve failed')
            return np.zeros(self._num_joints)

        return np.array([coriolis[i] + gravity[i] for i in range(self._num_joints)])

    def compute_observer_auxiliary(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        finite_difference_rad: float = 1.0e-5,
    ) -> np.ndarray:
        """Compute paper Eq. (7): a_i=g_i-0.5*dq.T*dM/dq_i*dq."""
        q = PyKDL.JntArray(self._num_joints)
        gravity = PyKDL.JntArray(self._num_joints)
        for i in range(self._num_joints):
            q[i] = positions[i]
        if self.dyn_solver.JntToGravity(q, gravity) < 0:
            raise RuntimeError("KDL gravity solve failed")
        out = np.array([gravity[i] for i in range(self._num_joints)])
        for i in range(self._num_joints):
            q_plus = np.array(positions, dtype=float)
            q_minus = np.array(positions, dtype=float)
            q_plus[i] += finite_difference_rad
            q_minus[i] -= finite_difference_rad
            derivative = (
                self.compute_mass_matrix(q_plus) - self.compute_mass_matrix(q_minus)
            ) / (2.0 * finite_difference_rad)
            out[i] -= 0.5 * float(velocities @ derivative @ velocities)
        return out
