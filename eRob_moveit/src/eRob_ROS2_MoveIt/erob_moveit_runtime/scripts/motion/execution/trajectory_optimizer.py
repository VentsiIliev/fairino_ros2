#!/usr/bin/env python3
"""Strategy seam for trajectory time parameterization."""

from abc import ABC, abstractmethod
import config


class ITrajectoryOptimizer(ABC):
    @abstractmethod
    def optimize(self, robot_controller, trajectory, vel_scaling, acc_scaling, callback):
        """Optimize a MoveIt trajectory and invoke callback with the result."""


class TotgTrajectoryOptimizer(ITrajectoryOptimizer):
    def __init__(self, apply_fn=None):
        self._apply_fn = apply_fn

    def optimize(self, robot_controller, trajectory, vel_scaling, acc_scaling, callback):
        apply_fn = self._apply_fn
        if apply_fn is None:
            from .trajectory_optimization import apply_ipp_totg
            apply_fn = apply_ipp_totg
        apply_fn(robot_controller, trajectory, vel_scaling, acc_scaling, callback)


class RuckigTrajectoryOptimizer(ITrajectoryOptimizer):
    def __init__(self, apply_fn=None):
        self._apply_fn = apply_fn

    def optimize(self, robot_controller, trajectory, vel_scaling, acc_scaling, callback):
        apply_fn = self._apply_fn
        if apply_fn is None:
            from .trajectory_optimization import apply_ruckig_service
            apply_fn = apply_ruckig_service
        apply_fn(robot_controller, trajectory, vel_scaling, acc_scaling, callback)


def build_trajectory_optimizer(name, node, fallback_name=None):
    requested = (name or "").upper()
    if requested == "TOTG":
        return TotgTrajectoryOptimizer()
    if requested == "RUCKIG":
        return RuckigTrajectoryOptimizer()

    if fallback_name:
        logger = getattr(node, "get_logger", lambda: None)()
        if logger is not None:
            logger.error(
                f"[TrajectoryOptimizer] Unknown optimizer '{name}', falling back to '{fallback_name}'"
            )
        return build_trajectory_optimizer(fallback_name, node=node)

    raise ValueError(f"Unknown trajectory optimizer: {name}")


def resolve_trajectory_optimizer(name, node, default_optimizer):
    requested = (name or "").upper()
    configured = str(getattr(config, "TRAJECTORY_OPTIMIZER", "") or "").upper()

    if configured == "TOTG" and requested == "RUCKIG":
        logger = getattr(node, "get_logger", lambda: None)()
        if logger is not None:
            logger.warning(
                "[TrajectoryOptimizer] Ignoring per-request optimizer 'RUCKIG' because runtime is pinned to 'TOTG'"
            )
        return default_optimizer

    if not requested:
        return default_optimizer
    return build_trajectory_optimizer(requested, node=node, fallback_name="TOTG")
