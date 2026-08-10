#!/usr/bin/env python3
"""Strategy seam for trajectory time parameterization."""

from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor

import numpy as np

import config

_DIAG_EXECUTOR = ThreadPoolExecutor(
    max_workers=1,
    thread_name_prefix="totg_path_diag",
)


def _log_joint_path_geometry(robot_controller, trajectory, *, label="TOTG"):
    """Log joint-space geometry that can make time parameterization fail.

    This is diagnostic-only: the trajectory is never modified.  In particular,
    report near-duplicate points and three-point reversals where consecutive
    joint-space direction vectors approach a 180 degree turn.
    """
    logger = getattr(robot_controller, "get_logger", lambda: None)()
    if logger is None:
        return

    joint_trajectory = getattr(trajectory, "joint_trajectory", None)
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    points = list(getattr(joint_trajectory, "points", []) or [])
    if len(points) < 2 or not joint_names:
        return

    try:
        positions = np.asarray([list(point.positions) for point in points], dtype=float)
    except Exception as exc:
        logger.warning(f"[{label}_PATH_DIAG] unable to inspect trajectory: {exc}")
        return

    if positions.ndim != 2 or positions.shape[0] != len(points) or positions.shape[1] != len(joint_names):
        logger.warning(
            f"[{label}_PATH_DIAG] malformed trajectory shape={positions.shape} "
            f"points={len(points)} joints={len(joint_names)}"
        )
        return

    if not np.all(np.isfinite(positions)):
        logger.warning(f"[{label}_PATH_DIAG] trajectory contains non-finite joint positions")
        return

    if bool(getattr(config, "TOTG_PATH_DIAG_ASYNC", True)):
        _DIAG_EXECUTOR.submit(
            _run_joint_path_geometry_diagnostics,
            logger,
            positions.copy(),
            list(joint_names),
            label,
        )
        return

    _log_joint_path_geometry_from_snapshot(
        logger,
        positions,
        joint_names,
        label,
    )


def _run_joint_path_geometry_diagnostics(logger, positions, joint_names, label):
    try:
        _log_joint_path_geometry_from_snapshot(
            logger,
            positions,
            joint_names,
            label,
        )
    except Exception as exc:
        logger.debug(f"[{label}_PATH_DIAG] background diagnostics failed: {exc}")


def _log_joint_path_geometry_from_snapshot(logger, positions, joint_names, label):
    """Log joint-space path diagnostics from copied primitive data."""
    if positions.ndim != 2 or positions.shape[0] < 2 or positions.shape[1] != len(joint_names):
        logger.warning(
            f"[{label}_PATH_DIAG] malformed trajectory snapshot shape={positions.shape} "
            f"joints={len(joint_names)}"
        )
        return

    steps = np.diff(positions, axis=0)
    step_norms = np.linalg.norm(steps, axis=1)
    duplicate_eps = 1e-8
    duplicate_indexes = np.flatnonzero(step_norms <= duplicate_eps)

    spans = np.ptp(positions, axis=0)
    endpoint_delta = positions[-1] - positions[0]
    max_abs_step_by_joint = np.max(np.abs(steps), axis=0)
    per_joint = " ".join(
        f"{name}:span={spans[i]:.6f},end={endpoint_delta[i]:+.6f},maxstep={max_abs_step_by_joint[i]:.6f}"
        for i, name in enumerate(joint_names)
    )
    logger.info(
        f"[{label}_PATH_DIAG] points={len(points)} joints={len(joint_names)} "
        f"near_duplicate_segments={len(duplicate_indexes)} {per_joint}"
    )

    if len(duplicate_indexes):
        shown = ",".join(str(int(index)) for index in duplicate_indexes[:12])
        suffix = "..." if len(duplicate_indexes) > 12 else ""
        logger.warning(
            f"[{label}_PATH_DIAG] near-duplicate segment indexes={shown}{suffix} "
            f"epsilon={duplicate_eps:.1e}rad"
        )

    # TOTG's '180 degree turn' is a geometric reversal of two adjacent path
    # segments in N-dimensional joint space.  Calculate the cosine between
    # d(q[i-1],q[i]) and d(q[i],q[i+1]); -1 means an exact 180 degree turn.
    candidates = []
    for middle in range(1, len(positions) - 1):
        d_prev = steps[middle - 1]
        d_next = steps[middle]
        n_prev = step_norms[middle - 1]
        n_next = step_norms[middle]
        if n_prev <= duplicate_eps or n_next <= duplicate_eps:
            continue
        cosine = float(np.dot(d_prev, d_next) / (n_prev * n_next))
        cosine = float(np.clip(cosine, -1.0, 1.0))
        candidates.append((cosine, middle, d_prev, d_next, n_prev, n_next))

    candidates.sort(key=lambda item: item[0])
    for rank, (cosine, middle, d_prev, d_next, n_prev, n_next) in enumerate(candidates[:8], start=1):
        angle_deg = float(np.degrees(np.arccos(cosine)))
        strongest_joint = int(np.argmax(np.abs(d_prev - d_next)))
        logger.warning(
            f"[{label}_PATH_DIAG] reversal_candidate rank={rank} middle={middle} "
            f"angle_deg={angle_deg:.6f} cos={cosine:.9f} "
            f"prev_norm={n_prev:.9f} next_norm={n_next:.9f} "
            f"strongest_joint={joint_names[strongest_joint]} "
            f"q_prev={[round(float(v), 9) for v in positions[middle - 1]]} "
            f"q_mid={[round(float(v), 9) for v in positions[middle]]} "
            f"q_next={[round(float(v), 9) for v in positions[middle + 1]]} "
            f"d_prev={[round(float(v), 9) for v in d_prev]} "
            f"d_next={[round(float(v), 9) for v in d_next]}"
        )

    severe = [item for item in candidates if item[0] <= -0.999]
    if severe:
        logger.error(
            f"[{label}_PATH_DIAG] detected {len(severe)} near-180deg joint-space reversal(s); "
            f"worst_middle={severe[0][1]} worst_cos={severe[0][0]:.9f}"
        )


class ITrajectoryOptimizer(ABC):
    @abstractmethod
    def optimize(self, robot_controller, trajectory, vel_scaling, acc_scaling, callback):
        """Optimize a MoveIt trajectory and invoke callback with the result."""


class TotgTrajectoryOptimizer(ITrajectoryOptimizer):
    def __init__(self, apply_fn=None):
        self._apply_fn = apply_fn

    def optimize(self, robot_controller, trajectory, vel_scaling, acc_scaling, callback):
        _log_joint_path_geometry(robot_controller, trajectory, label="TOTG")
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
    if not requested:
        return default_optimizer
    return build_trajectory_optimizer(requested, node=node, fallback_name="TOTG")
