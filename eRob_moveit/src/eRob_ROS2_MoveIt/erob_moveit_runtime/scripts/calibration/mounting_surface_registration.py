"""Rigid registration of measured mounting-surface points to CAD points.

The returned transform maps points expressed in the mounting/CAD frame into
the robot ``base_link`` frame::

    p_base = R_base_mounting @ p_mounting + t_base_mounting

This is deliberately independent of ROS messages so it can be tested before
the platform application and REST service are integrated.
"""

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class RegistrationResult:
    """Best-fit rigid transform and its point-fit diagnostics."""

    transform: np.ndarray
    rotation: np.ndarray
    translation_m: np.ndarray
    residuals_m: np.ndarray
    rms_error_m: float
    max_error_m: float

    @property
    def translation_mm(self) -> np.ndarray:
        return self.translation_m * 1000.0


def _unit(vector: Iterable[float], name: str) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must be a finite 3D vector")
    length = np.linalg.norm(value)
    if length < 1e-12:
        raise ValueError(f"{name} must not be zero")
    return value / length


def solve_mounting_surface_frame(
    measured_origin: Iterable[float],
    measured_x_point: Iterable[float],
    measured_y_point: Iterable[float],
    *,
    mounting_origin: Iterable[float] = (0.0, 0.0, 0.0),
    mounting_x_axis: Iterable[float] = (1.0, 0.0, 0.0),
    mounting_y_axis: Iterable[float] = (0.0, 1.0, 0.0),
    mounting_normal: Iterable[float] = (0.0, 0.0, 1.0),
) -> RegistrationResult:
    """Build a mounting-to-base frame from three guided robot touches.

    The touches are the physical mounting-surface center, a point in the
    surface +X direction, and a point in the surface +Y direction. Their
    distances from the center do not need to match CAD dimensions.

    The mounting axes normally come from the minimum rectangle of the CAD
    surface. ``mounting_origin`` is that rectangle's center.
    """

    origin = np.asarray(measured_origin, dtype=float)
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError("measured_origin must be a finite 3D point")
    x_touch = np.asarray(measured_x_point, dtype=float)
    y_touch = np.asarray(measured_y_point, dtype=float)
    if x_touch.shape != (3,) or y_touch.shape != (3,):
        raise ValueError("measured axis points must be 3D points")

    x_base = _unit(x_touch - origin, "measured_x_point - measured_origin")
    y_base_raw = y_touch - origin
    y_base_raw -= np.dot(y_base_raw, x_base) * x_base
    y_base = _unit(y_base_raw, "measured_y_point - measured_origin")
    z_base = _unit(np.cross(x_base, y_base), "measured surface normal")
    y_base = _unit(np.cross(z_base, x_base), "measured Y axis")

    x_mount = _unit(mounting_x_axis, "mounting_x_axis")
    y_mount_raw = np.asarray(mounting_y_axis, dtype=float)
    y_mount_raw -= np.dot(y_mount_raw, x_mount) * x_mount
    y_mount = _unit(y_mount_raw, "mounting_y_axis")
    z_mount = _unit(np.cross(x_mount, y_mount), "mounting surface normal")
    requested_normal = _unit(mounting_normal, "mounting_normal")
    if np.dot(z_mount, requested_normal) < 0.0:
        z_mount = -z_mount
        y_mount = -y_mount

    base_axes = np.column_stack((x_base, y_base, z_base))
    mounting_axes = np.column_stack((x_mount, y_mount, z_mount))
    rotation = base_axes @ mounting_axes.T
    mounting_origin = np.asarray(mounting_origin, dtype=float)
    if mounting_origin.shape != (3,) or not np.all(np.isfinite(mounting_origin)):
        raise ValueError("mounting_origin must be a finite 3D point")
    translation = origin - rotation @ mounting_origin

    # The frame method intentionally does not force the operator's touch
    # distances to equal CAD dimensions; these are direction-only samples.
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return RegistrationResult(
        transform=transform,
        rotation=rotation,
        translation_m=translation,
        residuals_m=np.zeros(3),
        rms_error_m=0.0,
        max_error_m=0.0,
    )


def _points(value: Iterable[Iterable[float]], name: str) -> np.ndarray:
    points = np.asarray(value, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {points.shape}")
    if points.shape[0] < 3:
        raise ValueError("At least three corresponding 3D points are required")
    if not np.all(np.isfinite(points)):
        raise ValueError(f"{name} contains non-finite values")
    return points


def solve_mounting_surface_registration(
    mounting_points: Iterable[Iterable[float]],
    measured_base_points: Iterable[Iterable[float]],
    *,
    collinear_tolerance_m: float = 1e-9,
) -> RegistrationResult:
    """Solve the rigid transform from mounting/CAD points to base-link points.

    ``mounting_points[i]`` and ``measured_base_points[i]`` must describe the
    same physical point. Three points are sufficient when they are not
    collinear. More points are accepted and solved by least squares, which is
    useful for checking measurement noise.
    """

    source = _points(mounting_points, "mounting_points")
    target = _points(measured_base_points, "measured_base_points")
    if source.shape != target.shape:
        raise ValueError(
            "mounting_points and measured_base_points must have identical shape"
        )

    source_centered = source - source.mean(axis=0)
    target_centered = target - target.mean(axis=0)
    source_rank = np.linalg.matrix_rank(source_centered, tol=collinear_tolerance_m)
    target_rank = np.linalg.matrix_rank(target_centered, tol=collinear_tolerance_m)
    if source_rank < 2 or target_rank < 2:
        raise ValueError(
            "Calibration points are degenerate; use three non-collinear points"
        )

    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(vt.T @ u.T) < 0.0:
        correction[-1, -1] = -1.0
    rotation = vt.T @ correction @ u.T
    translation = target.mean(axis=0) - rotation @ source.mean(axis=0)

    predicted = (rotation @ source.T).T + translation
    residuals = np.linalg.norm(predicted - target, axis=1)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    return RegistrationResult(
        transform=transform,
        rotation=rotation,
        translation_m=translation,
        residuals_m=residuals,
        rms_error_m=float(np.sqrt(np.mean(residuals**2))),
        max_error_m=float(np.max(residuals)),
    )
