#!/usr/bin/env python3
"""Cartesian blend construction for linked-LIN planning."""

from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation, Slerp

from utils.transformation_utils import TransformationUtils


def build_linked_lin_blend_poses(
    start_base: Sequence[float],
    targets_base: Sequence[Sequence[float]],
    blend_radii_mm: Sequence[float],
    *,
    max_translation_mm: float = 8.0,
    max_orientation_deg: float = 2.0,
    rotation_dominant_xyz_mm: float = 5.0,
    sample_count: int = 12,
) -> tuple[list[Pose], list[float]]:
    """Build one blended TCP waypoint stream for a contiguous LIN group.

    This mirrors the legacy ordered BlendBuilder semantics at Cartesian level:
    each internal target is trimmed on both adjacent LIN segments according to
    ``blendR`` and replaced with a quadratic Bezier transition. Orientation is
    interpolated through the requested junction orientation so rotation-only
    LIN segments keep their intended orientation change.

    Returned poses exclude the start pose because the C++ linked-LIN helper
    starts from the supplied seed joint state.
    """

    targets = [list(target[:6]) for target in targets_base]
    if len(targets) < 2:
        raise RuntimeError("linked-LIN blend requires at least two targets")
    if len(blend_radii_mm) != len(targets) - 1:
        raise RuntimeError(
            "linked-LIN blend radius count must equal target_count - 1"
        )

    transforms = [
        TransformationUtils.pose_to_transform(list(start_base[:6])),
        *[
            TransformationUtils.pose_to_transform(target)
            for target in targets
        ],
    ]

    segment_lengths_mm = [
        _translation_distance_mm(transforms[index], transforms[index + 1])
        for index in range(len(transforms) - 1)
    ]

    effective_radii: list[float] = []
    entries = []
    exits = []
    for junction in range(len(targets) - 1):
        requested = max(0.0, float(blend_radii_mm[junction]))
        if requested <= 0.0:
            raise RuntimeError(
                f"linked-LIN internal junction {junction + 1} has blendR=0"
            )

        left_length = segment_lengths_mm[junction]
        right_length = segment_lengths_mm[junction + 1]

        effective = requested
        if left_length > 1.0:
            effective = min(effective, 0.45 * left_length)
        if right_length > 1.0:
            effective = min(effective, 0.45 * right_length)
        if effective <= 1e-9:
            raise RuntimeError(
                f"linked-LIN effective blend radius at junction {junction + 1} "
                "collapsed to zero"
            )

        left_trim = _blend_trim_fraction(
            effective,
            left_length,
            rotation_dominant_xyz_mm,
        )
        right_trim = _blend_trim_fraction(
            effective,
            right_length,
            rotation_dominant_xyz_mm,
        )

        entries.append(
            _interpolate_transform(
                transforms[junction],
                transforms[junction + 1],
                1.0 - left_trim,
            )
        )
        exits.append(
            _interpolate_transform(
                transforms[junction + 1],
                transforms[junction + 2],
                right_trim,
            )
        )
        effective_radii.append(effective)

    max_translation_mm = max(0.1, float(max_translation_mm))
    max_orientation_deg = max(0.1, float(max_orientation_deg))
    sample_count = max(6, int(sample_count))

    poses: list[Pose] = []
    current = transforms[0]

    for junction in range(len(entries)):
        for pose in _poses_between_transforms(
            current,
            entries[junction],
            max_translation_mm=max_translation_mm,
            max_orientation_deg=max_orientation_deg,
        ):
            _append_pose_if_distinct(poses, pose)

        entry = entries[junction]
        corner = transforms[junction + 1]
        exit_transform = exits[junction]

        entry_rotation = Rotation.from_matrix(entry[:3, :3])
        corner_rotation = Rotation.from_matrix(corner[:3, :3])
        exit_rotation = Rotation.from_matrix(exit_transform[:3, :3])
        entry_to_corner = Slerp(
            [0.0, 1.0],
            Rotation.concatenate([entry_rotation, corner_rotation]),
        )
        corner_to_exit = Slerp(
            [0.0, 1.0],
            Rotation.concatenate([corner_rotation, exit_rotation]),
        )

        for sample in range(1, sample_count + 1):
            u = float(sample) / float(sample_count)
            a = (1.0 - u) ** 2
            b = 2.0 * (1.0 - u) * u
            c = u ** 2

            transform = entry.copy()
            transform[:3, 3] = (
                a * entry[:3, 3]
                + b * corner[:3, 3]
                + c * exit_transform[:3, 3]
            )

            if u <= 0.5:
                rotation = entry_to_corner([2.0 * u])[0]
            else:
                rotation = corner_to_exit([2.0 * u - 1.0])[0]
            transform[:3, :3] = rotation.as_matrix()
            _append_pose_if_distinct(poses, _pose_from_transform(transform))

        current = exit_transform

    for pose in _poses_between_transforms(
        current,
        transforms[-1],
        max_translation_mm=max_translation_mm,
        max_orientation_deg=max_orientation_deg,
    ):
        _append_pose_if_distinct(poses, pose)

    if not poses:
        raise RuntimeError("linked-LIN blended pose construction produced no poses")

    return poses, effective_radii


def _blend_trim_fraction(
    radius_mm: float,
    cartesian_length_mm: float,
    rotation_dominant_xyz_mm: float,
) -> float:
    radius_mm = max(0.0, float(radius_mm))
    cartesian_length_mm = max(0.0, float(cartesian_length_mm))
    rotation_dominant_xyz_mm = max(1.0, float(rotation_dominant_xyz_mm))
    if cartesian_length_mm > rotation_dominant_xyz_mm:
        return min(0.45, radius_mm / cartesian_length_mm)
    return 0.25


def _translation_distance_mm(a, b) -> float:
    return float(math.dist(a[:3, 3], b[:3, 3]) * 1000.0)


def _interpolate_transform(start, target, ratio: float):
    ratio = max(0.0, min(1.0, float(ratio)))
    transform = start.copy()
    transform[:3, 3] = (
        start[:3, 3] + (target[:3, 3] - start[:3, 3]) * ratio
    )
    start_rotation = Rotation.from_matrix(start[:3, :3])
    target_rotation = Rotation.from_matrix(target[:3, :3])
    slerp = Slerp(
        [0.0, 1.0],
        Rotation.concatenate([start_rotation, target_rotation]),
    )
    transform[:3, :3] = slerp([ratio])[0].as_matrix()
    return transform


def _poses_between_transforms(
    start,
    target,
    *,
    max_translation_mm: float,
    max_orientation_deg: float,
) -> list[Pose]:
    distance_mm = _translation_distance_mm(start, target)
    start_rotation = Rotation.from_matrix(start[:3, :3])
    target_rotation = Rotation.from_matrix(target[:3, :3])
    orientation_delta_deg = float(
        (start_rotation.inv() * target_rotation).magnitude() * 180.0 / math.pi
    )

    if distance_mm <= 1e-9 and orientation_delta_deg <= 1e-9:
        return []

    steps = max(
        1,
        int(math.ceil(distance_mm / max_translation_mm)),
        int(math.ceil(orientation_delta_deg / max_orientation_deg)),
    )
    slerp = Slerp(
        [0.0, 1.0],
        Rotation.concatenate([start_rotation, target_rotation]),
    )

    poses = []
    for step in range(1, steps + 1):
        ratio = float(step) / float(steps)
        transform = start.copy()
        transform[:3, 3] = (
            start[:3, 3] + (target[:3, 3] - start[:3, 3]) * ratio
        )
        transform[:3, :3] = slerp([ratio])[0].as_matrix()
        poses.append(_pose_from_transform(transform))
    return poses


def _append_pose_if_distinct(poses: list[Pose], pose: Pose) -> None:
    if poses and _same_pose(poses[-1], pose):
        return
    poses.append(pose)


def _same_pose(a: Pose, b: Pose) -> bool:
    if (
        abs(float(a.position.x) - float(b.position.x)) > 1e-10
        or abs(float(a.position.y) - float(b.position.y)) > 1e-10
        or abs(float(a.position.z) - float(b.position.z)) > 1e-10
    ):
        return False

    qa = [
        float(a.orientation.x),
        float(a.orientation.y),
        float(a.orientation.z),
        float(a.orientation.w),
    ]
    qb = [
        float(b.orientation.x),
        float(b.orientation.y),
        float(b.orientation.z),
        float(b.orientation.w),
    ]
    dot = abs(sum(x * y for x, y in zip(qa, qb)))
    return abs(1.0 - dot) <= 1e-10


def _pose_from_transform(transform) -> Pose:
    quat = TransformationUtils.matrix_to_quaternion(transform[:3, :3])
    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


__all__ = ["build_linked_lin_blend_poses"]
