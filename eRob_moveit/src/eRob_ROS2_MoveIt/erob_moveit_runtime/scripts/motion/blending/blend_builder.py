#!/usr/bin/env python3
"""Build blended ordered-chain joint trajectories."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from time import perf_counter
from typing import Any, Callable

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


StateValidityFn = Callable[[list[str], list[float]], Any]


@dataclass(frozen=True)
class BlendBuilderConfig:
    rotation_dominant_xyz_mm: float = 5.0
    junction_tolerance_rad: float = 0.02
    min_radius_mm: float = 0.5
    sample_count: int = 12


def xyz_distance_mm(a, b) -> float:
    return math.sqrt(
        (float(b[0]) - float(a[0])) ** 2
        + (float(b[1]) - float(a[1])) ** 2
        + (float(b[2]) - float(a[2])) ** 2
    )


def joint_path_distances(trajectory) -> list[float]:
    """Return cumulative Euclidean joint-space distances for a trajectory."""

    points = list(getattr(trajectory, "points", []) or [])
    if not points:
        return []

    distances = [0.0]
    for index in range(1, len(points)):
        previous = points[index - 1].positions
        current = points[index].positions
        step = math.sqrt(
            sum(
                (float(current[joint]) - float(previous[joint])) ** 2
                for joint in range(min(len(previous), len(current)))
            )
        )
        distances.append(distances[-1] + step)
    return distances


class BlendBuilder:
    def __init__(self, logger, state_validity_fn: StateValidityFn, config: BlendBuilderConfig | None = None):
        self._logger = logger
        self._state_validity_fn = state_validity_fn
        self._config = config or BlendBuilderConfig()

    def _blend_trim_fraction(self, radius_mm: float, cartesian_length_mm: float) -> float:
        radius_mm = max(0.0, float(radius_mm))
        cartesian_length_mm = max(0.0, float(cartesian_length_mm))
        rotation_dominant_xyz_mm = max(1.0, float(self._config.rotation_dominant_xyz_mm))
        if cartesian_length_mm > rotation_dominant_xyz_mm:
            return min(0.45, radius_mm / cartesian_length_mm)
        return 0.25

    def build(self, planned_segments):
        """
        Build one raw JointTrajectory from a contiguous LIN/PTP blend group.

        The input shape intentionally matches the current ordered-chain planned
        segment dictionaries so this extraction can remain behavior-preserving.
        """

        if len(planned_segments) < 2:
            raise RuntimeError("Blended group requires at least two segments")

        segment_count = len(planned_segments)
        trajectories = []
        joint_distances = []
        joint_lengths = []
        cartesian_lengths_mm = []

        for planned in planned_segments:
            trajectory = planned.get("trajectory")
            if trajectory is None:
                raise RuntimeError(f"Cannot blend empty/no-op segment {planned.get('label')!r}")

            points = list(getattr(trajectory, "points", []) or [])
            if len(points) < 2:
                raise RuntimeError(
                    f"Blend segment {planned.get('label')!r} requires at least 2 trajectory points; got {len(points)}"
                )

            if len(points) < 7:
                first_positions = [float(value) for value in points[0].positions]
                last_positions = [float(value) for value in points[-1].positions]
                if len(first_positions) != len(last_positions):
                    raise RuntimeError(f"Blend segment {planned.get('label')!r} has mismatched joint dimensions")

                original_count = len(points)
                densified_points = []
                for sample_index in range(7):
                    fraction = float(sample_index) / 6.0
                    point = JointTrajectoryPoint()
                    point.positions = [
                        start_value + (end_value - start_value) * fraction
                        for start_value, end_value in zip(first_positions, last_positions)
                    ]
                    point.velocities = []
                    point.accelerations = []
                    point.effort = []
                    point.time_from_start.sec = 0
                    point.time_from_start.nanosec = 0
                    densified_points.append(point)

                trajectory.points = densified_points
                points = list(trajectory.points)
                self._logger.info(
                    "[OrderedBlend] Densified short segment "
                    f"{planned.get('label')!r} from {original_count} to 7 joint-space points"
                )

            trajectories.append(trajectory)
            cartesian_length_mm = xyz_distance_mm(planned["start_position"], planned["target_position"])
            distances = joint_path_distances(trajectory)
            joint_length = distances[-1] if distances else 0.0
            if joint_length <= 1e-9:
                raise RuntimeError(f"Cannot blend true no-op segment {planned.get('label')!r}")

            cartesian_lengths_mm.append(cartesian_length_mm)
            joint_distances.append(distances)
            joint_lengths.append(joint_length)

        joint_names = list(trajectories[0].joint_names)
        if not joint_names:
            raise RuntimeError("Blended group has no joint names")

        for index, trajectory in enumerate(trajectories[1:], start=1):
            if list(trajectory.joint_names) != joint_names:
                raise RuntimeError(f"Joint-name/order mismatch at blend segment {index + 1}")

        junction_tolerance = float(self._config.junction_tolerance_rad)
        for junction in range(segment_count - 1):
            left_end = list(trajectories[junction].points[-1].positions)
            right_start = list(trajectories[junction + 1].points[0].positions)
            max_error = max(abs(float(a) - float(b)) for a, b in zip(left_end, right_start))
            if max_error > junction_tolerance:
                raise RuntimeError(
                    f"Blend junction {junction + 1}/{segment_count - 1} mismatch: "
                    f"{max_error:.6f}rad > {junction_tolerance:.6f}rad"
                )

        requested_radii = []
        effective_radii = []
        for junction in range(segment_count - 1):
            requested = max(0.0, float(planned_segments[junction].get("blendR", 0.0) or 0.0))
            if requested <= 0.0:
                raise RuntimeError(
                    f"Internal blend-group segment {planned_segments[junction].get('label')!r} has blendR=0"
                )

            effective = requested
            left_cartesian_mm = cartesian_lengths_mm[junction]
            right_cartesian_mm = cartesian_lengths_mm[junction + 1]
            if left_cartesian_mm > 1.0:
                effective = min(effective, 0.45 * left_cartesian_mm)
            if right_cartesian_mm > 1.0:
                effective = min(effective, 0.45 * right_cartesian_mm)
            requested_radii.append(requested)
            effective_radii.append(effective)

        min_radius = float(self._config.min_radius_mm)
        for junction, effective in enumerate(effective_radii):
            if effective < min_radius:
                raise RuntimeError(
                    f"Effective blend radius at junction {junction + 1} is too small: {effective:.3f}mm"
                )

        entry_indices = [None] * (segment_count - 1)
        exit_indices = [None] * (segment_count - 1)
        for junction in range(segment_count - 1):
            left_points = list(trajectories[junction].points)
            right_points = list(trajectories[junction + 1].points)
            radius = effective_radii[junction]
            left_trim_fraction = self._blend_trim_fraction(radius, cartesian_lengths_mm[junction])
            right_trim_fraction = self._blend_trim_fraction(radius, cartesian_lengths_mm[junction + 1])
            left_target_distance = joint_lengths[junction] * (1.0 - left_trim_fraction)
            right_target_distance = joint_lengths[junction + 1] * right_trim_fraction
            left_distances = joint_distances[junction]
            right_distances = joint_distances[junction + 1]

            entry_indices[junction] = min(
                range(1, len(left_points) - 1),
                key=lambda point_index: abs(left_distances[point_index] - left_target_distance),
            )
            exit_indices[junction] = min(
                range(1, len(right_points) - 1),
                key=lambda point_index: abs(right_distances[point_index] - right_target_distance),
            )

        for segment_index in range(1, segment_count - 1):
            start_index = exit_indices[segment_index - 1]
            end_index = entry_indices[segment_index]
            if start_index >= end_index:
                points = list(trajectories[segment_index].points)
                fallback_start = max(1, int(round(0.25 * (len(points) - 1))))
                fallback_end = min(len(points) - 2, int(round(0.75 * (len(points) - 1))))
                if fallback_start >= fallback_end:
                    raise RuntimeError(
                        f"Blend regions overlap in segment {segment_index + 1} "
                        f"{planned_segments[segment_index].get('label')!r}: "
                        f"start={start_index} end={end_index}; cannot create safe fallback with {len(points)} points"
                    )

                self._logger.warning(
                    "[OrderedBlend] Shrinking overlapping blend regions "
                    f"for segment {segment_index + 1} {planned_segments[segment_index].get('label')!r}: "
                    f"requested start={start_index} end={end_index} -> "
                    f"fallback start={fallback_start} end={fallback_end}"
                )
                exit_indices[segment_index - 1] = fallback_start
                entry_indices[segment_index] = fallback_end

        sample_count = max(6, int(self._config.sample_count))
        blends = []
        for junction in range(segment_count - 1):
            left = trajectories[junction]
            right = trajectories[junction + 1]
            q0 = [float(v) for v in left.points[entry_indices[junction]].positions]
            q1 = [float(v) for v in left.points[-1].positions]
            q2 = [float(v) for v in right.points[exit_indices[junction]].positions]
            blend_positions = []
            for sample in range(sample_count + 1):
                u = float(sample) / float(sample_count)
                a = (1.0 - u) ** 2
                b = 2.0 * (1.0 - u) * u
                c = u ** 2
                blend_positions.append([a * q0[joint] + b * q1[joint] + c * q2[joint] for joint in range(len(q0))])
            blends.append(blend_positions)

        validation_started = perf_counter()
        for junction, blend_positions in enumerate(blends):
            for sample_index, q in enumerate(blend_positions):
                validity = self._state_validity_fn(joint_names, q)
                if bool(getattr(validity, "valid", False)):
                    continue

                contacts = []
                for contact in list(getattr(validity, "contacts", []) or []):
                    body_1 = str(
                        getattr(contact, "contact_body_1", "")
                        or getattr(contact, "body_name_1", "")
                    )
                    body_2 = str(
                        getattr(contact, "contact_body_2", "")
                        or getattr(contact, "body_name_2", "")
                    )
                    if body_1 or body_2:
                        contacts.append(f"{body_1}<->{body_2}" if body_1 and body_2 else body_1 or body_2)

                detail = f" contacts={contacts}" if contacts else ""
                raise RuntimeError(f"Blend junction {junction + 1} sample {sample_index}/{sample_count} is invalid{detail}")

        merged = JointTrajectory()
        merged.joint_names = list(joint_names)

        def append_positions(positions):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            merged.points.append(point)

        for segment_index in range(segment_count):
            points = list(trajectories[segment_index].points)
            start_index = 0 if segment_index == 0 else exit_indices[segment_index - 1]
            end_index = len(points) - 1 if segment_index == segment_count - 1 else entry_indices[segment_index]

            for point_index in range(start_index, end_index + 1):
                positions = points[point_index].positions
                if merged.points:
                    previous = merged.points[-1].positions
                    if all(abs(float(a) - float(b)) <= 1e-12 for a, b in zip(previous, positions)):
                        continue
                append_positions(positions)

            if segment_index < segment_count - 1:
                for q in blends[segment_index][1:-1]:
                    append_positions(q)

        self._logger.info(
            "[OrderedBlend] Built raw group "
            f"segments={segment_count} "
            f"labels={[s.get('label') for s in planned_segments]} "
            f"requested_radii={[round(v, 3) for v in requested_radii]} "
            f"effective_radii={[round(v, 3) for v in effective_radii]} "
            f"cartesian_lengths_mm={[round(v, 3) for v in cartesian_lengths_mm]} "
            f"joint_lengths={[round(v, 6) for v in joint_lengths]} "
            f"entry_indices={entry_indices} "
            f"exit_indices={exit_indices} "
            f"merged_points={len(merged.points)} "
            f"validation_s={perf_counter() - validation_started:.3f}"
        )

        return merged, effective_radii


def wait_moveit_state_validity(
    planning_node,
    config_obj,
    joint_names,
    joint_positions,
    timeout_s: float = 2.0,
):
    """Call MoveIt state validity for ordered blend samples."""

    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    client = planning_node.get_state_validity_client()
    if client is None or not client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("MoveIt state-validity service unavailable")

    request = GetStateValidity.Request()
    joint_state = JointState()
    joint_state.name = list(joint_names)
    joint_state.position = [float(value) for value in joint_positions]
    request.robot_state.joint_state = joint_state
    request.robot_state.is_diff = True
    request.group_name = str(config_obj.PLANNING_GROUP)

    future = client.call_async(request)
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.001)

    raise TimeoutError("MoveIt state-validity request timed out")


def build_ordered_blend_builder(planning_node, config_obj) -> BlendBuilder:
    """Create the ordered-chain blend builder from runtime config."""

    return BlendBuilder(
        planning_node.get_logger(),
        lambda joint_names, joint_positions: wait_moveit_state_validity(
            planning_node,
            config_obj,
            joint_names,
            joint_positions,
        ),
        BlendBuilderConfig(
            rotation_dominant_xyz_mm=float(
                getattr(config_obj, "ORDERED_BLEND_ROTATION_DOMINANT_XYZ_MM", 5.0)
            ),
            junction_tolerance_rad=float(
                getattr(config_obj, "ORDERED_BLEND_JUNCTION_TOL_RAD", 0.02)
            ),
            min_radius_mm=float(
                getattr(config_obj, "ORDERED_BLEND_MIN_RADIUS_MM", 0.5)
            ),
            sample_count=int(getattr(config_obj, "ORDERED_BLEND_SAMPLES", 12)),
        ),
    )


__all__ = [
    "BlendBuilder",
    "BlendBuilderConfig",
    "build_ordered_blend_builder",
    "joint_path_distances",
    "wait_moveit_state_validity",
    "xyz_distance_mm",
]
