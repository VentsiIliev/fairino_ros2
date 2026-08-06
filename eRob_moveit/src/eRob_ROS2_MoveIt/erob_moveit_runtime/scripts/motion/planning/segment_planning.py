#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
import math
from threading import Event
import time
from time import perf_counter

from moveit_msgs.msg import RobotState

import config
from motion.execution.trajectory_optimizer import resolve_trajectory_optimizer
from motion.planning.direct_contour_ik import (
    _build_direct_contour_trajectory,
    _direct_ik_should_run,
    _log_report,
)
from motion.planning.planner_utils import _to_pose_list
from motion.planning.single_target import _compute_max_step, _wrapped_angle_delta_deg
from motion.planning.trajectory import _path_length_mm, _simplify_cartesian_waypoints
from motion.planning.trajectory_planner import _build_cartesian_request


@dataclass
class _PlannedSegment:
    index: int
    label: str
    target_position: list[float]
    joint_trajectory: object
    final_state: RobotState
    plan_elapsed_s: float
    optimize_elapsed_s: float


def _wait_future(future, timeout_s: float):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.005)
    raise TimeoutError(f"Timed out after {timeout_s:.1f}s waiting for MoveIt response")


def _wait_execution_complete(rc, timeout_s: float) -> int:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if not bool(getattr(rc, "is_executing", False)):
            return int(getattr(rc, "last_move_result", 0))
        time.sleep(0.01)
    rc.get_logger().error(f"[SegmentPlan] Timed out waiting for segment execution after {timeout_s:.1f}s")
    return -1


def _robot_state_from_trajectory_end(joint_trajectory) -> RobotState:
    state = RobotState()
    state.joint_state.name = list(joint_trajectory.joint_names)
    last_point = joint_trajectory.points[-1]
    state.joint_state.position = list(last_point.positions)
    state.joint_state.velocity = [0.0] * len(state.joint_state.name)
    state.joint_state.effort = []
    state.is_diff = False
    return state


def _optimize_sync(rc, trajectory, vel_scaling: float, acc_scaling: float, optimizer_name=None):
    done = Event()
    result_holder = {"trajectory": None}
    optimizer = resolve_trajectory_optimizer(optimizer_name, node=rc, default_optimizer=rc.trajectory_optimizer)
    started = perf_counter()

    def on_done(result):
        result_holder["trajectory"] = result
        done.set()

    optimizer.optimize(rc, trajectory, vel_scaling, acc_scaling, on_done)
    if not done.wait(timeout=float(getattr(config, "CUSTOM_SEQUENCE_OPTIMIZE_TIMEOUT_S", 10.0))):
        raise TimeoutError("Timed out waiting for trajectory optimizer")
    optimized = result_holder["trajectory"]
    if optimized is None:
        raise RuntimeError("Trajectory optimizer failed")
    return optimized, perf_counter() - started


def _densify_follow_waypoints(waypoints_6d: list[list[float]]) -> list[list[float]]:
    if len(waypoints_6d) < 2:
        return [list(point) for point in waypoints_6d]

    max_translation_mm = float(getattr(config, "FOLLOW_PATH_DENSIFY_MAX_TRANSLATION_MM", 1.0))
    max_orientation_deg = float(getattr(config, "FOLLOW_PATH_DENSIFY_MAX_ORIENTATION_DEG", 0.0))
    if max_translation_mm <= 0.0 and max_orientation_deg <= 0.0:
        return [list(point) for point in waypoints_6d]

    dense = [list(waypoints_6d[0])]
    for start, end in zip(waypoints_6d, waypoints_6d[1:]):
        start_pose = list(start)
        end_pose = list(end)
        dx = float(end_pose[0]) - float(start_pose[0])
        dy = float(end_pose[1]) - float(start_pose[1])
        dz = float(end_pose[2]) - float(start_pose[2])
        distance_mm = math.sqrt(dx * dx + dy * dy + dz * dz)
        translation_segments = (
            int(math.ceil(distance_mm / max_translation_mm))
            if max_translation_mm > 0.0
            else 1
        )
        orientation_deltas = [
            ((float(end_pose[index]) - float(start_pose[index]) + 180.0) % 360.0) - 180.0
            for index in range(3, 6)
        ]
        orientation_delta_deg = max(abs(delta) for delta in orientation_deltas)
        orientation_segments = (
            int(math.ceil(orientation_delta_deg / max_orientation_deg))
            if max_orientation_deg > 0.0
            else 1
        )
        segment_count = max(1, translation_segments, orientation_segments)
        for step in range(1, segment_count + 1):
            ratio = step / segment_count
            point = [
                float(start_pose[index]) + (float(end_pose[index]) - float(start_pose[index])) * ratio
                for index in range(3)
            ]
            point.extend([
                float(start_pose[index]) + orientation_deltas[index - 3] * ratio
                for index in range(3, 6)
            ])
            dense.append(point)
    return dense


def _format_direct_ik_failure(report) -> str:
    parts = [str(report.failure_reason or "unknown")]
    if getattr(report, "first_failed_index", None) is not None:
        parts.append(f"index={int(report.first_failed_index)}")
    details = str(getattr(report, "details", "") or "").strip()
    if details:
        parts.append(details)
    return " ".join(parts)


def _format_follow_waypoint_diagnostics(waypoints_6d: list[list[float]], failed_index) -> str:
    if failed_index is None:
        return ""
    try:
        index = int(failed_index)
    except (TypeError, ValueError):
        return ""
    if index < 0 or index >= len(waypoints_6d):
        return f" failed_index_out_of_range={index}/{len(waypoints_6d)}"

    failed = [float(value) for value in waypoints_6d[index][:6]]
    progress = index / max(1, len(waypoints_6d) - 1)
    parts = [
        f"failed_waypoint=[{', '.join(f'{value:.3f}' for value in failed)}]",
        f"progress={progress:.3f}",
    ]
    if index > 0:
        prev = [float(value) for value in waypoints_6d[index - 1][:6]]
        trans_prev = math.sqrt(sum((failed[axis] - prev[axis]) ** 2 for axis in range(3)))
        orient_prev = max(_wrapped_angle_delta_deg(prev[axis], failed[axis]) for axis in range(3, 6))
        parts.append(f"prev_delta_mm={trans_prev:.3f}")
        parts.append(f"prev_delta_deg={orient_prev:.3f}")
    if index + 1 < len(waypoints_6d):
        nxt = [float(value) for value in waypoints_6d[index + 1][:6]]
        trans_next = math.sqrt(sum((nxt[axis] - failed[axis]) ** 2 for axis in range(3)))
        orient_next = max(_wrapped_angle_delta_deg(failed[axis], nxt[axis]) for axis in range(3, 6))
        parts.append(f"next_delta_mm={trans_next:.3f}")
        parts.append(f"next_delta_deg={orient_next:.3f}")
    return " " + " ".join(parts)


def _build_follow_path_trajectory(
    rc,
    *,
    command_path: list[list[float]],
    start_state: RobotState,
    tool_transform,
    vel_scaling: float,
    acc_scaling: float,
    trajectory_optimizer_name=None,
):
    started = perf_counter()
    raw_waypoints_6d = [list(point[:6]) for point in command_path]
    simplified_waypoints_6d = _simplify_cartesian_waypoints(raw_waypoints_6d)
    waypoints_6d = _densify_follow_waypoints(simplified_waypoints_6d)
    if len(waypoints_6d) != len(simplified_waypoints_6d):
        rc.get_logger().info(
            f"[FollowPath] Densified waypoints for direct IK: "
            f"{len(simplified_waypoints_6d)} -> {len(waypoints_6d)} "
            f"(max_translation_mm={float(getattr(config, 'FOLLOW_PATH_DENSIFY_MAX_TRANSLATION_MM', 1.0)):.3f}, "
            f"max_orientation_deg={float(getattr(config, 'FOLLOW_PATH_DENSIFY_MAX_ORIENTATION_DEG', 0.0)):.3f})"
        )
    total_dist_mm = _path_length_mm(waypoints_6d)
    poses, err = _to_pose_list(rc, waypoints_6d, tool_transform, check_last_only=True)
    if err:
        raise RuntimeError(f"follow path pose conversion failed with result {err}")

    direct_ik_requested = _direct_ik_should_run(rc, waypoints_6d, total_dist_mm)
    if direct_ik_requested:
        result = _build_direct_contour_trajectory(rc, poses, seed_state=start_state)
        result.report.timings["total_before_optimizer_s"] = perf_counter() - started
        _log_report(rc, result.report)
        if result.report.ok:
            optimized, optimize_elapsed = _optimize_sync(
                rc,
                result.trajectory,
                vel_scaling,
                acc_scaling,
                optimizer_name=trajectory_optimizer_name,
            )
            rc.get_logger().info(
                f"[TIMING] follow_path_plan method=direct_contour_ik "
                f"waypoints={len(waypoints_6d)} optimize_s={optimize_elapsed:.3f} "
                f"elapsed_s={perf_counter() - started:.3f}"
            )
            return optimized.joint_trajectory
        direct_ik_failure = _format_direct_ik_failure(result.report)
        waypoint_diagnostics = _format_follow_waypoint_diagnostics(
            waypoints_6d,
            getattr(result.report, "first_failed_index", None),
        )
        if not bool(getattr(config, "FOLLOW_PATH_CARTESIAN_FALLBACK_ENABLED", True)):
            raise RuntimeError(
                f"follow path direct contour IK failed: {direct_ik_failure} "
                f"waypoints={len(waypoints_6d)} total_dist_mm={total_dist_mm:.3f}"
                f"{waypoint_diagnostics}"
            )
        rc.get_logger().warning(
            f"[FollowPath] Direct contour IK rejected follow path; falling back to Cartesian path: "
            f"{direct_ik_failure}{waypoint_diagnostics}"
        )

    avg_spacing_mm = total_dist_mm / max(len(waypoints_6d) - 1, 1)
    max_step_scale = float(getattr(config, "PATH_EEF_STEP_SCALE", 2.5))
    max_step_min_m = float(getattr(config, "PATH_EEF_STEP_MIN_M", 0.005))
    max_step_max_m = float(getattr(config, "PATH_EEF_STEP_MAX_M", 0.03))
    max_step = min(max((avg_spacing_mm / 1000.0) * max_step_scale, max_step_min_m), max_step_max_m)
    request = _build_cartesian_request(
        rc,
        poses,
        max_step,
        vel_scaling,
        acc_scaling,
        start_state=start_state,
        avoid_collisions=config.resolve_avoid_collisions(None),
    )
    response = _wait_future(
        rc.request_cartesian_path(request),
        timeout_s=float(getattr(config, "FOLLOW_PATH_CARTESIAN_TIMEOUT_S", 30.0)),
    )
    fraction = float(getattr(response, "fraction", 0.0))
    trajectory = getattr(response, "solution", None)
    joint_trajectory = getattr(trajectory, "joint_trajectory", None) if trajectory is not None else None
    point_count = len(getattr(joint_trajectory, "points", []) or [])
    if fraction < config.CARTESIAN_MIN_FRACTION or point_count <= 1:
        raise RuntimeError(f"follow path Cartesian planning failed: fraction={fraction:.4f} points={point_count}")

    optimized, optimize_elapsed = _optimize_sync(
        rc,
        trajectory,
        vel_scaling,
        acc_scaling,
        optimizer_name=trajectory_optimizer_name,
    )
    rc.get_logger().info(
        f"[TIMING] follow_path_plan method=cartesian_path waypoints={len(waypoints_6d)} "
        f"fraction={fraction:.4f} points={point_count} optimize_s={optimize_elapsed:.3f} "
        f"elapsed_s={perf_counter() - started:.3f}"
    )
    return optimized.joint_trajectory


def _plan_segment(
    rc,
    *,
    index: int,
    segment: dict,
    start_cartesian: list[float],
    start_state: RobotState | None,
    tool_transform,
) -> _PlannedSegment:
    started = perf_counter()
    target = list(segment["position"])
    label = str(segment.get("label") or f"segment_{index + 1}")
    vel_scaling = max(0.0, min(1.0, float(segment["vel"]) / 100.0))
    acc_scaling = max(0.0, min(1.0, float(segment["acc"]) / 100.0))

    pose_started = perf_counter()
    poses, err = _to_pose_list(rc, [start_cartesian, target], tool_transform)
    pose_elapsed = perf_counter() - pose_started
    if err:
        raise RuntimeError(f"pose conversion failed with result {err}")

    geometry_started = perf_counter()
    p0, p1 = poses[0].position, poses[-1].position
    delta_m = ((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2) ** 0.5
    orientation_delta_deg = max(
        _wrapped_angle_delta_deg(start_cartesian[3], target[3]),
        _wrapped_angle_delta_deg(start_cartesian[4], target[4]),
        _wrapped_angle_delta_deg(start_cartesian[5], target[5]),
    )
    max_step = _compute_max_step(delta_m, orientation_delta_deg)
    geometry_elapsed = perf_counter() - geometry_started
    request_started = perf_counter()
    request = _build_cartesian_request(
        rc,
        poses,
        max_step,
        vel_scaling,
        acc_scaling,
        start_state=start_state,
        avoid_collisions=config.resolve_avoid_collisions(None),
    )
    request_elapsed = perf_counter() - request_started

    rc.get_logger().info(
        f"[SegmentPlan] Planning segment {index + 1}: label='{label}' "
        f"vel={float(segment['vel']):.1f}% acc={float(segment['acc']):.1f}% "
        f"delta_mm={delta_m * 1000.0:.3f} orientation_delta_deg={orientation_delta_deg:.3f} "
        f"max_step_m={max_step:.6f} start_seed={'planned' if start_state is not None else 'live'}"
    )
    rc.get_logger().info(
        f"[SEGMENT_PLAN_TIMING] segment={index + 1} label='{label}' stage=request_ready "
        f"pose_s={pose_elapsed:.3f} geometry_s={geometry_elapsed:.3f} request_s={request_elapsed:.3f} "
        f"waypoints={len(getattr(request, 'waypoints', []) or [])} avoid_collisions={bool(getattr(request, 'avoid_collisions', False))}"
    )
    dispatch_started = perf_counter()
    future = rc.request_cartesian_path(request)
    dispatch_elapsed = perf_counter() - dispatch_started
    wait_started = perf_counter()
    response = _wait_future(
        future,
        timeout_s=float(getattr(config, "CUSTOM_SEQUENCE_PLAN_TIMEOUT_S", 10.0)),
    )
    wait_elapsed = perf_counter() - wait_started
    parse_started = perf_counter()
    fraction = float(getattr(response, "fraction", 0.0))
    trajectory = getattr(response, "solution", None)
    joint_trajectory = getattr(trajectory, "joint_trajectory", None) if trajectory is not None else None
    point_count = len(getattr(joint_trajectory, "points", []) or [])
    parse_elapsed = perf_counter() - parse_started
    plan_elapsed = perf_counter() - started
    rc.get_logger().info(
        f"[SEGMENT_PLAN_TIMING] segment={index + 1} label='{label}' stage=cartesian_response "
        f"dispatch_s={dispatch_elapsed:.3f} wait_s={wait_elapsed:.3f} parse_s={parse_elapsed:.3f} "
        f"fraction={fraction:.4f} points={point_count} total_before_opt_s={plan_elapsed:.3f}"
    )
    rc.get_logger().info(
        f"[TIMING] segment_plan segment={index + 1} label='{label}' "
        f"fraction={fraction:.4f} points={point_count} elapsed_s={plan_elapsed:.3f}"
    )
    if fraction < config.CARTESIAN_MIN_FRACTION or point_count <= 1:
        raise RuntimeError(f"cartesian planning failed: fraction={fraction:.4f} points={point_count}")

    optimize_started = perf_counter()
    optimized, optimize_elapsed = _optimize_sync(rc, trajectory, vel_scaling, acc_scaling)
    optimize_total_elapsed = perf_counter() - optimize_started
    rc.get_logger().info(
        f"[SEGMENT_PLAN_TIMING] segment={index + 1} label='{label}' stage=optimizer "
        f"reported_s={optimize_elapsed:.3f} optimizer_total_s={optimize_total_elapsed:.3f} "
        f"segment_total_s={perf_counter() - started:.3f}"
    )
    optimized_joint_trajectory = optimized.joint_trajectory
    return _PlannedSegment(
        index=index,
        label=label,
        target_position=target,
        joint_trajectory=optimized_joint_trajectory,
        final_state=_robot_state_from_trajectory_end(optimized_joint_trajectory),
        plan_elapsed_s=plan_elapsed,
        optimize_elapsed_s=optimize_elapsed,
    )
