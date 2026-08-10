#!/usr/bin/env python3
"""Small helpers for legacy ordered-chain Joint 6 unwind planning."""

from __future__ import annotations

from dataclasses import dataclass
import math
from time import perf_counter
from typing import Any, Callable, Mapping


@dataclass(frozen=True)
class OrderedUnwindConfig:
    axis_index: int
    joint_name: str
    min_delta_rad: float
    velocity_percent: float
    acceleration_percent: float
    velocity_scale: float
    acceleration_scale: float
    sign: float
    max_step_deg: float
    live_final_execution: bool


def joint_positions_by_name(state) -> dict[str, float]:
    names = list(getattr(state.joint_state, "name", []) or [])
    values = list(getattr(state.joint_state, "position", []) or [])
    return {name: float(value) for name, value in zip(names, values)}


def force_ordered_unwind_joint_branch(
    joint_trajectory,
    joint_name: str,
    start_value: float,
    target_value: float,
    *,
    logger=None,
) -> None:
    if joint_trajectory is None or not getattr(joint_trajectory, "points", None):
        return
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    if joint_name not in joint_names:
        return
    joint_index = joint_names.index(joint_name)
    points = list(joint_trajectory.points)
    if len(points) < 2:
        return

    start_value = float(start_value)
    target_value = float(target_value)
    delta = target_value - start_value
    for point_index, point in enumerate(points):
        positions = list(point.positions)
        if joint_index >= len(positions):
            continue
        fraction = point_index / max(1, len(points) - 1)
        positions[joint_index] = start_value + delta * fraction
        point.positions = positions

    if logger is not None:
        logger.info(
            "[UNWIND_J6] Forced ordered unwind joint branch before optimization: "
            f"{joint_name} {start_value:.3f} -> {target_value:.3f} rad "
            f"points={len(points)}"
        )


def build_ordered_unwind_direct_ik_planner(
    *,
    planning_node: Any,
    config_obj: Any,
    tool_transform: Any,
    rotational_path_waypoints_base: Callable[..., Any],
    optimize_sync: Callable[..., tuple[Any, float]],
    to_pose_list_fn: Callable[..., Any] | None = None,
    build_direct_contour_trajectory_fn: Callable[..., Any] | None = None,
    log_report_fn: Callable[..., None] | None = None,
) -> Callable[..., Any]:
    """Build the direct-IK trajectory callback used by ordered unwind planning."""

    if to_pose_list_fn is None:
        from motion.planning.planner_utils import _to_pose_list as to_pose_list_fn
    if build_direct_contour_trajectory_fn is None:
        from motion.planning.direct_contour_ik import (
            _build_direct_contour_trajectory as build_direct_contour_trajectory_fn,
        )
    if log_report_fn is None:
        from motion.planning.direct_contour_ik import _log_report as log_report_fn

    def _plan_unwind_direct_ik_trajectory(
        current_pos_wobj,
        target_pos_wobj,
        rotation_index,
        vel_scale,
        acc_scale,
        seed_state,
        joint_name=None,
        joint_start=None,
        joint_target=None,
    ):
        segment_started = perf_counter()
        direct_ik_step_deg = max(
            0.1,
            float(
                getattr(
                    config_obj,
                    "EXECUTOR_POST_UNWIND_DIRECT_IK_STEP_DEG",
                    4.0,
                )
            ),
        )
        waypoints_base = rotational_path_waypoints_base(
            current_pos_wobj,
            target_pos_wobj,
            rotation_index,
            max_step_override_deg=direct_ik_step_deg,
            apply_workobject_to_waypoints=False,
        )
        poses, err = to_pose_list_fn(
            planning_node,
            waypoints_base,
            tool_transform,
            check_last_only=True,
        )
        if err:
            raise RuntimeError(f"unwind pose conversion failed with result {err}")

        ik_result = build_direct_contour_trajectory_fn(
            planning_node,
            poses,
            seed_state=seed_state,
        )
        ik_result.report.timings["total_before_optimizer_s"] = (
            perf_counter() - segment_started
        )
        log_report_fn(planning_node, ik_result.report)
        if not ik_result.report.ok:
            raise RuntimeError(
                "unwind direct IK failed: "
                f"{ik_result.report.failure_reason} {ik_result.report.details}"
            )

        if (
            joint_name is not None
            and joint_start is not None
            and joint_target is not None
        ):
            force_ordered_unwind_joint_branch(
                ik_result.trajectory.joint_trajectory,
                str(joint_name),
                float(joint_start),
                float(joint_target),
                logger=planning_node.get_logger(),
            )

        optimizer_name = (
            str(
                getattr(
                    config_obj,
                    "EXECUTOR_POST_UNWIND_DIRECT_IK_OPTIMIZER",
                    "",
                )
                or ""
            )
            .strip()
            .upper()
            or None
        )
        optimized, optimize_elapsed = optimize_sync(
            planning_node,
            ik_result.trajectory,
            vel_scale,
            acc_scale,
            optimizer_name=optimizer_name,
        )
        planning_node.get_logger().info(
            f"[TIMING] ordered_chain_unwind_plan_piece waypoints={len(waypoints_base)} "
            f"points={len(getattr(optimized.joint_trajectory, 'points', []) or [])} "
            f"optimize_s={optimize_elapsed:.3f} "
            f"elapsed_s={perf_counter() - segment_started:.3f}"
        )
        return optimized.joint_trajectory

    return _plan_unwind_direct_ik_trajectory


def parse_ordered_unwind_config(
    segment: Mapping[str, Any],
    *,
    config_obj,
    clamp_percentage: Callable[[Any], float],
    is_final_segment: bool,
) -> OrderedUnwindConfig:
    """Parse ordered unwind settings without importing runtime config globally."""

    velocity_percent = clamp_percentage(
        segment.get(
            "vel",
            getattr(config_obj, "DEFAULT_VEL_PERCENT"),
        )
    )
    acceleration_percent = clamp_percentage(
        segment.get(
            "acc",
            getattr(config_obj, "DEFAULT_ACC_PERCENT"),
        )
    )
    sign = float(
        getattr(
            config_obj,
            "EXECUTOR_POST_UNWIND_ROTATION_AXIS_SIGN",
            1.0,
        )
    )
    if abs(sign) < 1e-9:
        sign = 1.0

    return OrderedUnwindConfig(
        axis_index=int(
            getattr(
                config_obj,
                "EXECUTOR_POST_UNWIND_ROTATION_AXIS_INDEX",
                5,
            )
        ),
        joint_name=str(
            getattr(
                config_obj,
                "EXECUTOR_POST_UNWIND_JOINT_NAME",
                "Joint_6",
            )
        ).strip(),
        min_delta_rad=float(
            getattr(
                config_obj,
                "EXECUTOR_POST_UNWIND_MIN_DELTA_RAD",
                0.5,
            )
        ),
        velocity_percent=velocity_percent,
        acceleration_percent=acceleration_percent,
        velocity_scale=velocity_percent / 100.0,
        acceleration_scale=acceleration_percent / 100.0,
        sign=sign,
        max_step_deg=max(
            1.0,
            abs(
                float(
                    getattr(
                        config_obj,
                        "EXECUTOR_POST_UNWIND_ROTATIONAL_SEGMENT_DEG",
                        180.0,
                    )
                )
            ),
        ),
        live_final_execution=bool(
            is_final_segment
            and bool(
                getattr(
                    config_obj,
                    "EXECUTOR_ORDERED_FINAL_UNWIND_LIVE_EXECUTION",
                    True,
                )
            )
        ),
    )


def build_ordered_unwind_runtime_result(
    *,
    segment_type: str,
    label: str,
    current_cartesian,
    current_state,
    segment: Mapping[str, Any],
    config_obj,
    plan_started: float,
    protected: bool,
) -> dict[str, Any]:
    return {
        "type": segment_type,
        "label": label,
        "target_position": list(current_cartesian[:6]),
        "final_state": current_state,
        "runtime_unwind": True,
        "vel": segment.get("vel", getattr(config_obj, "DEFAULT_VEL_PERCENT")),
        "acc": segment.get("acc", getattr(config_obj, "DEFAULT_ACC_PERCENT")),
        "trajectories": [],
        "trajectory_checks": [],
        "check": None,
        "plan_elapsed_s": perf_counter() - plan_started,
        "protected": protected,
        "blendR": 0.0,
    }


def plan_ordered_unwind_segment(
    planning_node,
    *,
    index: int,
    total_segments: int,
    segment: Mapping[str, Any],
    params: Any,
    current_cartesian,
    current_state,
    config_obj,
    clamp_percentage: Callable[[Any], float],
    canonical_angle: Callable[[float], float],
    plan_unwind_direct_ik_trajectory_fn: Callable[..., Any],
    robot_state_from_trajectory_end_fn: Callable[[Any], Any],
    plan_started: float,
) -> dict[str, Any]:
    segment_type = params.segment_type
    label = params.label
    blend_r = params.blend_radius
    protected = params.protected

    if blend_r > 0.0:
        raise RuntimeError(f"Ordered unwind segment {label!r} cannot use blendR")

    joint_names = list(getattr(config_obj, "JOINT_NAMES", []) or [])
    unwind_config = parse_ordered_unwind_config(
        segment,
        config_obj=config_obj,
        clamp_percentage=clamp_percentage,
        is_final_segment=index == total_segments - 1,
    )
    joint_name = unwind_config.joint_name

    if joint_name not in joint_names:
        raise RuntimeError(f"Joint {joint_name!r} is not configured")

    if unwind_config.live_final_execution:
        planning_node.get_logger().info(
            "[UNWIND_J6] Ordered final unwind will be planned live during execution"
        )
        return build_ordered_unwind_runtime_result(
            segment_type=segment_type,
            label=label,
            current_cartesian=current_cartesian,
            current_state=current_state,
            segment=segment,
            config_obj=config_obj,
            plan_started=plan_started,
            protected=protected,
        )

    axis_index = unwind_config.axis_index
    joint_index = joint_names.index(joint_name)
    by_name = joint_positions_by_name(current_state)
    current_value = float(by_name[joint_name])
    final_target = canonical_angle(current_value)
    min_delta = unwind_config.min_delta_rad
    remaining = final_target - current_value

    if abs(remaining) < min_delta:
        planning_node.get_logger().info(
            "[UNWIND_J6] Ordered-chain unwind skipped - no unwind needed "
            f"({joint_name} current={current_value:.4f}rad "
            f"target={final_target:.4f}rad "
            f"delta={remaining:.4f}rad "
            f"min_delta={min_delta:.4f}rad)"
        )
        return build_ordered_unwind_noop_result(
            segment_type=segment_type,
            label=label,
            current_cartesian=current_cartesian,
            current_state=current_state,
            plan_started=plan_started,
            protected=protected,
        )

    total_delta_deg = math.degrees(remaining) * unwind_config.sign
    segment_count = max(
        1,
        int(math.ceil(abs(total_delta_deg) / unwind_config.max_step_deg)),
    )

    planning_node.get_logger().info(
        "[UNWIND_J6] Planning ordered-chain rotational unwind: "
        f"{joint_name} {current_value:.3f} -> {final_target:.3f} rad "
        f"delta={remaining:.3f} rad "
        f"cart_axis={axis_index} "
        f"cart_delta={total_delta_deg:.3f}deg "
        f"segments={segment_count} "
        f"max_segment={unwind_config.max_step_deg:.1f}deg "
        f"vel={unwind_config.velocity_percent:.1f}% "
        f"acc={unwind_config.acceleration_percent:.1f}%"
    )

    trajectories = []
    trajectory_checks = []
    planning_state = current_state
    planning_cartesian = list(current_cartesian[:6])
    planning_value = current_value

    for unwind_index in range(1, segment_count + 1):
        remaining = final_target - planning_value
        remaining_deg = math.degrees(remaining) * unwind_config.sign
        if abs(remaining) < min_delta:
            break

        segment_delta_deg = math.copysign(
            min(abs(remaining_deg), unwind_config.max_step_deg),
            remaining_deg,
        )
        segment_joint_target = (
            planning_value + math.radians(segment_delta_deg) / unwind_config.sign
        )
        target_cartesian = list(planning_cartesian[:6])
        target_cartesian[axis_index] = float(target_cartesian[axis_index]) + segment_delta_deg

        planning_node.get_logger().info(
            f"[UNWIND_J6] Planning ordered unwind segment "
            f"{unwind_index}/{segment_count}: "
            f"{planning_value:.3f} -> {segment_joint_target:.3f} rad "
            f"(final={final_target:.3f}), "
            f"cart_delta={segment_delta_deg:.3f}deg"
        )

        joint_trajectory = plan_unwind_direct_ik_trajectory_fn(
            planning_cartesian,
            target_cartesian,
            axis_index,
            unwind_config.velocity_scale,
            unwind_config.acceleration_scale,
            planning_state,
            joint_name=joint_name,
            joint_start=planning_value,
            joint_target=segment_joint_target,
        )
        trajectories.append(joint_trajectory)
        trajectory_checks.append({
            "joint_names": joint_names,
            "joint_name": joint_name,
            "joint_index": joint_index,
            "target_value": segment_joint_target,
        })

        planning_state = robot_state_from_trajectory_end_fn(joint_trajectory)
        planning_cartesian = target_cartesian
        planning_value = float(
            joint_positions_by_name(planning_state).get(joint_name, planning_value)
        )

    return build_ordered_unwind_planned_result(
        segment_type=segment_type,
        label=label,
        planning_cartesian=planning_cartesian,
        planning_state=planning_state,
        trajectories=trajectories,
        trajectory_checks=trajectory_checks,
        joint_names=joint_names,
        joint_name=joint_name,
        joint_index=joint_index,
        final_target=final_target,
        plan_started=plan_started,
        protected=protected,
    )


def build_ordered_unwind_noop_result(
    *,
    segment_type: str,
    label: str,
    current_cartesian,
    current_state,
    plan_started: float,
    protected: bool,
) -> dict[str, Any]:
    return {
        "type": segment_type,
        "label": label,
        "target_position": list(current_cartesian[:6]),
        "final_state": current_state,
        "trajectories": [],
        "trajectory_checks": [],
        "check": None,
        "plan_elapsed_s": perf_counter() - plan_started,
        "protected": protected,
        "blendR": 0.0,
    }


def build_ordered_unwind_planned_result(
    *,
    segment_type: str,
    label: str,
    planning_cartesian,
    planning_state,
    trajectories,
    trajectory_checks,
    joint_names,
    joint_name: str,
    joint_index: int,
    final_target: float,
    plan_started: float,
    protected: bool,
) -> dict[str, Any]:
    return {
        "type": segment_type,
        "label": label,
        "target_position": list(planning_cartesian[:6]),
        "final_state": planning_state,
        "trajectories": trajectories,
        "trajectory_checks": trajectory_checks,
        "check": {
            "joint_names": joint_names,
            "joint_name": joint_name,
            "joint_index": joint_index,
            "target_value": final_target,
        },
        "plan_elapsed_s": perf_counter() - plan_started,
        "protected": protected,
        "blendR": 0.0,
    }


__all__ = [
    "OrderedUnwindConfig",
    "build_ordered_unwind_direct_ik_planner",
    "build_ordered_unwind_noop_result",
    "build_ordered_unwind_planned_result",
    "build_ordered_unwind_runtime_result",
    "force_ordered_unwind_joint_branch",
    "joint_positions_by_name",
    "parse_ordered_unwind_config",
    "plan_ordered_unwind_segment",
]
