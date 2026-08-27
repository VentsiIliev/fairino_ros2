#!/usr/bin/env python3
"""Small ordered-chain execution helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from time import perf_counter
import time
from typing import Any, Callable


@dataclass(frozen=True)
class OrderedTrajectoryTiming:
    duration_s: float
    controller_goal_tolerance_s: float
    wait_timeout_s: float


@dataclass(frozen=True)
class OrderedSegmentExecutionHooks:
    node: Any
    logger: Any
    set_ordered_motion_chain_status: Callable[..., None]
    ordered_chain_executing_status: Callable[..., dict[str, Any]]
    ordered_chain_segment_finished_status: Callable[..., dict[str, Any]]
    mark_motion_timing: Callable[..., None]
    publish_scheduler_updates: Callable[[dict[str, Any]], None]


@dataclass(frozen=True)
class OrderedControllerExecutionHooks:
    node: Any
    logger: Any
    mark_motion_timing: Callable[..., None]
    wait_point_match: Callable[[str, Any, Any, str], bool]
    send_trajectory: Callable[..., None]
    wait_execution_complete: Callable[[Any, float], Any]


@dataclass(frozen=True)
class OrderedUnwindFinalizationHooks:
    node: Any
    verify_explicit_unwind_complete: Callable[[Any], bool]


@dataclass(frozen=True)
class OrderedStateMatchHooks:
    node: Any
    logger: Any
    get_live_joint_state: Callable[[], Any]
    mark_motion_timing: Callable[..., None]
    enabled: bool
    tolerance_rad: float
    timeout_s: float


@dataclass(frozen=True)
class OrderedPlannedSequenceHooks:
    node: Any
    logger: Any
    set_ordered_motion_chain_status: Callable[..., None]
    ordered_chain_stopped_status: Callable[..., dict[str, Any]]
    mark_motion_timing: Callable[..., None]
    execute_planned_segment: Callable[..., int]


@dataclass(frozen=True)
class OrderedExecutionHookBundle:
    segment_hooks: OrderedSegmentExecutionHooks
    state_match_hooks: OrderedStateMatchHooks
    controller_hooks: OrderedControllerExecutionHooks
    unwind_finalization_hooks: OrderedUnwindFinalizationHooks


@dataclass(frozen=True)
class OrderedPlannedSegmentExecutorConfig:
    min_timeout_s: float
    timeout_multiplier: float
    motion_error_result: int
    default_velocity_percent: float
    default_acceleration_percent: float
    verification_failure_result: int = -6


def build_ordered_execution_hook_bundle(
    *,
    node: Any,
    logger: Any,
    config_obj: Any,
    set_ordered_motion_chain_status: Callable[..., None],
    ordered_chain_executing_status: Callable[..., dict[str, Any]],
    ordered_chain_segment_finished_status: Callable[..., dict[str, Any]],
    mark_motion_timing: Callable[..., None],
    publish_scheduler_updates: Callable[[dict[str, Any]], None],
    send_trajectory: Callable[..., None],
    wait_execution_complete: Callable[[Any, float], Any],
    verify_explicit_unwind_complete: Callable[[Any], bool],
) -> OrderedExecutionHookBundle:
    """Build ordered-chain execution hooks from backend-owned callbacks."""

    segment_hooks = OrderedSegmentExecutionHooks(
        node=node,
        logger=logger,
        set_ordered_motion_chain_status=set_ordered_motion_chain_status,
        ordered_chain_executing_status=ordered_chain_executing_status,
        ordered_chain_segment_finished_status=ordered_chain_segment_finished_status,
        mark_motion_timing=mark_motion_timing,
        publish_scheduler_updates=publish_scheduler_updates,
    )
    state_match_hooks = OrderedStateMatchHooks(
        node=node,
        logger=logger,
        get_live_joint_state=lambda: getattr(node, "current_joint_state", None),
        mark_motion_timing=mark_motion_timing,
        enabled=bool(getattr(config_obj, "EXECUTOR_ORDERED_START_MATCH_ENABLED", True)),
        tolerance_rad=max(
            0.0,
            float(getattr(config_obj, "EXECUTOR_ORDERED_START_MATCH_TOL_RAD", 0.02)),
        ),
        timeout_s=max(
            0.0,
            float(getattr(config_obj, "EXECUTOR_ORDERED_START_MATCH_TIMEOUT_S", 0.35)),
        ),
    )
    controller_hooks = OrderedControllerExecutionHooks(
        node=node,
        logger=logger,
        mark_motion_timing=mark_motion_timing,
        wait_point_match=lambda label, trajectory, point, phase: wait_ordered_trajectory_point_match(
            hooks=state_match_hooks,
            label=label,
            joint_trajectory=trajectory,
            point=point,
            phase=phase,
        ),
        send_trajectory=send_trajectory,
        wait_execution_complete=wait_execution_complete,
    )
    unwind_finalization_hooks = OrderedUnwindFinalizationHooks(
        node=node,
        verify_explicit_unwind_complete=verify_explicit_unwind_complete,
    )
    return OrderedExecutionHookBundle(
        segment_hooks=segment_hooks,
        state_match_hooks=state_match_hooks,
        controller_hooks=controller_hooks,
        unwind_finalization_hooks=unwind_finalization_hooks,
    )


def build_ordered_planned_segment_executor(
    *,
    execution_hooks: OrderedExecutionHookBundle,
    scheduler_bridge: Any,
    config: OrderedPlannedSegmentExecutorConfig,
    runtime_unwind: Callable[..., Any],
    now_s: Callable[[], float] = time.time,
) -> Callable[..., Any]:
    """Return a callback that executes one planned ordered-chain segment."""

    def _execute_planned_segment(
        index,
        total,
        planned,
        preplanned_ready_count=0,
    ):
        execution_started = perf_counter()
        return execute_ordered_planned_segment(
            segment_hooks=execution_hooks.segment_hooks,
            controller_hooks=execution_hooks.controller_hooks,
            unwind_finalization_hooks=execution_hooks.unwind_finalization_hooks,
            index=index,
            total=total,
            planned_segment=planned,
            preplanned_ready_count=preplanned_ready_count,
            execution_started_s=execution_started,
            mark_scheduler_executing=lambda: scheduler_bridge.mark_executing(
                index - 1,
                planned,
            ),
            mark_scheduler_finished=lambda result: scheduler_bridge.mark_finished(
                index - 1,
                planned,
                result,
            ),
            min_timeout_s=config.min_timeout_s,
            timeout_multiplier=config.timeout_multiplier,
            motion_error_result=config.motion_error_result,
            default_velocity_percent=config.default_velocity_percent,
            default_acceleration_percent=config.default_acceleration_percent,
            runtime_unwind=runtime_unwind,
            now_s=now_s,
            verification_failure_result=config.verification_failure_result,
        )

    return _execute_planned_segment


def _duration_to_seconds(duration_msg: Any) -> float:
    return float(duration_msg.sec) + float(duration_msg.nanosec) / 1e9


def _set_duration_from_seconds(duration_msg: Any, seconds: float) -> None:
    total_nanosec = int(round(max(0.0, float(seconds)) * 1e9))
    duration_msg.sec = total_nanosec // 1_000_000_000
    duration_msg.nanosec = total_nanosec % 1_000_000_000


def _scale_optional_sequence(values: Any, scale: float) -> Any:
    if not values:
        return values
    return [float(value) * float(scale) for value in values]


def _configured_joint_rate_limits() -> dict[str, float]:
    try:
        import config
    except Exception:
        return {}

    raw_limits = getattr(config, "ORDERED_BLEND_JOINT_RATE_LIMITS_RAD_S", {}) or {}
    if not isinstance(raw_limits, dict):
        return {}

    limits = {}
    for raw_name, raw_limit in raw_limits.items():
        name = str(raw_name or "").strip()
        if not name:
            continue
        try:
            limit = float(raw_limit)
        except (TypeError, ValueError):
            continue
        if limit > 0.0 and math.isfinite(limit):
            limits[name] = limit
    return limits


def _ordered_blend_rate_guard_enabled() -> bool:
    try:
        import config
    except Exception:
        return True
    return bool(getattr(config, "ORDERED_BLEND_JOINT_RATE_GUARD_ENABLED", True))


def _maybe_stretch_ordered_blend_joint_rates(
    joint_trajectory: Any, logger: Any, limits_override: dict[str, float] | None = None
) -> None:
    """Stretch a timed blended trajectory if configured joint interval rates are too high."""

    if not _ordered_blend_rate_guard_enabled():
        return

    points = list(getattr(joint_trajectory, "points", []) or [])
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    if len(points) < 2 or not joint_names:
        return

    limits = dict(limits_override or _configured_joint_rate_limits())
    peak_rate = 0.0
    peak_joint = ""
    peak_segment = 0
    peak_dt = 0.0
    peak_delta = 0.0

    required_scale = 1.0
    limited_joint = ""
    limited_rate = 0.0
    limited_limit = 0.0
    limited_segment = 0

    for segment_index, (previous, current) in enumerate(zip(points, points[1:]), start=1):
        prev_t = _duration_to_seconds(previous.time_from_start)
        current_t = _duration_to_seconds(current.time_from_start)
        dt = current_t - prev_t
        if dt <= 1e-9:
            continue

        previous_positions = list(getattr(previous, "positions", []) or [])
        current_positions = list(getattr(current, "positions", []) or [])
        if len(previous_positions) != len(joint_names) or len(current_positions) != len(joint_names):
            continue

        for joint_index, joint_name in enumerate(joint_names):
            delta = abs(float(current_positions[joint_index]) - float(previous_positions[joint_index]))
            rate = delta / dt
            if rate > peak_rate:
                peak_rate = rate
                peak_joint = str(joint_name)
                peak_segment = segment_index
                peak_dt = dt
                peak_delta = delta

            limit = limits.get(str(joint_name))
            if limit is None:
                continue
            scale = rate / limit
            if scale > required_scale:
                required_scale = scale
                limited_joint = str(joint_name)
                limited_rate = rate
                limited_limit = limit
                limited_segment = segment_index

    if peak_joint:
        logger.info(
            f"[OrderedBlend] Timed trajectory peak interval rate "
            f"{peak_joint}={peak_rate:.3f}rad/s "
            f"segment={peak_segment} dt={peak_dt:.3f}s delta={peak_delta:.4f}rad"
        )

    if required_scale <= 1.001:
        return

    for point in points:
        t = _duration_to_seconds(point.time_from_start)
        _set_duration_from_seconds(point.time_from_start, t * required_scale)
        point.velocities = _scale_optional_sequence(point.velocities, 1.0 / required_scale)
        point.accelerations = _scale_optional_sequence(
            point.accelerations,
            1.0 / (required_scale * required_scale),
        )

    logger.warning(
        f"[OrderedBlend] Stretched blended trajectory timing by {required_scale:.2f}x "
        f"to respect {limited_joint} rate limit "
        f"(peak {limited_rate:.3f}rad/s > {limited_limit:.3f}rad/s "
        f"at segment={limited_segment}); "
        f"new_duration={_duration_to_seconds(points[-1].time_from_start):.3f}s"
    )


def ordered_trajectory_point_match_error(
    *,
    live_state: Any,
    joint_trajectory: Any,
    point: Any,
) -> tuple[tuple[float, str, float, float] | None, str | None]:
    """Return worst joint error against a trajectory point, or a reason."""

    if live_state is None:
        return None, "no live joint state"
    state_names = list(getattr(live_state, "name", []) or [])
    state_positions = list(getattr(live_state, "position", []) or [])
    if not state_names or len(state_names) != len(state_positions):
        return None, "invalid live joint state"

    live_by_name = {name: float(value) for name, value in zip(state_names, state_positions)}
    joint_names = list(getattr(joint_trajectory, "joint_names", []) or [])
    planned_positions = list(getattr(point, "positions", []) or [])
    if not joint_names or len(planned_positions) < len(joint_names):
        return None, "invalid planned trajectory point"

    worst = None
    for joint_name, planned_value in zip(joint_names, planned_positions):
        if joint_name not in live_by_name:
            return None, f"live joint state missing {joint_name}"
        actual_value = live_by_name[joint_name]
        error = abs(actual_value - float(planned_value))
        if worst is None or error > worst[0]:
            worst = (error, joint_name, actual_value, float(planned_value))
    if worst is None:
        return None, "no joints to compare"
    return worst, None


def wait_ordered_trajectory_point_match(
    *,
    hooks: OrderedStateMatchHooks,
    label: str,
    joint_trajectory: Any,
    point: Any,
    phase: str,
) -> bool:
    """Wait for live state to match an ordered-chain trajectory point."""

    if not hooks.enabled:
        hooks.logger.info(
            f"[OrderedChain] {phase} state match check disabled for '{label}'"
        )
        hooks.mark_motion_timing(
            hooks.node,
            "ordered_state_match_skipped",
            label=label,
            phase=phase,
        )
        return True

    tolerance_rad = max(0.0, float(hooks.tolerance_rad))
    timeout_s = max(0.0, float(hooks.timeout_s))
    hooks.mark_motion_timing(
        hooks.node,
        "ordered_state_match_start",
        label=label,
        phase=phase,
        tolerance_rad=tolerance_rad,
        timeout_s=timeout_s,
    )
    hooks.logger.info(
        f"[OrderedChain] Waiting for {phase} state match for '{label}' "
        f"tolerance={tolerance_rad:.4f}rad timeout_s={timeout_s:.3f}"
    )
    started = perf_counter()
    last_worst = None
    last_reason = None
    while True:
        worst, reason = ordered_trajectory_point_match_error(
            live_state=hooks.get_live_joint_state(),
            joint_trajectory=joint_trajectory,
            point=point,
        )
        if worst is not None:
            last_worst = worst
            if worst[0] <= tolerance_rad:
                match_elapsed = perf_counter() - started
                hooks.logger.info(
                    f"[OrderedChain] {phase} state matched for '{label}': "
                    f"max_error={worst[0]:.4f}rad joint={worst[1]} "
                    f"elapsed_s={match_elapsed:.3f}"
                )
                hooks.mark_motion_timing(
                    hooks.node,
                    "ordered_state_match_done",
                    label=label,
                    phase=phase,
                    matched=True,
                    duration_s=match_elapsed,
                    max_error_rad=worst[0],
                    joint=worst[1],
                )
                return True
        else:
            last_reason = reason

        if perf_counter() - started >= timeout_s:
            match_elapsed = perf_counter() - started
            if last_worst is not None:
                error, joint_name, actual_value, planned_value = last_worst
                hooks.logger.error(
                    f"[OrderedChain] {phase} state mismatch for '{label}': "
                    f"max_error={error:.4f}rad tolerance={tolerance_rad:.4f}rad "
                    f"joint={joint_name} actual={actual_value:.6f} planned={planned_value:.6f} "
                    f"timeout_s={timeout_s:.3f}"
                )
                hooks.mark_motion_timing(
                    hooks.node,
                    "ordered_state_match_done",
                    label=label,
                    phase=phase,
                    matched=False,
                    duration_s=match_elapsed,
                    max_error_rad=error,
                    joint=joint_name,
                )
            else:
                hooks.logger.error(
                    f"[OrderedChain] {phase} state mismatch for '{label}': "
                    f"{last_reason or 'unknown'} timeout_s={timeout_s:.3f}"
                )
                hooks.mark_motion_timing(
                    hooks.node,
                    "ordered_state_match_done",
                    label=label,
                    phase=phase,
                    matched=False,
                    duration_s=match_elapsed,
                    reason=last_reason or "unknown",
                )
            return False
        time.sleep(0.01)


def execute_ordered_planned_sequence(
    *,
    hooks: OrderedPlannedSequenceHooks,
    segments: list[dict[str, Any]],
    scheduler_bridge: Any,
    planner_future: Any,
    stop_planning: Any,
    plan_timeout_s: float,
    previous_execution_suppress: bool,
    stop_requested_attr: str = "_ordered_motion_chain_stop_requested",
    suppress_post_success_attr: str = "_suppress_post_success_unwind",
    stopped_result: int = -14,
) -> int:
    """Consume planned ordered-chain segments and execute them in order.

    Contiguous segments with the same non-empty ``readiness_group`` are released
    only after every member of that group has been planned.  Planning still runs
    concurrently with preceding motion, so the wait occurs before the first
    group member rather than between members.
    """

    try:
        segments_count = len(segments)
        prefetched: dict[int, dict[str, Any]] = {}

        def _wait_for_index(index: int) -> dict[str, Any]:
            cached = prefetched.get(index)
            if cached is not None:
                return cached
            wait_started = perf_counter()
            hooks.mark_motion_timing(
                hooks.node,
                "ordered_plan_wait_start",
                index=index + 1,
                timeout_s=plan_timeout_s,
            )
            planned_segment = scheduler_bridge.wait_for_planned(
                index,
                timeout=plan_timeout_s,
            )
            prefetched[index] = planned_segment
            hooks.logger.info(
                f"[TIMING] ordered_motion_chain_plan_ready index={index + 1} "
                f"wait_before_execute_s={perf_counter() - wait_started:.3f}"
            )
            hooks.mark_motion_timing(
                hooks.node,
                "ordered_plan_ready",
                index=index + 1,
                label=planned_segment.get("label"),
                wait_before_execute_s=perf_counter() - wait_started,
                plan_s=float(planned_segment.get("plan_elapsed_s", 0.0) or 0.0),
            )
            return planned_segment

        for expected_index in range(segments_count):
            wait_started = perf_counter()
            planned = _wait_for_index(expected_index)
            readiness_group = str(segments[expected_index].get("readiness_group") or "").strip()
            if readiness_group:
                group_end = expected_index
                while (
                    group_end + 1 < segments_count
                    and str(segments[group_end + 1].get("readiness_group") or "").strip()
                    == readiness_group
                ):
                    group_end += 1
                for required_index in range(expected_index + 1, group_end + 1):
                    _wait_for_index(required_index)
                hooks.mark_motion_timing(
                    hooks.node,
                    "ordered_readiness_group_ready",
                    group=readiness_group,
                    first_index=expected_index + 1,
                    last_index=group_end + 1,
                    wait_before_execute_s=perf_counter() - wait_started,
                )
            preplanned_snapshot = scheduler_bridge.consume_planned(
                expected_index,
                current_index=expected_index + 1,
            )
            prefetched.pop(expected_index, None)
            hooks.set_ordered_motion_chain_status(**preplanned_snapshot)

            if bool(getattr(hooks.node, stop_requested_attr, False)):
                stop_planning.set()
                hooks.set_ordered_motion_chain_status(
                    **hooks.ordered_chain_stopped_status(
                        index=expected_index + 1,
                        planned_segment=planned,
                        result=stopped_result,
                    )
                )
                return stopped_result

            setattr(
                hooks.node,
                suppress_post_success_attr,
                expected_index + 1 < int(segments_count),
            )
            result = hooks.execute_planned_segment(
                expected_index + 1,
                int(segments_count),
                planned,
                preplanned_ready_count=preplanned_snapshot["preplanned_ready_count"],
            )
            if result != 0:
                stop_planning.set()
                return int(result)

        planner_future.result(timeout=1.0)
        return 0
    finally:
        setattr(
            hooks.node,
            suppress_post_success_attr,
            previous_execution_suppress,
        )
        stop_planning.set()


def ordered_trajectory_timing(
    joint_trajectory: Any,
    *,
    min_timeout_s: float,
    timeout_multiplier: float,
    wait_padding_s: float = 2.0,
) -> OrderedTrajectoryTiming:
    """Return legacy ordered-chain controller timeout values for a trajectory."""

    duration = joint_trajectory.points[-1].time_from_start
    duration_s = _duration_to_seconds(duration)
    controller_goal_tolerance_s = max(
        float(min_timeout_s),
        duration_s * float(timeout_multiplier),
    )
    return OrderedTrajectoryTiming(
        duration_s=duration_s,
        controller_goal_tolerance_s=controller_goal_tolerance_s,
        wait_timeout_s=duration_s + controller_goal_tolerance_s + float(wait_padding_s),
    )


def execute_ordered_timed_trajectory(
    *,
    hooks: OrderedControllerExecutionHooks,
    index: int,
    total: int,
    planned_segment: dict[str, Any],
    segment_type: str,
    timing: OrderedTrajectoryTiming,
    execution_started_s: float,
    motion_error_result: int,
) -> Any:
    """Execute one ordered-chain timed trajectory using backend-injected effects."""

    trajectory = planned_segment["trajectory"]
    label = planned_segment["label"]
    hooks.logger.info(
        f"[OrderedChain] Sending planned segment {index}/{total} label='{label}' "
        f"type={segment_type} points={len(trajectory.points)} duration_s={timing.duration_s:.3f} "
        f"controller_goal_tolerance_s={timing.controller_goal_tolerance_s:.3f} "
        f"wait_timeout_s={timing.wait_timeout_s:.3f} "
        f"plan_s={planned_segment['plan_elapsed_s']:.3f}"
    )
    if not hooks.wait_point_match(label, trajectory, trajectory.points[0], "start"):
        return motion_error_result

    hooks.mark_motion_timing(
        hooks.node,
        "ordered_controller_handoff_start",
        index=index,
        label=planned_segment.get("label"),
        points=len(getattr(trajectory, "points", []) or []),
    )
    hooks.send_trajectory(hooks.node, trajectory)

    hooks.mark_motion_timing(
        hooks.node,
        "ordered_wait_execution_start",
        index=index,
        label=planned_segment.get("label"),
        timeout_s=timing.wait_timeout_s,
    )
    result = hooks.wait_execution_complete(hooks.node, timing.wait_timeout_s)
    hooks.mark_motion_timing(
        hooks.node,
        "ordered_wait_execution_done",
        index=index,
        label=planned_segment.get("label"),
        result=int(result) if isinstance(result, int) else result,
        duration_s=perf_counter() - execution_started_s,
    )
    if result == 0:
        hooks.logger.info(
            f"[OrderedChain] Controller completed segment {index}/{total} "
            f"label='{label}', verifying live end state before next segment"
        )
        if not hooks.wait_point_match(label, trajectory, trajectory.points[-1], "end"):
            result = motion_error_result
    return result


def execute_ordered_unwind_trajectories(
    *,
    hooks: OrderedControllerExecutionHooks,
    planned_segment: dict[str, Any],
    min_timeout_s: float,
    timeout_multiplier: float,
) -> Any:
    """Execute the planned trajectory list for an ordered unwind segment."""

    result = 0
    trajectories = list(planned_segment["trajectories"])
    trajectory_checks = list(planned_segment.get("trajectory_checks") or [])
    for unwind_index, joint_trajectory in enumerate(trajectories, start=1):
        import config
        from motion.execution.unwind_dynamics_guard import enforce_unwind_joint_dynamics
        check = (
            trajectory_checks[unwind_index - 1]
            if unwind_index - 1 < len(trajectory_checks)
            else planned_segment.get("check") or {}
        )
        enforce_unwind_joint_dynamics(
            joint_trajectory,
            hooks.logger,
            joint_name=str(
                check.get("joint_name")
                or getattr(config, "EXECUTOR_POST_UNWIND_JOINT_NAME", "Joint_6")
            ),
            velocity_limit_rad_s=float(
                getattr(config, "EXECUTOR_POST_UNWIND_JOINT_RATE_LIMIT_RAD_S", 1.2)
            ),
            acceleration_limit_rad_s2=float(
                getattr(config, "EXECUTOR_POST_UNWIND_JOINT_ACCEL_LIMIT_RAD_S2", 2.5)
            ),
        )
        timing = ordered_trajectory_timing(
            joint_trajectory,
            min_timeout_s=min_timeout_s,
            timeout_multiplier=timeout_multiplier,
        )
        hooks.logger.info(
            f"[OrderedChain] Sending planned unwind {unwind_index}/{len(trajectories)} "
            f"points={len(joint_trajectory.points)} duration_s={timing.duration_s:.3f} "
            f"controller_goal_tolerance_s={timing.controller_goal_tolerance_s:.3f} "
            f"wait_timeout_s={timing.wait_timeout_s:.3f}"
        )
        hooks.send_trajectory(
            hooks.node,
            joint_trajectory,
            preserve_explicit_wrap=True,
            unwind_check=check,
        )
        result = hooks.wait_execution_complete(hooks.node, timing.wait_timeout_s)
        if result != 0:
            break
    return result


def finalize_ordered_unwind_result(
    *,
    hooks: OrderedUnwindFinalizationHooks,
    planned_segment: dict[str, Any],
    result: Any,
    now_s: float,
    verification_failure_result: int = -6,
) -> Any:
    """Apply legacy explicit-unwind verification and failure bookkeeping."""

    final_result = result
    if final_result == 0 and planned_segment.get("check") is not None:
        final_result = (
            0
            if hooks.verify_explicit_unwind_complete(planned_segment["check"])
            else verification_failure_result
        )
    if final_result != 0:
        setattr(hooks.node, "_last_ordered_unwind_failure_time", now_s)
        setattr(hooks.node, "_last_ordered_unwind_failure_result", int(final_result))
    return final_result


def execute_ordered_planned_segment(
    *,
    segment_hooks: OrderedSegmentExecutionHooks,
    controller_hooks: OrderedControllerExecutionHooks,
    unwind_finalization_hooks: OrderedUnwindFinalizationHooks,
    index: int,
    total: int,
    planned_segment: dict[str, Any],
    preplanned_ready_count: int,
    execution_started_s: float,
    mark_scheduler_executing: Callable[[], dict[str, Any]],
    mark_scheduler_finished: Callable[[Any], dict[str, Any]],
    min_timeout_s: float,
    timeout_multiplier: float,
    motion_error_result: int,
    default_velocity_percent: float,
    default_acceleration_percent: float,
    runtime_unwind: Callable[..., Any],
    now_s: Callable[[], float],
    verification_failure_result: int = -6,
) -> Any:
    """Execute one legacy ordered-chain planned segment and publish finish state."""

    segment_type = planned_segment["type"]
    start_ordered_segment_execution(
        hooks=segment_hooks,
        index=index,
        total=total,
        planned_segment=planned_segment,
        preplanned_ready_count=preplanned_ready_count,
        mark_scheduler_executing=mark_scheduler_executing,
    )
    if segment_type in {"linear", "ptp", "path", "blended", "linked_lin", "concatenated"}:
        if bool(planned_segment.get("noop", False)):
            segment_hooks.logger.info(
                f"[OrderedChain] Skipping no-op "
                f"segment {index}/{total} "
                f"label='{planned_segment['label']}' "
                f"type={segment_type}"
            )
            result = 0
        else:
            profile_limits = planned_segment.get("_joint_rate_limits_rad_s")
            if segment_type == "blended" or profile_limits:
                _maybe_stretch_ordered_blend_joint_rates(
                    planned_segment["trajectory"],
                    controller_hooks.logger,
                    limits_override=profile_limits,
                )
            timing = ordered_trajectory_timing(
                planned_segment["trajectory"],
                min_timeout_s=min_timeout_s,
                timeout_multiplier=timeout_multiplier,
            )
            result = execute_ordered_timed_trajectory(
                hooks=controller_hooks,
                index=index,
                total=total,
                planned_segment=planned_segment,
                segment_type=segment_type,
                timing=timing,
                execution_started_s=execution_started_s,
                motion_error_result=motion_error_result,
            )
    elif segment_type in {"blend_consumed", "concatenate_consumed"}:
        segment_hooks.logger.info(
            f"[OrderedChain] Segment {index}/{total} "
            f"label='{planned_segment['label']}' was executed inside the "
            "previous grouped controller trajectory"
        )
        result = 0
    elif segment_type == "unwind_joint6":
        if bool(planned_segment.get("runtime_unwind", False)):
            segment_hooks.logger.info(
                f"[OrderedChain] Executing live final unwind label='{planned_segment['label']}' "
                f"plan_s={planned_segment['plan_elapsed_s']:.3f}"
            )
            result = runtime_unwind(
                vel=planned_segment.get("vel", default_velocity_percent),
                acc=planned_segment.get("acc", default_acceleration_percent),
                queue_if_busy=False,
            )
        else:
            result = 0
        if result == 0:
            result = execute_ordered_unwind_trajectories(
                hooks=controller_hooks,
                planned_segment=planned_segment,
                min_timeout_s=min_timeout_s,
                timeout_multiplier=timeout_multiplier,
            )
        result = finalize_ordered_unwind_result(
            hooks=unwind_finalization_hooks,
            planned_segment=planned_segment,
            result=result,
            now_s=now_s(),
            verification_failure_result=verification_failure_result,
        )
    else:
        result = -1

    return finish_ordered_segment_execution(
        hooks=segment_hooks,
        index=index,
        planned_segment=planned_segment,
        segment_type=segment_type,
        result=result,
        execution_started_s=execution_started_s,
        mark_scheduler_finished=lambda: mark_scheduler_finished(result),
    )


def start_ordered_segment_execution(
    *,
    hooks: OrderedSegmentExecutionHooks,
    index: int,
    total: int,
    planned_segment: dict[str, Any],
    preplanned_ready_count: int,
    mark_scheduler_executing: Callable[[], dict[str, Any]],
) -> None:
    """Publish legacy ordered-chain start status/timing for one segment."""

    hooks.publish_scheduler_updates(mark_scheduler_executing())
    hooks.mark_motion_timing(
        hooks.node,
        "ordered_segment_execute_start",
        index=index,
        label=planned_segment.get("label"),
        segment_type=planned_segment["type"],
        preplanned_ready_count=int(preplanned_ready_count),
    )
    hooks.set_ordered_motion_chain_status(
        **hooks.ordered_chain_executing_status(
            index=index,
            total=total,
            planned_segment=planned_segment,
            preplanned_ready_count=preplanned_ready_count,
        )
    )


def finish_ordered_segment_execution(
    *,
    hooks: OrderedSegmentExecutionHooks,
    index: int,
    planned_segment: dict[str, Any],
    segment_type: str,
    result: Any,
    execution_started_s: float,
    mark_scheduler_finished: Callable[[], dict[str, Any]],
) -> Any:
    """Publish legacy ordered-chain completion status/timing for one segment."""

    hooks.set_ordered_motion_chain_status(
        **hooks.ordered_chain_segment_finished_status(
            index=index,
            planned_segment=planned_segment,
            result=result,
        )
    )
    elapsed_s = perf_counter() - execution_started_s
    hooks.logger.info(
        f"[TIMING] ordered_motion_chain_segment index={index} "
        f"label='{planned_segment['label']}' "
        f"type={segment_type} result={result} elapsed_s={elapsed_s:.3f}"
    )
    hooks.mark_motion_timing(
        hooks.node,
        "ordered_segment_execute_done",
        index=index,
        label=planned_segment.get("label"),
        segment_type=segment_type,
        result=int(result) if isinstance(result, int) else result,
        duration_s=elapsed_s,
    )
    hooks.publish_scheduler_updates(mark_scheduler_finished())
    return result


__all__ = [
    "OrderedControllerExecutionHooks",
    "OrderedExecutionHookBundle",
    "OrderedPlannedSegmentExecutorConfig",
    "OrderedSegmentExecutionHooks",
    "OrderedTrajectoryTiming",
    "OrderedUnwindFinalizationHooks",
    "build_ordered_execution_hook_bundle",
    "build_ordered_planned_segment_executor",
    "execute_ordered_planned_segment",
    "execute_ordered_timed_trajectory",
    "execute_ordered_unwind_trajectories",
    "finalize_ordered_unwind_result",
    "finish_ordered_segment_execution",
    "ordered_trajectory_timing",
    "start_ordered_segment_execution",
]
