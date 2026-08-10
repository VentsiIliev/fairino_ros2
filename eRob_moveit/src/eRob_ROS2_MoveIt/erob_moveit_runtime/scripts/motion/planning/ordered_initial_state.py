#!/usr/bin/env python3
"""Initial state helpers for legacy ordered-chain planning."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from time import perf_counter
from typing import Any


@dataclass(frozen=True)
class OrderedInitialPlanningState:
    start_cartesian: list[float]
    start_state: Any
    selected_optimizer: str | None
    previous_execution_suppress: bool
    duration_s: float


def build_ordered_initial_planning_state(
    *,
    node: Any,
    planning_node: Any,
    config_obj: Any,
    trajectory_optimizer: str | None = None,
) -> OrderedInitialPlanningState:
    """Capture the ordered-chain starting Cartesian pose and clean joint state."""

    from moveit_msgs.msg import RobotState

    init_started = perf_counter()
    start_cartesian = list(planning_node.prev_cartesian[:6])
    clean_joint_state = deepcopy(planning_node.current_joint_state)
    clean_joint_state.header.stamp = planning_node.get_clock().now().to_msg()
    clean_joint_state.velocity = [0.0] * (
        len(clean_joint_state.name) or len(clean_joint_state.position)
    )
    clean_joint_state.effort = []

    start_state = RobotState()
    start_state.joint_state = clean_joint_state
    start_state.is_diff = False

    selected_optimizer = trajectory_optimizer or (
        str(getattr(config_obj, "PATH_TRAJECTORY_OPTIMIZER", "") or "").strip().upper()
        or None
    )
    previous_execution_suppress = bool(
        getattr(node, "_suppress_post_success_unwind", False)
    )
    return OrderedInitialPlanningState(
        start_cartesian=start_cartesian,
        start_state=start_state,
        selected_optimizer=selected_optimizer,
        previous_execution_suppress=previous_execution_suppress,
        duration_s=perf_counter() - init_started,
    )


__all__ = [
    "OrderedInitialPlanningState",
    "build_ordered_initial_planning_state",
]
