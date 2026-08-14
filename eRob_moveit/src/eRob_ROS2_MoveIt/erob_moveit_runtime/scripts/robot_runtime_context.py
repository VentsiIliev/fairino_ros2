#!/usr/bin/env python3
"""Immutable per-robot runtime configuration context.

This module is intentionally configuration-only.

It does not:
- create ROS nodes,
- create action clients,
- publish topics,
- send trajectories,
- modify global runtime configuration.

It provides a safe bridge between the multi-robot configuration API in
config.py and future robot-specific runtime components.
"""

from __future__ import annotations

from dataclasses import dataclass

import config


@dataclass(frozen=True, slots=True)
class RobotRuntimeContext:
    """Resolved immutable configuration for one robot."""

    name: str
    joint_names: tuple[str, ...]
    planning_group: str
    base_link: str
    ee_link: str
    wrist_link: str
    cartesian_source_link: str
    collision_tip_link: str
    action_follow_trajectory: str

    @classmethod
    def from_config(
        cls,
        robot_name: str | None = None,
    ) -> "RobotRuntimeContext":
        """Create a context from the configured ROBOTS mapping.

        If robot_name is omitted, config.PRIMARY_ROBOT is used.

        Raises:
            RuntimeError:
                If the active runtime configuration does not define ROBOTS or
                no primary robot can be resolved.

            KeyError:
                If robot_name does not exist.

            ValueError:
                If the resolved configuration contains an invalid value.
        """
        resolved_name = str(robot_name or "").strip()

        if not resolved_name:
            resolved_name = config.get_primary_robot_name() or ""

        if not resolved_name:
            raise RuntimeError(
                "Unable to resolve robot runtime context: "
                "robot_name was not provided and PRIMARY_ROBOT is not configured"
            )

        robot_config = config.get_robot_config(resolved_name)

        joint_names = tuple(
            str(joint_name).strip()
            for joint_name in robot_config["joint_names"]
        )

        if not joint_names:
            raise ValueError(
                f"Robot {resolved_name!r} has no configured joints"
            )

        if any(not joint_name for joint_name in joint_names):
            raise ValueError(
                f"Robot {resolved_name!r} contains an empty joint name"
            )

        if len(set(joint_names)) != len(joint_names):
            raise ValueError(
                f"Robot {resolved_name!r} contains duplicate joint names"
            )

        planning_group = _required_string(
            robot_config,
            "planning_group",
            resolved_name,
        )

        base_link = _required_string(
            robot_config,
            "base_link",
            resolved_name,
        )

        ee_link = _required_string(
            robot_config,
            "ee_link",
            resolved_name,
        )

        wrist_link = _required_string(
            robot_config,
            "wrist_link",
            resolved_name,
        )

        cartesian_source_link = _required_string(
            robot_config,
            "cartesian_source_link",
            resolved_name,
        )

        collision_tip_link = _required_string(
            robot_config,
            "collision_tip_link",
            resolved_name,
        )

        action_follow_trajectory = _required_string(
            robot_config,
            "action_follow_trajectory",
            resolved_name,
        )

        if not action_follow_trajectory.startswith("/"):
            raise ValueError(
                f"Robot {resolved_name!r} action_follow_trajectory "
                "must be an absolute ROS action name"
            )

        return cls(
            name=resolved_name,
            joint_names=joint_names,
            planning_group=planning_group,
            base_link=base_link,
            ee_link=ee_link,
            wrist_link=wrist_link,
            cartesian_source_link=cartesian_source_link,
            collision_tip_link=collision_tip_link,
            action_follow_trajectory=action_follow_trajectory,
        )


def _required_string(
    robot_config: dict,
    key: str,
    robot_name: str,
) -> str:
    """Return one required non-empty string configuration value."""
    value = robot_config.get(key)

    if not isinstance(value, str):
        raise ValueError(
            f"Robot {robot_name!r} config value {key!r} "
            "must be a string"
        )

    value = value.strip()

    if not value:
        raise ValueError(
            f"Robot {robot_name!r} config value {key!r} "
            "must not be empty"
        )

    return value