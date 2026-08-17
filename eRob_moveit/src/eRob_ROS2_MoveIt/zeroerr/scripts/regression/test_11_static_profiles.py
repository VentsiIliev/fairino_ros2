#!/usr/bin/env python3
"""Regression test 11: static profile validation (pure config, no runtime).

Loads each configured robot profile (paint, welding, twin_robots, ...) through
the same merge pipeline the runtime uses (``config._merge`` + profile
runtime.yaml) WITHOUT constructing a robot runtime. Validates the static
structure: required keys, joint config, planner/action/toolchain fields, and
that every profile resolves a robot identity (explicit ROBOTS or the default
unscoped ``robot`` entry inherited from the base config/runtime.yaml).

Runs FIRST in ``run_all.sh``: pure-config, so it works even if a runtime
cannot construct under a given profile.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import common
from common import (
    EXIT_FAIL,
    EXIT_PASS,
    build_parser,
    print_env,
)

_OPTIMIZERS = {"TOTG", "RUCKIG"}
_JOINT_REQUIRED_FIELDS = ("joint_names", "planning_group", "base_link", "ee_link",
                          "action_follow_trajectory")


def _profile_names() -> list[str]:
    base_path = Path(getattr(common.config, "_RUNTIME_CONFIG_PATH", "") or "")
    config_dir = base_path.parent if base_path else None
    if config_dir is None or not config_dir.exists():
        return []
    names = sorted(
        entry.name
        for entry in config_dir.iterdir()
        if entry.is_dir() and (entry / "runtime.yaml").exists()
    )
    return names


def _load_profile_static(profile: str):
    """Mirror ``config._load_runtime_config`` for one named profile.

    Returns ``(merged, explicit_robot_names, error)`` where
    ``explicit_robot_names`` is the set of robots the profile's OWN
    runtime.yaml declares under ``ROBOTS`` (empty if it inherits the default).
    """
    base = common.config._load_yaml_file(common.config._runtime_yaml_path())
    merged = dict(common.config.DEFAULTS)
    merged = common.config._merge(merged, base)
    profile_path = common.config._profile_runtime_yaml_path(profile)
    if not profile_path or not profile_path.exists():
        return None, set(), f"profile runtime.yaml not found for {profile!r}"
    profile_config = common.config._load_yaml_file(profile_path)
    explicit = profile_config.get("ROBOTS")
    explicit_names = set(str(name) for name in explicit) if isinstance(explicit, dict) else set()
    merged = common.config._merge(merged, profile_config)
    for filename in ("contour_ik_config.yaml", "ptp_config.yaml", "jacobian_config.yaml"):
        extra = common.config._profile_config_yaml_path(profile, filename)
        if extra and extra.exists():
            merged = common.config._merge(merged, common.config._load_yaml_file(extra))
    return merged, explicit_names, None


def _check_topology(profile: str, merged: dict, explicit_names: set[str]) -> int:
    resolved = set(str(name) for name in (merged.get("ROBOTS") or {}))
    if not resolved:
        print(f"FAIL: profile {profile} resolved to an empty robot topology")
        return EXIT_FAIL

    if explicit_names:
        if resolved != explicit_names:
            inherited = sorted(resolved - explicit_names)
            missing = sorted(explicit_names - resolved)
            detail = []
            if inherited:
                detail.append(f"inherited robots not in explicit ROBOTS: {inherited}")
            if missing:
                detail.append(f"explicit ROBOTS missing after merge: {missing}")
            print(
                f"FAIL: profile {profile} resolved robot topology "
                f"{sorted(resolved)} does not match its explicit ROBOTS "
                f"{sorted(explicit_names)} ({'; '.join(detail)})"
            )
            return EXIT_FAIL
        print(f"    topology: {sorted(resolved)} (profile-owned ROBOTS)")
        return EXIT_PASS

    # Profile inherits the default single-robot identity from the base config.
    base_robots = common.config._load_yaml_file(
        common.config._runtime_yaml_path()
    ).get("ROBOTS") or {}
    inherited_names = set(str(name) for name in base_robots)
    if resolved != inherited_names:
        print(
            f"FAIL: profile {profile} resolved robot topology "
            f"{sorted(resolved)} does not match the inherited default "
            f"{sorted(inherited_names)}"
        )
        return EXIT_FAIL
    print(f"    topology: {sorted(resolved)} (inherited default identity)")
    return EXIT_PASS


def _check_profile(profile: str, merged: dict) -> int:
    print(f"\n== profile: {profile}")
    print("    ROBOTS:", merged.get("ROBOTS"))
    print("    PRIMARY_ROBOT:", merged.get("PRIMARY_ROBOT"))
    print("    URDF_PATH:", merged.get("URDF_PATH"))
    print("    SRDF_PATH:", merged.get("SRDF_PATH"))
    print("    TRAJECTORY_OPTIMIZER:", merged.get("TRAJECTORY_OPTIMIZER"))

    missing = sorted(
        key
        for key in common.config.REQUIRED_KEYS
        if key not in merged or merged[key] in (None, "")
    )
    if missing:
        print(f"FAIL: profile {profile} missing required keys: {missing}")
        return EXIT_FAIL

    primary = merged.get("PRIMARY_ROBOT")
    robots = merged.get("ROBOTS")
    if not primary or not isinstance(primary, str):
        print(f"FAIL: profile {profile} PRIMARY_ROBOT is empty or not a string: {primary!r}")
        return EXIT_FAIL
    if not isinstance(robots, dict) or primary not in robots:
        print(
            f"FAIL: profile {profile} PRIMARY_ROBOT {primary!r} "
            f"is not a key in ROBOTS ({sorted(robots) if isinstance(robots, dict) else robots!r})"
        )
        return EXIT_FAIL

    try:
        common.config._validate_robot_configs(merged)
    except RuntimeError as exc:
        print(f"FAIL: profile {profile} robot config validation: {exc}")
        return EXIT_FAIL

    optimizer = str(merged.get("TRAJECTORY_OPTIMIZER", "")).strip().upper()
    if optimizer and optimizer not in _OPTIMIZERS:
        print(f"FAIL: profile {profile} invalid TRAJECTORY_OPTIMIZER {optimizer!r}")
        return EXIT_FAIL

    vel = merged.get("DEFAULT_VEL_PERCENT")
    acc = merged.get("DEFAULT_ACC_PERCENT")
    if not (isinstance(vel, (int, float)) and 0 <= float(vel) <= 100):
        print(f"FAIL: profile {profile} DEFAULT_VEL_PERCENT {vel!r}")
        return EXIT_FAIL
    if not (isinstance(acc, (int, float)) and 0 <= float(acc) <= 100):
        print(f"FAIL: profile {profile} DEFAULT_ACC_PERCENT {acc!r}")
        return EXIT_FAIL

    queue_size = merged.get("MOTION_QUEUE_MAX_SIZE")
    if not isinstance(queue_size, int) or queue_size <= 0:
        print(f"FAIL: profile {profile} MOTION_QUEUE_MAX_SIZE {queue_size!r}")
        return EXIT_FAIL

    if isinstance(robots, dict) and robots:
        for name, robot_config in robots.items():
            for field in _JOINT_REQUIRED_FIELDS:
                if not robot_config.get(field):
                    print(f"FAIL: profile {profile} ROBOTS[{name!r}] missing {field!r}")
                    return EXIT_FAIL
        print(f"    multi-robot: {sorted(robots)}")
    else:
        legacy_missing = [
            field for field in ("JOINT_NAMES", "PLANNING_GROUP", "BASE_LINK", "EE_LINK",
                                "ACTION_FOLLOW_TRAJECTORY", "URDF_PATH", "SRDF_PATH")
            if not merged.get(field)
        ]
        if legacy_missing:
            print(f"FAIL: profile {profile} legacy fields missing: {legacy_missing}")
            return EXIT_FAIL
        joint_names = merged.get("JOINT_NAMES") or []
        num_joints = merged.get("NUM_JOINTS")
        if len(joint_names) != num_joints:
            print(
                f"FAIL: profile {profile} JOINT_NAMES({len(joint_names)}) "
                f"!= NUM_JOINTS({num_joints})"
            )
            return EXIT_FAIL
        print(
            "    legacy single-robot structure OK (no explicit ROBOTS); the "
            "profile inherits the default unscoped 'robot' identity from the "
            "base config/runtime.yaml."
        )
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 11: static profile validation").parse_args()
    print_env()

    profiles = _profile_names()
    if not profiles:
        print("FAIL: no config/<profile>/runtime.yaml profiles discovered")
        return EXIT_FAIL
    print("discovered profiles:", profiles)

    active_profile = str(getattr(common.config, "ACTIVE_PROFILE", "") or "")
    active_robot_names = common.config.get_robot_names()
    primary = common.config.get_primary_robot_name()
    print("active profile:", active_profile)
    print("active get_robot_names():", active_robot_names)
    print("active get_primary_robot_name():", primary)

    failures = 0
    for profile in profiles:
        merged, explicit_names, error = _load_profile_static(profile)
        if error is not None:
            print(f"FAIL: profile {profile}: {error}")
            failures += 1
            continue
        if _check_profile(profile, merged) != EXIT_PASS:
            failures += 1
            continue
        if _check_topology(profile, merged, explicit_names) != EXIT_PASS:
            failures += 1

    if failures:
        print(f"FAIL: {failures} profile(s) failed static validation")
        return EXIT_FAIL

    print("PASS: all discovered profiles are statically consistent")
    return EXIT_PASS


if __name__ == "__main__":
    raise SystemExit(_main())
