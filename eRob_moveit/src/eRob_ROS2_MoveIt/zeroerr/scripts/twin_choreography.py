#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def _add_runtime_to_path() -> None:
    runtime_dir = (
        Path(get_package_prefix("erob_moveit_runtime"))
        / "lib"
        / "erob_moveit_runtime"
    )
    sys.path.insert(0, str(runtime_dir))


_add_runtime_to_path()

from runtime_gateway.twin_local import TwinLocalRuntime  # noqa: E402


def _run_pair(call1, call2) -> tuple[int, int, float]:
    start_gate = threading.Barrier(3)
    results: dict[str, int] = {}
    started: dict[str, float] = {}
    errors: dict[str, BaseException] = {}

    def worker(name: str, call) -> None:
        try:
            start_gate.wait()
            started[name] = time.perf_counter()
            results[name] = int(call())
        except BaseException as exc:
            errors[name] = exc

    t1 = threading.Thread(target=worker, args=("robot1", call1), daemon=False)
    t2 = threading.Thread(target=worker, args=("robot2", call2), daemon=False)
    t1.start()
    t2.start()
    start_gate.wait()
    t1.join()
    t2.join()

    if errors:
        details = ", ".join(f"{name}: {exc}" for name, exc in errors.items())
        raise RuntimeError(f"concurrent choreography worker failed: {details}")

    separation = abs(started["robot1"] - started["robot2"])
    return results["robot1"], results["robot2"], separation


def _stop_both(robots) -> None:
    for name in ("robot1", "robot2"):
        try:
            robots.robot(name).stop_motion()
        except Exception:
            pass


def _master_offsets(
    x_span: float,
    y_span: float,
    z_span: float,
) -> list[tuple[float, float, float]]:
    """Closed offset loop relative to each robot's live HOME TCP."""
    return [
        (0.0, 0.0, 0.0),
        (+x_span, 0.0, 0.0),
        (+x_span, +y_span, +z_span),
        (0.0, +y_span, +2.0 * z_span),
        (-x_span, +y_span, +z_span),
        (-x_span, 0.0, 0.0),
        (-x_span, -y_span, -z_span),
        (0.0, -y_span, -2.0 * z_span),
        (+x_span, -y_span, -z_span),
        (+x_span, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    ]


def _path_from_offsets(
    home_xyz: list[float],
    offsets: list[tuple[float, float, float]],
) -> list[list[float]]:
    """Apply the same local-frame master offsets around one robot's HOME TCP.

    The twin URDF already mounts Robot 2 opposite Robot 1. The physical/world
    mirroring therefore comes from the robot base transforms; reflecting local
    Y here would mirror the path twice and can create a different reachability
    problem for Robot 2.
    """
    x0, y0, z0 = home_xyz
    return [[x0 + dx, y0 + dy, z0 + dz] for dx, dy, dz in offsets]


def _joint_positions(gateway) -> list[float]:
    state = gateway.node.current_joint_state
    if state is None:
        return []
    return [float(v) for v in (getattr(state, "position", []) or [])]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Twin looping Cartesian choreography using identical local paths"
    )
    parser.add_argument("--cycles", type=int, default=4, help="number of complete path loops")
    parser.add_argument("--x-span", type=float, default=35.0, help="master path X excursion in mm")
    parser.add_argument("--y-span", type=float, default=45.0, help="master path Y excursion in mm")
    parser.add_argument("--z-span", type=float, default=20.0, help="master path Z excursion in mm")
    parser.add_argument("--vel", type=float, default=20.0, help="path velocity percentage")
    parser.add_argument("--acc", type=float, default=20.0, help="path acceleration percentage")
    parser.add_argument("--pause", type=float, default=0.10, help="pause between full loops in seconds")
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument(
        "--allow-real-hardware",
        action="store_true",
        help="required when ZEROERR_USE_FAKE_HARDWARE is not true",
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if args.cycles < 1:
        raise ValueError("--cycles must be >= 1")
    for name in ("x_span", "y_span", "z_span"):
        if getattr(args, name) <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be > 0")
    for name in ("vel", "acc"):
        value = getattr(args, name)
        if not 0.0 < value <= 100.0:
            raise ValueError(f"--{name} must be in (0, 100]")
    if args.pause < 0.0:
        raise ValueError("--pause must be >= 0")

    fake = os.environ.get("ZEROERR_USE_FAKE_HARDWARE", "").strip().lower() in {
        "1", "true", "yes", "on"
    }
    if not fake and not args.allow_real_hardware:
        raise RuntimeError(
            "Refusing to execute choreography on non-fake hardware. "
            "Pass --allow-real-hardware only after verifying clearances and limits."
        )


def main() -> int:
    args = _parse_args()
    try:
        _validate_args(args)
    except (ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}")
        return 2

    offsets = _master_offsets(args.x_span, args.y_span, args.z_span)

    print(
        "Twin choreography from live HOME with identical local paths: "
        f"cycles={args.cycles} waypoints={len(offsets)} "
        f"Xspan={args.x_span:.1f}mm Yspan={args.y_span:.1f}mm "
        f"Zspan={args.z_span:.1f}mm vel={args.vel:.1f}% acc={args.acc:.1f}%"
    )
    print(
        f"Environment: ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<unset>')} "
        f"fake={os.environ.get('ZEROERR_USE_FAKE_HARDWARE', '<unset>')}"
    )

    with TwinLocalRuntime() as robots:
        if not robots.wait_until_ready(args.ready_timeout):
            print("FAIL: twin runtime did not become ready")
            print(robots.readiness())
            return 3

        robot1 = robots.robot1
        robot2 = robots.robot2
        print("READY:", robots.readiness())

        try:
            # The twin fake hardware starts both arms with the same six-joint
            # state. Robot 2's base is already rotated 180 degrees in the URDF,
            # so identical local Cartesian paths appear mirrored in world space.
            home_q1 = _joint_positions(robot1)
            home_q2 = _joint_positions(robot2)
            home_pose1 = robot1.get_current_position()
            home_pose2 = robot2.get_current_position()

            if len(home_q1) != 6 or len(home_q2) != 6:
                print("FAIL: six-joint HOME state unavailable")
                return 4
            if home_pose1 is None or home_pose2 is None:
                print("FAIL: HOME Cartesian pose unavailable")
                return 4
            if len(home_pose1) < 6 or len(home_pose2) < 6:
                print("FAIL: HOME Cartesian pose does not contain XYZ/RPY")
                return 4

            print("\nUsing live symmetric HOME; no guessed Cartesian PTP is required.")
            print("robot1 HOME joints:", home_q1)
            print("robot2 HOME joints:", home_q2)
            print("robot1 HOME TCP:", home_pose1)
            print("robot2 HOME TCP:", home_pose2)

            robot1_path = _path_from_offsets(
                [float(v) for v in home_pose1[:3]],
                offsets,
            )
            robot2_path = _path_from_offsets(
                [float(v) for v in home_pose2[:3]],
                offsets,
            )

            rx1, ry1, rz1 = [float(v) for v in home_pose1[3:6]]
            rx2, ry2, rz2 = [float(v) for v in home_pose2[3:6]]

            print("\nRobot 1 / Robot 2 identical local master paths:")
            for index, (p1, p2) in enumerate(zip(robot1_path, robot2_path)):
                print(f"  {index:02d}: R1={p1}  R2={p2}")

            print("\nStarting twin path loop. Ctrl+C stops both robots.")
            for cycle in range(1, args.cycles + 1):
                print(f"\n===== TWIN LOOP {cycle}/{args.cycles} =====")
                r1, r2, separation = _run_pair(
                    lambda: robot1.execute_path(
                        robot1_path,
                        rx=rx1,
                        ry=ry1,
                        rz=rz1,
                        vel=args.vel,
                        acc=args.acc,
                        blocking=True,
                        trajectory_optimizer="RUCKIG",
                        orientation_mode="constant",
                    ),
                    lambda: robot2.execute_path(
                        robot2_path,
                        rx=rx2,
                        ry=ry2,
                        rz=rz2,
                        vel=args.vel,
                        acc=args.acc,
                        blocking=True,
                        trajectory_optimizer="RUCKIG",
                        orientation_mode="constant",
                    ),
                )
                print(
                    f"loop results: robot1={r1} robot2={r2} "
                    f"dispatch_separation={separation * 1000.0:.2f}ms"
                )
                if r1 != 0 or r2 != 0:
                    print("FAIL: twin choreography stopped because a path returned an error")
                    _stop_both(robots)
                    return 5
                if args.pause:
                    time.sleep(args.pause)

            final_q1 = _joint_positions(robot1)
            final_q2 = _joint_positions(robot2)
            print("\nFinal robot1 joints:", final_q1)
            print("Final robot2 joints:", final_q2)
            print("PASS: twin choreography completed")
            return 0
        except KeyboardInterrupt:
            print("\nInterrupted: stopping both robots")
            _stop_both(robots)
            return 130
        except Exception as exc:
            print(f"\nFAIL: {exc}")
            _stop_both(robots)
            return 6


if __name__ == "__main__":
    raise SystemExit(main())
