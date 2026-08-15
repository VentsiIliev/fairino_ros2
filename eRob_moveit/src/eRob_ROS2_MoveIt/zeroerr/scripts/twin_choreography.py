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


def _master_loop(
    x0: float,
    y0: float,
    z0: float,
    x_span: float,
    y_span: float,
    z_span: float,
) -> list[list[float]]:
    """Closed Robot-1 Cartesian path around the inward-facing start pose."""
    return [
        [x0, y0, z0],
        [x0 + x_span, y0, z0],
        [x0 + x_span, y0 + y_span, z0 + z_span],
        [x0, y0 + y_span, z0 + 2.0 * z_span],
        [x0 - x_span, y0 + y_span, z0 + z_span],
        [x0 - x_span, y0, z0],
        [x0 - x_span, y0 - y_span, z0 - z_span],
        [x0, y0 - y_span, z0 - 2.0 * z_span],
        [x0 + x_span, y0 - y_span, z0 - z_span],
        [x0 + x_span, y0, z0],
        [x0, y0, z0],
    ]


def _mirror_path_for_robot2(path: list[list[float]], mirror_y: float) -> list[list[float]]:
    """Mirror Robot-1 local path into Robot-2 local coordinates.

    The twin URDF places Robot 2 at the opposite side with a 180-degree yaw.
    A world-space mirror across the center plane therefore maps local coordinates as:
        x2 = x1
        y2 = 2*mirror_y - y1
        z2 = z1
    """
    return [[x, 2.0 * mirror_y - y, z] for x, y, z in path]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mirrored looping Cartesian choreography for TwinLocalRuntime"
    )
    parser.add_argument("--cycles", type=int, default=4, help="number of complete mirrored path loops")
    parser.add_argument("--start-x", type=float, default=300.0, help="inward-facing local X start in mm")
    parser.add_argument("--start-y", type=float, default=0.0, help="local Y center in mm")
    parser.add_argument("--start-z", type=float, default=300.0, help="start height in mm")
    parser.add_argument("--rx", type=float, default=180.0, help="constant tool RX in degrees")
    parser.add_argument("--ry", type=float, default=0.0, help="constant tool RY in degrees")
    parser.add_argument("--rz", type=float, default=0.0, help="constant tool RZ in degrees")
    parser.add_argument("--x-span", type=float, default=35.0, help="master path X excursion in mm")
    parser.add_argument("--y-span", type=float, default=45.0, help="master path Y excursion in mm")
    parser.add_argument("--z-span", type=float, default=20.0, help="master path Z excursion in mm")
    parser.add_argument("--vel", type=float, default=20.0, help="path velocity percentage")
    parser.add_argument("--acc", type=float, default=20.0, help="path acceleration percentage")
    parser.add_argument("--start-vel", type=float, default=15.0, help="PTP start velocity percentage")
    parser.add_argument("--start-acc", type=float, default=15.0, help="PTP start acceleration percentage")
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
    for name in ("vel", "acc", "start_vel", "start_acc"):
        value = getattr(args, name)
        if not 0.0 < value <= 100.0:
            raise ValueError(f"--{name.replace('_', '-')} must be in (0, 100]")
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

    start_pose = [
        args.start_x,
        args.start_y,
        args.start_z,
        args.rx,
        args.ry,
        args.rz,
    ]
    # Because Robot 2's base is rotated 180 degrees in the twin URDF, the same
    # local start pose places the two TCPs symmetrically and facing inward.
    robot1_start = list(start_pose)
    robot2_start = list(start_pose)

    robot1_path = _master_loop(
        args.start_x,
        args.start_y,
        args.start_z,
        args.x_span,
        args.y_span,
        args.z_span,
    )
    robot2_path = _mirror_path_for_robot2(robot1_path, args.start_y)

    print(
        "Twin mirrored choreography: "
        f"cycles={args.cycles} waypoints={len(robot1_path)} "
        f"start={start_pose} vel={args.vel:.1f}% acc={args.acc:.1f}%"
    )
    print(
        f"Environment: ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<unset>')} "
        f"fake={os.environ.get('ZEROERR_USE_FAKE_HARDWARE', '<unset>')}"
    )
    print("Robot 1 master path:")
    for index, point in enumerate(robot1_path):
        print(f"  {index:02d}: {point}")
    print("Robot 2 mirrored path:")
    for index, point in enumerate(robot2_path):
        print(f"  {index:02d}: {point}")

    with TwinLocalRuntime() as robots:
        if not robots.wait_until_ready(args.ready_timeout):
            print("FAIL: twin runtime did not become ready")
            print(robots.readiness())
            return 3

        robot1 = robots.robot1
        robot2 = robots.robot2
        print("READY:", robots.readiness())

        try:
            print("\nMoving both robots to symmetric inward-facing start pose...")
            r1, r2, separation = _run_pair(
                lambda: robot1.move_ptp(
                    robot1_start,
                    vel=args.start_vel,
                    acc=args.start_acc,
                    blocking=True,
                ),
                lambda: robot2.move_ptp(
                    robot2_start,
                    vel=args.start_vel,
                    acc=args.start_acc,
                    blocking=True,
                ),
            )
            print(
                f"start results: robot1={r1} robot2={r2} "
                f"dispatch_separation={separation * 1000.0:.2f}ms"
            )
            if r1 != 0 or r2 != 0:
                print("FAIL: could not reach symmetric choreography start pose")
                _stop_both(robots)
                return 4

            print("\nStarting mirrored path loop. Ctrl+C stops both robots.")
            for cycle in range(1, args.cycles + 1):
                print(f"\n===== MIRRORED LOOP {cycle}/{args.cycles} =====")
                r1, r2, separation = _run_pair(
                    lambda: robot1.execute_path(
                        robot1_path,
                        rx=args.rx,
                        ry=args.ry,
                        rz=args.rz,
                        vel=args.vel,
                        acc=args.acc,
                        blocking=True,
                        trajectory_optimizer="RUCKIG",
                        orientation_mode="constant",
                    ),
                    lambda: robot2.execute_path(
                        robot2_path,
                        rx=args.rx,
                        ry=args.ry,
                        rz=args.rz,
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
                    print("FAIL: mirrored choreography stopped because a path returned an error")
                    _stop_both(robots)
                    return 5
                if args.pause:
                    time.sleep(args.pause)

            print("\nPASS: mirrored twin choreography completed")
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
