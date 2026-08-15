#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from dataclasses import dataclass
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

from enums import Direction, RobotAxis  # noqa: E402
from runtime_gateway.twin_local import TwinLocalRuntime  # noqa: E402


@dataclass(frozen=True)
class JogMove:
    axis: RobotAxis
    direction: Direction
    step: float


@dataclass(frozen=True)
class ChoreoStep:
    name: str
    robot1: JogMove
    robot2: JogMove


def _opposite(direction: Direction) -> Direction:
    return Direction.MINUS if direction == Direction.PLUS else Direction.PLUS


def _build_choreography(x_mm: float, y_mm: float, z_mm: float) -> list[ChoreoStep]:
    """Build a symmetric Cartesian choreography that returns both robots to start."""
    xp = Direction.PLUS
    xm = Direction.MINUS
    yp = Direction.PLUS
    ym = Direction.MINUS
    zp = Direction.PLUS
    zm = Direction.MINUS

    return [
        ChoreoStep(
            "open X",
            JogMove(RobotAxis.X, xp, x_mm),
            JogMove(RobotAxis.X, xm, x_mm),
        ),
        ChoreoStep(
            "rise together",
            JogMove(RobotAxis.Z, zp, z_mm),
            JogMove(RobotAxis.Z, zp, z_mm),
        ),
        ChoreoStep(
            "sway apart Y",
            JogMove(RobotAxis.Y, yp, y_mm),
            JogMove(RobotAxis.Y, ym, y_mm),
        ),
        ChoreoStep(
            "cross X",
            JogMove(RobotAxis.X, xm, x_mm * 2.0),
            JogMove(RobotAxis.X, xp, x_mm * 2.0),
        ),
        ChoreoStep(
            "sway through Y",
            JogMove(RobotAxis.Y, ym, y_mm * 2.0),
            JogMove(RobotAxis.Y, yp, y_mm * 2.0),
        ),
        ChoreoStep(
            "lower together",
            JogMove(RobotAxis.Z, zm, z_mm * 2.0),
            JogMove(RobotAxis.Z, zm, z_mm * 2.0),
        ),
        ChoreoStep(
            "cross back X",
            JogMove(RobotAxis.X, xp, x_mm * 2.0),
            JogMove(RobotAxis.X, xm, x_mm * 2.0),
        ),
        ChoreoStep(
            "sway back Y",
            JogMove(RobotAxis.Y, yp, y_mm * 2.0),
            JogMove(RobotAxis.Y, ym, y_mm * 2.0),
        ),
        ChoreoStep(
            "rise through center",
            JogMove(RobotAxis.Z, zp, z_mm * 2.0),
            JogMove(RobotAxis.Z, zp, z_mm * 2.0),
        ),
        ChoreoStep(
            "close X",
            JogMove(RobotAxis.X, xm, x_mm),
            JogMove(RobotAxis.X, xp, x_mm),
        ),
        ChoreoStep(
            "center Y",
            JogMove(RobotAxis.Y, ym, y_mm),
            JogMove(RobotAxis.Y, yp, y_mm),
        ),
        ChoreoStep(
            "settle Z",
            JogMove(RobotAxis.Z, zm, z_mm),
            JogMove(RobotAxis.Z, zm, z_mm),
        ),
    ]


def _run_pair(robots, step: ChoreoStep, vel: float, acc: float) -> tuple[int, int, float]:
    start_gate = threading.Barrier(3)
    results: dict[str, int] = {}
    started: dict[str, float] = {}
    errors: dict[str, BaseException] = {}

    def worker(robot_name: str, move: JogMove) -> None:
        try:
            gateway = robots.robot(robot_name)
            start_gate.wait()
            started[robot_name] = time.perf_counter()
            results[robot_name] = int(
                gateway.jog(move.axis, move.direction, move.step, vel, acc)
            )
        except BaseException as exc:  # surface worker failures in the main thread
            errors[robot_name] = exc

    t1 = threading.Thread(target=worker, args=("robot1", step.robot1), daemon=False)
    t2 = threading.Thread(target=worker, args=("robot2", step.robot2), daemon=False)
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
            robots.robot(name).stop()
        except Exception:
            pass


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Concurrent two-robot Cartesian choreography using TwinLocalRuntime"
    )
    parser.add_argument("--cycles", type=int, default=2, help="number of complete closed-loop cycles")
    parser.add_argument("--x", type=float, default=15.0, help="base X displacement in mm")
    parser.add_argument("--y", type=float, default=12.0, help="base Y displacement in mm")
    parser.add_argument("--z", type=float, default=10.0, help="base Z displacement in mm")
    parser.add_argument("--vel", type=float, default=15.0, help="velocity percentage")
    parser.add_argument("--acc", type=float, default=15.0, help="acceleration percentage")
    parser.add_argument("--pause", type=float, default=0.15, help="pause between paired moves in seconds")
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
    for name in ("x", "y", "z"):
        if getattr(args, name) <= 0.0:
            raise ValueError(f"--{name} must be > 0")
    if not 0.0 < args.vel <= 100.0:
        raise ValueError("--vel must be in (0, 100]")
    if not 0.0 < args.acc <= 100.0:
        raise ValueError("--acc must be in (0, 100]")
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

    choreography = _build_choreography(args.x, args.y, args.z)
    total_steps = len(choreography) * args.cycles

    print(
        "Twin choreography: "
        f"cycles={args.cycles} paired_steps={total_steps} "
        f"X={args.x:.1f}mm Y={args.y:.1f}mm Z={args.z:.1f}mm "
        f"vel={args.vel:.1f}% acc={args.acc:.1f}%"
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

        print("READY:", robots.readiness())
        print("Starting choreography. Ctrl+C requests stop on both robots.\n")

        try:
            step_number = 0
            for cycle in range(1, args.cycles + 1):
                print(f"===== CYCLE {cycle}/{args.cycles} =====")
                for choreo in choreography:
                    step_number += 1
                    print(
                        f"[{step_number:02d}/{total_steps:02d}] {choreo.name}: "
                        f"R1 {choreo.robot1.axis.name}{'+' if choreo.robot1.direction == Direction.PLUS else '-'} "
                        f"{choreo.robot1.step:.1f}, "
                        f"R2 {choreo.robot2.axis.name}{'+' if choreo.robot2.direction == Direction.PLUS else '-'} "
                        f"{choreo.robot2.step:.1f}"
                    )
                    r1, r2, separation = _run_pair(robots, choreo, args.vel, args.acc)
                    print(
                        f"    results: robot1={r1} robot2={r2} "
                        f"dispatch_separation={separation * 1000.0:.2f}ms"
                    )
                    if r1 != 0 or r2 != 0:
                        print("FAIL: choreography stopped because a robot returned a motion error")
                        _stop_both(robots)
                        return 4
                    if args.pause:
                        time.sleep(args.pause)

            print("\nPASS: twin choreography completed")
            return 0
        except KeyboardInterrupt:
            print("\nInterrupted: stopping both robots")
            _stop_both(robots)
            return 130
        except Exception as exc:
            print(f"\nFAIL: {exc}")
            _stop_both(robots)
            return 5


if __name__ == "__main__":
    raise SystemExit(main())
