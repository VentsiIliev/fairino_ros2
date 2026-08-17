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
    """Closed, more dynamic offset loop around the staged face-to-face pose."""
    return [
        (0.0, 0.0, 0.0),
        (+x_span, 0.0, +0.35 * z_span),
        (+0.55 * x_span, +y_span, +z_span),
        (-0.35 * x_span, +0.70 * y_span, +1.45 * z_span),
        (-x_span, 0.0, +0.45 * z_span),
        (-0.55 * x_span, -y_span, -0.55 * z_span),
        (+0.35 * x_span, -0.70 * y_span, -1.20 * z_span),
        (+x_span, 0.0, -0.35 * z_span),
        (+0.45 * x_span, +0.55 * y_span, +0.45 * z_span),
        (-0.45 * x_span, -0.55 * y_span, -0.45 * z_span),
        (0.0, 0.0, 0.0),
    ]


def _path_from_offsets(
    home_xyz: list[float],
    offsets: list[tuple[float, float, float]],
) -> list[list[float]]:
    """Apply identical local offsets around one robot's staged TCP.

    Robot 2 is already mounted opposite Robot 1 by the twin URDF. Identical
    local-frame geometry therefore becomes mirrored choreography in world space.
    """
    x0, y0, z0 = home_xyz
    return [[x0 + dx, y0 + dy, z0 + dz] for dx, dy, dz in offsets]


def _joint_positions(gateway) -> list[float]:
    state = gateway.node.current_joint_state
    if state is None:
        return []
    return [float(v) for v in (getattr(state, "position", []) or [])]


def _stage_xyz_candidates() -> list[tuple[float, float, float]]:
    """Common local TCP positions that place both arms toward the center.

    Both robots use the same local coordinates. Robot 2's 180-degree mounting
    transform supplies the world-space mirror.
    """
    return [
        (360.0, 0.0, 540.0),
        (320.0, 0.0, 560.0),
        (400.0, 0.0, 520.0),
        (300.0, 0.0, 600.0),
        (420.0, 0.0, 580.0),
        (340.0, 40.0, 560.0),
        (340.0, -40.0, 560.0),
        (280.0, 0.0, 520.0),
    ]


def _find_shared_stage_pose(robot1, robot2, pose1, pose2):
    """Find one common local XYZ candidate valid for both robot runtimes."""
    start1 = [float(v) for v in pose1[:6]]
    start2 = [float(v) for v in pose2[:6]]
    rpy1 = [float(v) for v in pose1[3:6]]
    rpy2 = [float(v) for v in pose2[3:6]]

    print("\nSearching for a shared center-facing staging pose...")
    for index, xyz in enumerate(_stage_xyz_candidates()):
        target1 = [float(xyz[0]), float(xyz[1]), float(xyz[2]), *rpy1]
        target2 = [float(xyz[0]), float(xyz[1]), float(xyz[2]), *rpy2]

        check1 = robot1.validate_pose(start1, target1)
        check2 = robot2.validate_pose(start2, target2)
        ok1 = bool(check1.get("reachable", False))
        ok2 = bool(check2.get("reachable", False))
        reason1 = check1.get("reason", "unknown")
        reason2 = check2.get("reason", "unknown")
        print(
            f"  candidate {index:02d} XYZ={list(xyz)} "
            f"robot1={ok1}({reason1}) robot2={ok2}({reason2})"
        )

        if ok1 and ok2:
            return target1, target2

    return None, None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Twin choreography: stage both robots face-to-face, then run mirrored local paths"
    )
    parser.add_argument("--cycles", type=int, default=4, help="number of complete path loops")
    parser.add_argument("--x-span", type=float, default=55.0, help="master path X excursion in mm")
    parser.add_argument("--y-span", type=float, default=70.0, help="master path Y excursion in mm")
    parser.add_argument("--z-span", type=float, default=35.0, help="master path Z excursion in mm")
    parser.add_argument("--vel", type=float, default=100.0, help="path velocity percentage")
    parser.add_argument("--acc", type=float, default=60.0, help="path acceleration percentage")
    parser.add_argument("--stage-vel", type=float, default=20.0, help="staging PTP velocity percentage")
    parser.add_argument("--stage-acc", type=float, default=20.0, help="staging PTP acceleration percentage")
    parser.add_argument("--pause", type=float, default=0.05, help="pause between full loops in seconds")
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
    for name in ("vel", "acc", "stage_vel", "stage_acc"):
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

    offsets = _master_offsets(args.x_span, args.y_span, args.z_span)

    print(
        "Twin face-to-face choreography: "
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
            initial_q1 = _joint_positions(robot1)
            initial_q2 = _joint_positions(robot2)
            initial_pose1 = robot1.get_current_position()
            initial_pose2 = robot2.get_current_position()

            if len(initial_q1) != 6 or len(initial_q2) != 6:
                print("FAIL: six-joint startup state unavailable")
                return 4
            if initial_pose1 is None or initial_pose2 is None:
                print("FAIL: startup Cartesian pose unavailable")
                return 4
            if len(initial_pose1) < 6 or len(initial_pose2) < 6:
                print("FAIL: startup Cartesian pose does not contain XYZ/RPY")
                return 4

            print("\nStartup robot1 joints:", initial_q1)
            print("Startup robot2 joints:", initial_q2)
            print("Startup robot1 TCP:", initial_pose1)
            print("Startup robot2 TCP:", initial_pose2)

            stage1, stage2 = _find_shared_stage_pose(
                robot1,
                robot2,
                initial_pose1,
                initial_pose2,
            )
            if stage1 is None or stage2 is None:
                print("FAIL: no shared center-facing staging pose was reachable by both robots")
                return 5

            print("\nSelected staging targets:")
            print("  robot1:", stage1)
            print("  robot2:", stage2)
            print("Moving both robots to face each other...")

            s1, s2, stage_separation = _run_pair(
                lambda: robot1.move_ptp(
                    stage1,
                    vel=args.stage_vel,
                    acc=args.stage_acc,
                    blocking=True,
                    trajectory_optimizer="RUCKIG",
                ),
                lambda: robot2.move_ptp(
                    stage2,
                    vel=args.stage_vel,
                    acc=args.stage_acc,
                    blocking=True,
                    trajectory_optimizer="RUCKIG",
                ),
            )
            print(
                f"staging results: robot1={s1} robot2={s2} "
                f"dispatch_separation={stage_separation * 1000.0:.2f}ms"
            )
            if s1 != 0 or s2 != 0:
                print("FAIL: staging motion failed")
                _stop_both(robots)
                return 6

            time.sleep(0.20)
            home_pose1 = robot1.get_current_position()
            home_pose2 = robot2.get_current_position()
            if home_pose1 is None or home_pose2 is None:
                print("FAIL: staged Cartesian pose unavailable")
                return 7

            print("\nFace-to-face staged TCPs:")
            print("  robot1:", home_pose1)
            print("  robot2:", home_pose2)

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

            print("\nDynamic identical-local choreography paths:")
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
                    return 8
                if args.pause:
                    time.sleep(args.pause)

            final_q1 = _joint_positions(robot1)
            final_q2 = _joint_positions(robot2)
            print("\nFinal robot1 joints:", final_q1)
            print("Final robot2 joints:", final_q2)
            print("PASS: face-to-face twin choreography completed")
            return 0
        except KeyboardInterrupt:
            print("\nInterrupted: stopping both robots")
            _stop_both(robots)
            return 130
        except Exception as exc:
            print(f"\nFAIL: {exc}")
            _stop_both(robots)
            return 9


if __name__ == "__main__":
    raise SystemExit(main())
