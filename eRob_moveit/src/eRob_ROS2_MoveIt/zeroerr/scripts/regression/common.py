#!/usr/bin/env python3
"""Shared helpers for the single-robot regression suite.

The suite proves the existing single-robot profiles (paint, welding) keep
working after the prepared/synchronized-execution refactor on the
``twin_robots`` branch. Tests bring up one in-process RobotController +
LocalRuntimeGateway, mirroring ``runtime_gateway/twin_local.py`` but for a
single robot and WITHOUT the twin-only ``prepare_pair`` APIs.

Single-robot profiles inherit a default logical robot identity
(``PRIMARY_ROBOT``/``ROBOTS`` with the unscoped ``robot`` entry) from the base
``config/runtime.yaml``, so ``RobotRuntimeContext.from_config()`` constructs
cleanly. The suite detects whether the resolved identity is scoped (twin-style
``robot1_arm`` prefixes) or unscoped (legacy ``manipulator``) and wires topics
and the active-TCP frame accordingly.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from threading import Thread

from ament_index_python.packages import get_package_prefix

REGEX_DEFAULT_CONFIG_PACKAGE = "zeroerr"


def _add_runtime_to_path() -> None:
    runtime_dir = (
        Path(get_package_prefix("erob_moveit_runtime"))
        / "lib"
        / "erob_moveit_runtime"
    )
    sys.path.insert(0, str(runtime_dir))


_add_runtime_to_path()

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

import config  # noqa: E402
from backend.backend_factory import create_robot_backend  # noqa: E402
from robot_controller import RobotController  # noqa: E402
from robot_runtime_context import RobotRuntimeContext  # noqa: E402
from runtime_gateway.local import LocalRuntimeGateway  # noqa: E402
from status.robot_scoped_state_publisher import RobotScopedStatePublisher  # noqa: E402
from utils.work_object import WorkObject  # noqa: E402

# Exit codes
EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_SETUP = 2  # infrastructure / setup failure -> run_all.sh stops
EXIT_USAGE = 3
MOTION_ERROR_DESCRIPTIONS = {
    0: "success",
    -1: "Busy / invalid input / generic error",
    -2: "MoveIt service unavailable",
    -3: "Safety violation: target outside workspace",
    -4: "No current robot position available",
    -5: "Motion queue full",
    -6: "Path planning failed: MoveIt returned no trajectory",
    -7: "Time parameterization failed (TOTG/Ruckig)",
    -8: "Jacobian fallback path planning failed",
    -9: "Near-singularity detected",
    -10: "Collision detected during Jacobian check",
    -11: "Cartesian path planning failed: target unreachable, collision, or joint-limit constraint",
    -12: "Hardware not ready: EtherCAT slave not in OP",
    -13: "Drive operation is not enabled; call /drive/enable before motion",
    -14: "Controller execution failed: trajectory tolerance or controller action error",
    -15: "Prepared start mismatch: live joint state deviates beyond tolerance",
}


def describe_motion_result(result) -> str:
    code = int(result)
    description = MOTION_ERROR_DESCRIPTIONS.get(code, f"Unknown error code {code}")
    return f"result={code} ({description})"


def is_scoped_robot(context) -> bool:
    """True when a robot identity carries the twin-style name prefixes.

    Twin robots scope everything under their name (``robot1_arm``,
    ``robot1_base_link``, ``/robot1_...``). Legacy single-robot profiles use
    the default unscoped identity (``manipulator``, ``base_link``,
    ``/joint_states``) whose name is NOT a prefix of its own links/groups.
    """
    name = str(getattr(context, "name", "") or "")
    if not name:
        return False
    for value in (
        str(getattr(context, "planning_group", "") or ""),
        str(getattr(context, "base_link", "") or ""),
        str(getattr(context, "ee_link", "") or ""),
    ):
        if value.startswith(f"{name}_"):
            return True
    return False


def require_fake_hardware(args) -> int | None:
    """Return an exit code if the hardware safety guard blocks the run.

    Motion tests default to fake hardware. ``--allow-real-hardware`` opts into
    a real robot; the operator is then responsible for drives/interlocks.
    """
    fake = os.environ.get("ZEROERR_USE_FAKE_HARDWARE", "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    if fake:
        return None
    if getattr(args, "allow_real_hardware", False):
        print(
            "WARNING: running against REAL hardware. Verify drive-enable and "
            "motion interlocks are released before any move command.",
            flush=True,
        )
        return None
    print(
        "SETUP-FAIL: refusing to run motion tests without fake hardware. "
        "Set ZEROERR_USE_FAKE_HARDWARE=1 (e.g. launch the stack with "
        "./launch_zeroerr.sh --fake), or pass --allow-real-hardware to accept "
        "full responsibility for the real robot.",
        flush=True,
    )
    return EXIT_SETUP


class SingleRobotRuntime:
    """In-process single-robot runtime.

    Mirrors ``TwinLocalRuntime`` but composes exactly one RobotController and
    one LocalRuntimeGateway. Twin-only primitives (prepare_pair /
    execute_prepared_pair) are intentionally absent.
    """

    def __init__(
        self,
        robot_name: str | None = None,
        *,
        executor_threads: int = 4,
        state_publish_hz: float | None = None,
        node_suffix: str = "regression",
    ):
        names = config.get_robot_names()
        if robot_name is not None and robot_name not in names:
            raise ValueError(
                f"Unknown robot {robot_name!r}; available={names}"
            )
        resolved_name = robot_name or config.get_primary_robot_name()
        if not resolved_name:
            raise RuntimeError(
                "No robot selected and no PRIMARY_ROBOT configured; "
                f"get_robot_names()={names}"
            )

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self._executor = MultiThreadedExecutor(
            num_threads=max(2, int(executor_threads))
        )
        self._spin_thread = None
        self._started = False
        self._closed = False
        self._nodes = []

        self.robot_name = str(resolved_name)
        self.context = RobotRuntimeContext.from_config(self.robot_name)
        self.scoped = is_scoped_robot(self.context)
        self.topic_prefix = (
            f"/{self.robot_name}" if self.scoped else ""
        )
        self.active_tcp_frame = (
            f"{self.robot_name}_active_tcp" if self.scoped else None
        )

        wo_params = list(
            getattr(config, "DEFAULT_WORKOBJECT", [0, 0, 0, 0, 0, 0])
        )
        work_object = (
            WorkObject(*wo_params)
            if any(value != 0 for value in wo_params)
            else None
        )

        suffix = str(node_suffix).strip() or "regression"
        state_publisher = RobotScopedStatePublisher(
            self.context,
            topic_prefix=self.topic_prefix,
            publish_hz=state_publish_hz,
            node_name=f"{self.robot_name}_{suffix}_state_publisher",
        )

        controller = RobotController(
            robot_context=self.context,
            node_name=f"{self.robot_name}_{suffix}_runtime",
            state_topic_prefix=self.topic_prefix,
            active_tcp_frame=self.active_tcp_frame,
        )

        backend = create_robot_backend(
            node=controller,
            workobject=work_object,
            ip=getattr(config, "ROBOT_IP", "192.168.58.2"),
        )
        controller.robot = backend

        self.gateway = LocalRuntimeGateway(
            robot=backend,
            node=controller,
        )

        self._nodes.extend([state_publisher, controller])
        for node in self._nodes:
            self._executor.add_node(node)

    # --- lifecycle ---------------------------------------------------------

    def start(self):
        if self._closed:
            raise RuntimeError("SingleRobotRuntime is closed")
        if self._started:
            return self

        self._spin_thread = Thread(
            target=self._executor.spin,
            daemon=True,
            name="single_robot_runtime_executor",
        )
        self._spin_thread.start()
        self._started = True
        return self

    def wait_until_ready(self, timeout_s: float = 15.0) -> bool:
        self.start()
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            if self.gateway.is_motion_stack_ready():
                return True
            time.sleep(0.05)
        return bool(self.gateway.is_motion_stack_ready())

    def readiness(self) -> dict:
        return {
            self.robot_name: {
                "ready": self.gateway.is_motion_stack_ready(),
                "fault": self.gateway.get_motion_stack_fault_reason(),
            }
        }

    def close(self):
        if self._closed:
            return
        self._closed = True

        try:
            self._executor.shutdown(timeout_sec=2.0)
        except TypeError:
            self._executor.shutdown()
        except Exception:
            pass

        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

        for node in reversed(self._nodes):
            try:
                node.destroy_node()
            except Exception:
                pass

        self._nodes.clear()

        if self._owns_rclpy and rclpy.ok():
            rclpy.try_shutdown()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def run_with_runtime(args, body):
    """Execute ``body(runtime, args) -> int`` inside a SingleRobotRuntime."""
    with SingleRobotRuntime(args.robot) as runtime:
        return body(runtime, args)


# --- shared measurement helpers (mirror twin_test.py) ------------------------


def scoped_joint_positions(gateway) -> tuple[list[str], list[float]]:
    state = gateway.node.current_joint_state
    if state is None:
        return [], []
    return (
        list(getattr(state, "name", []) or []),
        [float(value) for value in (getattr(state, "position", []) or [])],
    )


def context_joint_positions(gateway) -> list[float]:
    """Return the robot's joint positions ordered by its runtime context.

    The raw ``/joint_states`` message may carry other robots' joints (twin /
    combined graph); this filters to ``robot_context.joint_names`` only, which
    matches the order used by ``PreparedTrajectory.start_positions``.
    """
    state = gateway.node.current_joint_state
    if state is None:
        return []
    names = list(getattr(state, "name", []) or [])
    positions = list(getattr(state, "position", []) or [])
    if len(names) != len(positions):
        return []
    by_name = dict(zip(names, positions))
    ordered = []
    for name in gateway.node.robot_context.joint_names:
        if name not in by_name:
            return []
        ordered.append(by_name[name])
    return ordered


def max_delta(before: list[float], after: list[float]) -> float:
    if len(before) != len(after) or not before:
        return float("inf")
    return max(abs(a - b) for a, b in zip(after, before))


def wait_for_idle(gateway, timeout_s: float, require_motion: bool = False) -> bool:
    """Wait until the gateway's controller is no longer executing.

    With ``require_motion``, return False if the node never left the executing
    state (guards against trivially-passing idle checks).
    """
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    saw_motion = False
    while time.monotonic() < deadline:
        executing = bool(getattr(gateway.node, "is_executing", False))
        saw_motion = saw_motion or executing
        if saw_motion and not executing:
            return True
        time.sleep(0.05)
    idle = not bool(getattr(gateway.node, "is_executing", False))
    if require_motion and not saw_motion:
        return False
    return idle


def settle(gateway, settle_s: float) -> None:
    """Small post-idle pause so a motion is fully committed to joint state."""
    time.sleep(max(0.0, float(settle_s)))


def current_pose(gateway) -> list[float] | None:
    position = gateway.get_current_position()
    if position is None:
        return None
    values = [float(value) for value in position]
    return values if len(values) == 6 else None


# --- orientation / angle helpers ------------------------------------------------


def wrapped_angle_delta_deg(a: float, b: float) -> float:
    """Shortest signed angular distance between two degree values."""
    return ((a - b + 180.0) % 360.0) - 180.0


def max_orientation_delta_deg(pose_a: list[float], pose_b: list[float]) -> float:
    """Max absolute angular delta across rx/ry/rz (indices 3..5, degrees)."""
    return max(
        abs(wrapped_angle_delta_deg(a, b))
        for a, b in zip(pose_a[3:6], pose_b[3:6])
    )


# --- CLI ----------------------------------------------------------------------


def build_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--robot",
        default=None,
        help="robot name for multi-robot profiles (default: PRIMARY_ROBOT)",
    )
    parser.add_argument(
        "--vel",
        type=float,
        default=None,
        help="velocity percent (0-100); defaults per profile",
    )
    parser.add_argument(
        "--acc",
        type=float,
        default=None,
        help="acceleration percent (0-100); defaults per profile",
    )
    parser.add_argument(
        "--ready-timeout",
        type=float,
        default=30.0,
        help="seconds to wait for motion-stack readiness",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.25,
        help="post-idle settle pause before joint-delta checks",
    )
    parser.add_argument(
        "--move-mm",
        type=float,
        default=20.0,
        help="default Cartesian move distance for tests that move",
    )
    parser.add_argument(
        "--move-deg",
        type=float,
        default=5.0,
        help="default orientation move for tests that rotate",
    )
    parser.add_argument(
        "--allow-real-hardware",
        action="store_true",
        help="run against real hardware (operator accepts responsibility)",
    )
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="continue a multi-step test after an assertion (best-effort mode)",
    )
    return parser


def velocity_acceleration(args) -> tuple[float, float]:
    """Return (vel, acc) percentages, normalized for fake hardware.

    With fake hardware the joint-controller step executes near-instantly, so
    scaled-down values keep the observations meaningful.
    """
    vel = float(args.vel if args.vel is not None else getattr(config, "DEFAULT_VEL_PERCENT", 30.0))
    acc = float(args.acc if args.acc is not None else getattr(config, "DEFAULT_ACC_PERCENT", 30.0))
    fake = os.environ.get("ZEROERR_USE_FAKE_HARDWARE", "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    if fake:
        vel = min(vel, 15.0)
        acc = min(acc, 15.0)
    return max(0.0, min(100.0, vel)), max(0.0, min(100.0, acc))


def print_env() -> None:
    print(
        f"REGRESSION-ENV: ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<unset>')} "
        f"ROBOT_BACKEND={getattr(config, 'ROBOT_BACKEND', '<unset>')} "
        f"ACTIVE_PROFILE={getattr(config, 'ACTIVE_PROFILE', '<unset>')} "
        f"fake={os.environ.get('ZEROERR_USE_FAKE_HARDWARE', '<unset>')}"
    )


def main() -> int:
    parser = build_parser("regression suite helper (standalone smoke check)")
    args = parser.parse_args()
    print_env()

    guard = require_fake_hardware(args)
    if guard is not None:
        return guard

    with SingleRobotRuntime(args.robot) as runtime:
        if not runtime.wait_until_ready(args.ready_timeout):
            print("FAIL: runtime did not become ready")
            print(runtime.readiness())
            return EXIT_FAIL
        names, positions = scoped_joint_positions(runtime.gateway)
        print(f"PASS: {runtime.robot_name} ready")
        print("joint_names:", names)
        print("joint_positions:", positions)
        snapshot = runtime.gateway.state_snapshot()
        print("cartesian_position:", snapshot.get("position"))
        print("motion_stack_fault:", snapshot.get("motion_stack_fault"))
    return EXIT_PASS


if __name__ == "__main__":
    raise SystemExit(main())
