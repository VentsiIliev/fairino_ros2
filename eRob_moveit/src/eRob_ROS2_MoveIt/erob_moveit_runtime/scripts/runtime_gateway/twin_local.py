#!/usr/bin/env python3
"""Twin-robot local runtime composition.

Creates two normal RobotController instances in one Python process. Each
controller owns one RobotRuntimeContext and one LocalRuntimeGateway. The generic
motion stack remains single-robot: robot selection is completed before commands
enter planners, validation, optimization, or execution.

Synchronized execution uses the plan-now / execute-later split:
``prepare_pair`` plans both robots' paths concurrently through
``prepare_path`` (no controller contact), then ``execute_prepared_pair`` submits
both captured trajectories with a single shared future ``header.stamp`` so both
controllers start at the same instant on the shared clock.
"""

from __future__ import annotations

from threading import Thread
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

import config
from backend.backend_factory import create_robot_backend
from robot_controller import RobotController
from robot_runtime_context import RobotRuntimeContext
from runtime_gateway.local import LocalRuntimeGateway
from status.robot_scoped_state_publisher import RobotScopedStatePublisher
from utils.work_object import WorkObject


class TwinLocalRuntime:
    """Own two robot runtimes and expose their existing local gateways."""

    def __init__(
        self,
        robot_names=("robot1", "robot2"),
        *,
        executor_threads: int = 4,
        state_publish_hz: float | None = None,
    ):
        names = tuple(str(name).strip() for name in robot_names if str(name).strip())
        if len(names) != 2 or len(set(names)) != 2:
            raise ValueError("TwinLocalRuntime requires exactly two distinct robot names")

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
        self._state_publishers = []
        self._controllers = []
        self._gateways = {}

        wo_params = list(
            getattr(config, "DEFAULT_WORKOBJECT", [0, 0, 0, 0, 0, 0])
        )

        for robot_name in names:
            context = RobotRuntimeContext.from_config(robot_name)
            topic_prefix = f"/{context.name}"
            work_object = (
                WorkObject(*wo_params)
                if any(value != 0 for value in wo_params)
                else None
            )

            state_publisher = RobotScopedStatePublisher(
                context,
                topic_prefix=topic_prefix,
                publish_hz=state_publish_hz,
                node_name=f"{context.name}_state_publisher",
            )

            controller = RobotController(
                robot_context=context,
                node_name=f"{context.name}_runtime",
                state_topic_prefix=topic_prefix,
                active_tcp_frame=f"{context.name}_active_tcp",
            )

            backend = create_robot_backend(
                node=controller,
                workobject=work_object,
                ip=getattr(config, "ROBOT_IP", "192.168.58.2"),
            )
            controller.robot = backend

            gateway = LocalRuntimeGateway(
                robot=backend,
                node=controller,
            )

            self._state_publishers.append(state_publisher)
            self._controllers.append(controller)
            self._gateways[context.name] = gateway
            self._nodes.extend([state_publisher, controller])

        for node in self._nodes:
            self._executor.add_node(node)

        self.robot1 = self._gateways.get("robot1")
        self.robot2 = self._gateways.get("robot2")

    @property
    def gateways(self):
        return dict(self._gateways)

    def robot(self, robot_name: str) -> LocalRuntimeGateway:
        try:
            return self._gateways[str(robot_name)]
        except KeyError as exc:
            raise KeyError(
                f"Unknown robot {robot_name!r}; available={list(self._gateways)}"
            ) from exc

    def start(self):
        if self._closed:
            raise RuntimeError("TwinLocalRuntime is closed")
        if self._started:
            return self

        self._spin_thread = Thread(
            target=self._executor.spin,
            daemon=True,
            name="twin_robot_runtime_executor",
        )
        self._spin_thread.start()
        self._started = True
        return self

    def wait_until_ready(self, timeout_s: float = 15.0) -> bool:
        self.start()
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            if all(
                gateway.is_motion_stack_ready()
                for gateway in self._gateways.values()
            ):
                return True
            time.sleep(0.05)
        return all(
            gateway.is_motion_stack_ready()
            for gateway in self._gateways.values()
        )

    def readiness(self) -> dict:
        return {
            name: {
                "ready": gateway.is_motion_stack_ready(),
                "fault": gateway.get_motion_stack_fault_reason(),
            }
            for name, gateway in self._gateways.items()
        }

    # --- synchronized pair planning / execution -----------------------------

    def _is_gateway_idle(self, name: str) -> bool:
        gateway = self._gateways[str(name)]
        node = gateway.node
        if node is None:
            return False
        if getattr(node, "is_executing", False):
            return False
        queue_size = getattr(node, "motion_queue", None)
        if queue_size is not None:
            status = queue_size.get_status()
            if int(status.get("queue_size", 0)) > 0:
                return False
        return True

    def prepare_pair(self, *, robot1: dict, robot2: dict) -> dict:
        """Plan both robots' paths concurrently without contacting controllers.

        ``robot1``/``robot2`` are kwargs dicts forwarded to each gateway's
        ``prepare_path``. Returns ``{name: PreparedTrajectory}``. No-op prepares
        are valid successes. Raises ``RuntimeError`` if either robot fails to
        prepare (fail-fast).
        """
        self.start()
        results: dict[str, object] = {}
        errors: dict[str, BaseException] = {}

        def _prepare(name: str, kwargs: dict) -> None:
            try:
                results[name] = self._gateways[name].prepare_path(**kwargs)
            except BaseException as exc:
                errors[name] = exc

        threads = [
            Thread(target=_prepare, args=(name, kwargs), daemon=False)
            for name, kwargs in (
                ("robot1", dict(robot1)),
                ("robot2", dict(robot2)),
            )
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        if errors:
            details = ", ".join(f"{name}: {exc}" for name, exc in errors.items())
            raise RuntimeError(f"prepare_pair failed: {details}")

        failed = [
            name
            for name, result in results.items()
            if isinstance(result, int) and result < 0
        ]
        if failed:
            codes = ", ".join(f"{name}={results[name]}" for name in failed)
            raise RuntimeError(f"prepare_pair failed: {codes}")

        for name, prepared in results.items():
            self._gateways[name].node.get_logger().info(
                f"[TWIN] Prepared {name}: noop={getattr(prepared, 'noop', None)} "
                f"points={getattr(prepared, 'point_count', None)} "
                f"duration_s={getattr(prepared, 'duration_s', 0.0):.3f}"
            )
        return dict(results)

    def execute_prepared_pair(
        self,
        prepared1,
        prepared2,
        *,
        blocking: bool = True,
        start_time=None,
        offset_s: float | None = None,
        start_policy: str | None = None,
    ) -> dict:
        """Execute both prepared trajectories with one shared future start time.

        Both trajectories are re-prepared and submitted through the generic
        :class:`SynchronizedTrajectoryExecutor`, which computes ONE common
        ``header.stamp`` (either the provided ``start_time`` or the shared clock
        advanced by ``offset_s``; when ``offset_s`` is None the executor itself
        resolves the generic ``SYNCHRONIZED_EXECUTION_START_DELAY_S``), enforces
        an acceptance barrier (cancelling accepted partners if either goal is
        rejected before the stamp), and waits for both to complete. Because
        both robots run on the same machine / clock, their controllers start
        simultaneously.

        ``start_policy`` selects the start handling per trajectory:
        ``live_anchor`` (default) re-anchors point 0 to live joint state;
        ``require_exact`` verifies the live state matches the cached first
        point and fails with ``MOTION_ERROR_PREPARED_START_MISMATCH`` otherwise.
        When None, each prepared trajectory's own ``metadata["start_policy"]``
        is used if present.

        Returns ``{name: result_code, "_dispatch_separation_s": float}``.
        Raises ``RuntimeError`` if either robot is busy (execution must not be
        queued for a synchronized start), if the acceptance barrier fails, or
        if a goal cannot be prepared/dispatched.
        """
        self.start()
        busy = [
            name
            for name in ("robot1", "robot2")
            if not self._is_gateway_idle(name)
        ]
        if busy:
            raise RuntimeError(
                f"execute_prepared_pair refused: robot(s) not idle: {', '.join(busy)}"
            )

        if start_policy is not None:
            if start_policy not in ("live_anchor", "require_exact"):
                raise ValueError(f"Unsupported start_policy: {start_policy!r}")
            for prepared in (prepared1, prepared2):
                if getattr(prepared, "metadata", None) is not None:
                    prepared.metadata["start_policy"] = start_policy

        from motion.execution.synchronized_trajectory_executor import (
            SynchronizedTrajectoryExecutor,
        )

        sync_executor = SynchronizedTrajectoryExecutor(
            self._nodes[0].get_clock()
        )
        items = [
            (self._gateways["robot1"].node.trajectory_executor, prepared1, "robot1"),
            (self._gateways["robot2"].node.trajectory_executor, prepared2, "robot2"),
        ]
        results = sync_executor.execute(
            items,
            start_time=start_time,
            offset_s=offset_s,
        )
        for name, result in results.items():
            if name.startswith("_"):
                continue
            self._gateways[name].node.get_logger().info(
                f"[TWIN_SYNC] execute_prepared {name} result={result}"
            )
        return dict(results)

    def stop_pair(self) -> dict:
        """Request an immediate stop on both robots (best-effort)."""
        results = {}
        for name, gateway in self._gateways.items():
            try:
                results[name] = gateway.stop_motion()
            except Exception as exc:
                results[name] = {"error": str(exc)}
        return results

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
        self._controllers.clear()
        self._state_publishers.clear()

        if self._owns_rclpy and rclpy.ok():
            rclpy.try_shutdown()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
