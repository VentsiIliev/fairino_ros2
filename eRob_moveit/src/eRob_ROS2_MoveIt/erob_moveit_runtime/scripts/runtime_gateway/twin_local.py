#!/usr/bin/env python3
"""Twin-robot local runtime composition.

Creates two normal RobotController instances in one Python process. Each
controller owns one RobotRuntimeContext and one LocalRuntimeGateway. The generic
motion stack remains single-robot: robot selection is completed before commands
enter planners, validation, optimization, or execution.
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

        wo_params = list(getattr(config, "DEFAULT_WORKOBJECT", [0, 0, 0, 0, 0, 0]))
        work_object = WorkObject(*wo_params) if any(value != 0 for value in wo_params) else None

        for robot_name in names:
            context = RobotRuntimeContext.from_config(robot_name)
            topic_prefix = f"/{context.name}"

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
            if all(gateway.is_motion_stack_ready() for gateway in self._gateways.values()):
                return True
            time.sleep(0.05)
        return all(gateway.is_motion_stack_ready() for gateway in self._gateways.values())

    def readiness(self) -> dict:
        return {
            name: {
                "ready": gateway.is_motion_stack_ready(),
                "fault": gateway.get_motion_stack_fault_reason(),
            }
            for name, gateway in self._gateways.items()
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
        self._controllers.clear()
        self._state_publishers.clear()

        if self._owns_rclpy and rclpy.ok():
            rclpy.try_shutdown()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
