from __future__ import annotations

import asyncio
import itertools
import json
import logging
import threading
import time
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from backend.i_robot_backend import IRobotBackend
    from robot_controller import RobotController


logger = logging.getLogger("erob_moveit_rest_server")


def _to_jsonable(value):
    if isinstance(value, dict):
        return {str(k): _to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(v) for v in value]
    if hasattr(value, "tolist") and callable(getattr(value, "tolist")):
        try:
            return _to_jsonable(value.tolist())
        except Exception:
            pass
    if hasattr(value, "item") and callable(getattr(value, "item")):
        try:
            return _to_jsonable(value.item())
        except Exception:
            pass
    return value


def execution_payload(robot: IRobotBackend | None, node: RobotController | None, sequence: int) -> dict:
    payload = {
        "type": "execution_status",
        "sequence": int(sequence),
        "timestamp": time.time(),
    }
    if robot is None or node is None:
        payload.update({
            "success": False,
            "runtime_ready": False,
            "runtime_initialized": False,
            "motion_stack_ready": False,
            "motion_stack_fault": "robot runtime is still starting",
            "unavailable_fields": ["runtime"],
        })
        return payload

    motion_stack_ready = bool(node.is_motion_stack_ready())
    status_publisher = getattr(node, "status_publisher", None)
    get_status_dict = getattr(status_publisher, "get_status_dict", None)
    if not callable(get_status_dict):
        payload.update({
            "success": False,
            "runtime_ready": motion_stack_ready,
            "runtime_initialized": True,
            "motion_stack_ready": motion_stack_ready,
            "motion_stack_fault": None if motion_stack_ready else node.get_motion_stack_fault_reason(),
            "unavailable_fields": ["status_publisher"],
        })
        return payload

    try:
        status = _to_jsonable(get_status_dict())
        get_ordered_status = getattr(robot, "get_ordered_motion_chain_status", None)
        if callable(get_ordered_status):
            ordered_status = _to_jsonable(get_ordered_status())
            if isinstance(ordered_status, dict) and bool(ordered_status.get("active", False)):
                status["ordered_motion_chain"] = ordered_status
        status["last_submitted_task_id"] = getattr(node, "last_submitted_task_id", None)
        payload.update({
            "success": True,
            "runtime_ready": motion_stack_ready,
            "runtime_initialized": True,
            "motion_stack_ready": motion_stack_ready,
            "motion_stack_fault": None if motion_stack_ready else node.get_motion_stack_fault_reason(),
            "unavailable_fields": [],
            "status": status,
        })
        return payload
    except Exception as exc:
        payload.update({
            "success": False,
            "runtime_ready": motion_stack_ready,
            "runtime_initialized": True,
            "motion_stack_ready": motion_stack_ready,
            "motion_stack_fault": None if motion_stack_ready else node.get_motion_stack_fault_reason(),
            "unavailable_fields": ["status"],
            "error": str(exc),
        })
        return payload


def start_execution_websocket_server(
    *,
    robot: IRobotBackend | None,
    node: RobotController | None,
    config,
    fallback_host: str,
    fallback_port: int,
    robot_getter: Callable[[], IRobotBackend | None] | None = None,
    node_getter: Callable[[], RobotController | None] | None = None,
):
    if not bool(getattr(config, "REST_WS_EXECUTION_ENABLED", True)):
        logger.info("Execution WebSocket disabled by REST_WS_EXECUTION_ENABLED")
        return
    try:
        import websockets
        from websockets.exceptions import ConnectionClosed
    except Exception as exc:
        logger.error(f"Execution WebSocket unavailable: {exc}")
        return

    ws_host = str(getattr(config, "REST_WS_EXECUTION_HOST", fallback_host) or fallback_host)
    ws_port = int(getattr(config, "REST_WS_EXECUTION_PORT", int(fallback_port) + 2))
    ws_rate_hz = max(float(getattr(config, "REST_WS_EXECUTION_RATE_HZ", 10.0)), 0.1)
    ws_period_s = 1.0 / ws_rate_hz
    sequence_counter = itertools.count(1)

    async def execution_handler(connection):
        request_obj = getattr(connection, "request", None)
        request_path = str(getattr(request_obj, "path", "") or "")
        path_only = request_path.split("?", 1)[0]
        if path_only != "/ws/execution":
            await connection.close(code=1008, reason="unsupported websocket path")
            return

        try:
            await connection.send(json.dumps({
                "type": "hello",
                "endpoint": "/ws/execution",
                "rate_hz": ws_rate_hz,
                "timestamp": time.time(),
            }, separators=(",", ":")))
            while True:
                sequence = next(sequence_counter)
                current_robot = robot_getter() if robot_getter is not None else robot
                current_node = node_getter() if node_getter is not None else node
                payload = execution_payload(current_robot, current_node, sequence)
                await connection.send(json.dumps(payload, separators=(",", ":")))
                await asyncio.sleep(ws_period_s)
        except ConnectionClosed:
            return
        except Exception as exc:
            logger.exception(f"Execution WebSocket client handler failed: {exc}")

    async def run_server():
        async with websockets.serve(
            execution_handler,
            ws_host,
            ws_port,
            ping_interval=20,
            ping_timeout=20,
            max_size=1024 * 1024,
        ):
            logger.info(f"Execution WebSocket running on ws://{ws_host}:{ws_port}/ws/execution")
            await asyncio.Future()

    def websocket_thread():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(run_server())
        except Exception as exc:
            logger.exception(f"Execution WebSocket server failed: {exc}")
        finally:
            loop.close()

    threading.Thread(
        target=websocket_thread,
        daemon=True,
        name="ExecutionWebSocketServer",
    ).start()
