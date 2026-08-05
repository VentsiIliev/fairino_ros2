#!/usr/bin/env python3
"""
ROS2 Bridge Server - Exposes the shared MoveIt robot backend via REST API.
"""
import logging
import sys
import threading
import time
import traceback

from flask import Flask, Response, jsonify, request
import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_controller import RobotController
from backend.backend_factory import create_robot_backend
from backend.i_robot_backend import IRobotBackend
from utils.work_object import WorkObject
import config
from rest_api_support import (
    motion_error_response,
    parse_execute_ordered_motion_chain_request,
    parse_execute_path_request,
    parse_execute_sequence_request,
    parse_jog_request,
    parse_move_linear_request,
    validate_pose_from_start,
)


LOG_FILE = config.REST_LOG
logger = logging.getLogger("erob_moveit_rest_server")
logger.setLevel(logging.INFO)
logging.getLogger("werkzeug").setLevel(logging.WARNING)

if not logger.handlers:
    file_handler = logging.FileHandler(LOG_FILE)
    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s"
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)


OPENAPI_SPEC = {
    "openapi": "3.0.3",
    "info": {
        "title": "ZeroErr Robot Runtime API",
        "version": "1.0.0",
        "description": "REST API for startup polling, robot motion, state, tools, safety walls, drives, and interlocks.",
    },
    "servers": [{"url": "/"}],
    "paths": {
        "/health": {"get": {"tags": ["startup"], "summary": "HTTP/runtime health"}},
        "/startup/status": {"get": {"tags": ["startup"], "summary": "Startup progress for frontend polling"}},
        "/status": {"get": {"tags": ["state"], "summary": "Robot execution, queue, runtime, drive, and interlock status"}},
        "/state/snapshot": {"get": {"tags": ["state"], "summary": "Combined UI state snapshot"}},
        "/state/kinematics": {"get": {"tags": ["state"], "summary": "Current TCP position, velocity, and acceleration"}},
        "/position/current": {"get": {"tags": ["state"], "summary": "Current TCP position"}},
        "/position/flange": {"get": {"tags": ["state"], "summary": "Current flange position"}},
        "/velocity/current": {"get": {"tags": ["state"], "summary": "Current TCP velocity"}},
        "/move/linear": {"post": {"tags": ["motion"], "summary": "Queue or execute a linear move"}},
        "/move/ptp": {"post": {"tags": ["motion"], "summary": "Queue or execute a point-to-point move"}},
        "/execute/path": {"post": {"tags": ["motion"], "summary": "Execute a waypoint path"}},
        "/execute/sequence": {"post": {"tags": ["motion"], "summary": "Execute mixed motion segments"}},
        "/execute/ordered_motion_chain": {"post": {"tags": ["motion"], "summary": "Execute ordered motion chain"}},
        "/execute/ordered_motion_chain/status": {"get": {"tags": ["motion"], "summary": "Ordered motion chain status"}},
        "/unwind/joint6": {"post": {"tags": ["motion"], "summary": "Unwind joint 6"}},
        "/jog": {"post": {"tags": ["motion"], "summary": "Jog along one robot axis"}},
        "/stop": {"post": {"tags": ["motion"], "summary": "Stop active motion and clear queued work"}},
        "/reachability/pose": {"post": {"tags": ["planning"], "summary": "Validate pose reachability from a start pose"}},
        "/workobject/set": {"post": {"tags": ["frames"], "summary": "Set active work object origin"}},
        "/tool/registry": {"get": {"tags": ["tools"], "summary": "Get tool registry"}},
        "/tool/registry/{tool_id}": {
            "post": {
                "tags": ["tools"],
                "summary": "Update one tool registry entry",
                "parameters": [{"name": "tool_id", "in": "path", "required": True, "schema": {"type": "integer"}}],
            }
        },
        "/tool/active": {
            "get": {"tags": ["tools"], "summary": "Get active tool"},
            "post": {"tags": ["tools"], "summary": "Set active tool"},
        },
        "/safety/walls/enabled": {"get": {"tags": ["safety"], "summary": "Check whether safety walls are enabled"}},
        "/safety/walls/status": {"get": {"tags": ["safety"], "summary": "Safety wall status"}},
        "/safety/walls/enable": {"post": {"tags": ["safety"], "summary": "Enable safety walls"}},
        "/safety/walls/disable": {"post": {"tags": ["safety"], "summary": "Disable safety walls"}},
        "/io/digital_output": {"post": {"tags": ["io"], "summary": "Set digital output"}},
        "/drive/status": {"get": {"tags": ["drives"], "summary": "Drive operation-enable status"}},
        "/drive/enable": {"post": {"tags": ["drives"], "summary": "Request and verify drive operation enable"}},
        "/drive/disable": {"post": {"tags": ["drives"], "summary": "Request and verify drive operation disable"}},
        "/motion/interlock/status": {"get": {"tags": ["interlock"], "summary": "Motion interlock status"}},
        "/motion/interlock/reset": {"post": {"tags": ["interlock"], "summary": "Reset motion interlock"}},
    },
}


def _json_schema(schema_type="object", **extra):
    return {"type": schema_type, **extra}


def _json_request_body(example: dict | None = None, required: bool = False) -> dict:
    media_type = {"schema": _json_schema("object")}
    if example is not None:
        media_type["example"] = example
    return {
        "required": required,
        "content": {
            "application/json": media_type,
        },
    }


def _apply_openapi_details():
    default_response = {
        "description": "JSON response",
        "content": {
            "application/json": {
                "schema": _json_schema("object"),
            },
        },
    }
    for path_item in OPENAPI_SPEC["paths"].values():
        for operation in path_item.values():
            operation.setdefault("responses", {"200": default_response})

    post_examples = {
        "/move/linear": {
            "position": [300, 0, 300, 180, 0, 0],
            "tool": 0,
            "user": 0,
            "vel": 20,
            "acc": 20,
            "blocking": False,
            "trajectory_optimizer": "RUCKIG",
        },
        "/move/ptp": {
            "position": [300, 0, 300, 180, 0, 0],
            "tool": 0,
            "user": 0,
            "vel": 20,
            "acc": 20,
            "blocking": False,
        },
        "/execute/path": {
            "path": [[300, 0, 300, 180, 0, 0], [320, 0, 300, 180, 0, 0]],
            "vel": 20,
            "acc": 20,
            "blocking": False,
            "orientation_mode": "constant",
        },
        "/execute/sequence": {
            "segments": [
                {"motion_type": "linear", "position": [300, 0, 300, 180, 0, 0], "vel": 20, "acc": 20},
                {"motion_type": "ptp", "position": [320, 0, 300, 180, 0, 0], "vel": 20, "acc": 20},
            ],
            "tool": 0,
            "user": 0,
            "blocking": False,
        },
        "/execute/ordered_motion_chain": {
            "segments": [
                {"type": "linear", "label": "approach", "position": [300, 0, 300, 180, 0, 0], "vel": 20, "acc": 20}
            ],
            "tool": 0,
            "user": 0,
            "blocking": True,
            "trajectory_optimizer": "RUCKIG",
        },
        "/unwind/joint6": {"blocking": True, "queue_if_busy": True, "vel": 20, "acc": 20},
        "/jog": {"axis": "X", "direction": "POSITIVE", "step": 10, "vel": 10, "acc": 10},
        "/reachability/pose": {
            "target_position": [300, 0, 300, 180, 0, 0],
            "start_position": [280, 0, 300, 180, 0, 0],
            "tool": 0,
            "user": 0,
        },
        "/workobject/set": {"origin": [0, 0, 0, 0, 0, 0], "user_id": 0},
        "/tool/registry/{tool_id}": {
            "name": "TOOL_1",
            "transform": [0, 0, 170, 0, 0, 0],
            "persist": False,
        },
        "/tool/active": {"tool_id": 1},
        "/io/digital_output": {"port": 0, "value": 1},
        "/stop": {},
        "/safety/walls/enable": {},
        "/safety/walls/disable": {},
        "/drive/enable": {},
        "/drive/disable": {},
        "/motion/interlock/reset": {},
    }
    for path, example in post_examples.items():
        operation = OPENAPI_SPEC["paths"].get(path, {}).get("post")
        if operation is not None:
            operation["requestBody"] = _json_request_body(example, required=bool(example))

    OPENAPI_SPEC["paths"]["/jog"]["post"]["requestBody"] = {
        "required": True,
        "content": {
            "application/json": {
                "schema": {
                    "type": "object",
                    "required": ["axis", "direction", "step", "vel", "acc"],
                    "properties": {
                        "axis": {"type": "string", "enum": ["X", "Y", "Z", "RX", "RY", "RZ"]},
                        "direction": {"type": "string", "enum": ["PLUS", "MINUS"]},
                        "step": {"type": "number"},
                        "vel": {"type": "number"},
                        "acc": {"type": "number"},
                    },
                },
                "example": {"axis": "X", "direction": "PLUS", "step": 10, "vel": 10, "acc": 10},
            },
        },
    }

    drive_command_responses = {
        "200": {"description": "Command verified against current drive status"},
        "202": {"description": "Command accepted but drive status has not matched yet"},
        "500": {"description": "Command failed or returned unexpected state"},
        "503": {"description": "Hardware not ready"},
    }
    OPENAPI_SPEC["paths"]["/drive/enable"]["post"]["responses"] = drive_command_responses
    OPENAPI_SPEC["paths"]["/drive/disable"]["post"]["responses"] = drive_command_responses
    OPENAPI_SPEC["paths"]["/execute/sequence"]["post"]["responses"] = {
        "200": {"description": "Blocking sequence completed with final result 0"},
        "202": {"description": "Sequence queued or accepted for asynchronous planning/execution"},
        "400": {"description": "Invalid sequence request"},
        "409": {"description": "Drive not enabled or controller execution failure"},
        "500": {"description": "Internal or planning failure"},
        "503": {"description": "MoveIt service, queue, or hardware not ready"},
    }


_apply_openapi_details()


SWAGGER_HTML = """<!doctype html>
<html>
<head>
  <title>ZeroErr Robot Runtime API</title>
  <link rel="stylesheet" href="https://unpkg.com/swagger-ui-dist@5/swagger-ui.css">
  <style>body { margin: 0; background: #fff; }</style>
</head>
<body>
  <div id="swagger-ui"></div>
  <script src="https://unpkg.com/swagger-ui-dist@5/swagger-ui-bundle.js"></script>
  <script>
    window.onload = function() {
      SwaggerUIBundle({
        url: "/openapi.json",
        dom_id: "#swagger-ui",
        deepLinking: true,
        displayRequestDuration: true,
        tryItOutEnabled: true,
        supportedSubmitMethods: ["get", "post"]
      });
    };
  </script>
</body>
</html>
"""


def _to_jsonable(value):
    """Convert nested ROS / numpy-ish values into plain JSON-safe Python types."""
    if isinstance(value, dict):
        return {str(k): _to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(v) for v in value]
    if hasattr(value, "item") and callable(getattr(value, "item")):
        try:
            return _to_jsonable(value.item())
        except Exception:
            pass
    return value


def _response_error(message: str, status_code: int, **extra):
    return jsonify({"success": False, "error": message, **extra}), status_code


def _http_status_for_state(state: str | None, default: int = 500) -> int:
    if state in {"HARDWARE_NOT_READY"}:
        return 503
    if state in {"UNSUPPORTED"}:
        return 501
    return default


def _as_dict(value, error_message: str) -> dict:
    value = _to_jsonable(value)
    if isinstance(value, dict):
        return value
    return {"success": False, "error": error_message}

def start_rest_server(
        robot: IRobotBackend | None = None,
        node: RobotController | None = None,
        host: str = config.REST_HOST,
        port: int = config.REST_PORT,
        start_ros: bool = True,
        runtime_initializer=None,
        allow_starting_without_robot: bool = False,
):
    """
    Start REST server.

    Modes:
    - Standalone: start_ros=True, robot=None
    - Embedded:   start_ros=False, robot provided
    """

    app = Flask(__name__)
    startup_lock = threading.Lock()
    startup_state = {
        "phase": "initializing_http",
        "message": "HTTP server is starting",
        "ready": False,
        "error": None,
        "started_at": time.time(),
        "updated_at": time.time(),
    }

    def update_startup_status(phase: str, message: str | None = None, **extra):
        with startup_lock:
            startup_state.update({
                "phase": phase,
                "message": message if message is not None else phase,
                "updated_at": time.time(),
            })
            startup_state.update(extra)

    def get_startup_status():
        with startup_lock:
            status = dict(startup_state)
        status["ros2_active"] = robot is not None
        return status

    def runtime_state_snapshot() -> dict:
        if robot is None or node is None:
            return {
                "runtime_ready": False,
                "startup": get_startup_status(),
            }
        drive_status = _to_jsonable(node.get_drive_operation_status())
        motion_interlock = _to_jsonable(node.get_motion_interlock_status())
        hardware_ready = bool(node.is_hardware_ready_for_motion())
        motion_stack_ready = bool(node.is_motion_stack_ready())
        return {
            "runtime_ready": True,
            "hardware_ready": hardware_ready,
            "hardware_fault": None if hardware_ready else node.get_hardware_fault_reason(),
            "motion_stack_ready": motion_stack_ready,
            "motion_stack_fault": None if motion_stack_ready else node.get_motion_stack_fault_reason(),
            "drive": drive_status,
            "motion_interlock": motion_interlock,
        }

    def drive_command_response(command_result: dict, desired_enabled: bool):
        command_result = _to_jsonable(command_result)
        if not isinstance(command_result, dict):
            return jsonify({
                "success": False,
                "command_accepted": False,
                "desired_enabled": bool(desired_enabled),
                "error": f"unexpected drive command result type: {type(command_result).__name__}",
            }), 500
        command_accepted = bool(command_result.get("success", False))
        verify_timeout_s = max(
            float(getattr(config, "STARTUP_AUTO_ENABLE_DRIVES_VERIFY_TIMEOUT_S", 5.0)),
            0.1,
        )
        verify_deadline = time.monotonic() + verify_timeout_s
        drive_status = _as_dict(
            node.get_drive_operation_status(),
            "unexpected drive status result type",
        )

        while command_accepted and time.monotonic() < verify_deadline:
            actual_enabled = bool(drive_status.get("actual_enabled", False))
            requested_enabled = bool(drive_status.get("requested_enabled", False))
            if desired_enabled and actual_enabled and requested_enabled:
                break
            if not desired_enabled and not actual_enabled and not requested_enabled:
                break
            time.sleep(0.05)
            drive_status = _as_dict(
                node.get_drive_operation_status(),
                "unexpected drive status result type",
            )
            if not drive_status.get("success", True):
                break

        actual_enabled = bool(drive_status.get("actual_enabled", False))
        requested_enabled = bool(drive_status.get("requested_enabled", False))
        verified = (
            actual_enabled and requested_enabled
            if desired_enabled
            else not actual_enabled and not requested_enabled
        )
        response = {
            **drive_status,
            "success": bool(command_accepted and verified),
            "command_accepted": command_accepted,
            "desired_enabled": bool(desired_enabled),
            "request": command_result,
        }
        if response["success"]:
            return jsonify(response), 200
        if not command_accepted:
            error = str(command_result.get("error") or "drive command was rejected")
            status_code = _http_status_for_state(command_result.get("state"))
            response["error"] = error
            return jsonify(response), status_code
        response["error"] = (
            "drive enable command accepted, but drives are not operation_enabled"
            if desired_enabled
            else "drive disable command accepted, but drives still report operation_enabled"
        )
        return jsonify(response), 202

    # ------------------------------------------------------------------
    # ROS2 initialization (standalone mode)
    # ------------------------------------------------------------------

    executor = None

    if start_ros:
        rclpy.init()
        node = RobotController()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        def ros_spin():
            executor.spin()

        threading.Thread(target=ros_spin, daemon=True).start()

        node.wait_for_monitor(timeout_sec=config.MONITOR_WAIT_TIMEOUT_S)

        robot = create_robot_backend(node=node, workobject=None, ip="0.0.0.0")

        time.sleep(1.0)

    if runtime_initializer is not None:
        def initialize_runtime():
            nonlocal robot, node
            try:
                update_startup_status(
                    "initializing_runtime",
                    "ROS runtime is initializing",
                )
                initialized = runtime_initializer(update_startup_status)
                if initialized is not None:
                    robot, node = initialized
                if robot is None or node is None:
                    raise RuntimeError("runtime initializer did not return a robot and node")
                update_startup_status(
                    "ready",
                    "Robot runtime is ready",
                    ready=True,
                    error=None,
                )
            except Exception as exc:
                update_startup_status(
                    "error",
                    f"Runtime initialization failed: {exc}",
                    ready=False,
                    error=str(exc),
                )
                logger.exception("Runtime initialization failed")

        threading.Thread(
            target=initialize_runtime,
            daemon=True,
            name="RuntimeInitializer",
        ).start()

    if robot is None and runtime_initializer is None and not allow_starting_without_robot:
        raise RuntimeError("REST server started without a robot instance")

    @app.before_request
    def require_runtime_ready():
        allowed_endpoints = {"health", "startup_status", "swagger_docs", "openapi_json", "static"}
        if request.endpoint in allowed_endpoints:
            return None
        if robot is None or node is None:
            return jsonify({
                "success": False,
                "error": "robot runtime is still starting",
                "startup": get_startup_status(),
            }), 503
        return None

    # ------------------------------------------------------------------
    # Routes
    # ------------------------------------------------------------------

    @app.route("/health", methods=["GET"])
    def health():
        status = get_startup_status()
        http_status = 200 if status.get("error") is None else 500
        return jsonify({"status": "ok" if status["ready"] else status["phase"], **status}), http_status

    @app.route("/startup/status", methods=["GET"])
    def startup_status():
        status = get_startup_status()
        http_status = 200 if status.get("error") is None else 500
        return jsonify(status), http_status

    @app.route("/docs", methods=["GET"])
    @app.route("/api/docs", methods=["GET"])
    def swagger_docs():
        return Response(SWAGGER_HTML, mimetype="text/html")

    @app.route("/openapi.json", methods=["GET"])
    def openapi_json():
        return jsonify(OPENAPI_SPEC)

    @app.route("/move/linear", methods=["POST"])
    def move_linear():
        try:
            payload = parse_move_linear_request(request.json)
        except ValueError as exc:
            return _response_error(str(exc), 400)

        logger.info(f"Received move/linera/ request with data {request.json}")

        result = robot.move_liner(
            payload["position"],
            tool=payload["tool"],
            user=payload["user"],
            vel=payload["vel"],
            acc=payload["acc"],
            blocking=payload["blocking"],
            trajectory_optimizer=payload["trajectory_optimizer"],
        )
        task_id = getattr(robot.node, 'last_submitted_task_id', None)

        if result > 0:
            logger.debug(f"move_linear queued with result {result}")
            return jsonify({"result": result, "success": True, "queued": True, "queue_position": result, "task_id": task_id}), 202
        elif result == 0:
            logger.debug(f"move_linear queued with result {result}")
            return jsonify({"result": result, "success": True, "queued": False, "task_id": task_id}), 200
        else:
            return motion_error_response(result)

    @app.route("/move/ptp", methods=["POST"])
    def move_ptp():
        try:
            payload = parse_move_linear_request(request.json)
        except ValueError as exc:
            return _response_error(str(exc), 400)

        logger.info(f"Received move/ptp request with data {request.json}")

        result = robot.move_ptp(
            payload["position"],
            tool=payload["tool"],
            user=payload["user"],
            vel=payload["vel"],
            acc=payload["acc"],
            blocking=payload["blocking"],
            trajectory_optimizer=payload["trajectory_optimizer"],
        )
        task_id = getattr(robot.node, 'last_submitted_task_id', None)

        if result > 0:
            return jsonify({"result": result, "success": True, "queued": True, "queue_position": result, "task_id": task_id}), 202
        elif result == 0:
            return jsonify({"result": result, "success": True, "queued": False, "task_id": task_id}), 200
        else:
            return motion_error_response(result)


    @app.route("/execute/path", methods=["POST"])
    def execute_path():
        try:
            payload = parse_execute_path_request(request.json)
        except ValueError as exc:
            return _response_error(str(exc), 400)

        robot.node.get_logger().info(
            f"Executing path with {len(payload['path'])} waypoints, vel={payload['vel']}, acc={payload['acc']}")

        try:
            result = robot.execute_path(
                payload["path"],
                rx=payload["rx"],
                ry=payload["ry"],
                rz=payload["rz"],
                vel=payload["vel"],
                acc=payload["acc"],
                blocking=payload["blocking"],
                trajectory_optimizer=payload["trajectory_optimizer"],
                orientation_mode=payload["orientation_mode"],
            )
        except Exception as e:
            import traceback
            traceback.print_exc()
            robot.node.get_logger().error(f"Error executing path: {e}")
            return jsonify({"result": -1, "success": False, "error": str(e)}), 500

        task_id = getattr(robot.node, 'last_submitted_task_id', None)
        if result > 0:
            return jsonify({"result": result, "success": True, "queued": True, "queue_position": result, "task_id": task_id}), 202
        elif result == 0:
            return jsonify({"result": result, "success": True, "queued": False, "task_id": task_id}), 200
        else:
            return motion_error_response(result)

    @app.route("/execute/sequence", methods=["POST"])
    def execute_sequence():
        try:
            payload = parse_execute_sequence_request(request.json)
        except ValueError as exc:
            return _response_error(str(exc), 400)

        robot.node.get_logger().info(
            f"Executing sequence with {len(payload['segments'])} segments")

        try:
            result = robot.execute_sequence(
                payload["segments"],
                tool=payload["tool"],
                user=payload["user"],
                blocking=payload["blocking"],
            )
        except Exception as e:
            import traceback
            traceback.print_exc()
            robot.node.get_logger().error(f"Error executing sequence: {e}")
            return jsonify({"result": -1, "success": False, "error": str(e)}), 500

        task_id = getattr(robot.node, 'last_submitted_task_id', None)
        if result > 0:
            return jsonify({
                "result": result,
                "success": True,
                "accepted": True,
                "final": False,
                "state": "QUEUED",
                "queued": True,
                "queue_position": result,
                "task_id": task_id,
                "status_url": "/status",
                "message": "sequence queued; poll /status for current_task_id, last_completed_task_id, and last_completed_result",
            }), 202
        elif result == 0:
            if payload["blocking"]:
                return jsonify({
                    "result": result,
                    "success": True,
                    "accepted": True,
                    "final": True,
                    "state": "COMPLETED",
                    "queued": False,
                    "task_id": task_id,
                }), 200
            return jsonify({
                "result": result,
                "success": True,
                "accepted": True,
                "final": False,
                "state": "ACCEPTED_ASYNC",
                "queued": False,
                "task_id": task_id,
                "status_url": "/status",
                "message": "sequence accepted; planning/execution completes asynchronously, poll /status for final result",
            }), 202
        else:
            return motion_error_response(
                result,
                accepted=False,
                final=True,
                state="REJECTED",
                queued=False,
                task_id=task_id,
            )

    @app.route("/execute/ordered_motion_chain", methods=["POST"])
    def execute_ordered_motion_chain():
        try:
            payload = parse_execute_ordered_motion_chain_request(request.json)
        except ValueError as exc:
            return _response_error(str(exc), 400)

        robot.node.get_logger().info(
            f"Executing ordered motion chain with {len(payload['segments'])} segments")

        try:
            result = robot.execute_ordered_motion_chain(
                segments=payload["segments"],
                tool=payload["tool"],
                user=payload["user"],
                blocking=payload["blocking"],
                trajectory_optimizer=payload["trajectory_optimizer"],
            )
        except Exception as e:
            traceback.print_exc()
            robot.node.get_logger().error(f"Error executing ordered motion chain: {e}")
            return jsonify({"result": -1, "success": False, "error": str(e)}), 500

        task_id = getattr(robot.node, 'last_submitted_task_id', None)
        if result > 0:
            return jsonify({"result": result, "success": True, "queued": True, "queue_position": result, "task_id": task_id}), 202
        elif result == 0:
            return jsonify({"result": result, "success": True, "queued": False, "task_id": task_id}), 200
        else:
            return motion_error_response(result)

    @app.route("/unwind/joint6", methods=["POST"])
    def unwind_joint6():
        data = request.json or {}
        blocking = bool(data.get("blocking", True))
        queue_if_busy = bool(data.get("queue_if_busy", True))
        vel = data.get("vel")
        acc = data.get("acc")

        try:
            vel = None if vel is None else float(vel)
            acc = None if acc is None else float(acc)
        except (TypeError, ValueError):
            return jsonify({
                "result": -1,
                "success": False,
                "error": "vel and acc must be numeric when provided",
            }), 400

        try:
            result = robot.unwind_joint6(
                blocking=blocking,
                queue_if_busy=queue_if_busy,
                vel=vel,
                acc=acc,
            )
        except Exception as exc:
            traceback.print_exc()
            robot.node.get_logger().error(f"Error executing explicit Joint_6 unwind: {exc}")
            return jsonify({"result": -1, "success": False, "error": str(exc)}), 500

        task_id = getattr(robot.node, 'last_submitted_task_id', None)
        if result > 0:
            return jsonify({
                "result": result,
                "success": True,
                "queued": True,
                "queue_position": result,
                "task_id": task_id,
            }), 202
        if result == 0:
            return jsonify({
                "result": result,
                "success": True,
                "queued": False,
                "task_id": task_id,
            }), 200
        return motion_error_response(result)


    @app.route("/safety/walls/enabled", methods=["GET"])
    def safety_walls_enabled():
        body = _as_dict(robot.get_safety_walls_status(), "invalid safety wall status")
        success = not body.get("error")
        return jsonify({**body, "success": success, "enabled": bool(body.get("enabled", False))}), 200 if success else 503

    @app.route("/safety/walls/status", methods=["GET"])
    def safety_walls_status():
        body = _as_dict(robot.get_safety_walls_status(), "invalid safety wall status")
        success = not body.get("error")
        return jsonify({**body, "success": success}), 200 if success else 503

    @app.route("/safety/walls/enable", methods=["POST"])
    def enable_safety_walls():
        body = _as_dict(robot.enable_safety_walls(), "invalid safety wall status")
        success = bool(body.get("enabled", False)) and not body.get("error")
        return jsonify({**body, "success": success}), 200 if success else 500

    @app.route("/safety/walls/disable", methods=["POST"])
    def disable_safety_walls():
        body = _as_dict(robot.disable_safety_walls(), "invalid safety wall status")
        success = not bool(body.get("enabled", True)) and not body.get("error")
        return jsonify({**body, "success": success}), 200 if success else 500

    @app.route("/position/current", methods=["GET"])
    def get_position():
        pos = robot.get_current_position()
        if pos is None:
            return _response_error("current position unavailable", 503)
        return jsonify({"success": True, "position": pos})

    @app.route("/position/flange", methods=["GET"])
    def get_flange_position():
        pos = robot.get_current_flange_position()
        if pos is None:
            return _response_error("current flange position unavailable", 503)
        return jsonify({"success": True, "position": pos})

    @app.route("/tool/registry", methods=["GET"])
    def get_tool_registry():
        return jsonify({"success": True, **config.get_tool_registry_snapshot()})

    @app.route("/tool/active", methods=["GET"])
    def get_active_tool():
        return jsonify({
            "success": True,
            "tool_name": getattr(node, "active_tool_name", "TOOL_0"),
        })

    @app.route("/tool/active", methods=["POST"])
    def set_active_tool():
        try:
            payload = request.json or {}
            if "tool_id" in payload:
                tool_name = config.resolve_tool_name(payload.get("tool_id"))
            else:
                tool_name = str(payload.get("name") or payload.get("tool_name") or "").strip()
                if not tool_name:
                    raise ValueError("tool_id or tool_name is required")
            node.set_tool(tool_name)
            return jsonify({
                "success": True,
                "tool_name": getattr(node, "active_tool_name", tool_name),
            })
        except ValueError as exc:
            return jsonify({"success": False, "error": str(exc)}), 400
        except Exception as exc:
            traceback_text = traceback.format_exc()
            node.get_logger().error(
                "REST /tool/active exception: "
                f"{exc}\n{traceback_text}"
            )
            return jsonify({"success": False, "error": str(exc)}), 500

    @app.route("/tool/registry/<int:tool_id>", methods=["POST"])
    def update_tool_registry(tool_id):
        try:
            payload = request.json or {}
            transform = payload.get("transform")
            name = payload.get("name")
            persist = bool(payload.get("persist", False))
            snapshot = config.update_tool_registry(
                tool_id=tool_id,
                name=name,
                transform=transform,
                persist=persist,
            )
            if getattr(node, "active_tool_name", None) == config.resolve_tool_name(tool_id):
                node.set_tool(config.resolve_tool_name(tool_id))
            return jsonify({"success": True, **snapshot})
        except ValueError as exc:
            return jsonify({"success": False, "error": str(exc)}), 400
        except Exception as exc:
            traceback_text = traceback.format_exc()
            node.get_logger().error(
                "REST /tool/registry exception: "
                f"{exc}\n{traceback_text}"
            )
            return jsonify({"success": False, "error": str(exc)}), 500

    @app.route("/reachability/pose", methods=["POST"])
    def validate_pose():
        try:
            data = request.json or {}
            target_position = data.get("target_position") or data.get("position")
            start_position = data.get("start_position")
            node.get_logger().info(
                "REST /reachability/pose request: "
                f"start={start_position} target={target_position} "
                f"tool={data.get('tool', 0)} user={data.get('user', 0)} "
                f"has_seed_joint_state={bool(data.get('start_joint_state'))}"
            )
            if not target_position or len(target_position) != 6:
                return _response_error("Invalid target_position format", 400, reachable=False)
            if start_position is None:
                start_position = robot.get_current_position()
            if not start_position or len(start_position) != 6:
                return _response_error("Invalid or unavailable start_position", 400, reachable=False)

            result = validate_pose_from_start(
                node,
                robot,
                start_position=start_position,
                target_position=target_position,
                tool=data.get("tool", 0),
                user=data.get("user", 0),
                start_joint_state_payload=data.get("start_joint_state"),
            )
            result = _to_jsonable(result)
            http_status = 200 if result.get("reachable") else 409 if result.get("reason") == "cartesian_path_partial" else 400
            log_fn = node.get_logger().info if result.get("reachable") else node.get_logger().warning
            log_fn(f"REST /reachability/pose response: http={http_status} result={result}")
            return jsonify({**result, "success": bool(result.get("reachable"))}), http_status
        except Exception as exc:
            traceback_text = traceback.format_exc()
            node.get_logger().error(
                "REST /reachability/pose exception: "
                f"{exc}\n{traceback_text}"
            )
            return jsonify({
                "success": False,
                "reachable": False,
                "reason": "rest_handler_exception",
                "error": str(exc),
            }), 500

    @app.route("/velocity/current", methods=["GET"])
    def get_velocity():
        vel = robot.get_current_velocity()
        if vel is None:
            return _response_error("current velocity unavailable", 503)
        return jsonify({"success": True, "velocity": vel})

    @app.route("/stop", methods=["POST"])
    def stop_motion():
        robot.node.get_logger().info("[rest_server.py] Stopping motion")
        stop_result = robot.stop_motion()
        if not isinstance(stop_result, dict):
            stop_result = {
                "state": "ERROR",
                "result": -2,
                "success": False,
                "stopped": False,
                "error": f"Unexpected stop result type: {type(stop_result).__name__}",
            }
        return jsonify({
            "stop_state": stop_result.get("state", "ERROR"),
            "stopped": bool(stop_result.get("stopped", False)),
            "result": stop_result.get("result", -2),
            "success": bool(stop_result.get("success", False)),
            "queue_cleared": int(stop_result.get("queue_cleared", 0)),
            **({"error": stop_result["error"]} if stop_result.get("error") else {}),
        })

    @app.route("/workobject/set", methods=["POST"])
    def set_workobject():
        data = request.json or {}
        origin = data.get("origin")

        if not origin or len(origin) != 6:
            return _response_error("Invalid origin format", 400)

        try:
            workobject = WorkObject(origin=origin)
            robot.set_workobject(workobject, user_id=data.get("user_id", 0))
        except Exception as exc:
            node.get_logger().error(f"Workobject endpoint error: {exc}")
            return _response_error(str(exc), 500)
        return jsonify({"success": True, "origin": origin, "user_id": data.get("user_id", 0)})

    @app.route("/status", methods=["GET"])
    def get_status():
        """Get robot execution status and queue size."""
        status = robot.node.status_publisher.get_status_dict()
        get_ordered_status = getattr(robot, "get_ordered_motion_chain_status", None)
        if callable(get_ordered_status):
            status["ordered_motion_chain"] = _to_jsonable(get_ordered_status())
        status["success"] = True
        status.update(runtime_state_snapshot())
        return jsonify(status)

    @app.route("/execute/ordered_motion_chain/status", methods=["GET"])
    def get_ordered_motion_chain_status():
        get_ordered_status = getattr(robot, "get_ordered_motion_chain_status", None)
        if not callable(get_ordered_status):
            return jsonify({"success": True, "supported": False, "active": False})
        status = _as_dict(get_ordered_status(), "invalid ordered motion chain status")
        success = not status.get("error")
        return jsonify({**status, "success": success}), 200 if success else 500

    @app.route("/state/snapshot", methods=["GET"])
    def get_state_snapshot():
        """Get common UI/runtime state in one request."""
        position = robot.get_current_position()
        flange_position = robot.get_current_flange_position()
        velocity = robot.get_current_velocity()
        unavailable = []
        if position is None:
            unavailable.append("position")
        if flange_position is None:
            unavailable.append("flange_position")
        if velocity is None:
            unavailable.append("velocity")
        return jsonify({
            "success": not unavailable,
            "partial": bool(unavailable),
            "unavailable_fields": unavailable,
            "position": position,
            "flange_position": flange_position,
            "velocity": velocity,
            "status": robot.node.status_publisher.get_status_dict(),
            "active_tool": getattr(node, "active_tool_name", "TOOL_0"),
            "safety_walls": robot.get_safety_walls_status(),
            **runtime_state_snapshot(),
        })

    @app.route("/state/kinematics", methods=["GET"])
    def get_state_kinematics():
        """Get current TCP kinematic state in one request."""
        position = robot.get_current_position()
        velocity = robot.get_current_velocity()
        acceleration = robot.get_current_acceleration()

        if position is None:
            return jsonify({
                "success": False,
                "error": "current position unavailable",
            }), 503

        unavailable = []
        if velocity is None:
            unavailable.append("velocity")
        if acceleration is None:
            unavailable.append("acceleration")

        return jsonify({
            "success": not unavailable,
            "partial": bool(unavailable),
            "unavailable_fields": unavailable,
            "position": position,
            "velocity": velocity,
            "acceleration": acceleration,
        }), 200 if not unavailable else 206


    @app.route("/jog", methods=["POST"])
    def jog():
        try:
            axis, direction, step, vel, acc = parse_jog_request(request.json)
            result = robot.start_jog(axis, direction, step, vel, acc)
            if result == 0:
                return jsonify({"result": result, "success": True}), 200
            return motion_error_response(result)
        except ValueError as exc:
            return jsonify({"result": -1, "success": False, "error": str(exc)}), 400
        except Exception as exc:
            robot.node.get_logger().error(f"Jog endpoint error: {exc}")
            return jsonify({"result": -1, "success": False, "error": str(exc)}), 500

    @app.route("/io/digital_output", methods=["POST"])
    def set_digital_output():
        try:
            data = request.json or {}
            port = int(data["port"])
            value = int(data["value"])
        except KeyError as exc:
            return jsonify({"result": -1, "success": False, "error": f"Missing field: {exc.args[0]}"}), 400
        except (TypeError, ValueError):
            return jsonify({"result": -1, "success": False, "error": "port and value must be integers"}), 400

        if port < 0:
            return jsonify({"result": -1, "success": False, "error": "port must be >= 0"}), 400
        if value not in (0, 1):
            return jsonify({"result": -1, "success": False, "error": "value must be 0 or 1"}), 400

        try:
            result = robot.setDigitalOutput(port, value)
            if result == 0:
                return jsonify({"result": 0, "success": True, "port": port, "value": value}), 200
            return jsonify({"result": result, "success": False, "port": port, "value": value}), 500
        except Exception as exc:
            robot.node.get_logger().error(f"Digital output endpoint error: {exc}")
            return jsonify({"result": -1, "success": False, "error": str(exc)}), 500

    @app.route("/drive/enable", methods=["POST"])
    def enable_drive_operation():
        try:
            result = _to_jsonable(node.set_drive_operation_enabled(True))
            return drive_command_response(result, desired_enabled=True)
        except Exception as exc:
            node.get_logger().error(f"Drive enable endpoint error: {exc}")
            logger.exception("Drive enable endpoint error")
            return jsonify({"success": False, "error": str(exc)}), 500

    @app.route("/drive/disable", methods=["POST"])
    def disable_drive_operation():
        try:
            result = _to_jsonable(node.set_drive_operation_enabled(False))
            return drive_command_response(result, desired_enabled=False)
        except Exception as exc:
            node.get_logger().error(f"Drive disable endpoint error: {exc}")
            logger.exception("Drive disable endpoint error")
            return jsonify({"success": False, "error": str(exc)}), 500

    @app.route("/drive/status", methods=["GET"])
    def drive_operation_status():
        try:
            return jsonify({
                **_to_jsonable(node.get_drive_operation_status()),
                "hardware_ready": bool(node.is_hardware_ready_for_motion()),
                "hardware_fault": None if node.is_hardware_ready_for_motion() else node.get_hardware_fault_reason(),
            })
        except Exception as exc:
            node.get_logger().error(f"Drive status endpoint error: {exc}")
            logger.exception("Drive status endpoint error")
            return jsonify({"success": False, "error": str(exc)}), 500

    @app.route("/motion/interlock/status", methods=["GET"])
    def motion_interlock_status():
        return jsonify({"success": True, **_to_jsonable(node.get_motion_interlock_status())})

    @app.route("/motion/interlock/reset", methods=["POST"])
    def reset_motion_interlock():
        result = _to_jsonable(node.reset_motion_interlock())
        return jsonify({
            **result,
            "hardware_ready": bool(node.is_hardware_ready_for_motion()),
            "hardware_fault": None if node.is_hardware_ready_for_motion() else node.get_hardware_fault_reason(),
            "motion_interlock": _to_jsonable(node.get_motion_interlock_status()),
        })

    # ------------------------------------------------------------------
    # Run server
    # ------------------------------------------------------------------

    logger.info("=" * 60)
    logger.info("eRob MoveIt Runtime Server")
    logger.info(f"Server running on http://{host}:{port}")
    update_startup_status("http_ready", "HTTP server is ready for startup polling")
    logger.info("=" * 60)

    with open(LOG_FILE, "a") as f:
        sys.stdout = f
        sys.stderr = f
        app.run(host=host, port=port, threaded=True, use_reloader=False)
