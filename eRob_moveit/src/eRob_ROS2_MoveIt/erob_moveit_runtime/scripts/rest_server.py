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
from werkzeug.exceptions import HTTPException

from robot_controller import RobotController
from backend.backend_factory import create_robot_backend
from backend.i_robot_backend import IRobotBackend
from runtime_api.handlers import RuntimeApi
from runtime_websockets.execution_server import start_execution_websocket_server
from runtime_websockets.state_server import start_state_websocket_server
import config


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
        if robot is not None and node is not None and status.get("error") is None:
            motion_stack_ready = bool(node.is_motion_stack_ready())
            status["motion_stack_ready"] = motion_stack_ready
            if motion_stack_ready:
                status["phase"] = "ready"
                status["message"] = "Robot runtime is ready"
                status["ready"] = True
                status["motion_stack_fault"] = None
            else:
                status["phase"] = "motion_stack_warming"
                status["message"] = "Robot runtime is initialized; motion stack is still warming up"
                status["ready"] = False
                status["motion_stack_fault"] = node.get_motion_stack_fault_reason()
        return status

    def runtime_state_snapshot() -> dict:
        if robot is None or node is None:
            return {
                "runtime_ready": False,
                "runtime_initialized": False,
                "startup": get_startup_status(),
            }
        drive_status = _to_jsonable(node.get_drive_operation_status())
        motion_interlock = _to_jsonable(node.get_motion_interlock_status())
        hardware_ready = bool(node.is_hardware_ready_for_motion())
        motion_stack_ready = bool(node.is_motion_stack_ready())
        return {
            "runtime_ready": motion_stack_ready,
            "runtime_initialized": True,
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
                    "runtime_initialized",
                    "Robot runtime is initialized; waiting for motion stack",
                    ready=False,
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

    @app.errorhandler(Exception)
    def json_error_handler(exc):
        if isinstance(exc, HTTPException):
            return jsonify({
                "success": False,
                "status": "error",
                "error": exc.description,
                "code": exc.code,
                "startup": get_startup_status(),
            }), exc.code
        logger.exception("Unhandled REST exception")
        return jsonify({
            "success": False,
            "status": "error",
            "error": str(exc),
            "startup": get_startup_status(),
        }), 500

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
        # Legacy GUI clients use /health.status == "ok" as their interaction
        # gate, so only report ok once the motion stack is ready.
        health_status = "error" if status.get("error") else "ok" if status.get("ready") else status.get("phase", "starting")
        return jsonify({"status": health_status, **status}), http_status

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

    runtime_api = RuntimeApi(
        robot_getter=lambda: robot,
        node_getter=lambda: node,
        startup_status_getter=get_startup_status,
        runtime_state_snapshot_getter=runtime_state_snapshot,
        port=port,
    )

    def api_response(response):
        return jsonify(response.body), response.status

    @app.route("/move/linear", methods=["POST"])
    def move_linear():
        return api_response(runtime_api.move_linear(request.json))


    @app.route("/move/ptp", methods=["POST"])
    def move_ptp():
        return api_response(runtime_api.move_ptp(request.json))

    @app.route("/execute/path", methods=["POST"])
    def execute_path():
        return api_response(runtime_api.execute_path(request.json))

    @app.route("/execute/sequence", methods=["POST"])
    def execute_sequence():
        return api_response(runtime_api.execute_sequence(request.json))

    @app.route("/execute/ordered_motion_chain", methods=["POST"])
    def execute_ordered_motion_chain():
        return api_response(runtime_api.execute_ordered_motion_chain(request.json))

    @app.route("/unwind/joint6", methods=["POST"])
    def unwind_joint6():
        return api_response(runtime_api.unwind_joint6(request.json))

    @app.route("/safety/walls/enabled", methods=["GET"])
    def safety_walls_enabled():
        return api_response(runtime_api.safety_walls_enabled())

    @app.route("/safety/walls/status", methods=["GET"])
    def safety_walls_status():
        return api_response(runtime_api.safety_walls_status())

    @app.route("/safety/walls/enable", methods=["POST"])
    def enable_safety_walls():
        return api_response(runtime_api.enable_safety_walls())

    @app.route("/safety/walls/disable", methods=["POST"])
    def disable_safety_walls():
        return api_response(runtime_api.disable_safety_walls())

    @app.route("/position/current", methods=["GET"])
    def get_position():
        return api_response(runtime_api.current_position())

    @app.route("/position/flange", methods=["GET"])
    def get_flange_position():
        return api_response(runtime_api.flange_position())

    @app.route("/tool/registry", methods=["GET"])
    def get_tool_registry():
        return api_response(runtime_api.tool_registry())

    @app.route("/tool/active", methods=["GET"])
    def get_active_tool():
        return api_response(runtime_api.active_tool())

    @app.route("/tool/active", methods=["POST"])
    def set_active_tool():
        return api_response(runtime_api.set_active_tool(request.json))

    @app.route("/tool/registry/<int:tool_id>", methods=["POST"])
    def update_tool_registry(tool_id):
        return api_response(runtime_api.update_tool_registry(tool_id, request.json))

    @app.route("/reachability/pose", methods=["POST"])
    def validate_pose():
        return api_response(runtime_api.validate_pose(request.json))

    @app.route("/velocity/current", methods=["GET"])
    def get_velocity():
        return api_response(runtime_api.current_velocity())

    @app.route("/stop", methods=["POST"])
    def stop_motion():
        return api_response(runtime_api.stop_motion())

    @app.route("/workobject/set", methods=["POST"])
    def set_workobject():
        return api_response(runtime_api.set_workobject(request.json))

    @app.route("/status", methods=["GET"])
    def get_status():
        return api_response(runtime_api.status())

    @app.route("/execute/ordered_motion_chain/status", methods=["GET"])
    def get_ordered_motion_chain_status():
        return api_response(runtime_api.ordered_motion_chain_status())

    @app.route("/state/snapshot", methods=["GET"])
    def get_state_snapshot():
        return api_response(runtime_api.state_snapshot())

    @app.route("/state/kinematics", methods=["GET"])
    def get_state_kinematics():
        return api_response(runtime_api.state_kinematics())

    @app.route("/jog", methods=["POST"])
    def jog():
        return api_response(runtime_api.jog(request.json))

    @app.route("/io/digital_output", methods=["POST"])
    def set_digital_output():
        return api_response(runtime_api.set_digital_output(request.json))

    @app.route("/drive/enable", methods=["POST"])
    def enable_drive_operation():
        return api_response(runtime_api.enable_drive_operation())

    @app.route("/drive/disable", methods=["POST"])
    def disable_drive_operation():
        return api_response(runtime_api.disable_drive_operation())

    @app.route("/drive/status", methods=["GET"])
    def drive_operation_status():
        return api_response(runtime_api.drive_operation_status())

    @app.route("/motion/interlock/status", methods=["GET"])
    def motion_interlock_status():
        return api_response(runtime_api.motion_interlock_status())

    @app.route("/motion/interlock/reset", methods=["POST"])
    def reset_motion_interlock():
        return api_response(runtime_api.reset_motion_interlock())

    # ------------------------------------------------------------------
    # Run server
    # ------------------------------------------------------------------

    logger.info("=" * 60)
    logger.info("eRob MoveIt Runtime Server")
    logger.info(f"Server running on http://{host}:{port}")
    update_startup_status("http_ready", "HTTP server is ready for startup polling")
    start_state_websocket_server(
        robot=robot,
        node=node,
        config=config,
        fallback_host=host,
        fallback_port=port,
        robot_getter=lambda: robot,
        node_getter=lambda: node,
    )
    start_execution_websocket_server(
        robot=robot,
        node=node,
        config=config,
        fallback_host=host,
        fallback_port=port,
        robot_getter=lambda: robot,
        node_getter=lambda: node,
    )
    logger.info("=" * 60)

    with open(LOG_FILE, "a") as f:
        sys.stdout = f
        sys.stderr = f
        app.run(host=host, port=port, threaded=True, use_reloader=False)
