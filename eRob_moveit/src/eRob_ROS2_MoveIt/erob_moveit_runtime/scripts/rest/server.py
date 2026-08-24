#!/usr/bin/env python3
"""
ROS2 Bridge Server - Exposes the shared MoveIt robot backend via REST API.
"""
import logging
import socket
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
from runtime_gateway.local import LocalRuntimeGateway
from runtime_websockets.execution_server import start_execution_websocket_server
from runtime_websockets.state_server import start_state_websocket_server
from rest.openapi import OPENAPI_SPEC, SWAGGER_HTML
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


def _configured_tcp_servers(host: str, port: int) -> list[tuple[str, str, int]]:
    servers = [("REST HTTP", str(host or "0.0.0.0"), int(port))]
    if bool(getattr(config, "REST_WS_STATE_ENABLED", True)):
        servers.append((
            "state WebSocket",
            str(getattr(config, "REST_WS_STATE_HOST", host) or host or "0.0.0.0"),
            int(getattr(config, "REST_WS_STATE_PORT", int(port) + 1)),
        ))
    if bool(getattr(config, "REST_WS_EXECUTION_ENABLED", True)):
        servers.append((
            "execution WebSocket",
            str(getattr(config, "REST_WS_EXECUTION_HOST", host) or host or "0.0.0.0"),
            int(getattr(config, "REST_WS_EXECUTION_PORT", int(port) + 2)),
        ))
    return servers


def _assert_tcp_servers_available(servers: list[tuple[str, str, int]]) -> None:
    seen: set[tuple[str, int]] = set()
    conflicts = []
    for name, bind_host, bind_port in servers:
        key = (bind_host, bind_port)
        if key in seen:
            continue
        seen.add(key)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind((bind_host, bind_port))
            except OSError as exc:
                conflicts.append(f"{name} {bind_host}:{bind_port} ({exc.strerror or exc})")
    if conflicts:
        conflict_text = "; ".join(conflicts)
        raise RuntimeError(
            "REST runtime cannot start because required TCP port(s) are unavailable: "
            f"{conflict_text}"
        )


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

    gateway = LocalRuntimeGateway(
        robot_getter=lambda: robot,
        node_getter=lambda: node,
    )

    try:
        _assert_tcp_servers_available(_configured_tcp_servers(host, port))
    except RuntimeError as exc:
        update_startup_status("error", str(exc), ready=False, error=str(exc))
        logger.error(str(exc))
        raise

    def get_startup_status():
        with startup_lock:
            status = dict(startup_state)
        runtime_status = gateway.startup_status()
        status["ros2_active"] = bool(runtime_status.get("ros2_active", False))
        if status.get("error") is not None:
            return status
        if runtime_status.get("ros2_active"):
            motion_stack_ready = bool(runtime_status.get("motion_stack_ready", False))
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
                status["motion_stack_fault"] = runtime_status.get("motion_stack_fault")
        return status

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
        runtime_state_snapshot_getter=gateway.runtime_state_snapshot,
        port=port,
        gateway=gateway,
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

    @app.route("/execute/ordered_motion_chain/prepare", methods=["POST"])
    def prepare_ordered_motion_chain():
        return api_response(runtime_api.prepare_ordered_motion_chain(request.json))

    @app.route("/execute/ordered_motion_chain/prepared/<plan_id>/execute", methods=["POST"])
    def execute_prepared_ordered_motion_chain(plan_id):
        return api_response(runtime_api.execute_prepared_ordered_motion_chain(plan_id))

    @app.route("/execute/ordered_motion_chain/prepared/<plan_id>", methods=["GET", "DELETE"])
    def prepared_ordered_motion_chain(plan_id):
        if request.method == "DELETE":
            return api_response(runtime_api.discard_prepared_ordered_motion_chain(plan_id))
        return api_response(runtime_api.prepared_ordered_motion_chain_status(plan_id))

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

    @app.route("/position/base_tcp", methods=["GET"])
    def get_base_tcp_position():
        return api_response(runtime_api.base_tcp_position())

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

    @app.route("/workobject/registry", methods=["GET"])
    def get_workobject_registry():
        return api_response(runtime_api.workobject_registry())

    @app.route("/workobject/registry/<int:user_id>", methods=["POST"])
    def update_workobject_registry(user_id):
        return api_response(runtime_api.update_workobject_registry(user_id, request.json))

    @app.route("/workobject/active", methods=["GET"])
    def get_active_workobject():
        return api_response(runtime_api.active_workobject())

    @app.route("/workobject/active", methods=["POST"])
    def set_active_workobject():
        return api_response(runtime_api.set_active_workobject(request.json))

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

    @app.route("/servo/cartesian/start", methods=["POST"])
    def cartesian_servo_start():
        return api_response(
            runtime_api.cartesian_servo_start(request.get_json(silent=True))
        )

    @app.route("/servo/cartesian/update", methods=["POST"])
    def cartesian_servo_update():
        return api_response(
            runtime_api.cartesian_servo_update(request.get_json(silent=True))
        )

    @app.route("/servo/cartesian/stop", methods=["POST"])
    def cartesian_servo_stop():
        return api_response(
            runtime_api.cartesian_servo_stop()
        )

    @app.route("/jog", methods=["POST"])
    def jog():
        return api_response(runtime_api.jog(request.get_json(silent=True)))

    @app.route("/jog/joint", methods=["POST"])
    def joint_jog():
        return api_response(runtime_api.joint_jog(request.get_json(silent=True)))

    @app.route("/servojog/start", methods=["POST"])
    def servo_jog_start():
        return api_response(runtime_api.servo_jog_start(request.get_json(silent=True)))

    @app.route("/servojog/stop", methods=["POST"])
    def servo_jog_stop():
        return api_response(runtime_api.servo_jog_stop())

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
