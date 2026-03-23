#!/usr/bin/env python3
"""
ROS2 Bridge Server - Exposes FairinoRos2Robot via REST API
"""
import logging
import sys
import threading
import time

from flask import Flask, jsonify, request
import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_controller import RobotController
from fairino_ros2_robot import FairinoRos2Robot
from utils.work_object import WorkObject
import config
from rest_api_support import (
    motion_error_response,
    parse_execute_path_request,
    parse_jog_request,
    parse_move_linear_request,
    validate_pose_from_start,
)


LOG_FILE = config.REST_LOG
logger = logging.getLogger("fairino_rest_server")
logger.setLevel(logging.INFO)

if not logger.handlers:
    file_handler = logging.FileHandler(LOG_FILE)
    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s"
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

def start_rest_server(
        robot: FairinoRos2Robot | None = None,
        node: RobotController | None = None,
        host: str = config.REST_HOST,
        port: int = config.REST_PORT,
        start_ros: bool = True,
):
    """
    Start REST server.

    Modes:
    - Standalone: start_ros=True, robot=None
    - Embedded:   start_ros=False, robot provided
    """

    app = Flask(__name__)

    # ------------------------------------------------------------------
    # ROS2 initialization (standalone mode)
    # ------------------------------------------------------------------

    executor = None

    if start_ros:
        rclpy.init()
        node = RobotController()
        node.wait_for_monitor(timeout_sec=config.MONITOR_WAIT_TIMEOUT_S)

        robot = FairinoRos2Robot(ip="0.0.0.0", node=node, workobject=None)

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        def ros_spin():
            executor.spin()

        threading.Thread(target=ros_spin, daemon=True).start()

        time.sleep(1.0)

    if robot is None:
        raise RuntimeError("REST server started without a robot instance")

    # ------------------------------------------------------------------
    # Routes
    # ------------------------------------------------------------------

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify({"status": "ok", "ros2_active": robot is not None})



    @app.route("/move/linear", methods=["POST"])
    def move_linear():
        try:
            payload = parse_move_linear_request(request.json)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

        logger.info(f"Received move/linera/ request with data {request.json}")

        result = robot.move_liner(
            payload["position"],
            tool=payload["tool"],
            user=payload["user"],
            vel=payload["vel"],
            acc=payload["acc"],
            blocking=payload["blocking"],
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


    @app.route("/execute/path", methods=["POST"])
    def execute_path():
        try:
            payload = parse_execute_path_request(request.json)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

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


    @app.route("/safety/walls/enabled", methods=["GET"])
    def safety_walls_enabled():
        status = robot.get_safety_walls_status()
        return jsonify({"enabled": bool(status.get("enabled", False))})

    @app.route("/safety/walls/status", methods=["GET"])
    def safety_walls_status():
        return jsonify(robot.get_safety_walls_status())

    @app.route("/safety/walls/enable", methods=["POST"])
    def enable_safety_walls():
        status = robot.enable_safety_walls()
        return jsonify({"success": True, **status})

    @app.route("/safety/walls/disable", methods=["POST"])
    def disable_safety_walls():
        status = robot.disable_safety_walls()
        return jsonify({"success": True, **status})

    @app.route("/position/current", methods=["GET"])
    def get_position():
        pos = robot.get_current_position()
        if pos is None:
            return jsonify({"error": "Failed to get position"}), 500
        return jsonify({"position": pos})

    @app.route("/reachability/pose", methods=["POST"])
    def validate_pose():
        data = request.json or {}
        target_position = data.get("target_position") or data.get("position")
        start_position = data.get("start_position")
        if not target_position or len(target_position) != 6:
            return jsonify({"error": "Invalid target_position format"}), 400
        if start_position is None:
            start_position = robot.get_current_position()
        if not start_position or len(start_position) != 6:
            return jsonify({"error": "Invalid or unavailable start_position"}), 400

        result = validate_pose_from_start(node, robot, 
            start_position=start_position,
            target_position=target_position,
            tool=data.get("tool", 0),
            user=data.get("user", 0),
            start_joint_state_payload=data.get("start_joint_state"),
        )
        http_status = 200 if result.get("reachable") else 409 if result.get("reason") == "cartesian_path_partial" else 400
        return jsonify({"success": True, **result}), http_status

    @app.route("/velocity/current", methods=["GET"])
    def get_velocity():
        vel = robot.get_current_velocity()
        if vel is None:
            return jsonify({"error": "Failed to get velocity"}), 500
        return jsonify({"velocity": vel})

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
        data = request.json
        origin = data.get("origin")

        if not origin or len(origin) != 6:
            return jsonify({"error": "Invalid origin format"}), 400

        workobject = WorkObject(origin=origin)
        robot.set_workobject(workobject, user_id=data.get("user_id", 0))
        return jsonify({"success": True})

    @app.route("/status", methods=["GET"])
    def get_status():
        """Get robot execution status and queue size."""
        status = robot.node.status_publisher.get_status_dict()
        return jsonify(status)


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
            return jsonify({"result": -1, "success": False, "error": str(exc)})

    # ------------------------------------------------------------------
    # Run server
    # ------------------------------------------------------------------

    logger.info("=" * 60)
    logger.info("Fairino ROS2 Bridge Server")
    logger.info(f"Server running on http://{host}:{port}")
    logger.info("=" * 60)

    with open(LOG_FILE, "a") as f:
        sys.stdout = f
        sys.stderr = f
        app.run(host=host, port=port, threaded=True, use_reloader=False)
