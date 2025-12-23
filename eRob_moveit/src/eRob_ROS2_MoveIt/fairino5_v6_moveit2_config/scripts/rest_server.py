#!/usr/bin/env python3
"""
ROS2 Bridge Server - Exposes FairinoRos2Robot via REST API
"""
import logging
import sys
import os

import threading
import time
from flask import Flask, request, jsonify

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_controller import FairinoRos2Robot, RobotController
from utils.work_object import WorkObject
from enums import RobotAxis, Direction


LOG_FILE = "/tmp/fairino_rest_server.log"
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
        host: str = "0.0.0.0",
        port: int = 5000,
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
        node.wait_for_monitor(timeout_sec=10.0)

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

    @app.route("/move/cartesian", methods=["POST"])
    def move_cartesian():
        data = request.json
        position = data.get("position")

        if not position or len(position) != 6:
            return jsonify({"error": "Invalid position format"}), 400

        # result = robot.move_cartesian(
        #     position,
        #     tool=data.get("tool", 0),
        #     # user=data.get("user", 0),
        #     user=0,# TEMP USE USER 0 BECAUSE THE CALIB AND HOME POS IN COBOT SOFT ARE CAPTURED IN FRAME 0
        #     vel=data.get("vel", 30),
        #     acc=data.get("acc", 30),
        # )

        result = robot.move_liner(
            position,
            tool=data.get("tool", 0),
            # user=data.get("user", 0),
            user=0,  # TEMP USE USER 0 BECAUSE THE CALIB AND HOME POS IN COBOT SOFT ARE CAPTURED IN FRAME 0
            vel=data.get("vel", 30),
            acc=data.get("acc", 30),
        )
        return jsonify({"result": result, "success": result == 0})

    @app.route("/move/linear", methods=["POST"])
    def move_linear():
        data = request.json
        position = data.get("position")

        if not position or len(position) != 6:
            return jsonify({"error": "Invalid position format"}), 400

        result = robot.move_liner(
            position,
            tool=data.get("tool", 0),
            # user=data.get("user", 0),
            user=0,  # TEMP USE USER 0 BECAUSE THE CALIB AND HOME POS IN COBOT SOFT ARE CAPTURED IN FRAME 0
            vel=data.get("vel", 30),
            acc=data.get("acc", 30),
        )
        return jsonify({"result": result, "success": result == 0})

    @app.route("/execute/path", methods=["POST"])
    def execute_path():
        data = request.json
        path = data.get("path")

        if not path:
            return jsonify({"error": "No path provided"}), 400

        # Flatten path if it's nested (client sends [[[waypoints]]])
        if path and isinstance(path, list) and len(path) > 0:
            if isinstance(path[0], list) and len(path[0]) > 0 and isinstance(path[0][0], list):
                # Path is nested: [[[wp1], [wp2], ...]] -> flatten to [[wp1], [wp2], ...]
                path = path[0]
                logger.info(f"Flattened nested path, now has {len(path)} waypoints")

        # Ensure vel and acc are floats and normalize to 0.0-1.0 range
        vel = float(data.get("vel", 0.6))
        acc = float(data.get("acc", 0.4))

        # If values are > 1.0, assume they're percentages (0-100) and convert to scaling factors (0.0-1.0)
        if vel > 1.0:
            vel = vel / 100.0
        if acc > 1.0:
            acc = acc / 100.0
        robot.node.get_logger().info(f"Executing path with {len(path)} waypoints, vel={vel}, acc={acc}")
        result = robot.execute_path(
            path,
            rx=data.get("rx"),
            ry=data.get("ry"),
            rz=data.get("rz"),
            vel=vel,
            acc=acc,
            blocking=data.get("blocking", False),
        )
        return jsonify({"result": result, "success": result == 0})

    @app.route("/position/current", methods=["GET"])
    def get_position():
        pos = robot.get_current_position()
        if pos is None:
            return jsonify({"error": "Failed to get position"}), 500
        return jsonify({"position": pos})

    @app.route("/velocity/current", methods=["GET"])
    def get_velocity():
        vel = robot.get_current_velocity()
        if vel is None:
            return jsonify({"error": "Failed to get velocity"}), 500
        return jsonify({"velocity": vel})

    @app.route("/stop", methods=["POST"])
    def stop_motion():
        result = robot.stop_motion()
        return jsonify({"result": result, "success": result == 0})

    @app.route("/workobject/set", methods=["POST"])
    def set_workobject():
        data = request.json
        origin = data.get("origin")

        if not origin or len(origin) != 6:
            return jsonify({"error": "Invalid origin format"}), 400

        workobject = WorkObject(origin=origin)
        robot.set_workobject(workobject, user_id=data.get("user_id", 0))
        return jsonify({"success": True})

    @app.route("/jog", methods=["POST"])
    def jog():
        data = request.json

        try:
            # Validate axis
            axis_val = data.get("axis", None)
            if axis_val is None:
                return jsonify({"result": -1, "success": False, "error": "Missing 'axis'"}), 400
            try:
                axis = RobotAxis(axis_val)
            except ValueError:
                return jsonify({"result": -1, "success": False, "error": f"Invalid 'axis': {axis_val}"}), 400

            # Validate direction
            dir_val = data.get("direction", None)
            if dir_val is None:
                return jsonify({"result": -1, "success": False, "error": "Missing 'direction'"}), 400
            try:
                direction = Direction(dir_val)
            except ValueError:
                return jsonify({"result": -1, "success": False, "error": f"Invalid 'direction': {dir_val}"}), 400

            # Validate step, vel, acc
            try:
                step = float(data.get("step"))
                vel = float(data.get("vel"))
                acc = float(data.get("acc"))
            except (TypeError, ValueError):
                return jsonify({"result": -1, "success": False, "error": "Invalid step/vel/acc"}), 400

            """ Z AXIS JOG INVERSION HANDLING """
            # Invert Z axis
            if axis == RobotAxis.Z:
                step = -step

            # Call jog
            result = robot.start_jog(axis, direction, step, vel, acc)

            return jsonify({"result": result, "success": result == 0})

        except Exception as e:
            robot.node.get_logger().error(f"Jog endpoint error: {e}")
            return jsonify({"result": -1, "success": False, "error": str(e)})

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

    app.run(host=host, port=port, threaded=True, use_reloader=False)

