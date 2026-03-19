#!/usr/bin/env python3
"""
ROS2 Bridge Server - Exposes FairinoRos2Robot via REST API
"""
import logging
import sys
import os
from copy import deepcopy

import threading
import time
from flask import Flask, request, jsonify
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import interp1d
from moveit_msgs.msg import MoveItErrorCodes, RobotState

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_controller import RobotController
from fairino_ros2_robot import FairinoRos2Robot
from utils.work_object import WorkObject
from enums import RobotAxis, Direction
import config
from motion.planning.trajectory_planner import _build_cartesian_request
from motion.planning.planner_utils import _to_pose_list


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

    MOTION_ERROR_DESCRIPTIONS = {
        -1:  "Busy / invalid input / generic error",
        -2:  "MoveIt service unavailable",
        -3:  "Safety violation: target outside workspace",
        -4:  "No current robot position available",
        -5:  "Motion queue full",
        -6:  "Path planning failed: MoveIt returned no trajectory",
        -7:  "Time parameterization failed (TOTG/Ruckig)",
        -8:  "Jacobian fallback path planning failed",
        -9:  "Near-singularity detected",
        -10: "Collision detected during Jacobian check",
        -11: "Cartesian path planning failed: target unreachable, collision, or joint-limit constraint",
    }

    def motion_error_response(result):
        description = MOTION_ERROR_DESCRIPTIONS.get(result, f"Unknown error code {result}")
        http_status = 503 if result in (-2, -5) else 400 if result in (-3, -11) else 500
        return jsonify({"result": result, "success": False, "error": description}), http_status

    def _wait_future(future, timeout_s=10.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.01)
        raise TimeoutError(f"Timed out waiting for MoveIt service response after {timeout_s:.1f}s")

    def _request_ik_for_pose(pose, timeout_s=3.0, seed_joint_state=None):
        """!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # the  timeout_s provided here overrides the timeout set in kinematics.yaml !!!"""

        ik_client = node.get_ik_client()
        if ik_client is None or not ik_client.wait_for_service(timeout_sec=1.0):
            raise TimeoutError('IK service unavailable')

        from moveit_msgs.srv import GetPositionIK

        req = GetPositionIK.Request()
        req.ik_request.group_name = config.PLANNING_GROUP
        req.ik_request.ik_link_name = config.EE_LINK
        req.ik_request.pose_stamped.header.frame_id = config.BASE_LINK
        req.ik_request.pose_stamped.header.stamp = node.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = int(timeout_s)
        req.ik_request.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1_000_000_000)

        if seed_joint_state is not None:
            state = deepcopy(seed_joint_state)
            state.header.stamp = node.get_clock().now().to_msg()
            req.ik_request.robot_state.joint_state = state
            req.ik_request.robot_state.is_diff = False
        elif node.current_joint_state is not None:
            state = deepcopy(node.current_joint_state)
            state.header.stamp = node.get_clock().now().to_msg()
            req.ik_request.robot_state.joint_state = state
            req.ik_request.robot_state.is_diff = False

        future = ik_client.call_async(req)
        return _wait_future(future, timeout_s=timeout_s + 1.0)

    def _check_state_validity(joint_names, joint_positions, timeout_s=2.0):
        state_validity_client = node.get_state_validity_client()
        if state_validity_client is None or not state_validity_client.wait_for_service(timeout_sec=1.0):
            raise TimeoutError('State validity service unavailable')

        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        req = GetStateValidity.Request()
        js = JointState()
        js.name = list(joint_names)
        js.position = list(joint_positions)
        req.robot_state.joint_state = js
        req.group_name = config.PLANNING_GROUP

        future = state_validity_client.call_async(req)
        return _wait_future(future, timeout_s=timeout_s + 1.0)

    def _validate_pose_from_start(start_position, target_position, tool=0, user=0, start_joint_state_payload=None):
        if len(start_position) != 6 or len(target_position) != 6:
            return {"reachable": False, "reason": "invalid_pose_format", "fraction": 0.0}

        tool_transform = node.get_tool_transform(int(tool))
        start_base = robot.apply_workobject(start_position, user_id=int(user))
        target_base = robot.apply_workobject(target_position, user_id=int(user))

        target_poses, err = _to_pose_list(node, [target_base], tool_transform, check_last_only=True)
        if err:
            return {"reachable": False, "reason": "target_pose_safety_rejected", "fraction": 0.0, "result": err}

        start_joint_state = None
        if isinstance(start_joint_state_payload, dict):
            start_names = list(start_joint_state_payload.get("name", []) or [])
            start_positions = list(start_joint_state_payload.get("position", []) or [])
            if start_names and start_positions and len(start_names) == len(start_positions):
                from sensor_msgs.msg import JointState
                start_joint_state = JointState()
                start_joint_state.name = start_names
                start_joint_state.position = start_positions

        if start_joint_state is None:
            start_poses, err = _to_pose_list(node, [start_base], tool_transform, check_last_only=True)
            if err:
                return {"reachable": False, "reason": "start_pose_safety_rejected", "fraction": 0.0, "result": err}

            try:
                start_ik_resp = _request_ik_for_pose(start_poses[0], timeout_s=2.0)
            except Exception as exc:
                return {"reachable": False, "reason": f"ik_service_error: {exc}", "fraction": 0.0, "result": -2}

            start_ik_error = int(getattr(getattr(start_ik_resp, "error_code", None), "val", 0))
            start_solution = getattr(start_ik_resp, "solution", None)
            start_joint_state = getattr(start_solution, "joint_state", None)
            start_positions = list(getattr(start_joint_state, "position", [])) if start_joint_state is not None else []
            start_names = list(getattr(start_joint_state, "name", [])) if start_joint_state is not None else []
            if start_ik_error != MoveItErrorCodes.SUCCESS or not start_positions or not start_names:
                return {"reachable": False, "reason": "start_pose_ik_failed", "fraction": 0.0, "result": -11}

        try:
            target_ik_resp = _request_ik_for_pose(
                target_poses[0],
                timeout_s=0.1,
                seed_joint_state=start_joint_state,
            )
        except Exception as exc:
            return {"reachable": False, "reason": f"ik_service_error: {exc}", "fraction": 0.0, "result": -2}

        target_ik_error = int(getattr(getattr(target_ik_resp, "error_code", None), "val", 0))
        target_solution = getattr(target_ik_resp, "solution", None)
        target_joint_state = getattr(target_solution, "joint_state", None)
        target_positions = list(getattr(target_joint_state, "position", [])) if target_joint_state is not None else []
        target_names = list(getattr(target_joint_state, "name", [])) if target_joint_state is not None else []
        if target_ik_error != MoveItErrorCodes.SUCCESS or not target_positions or not target_names:
            return {"reachable": False, "reason": "target_pose_ik_failed", "fraction": 0.0, "result": -11}

        try:
            validity_resp = _check_state_validity(target_names, target_positions, timeout_s=2.0)
        except Exception as exc:
            return {"reachable": False, "reason": f"state_validity_error: {exc}", "fraction": 0.0, "result": -2}

        reachable = bool(getattr(validity_resp, "valid", False))
        reason = "ok" if reachable else "target_state_in_collision"
        return {
            "reachable": reachable,
            "reason": reason,
            "fraction": 1.0 if reachable else 0.0,
            "num_points": 1,
            "result": 0 if reachable else -11,
            "start_position": start_base,
            "target_position": target_base,
            "target_joint_state": {
                "name": target_names,
                "position": target_positions,
            } if reachable else None,
        }

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify({"status": "ok", "ros2_active": robot is not None})



    @app.route("/move/linear", methods=["POST"])
    def move_linear():
        data = request.json
        position = data.get("position")

        if not position or len(position) != 6:
            return jsonify({"error": "Invalid position format"}), 400

        logger.info(f"Received move/linera/ request with data {data}")

        result = robot.move_liner(
            position,
            tool=data.get("tool", 0),
            user=0,
            vel=data.get("vel", config.DEFAULT_VEL_PERCENT),
            acc=data.get("acc", config.DEFAULT_ACC_PERCENT),
            blocking=data.get("blocking", True),
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
        data = request.json
        path = data.get("path")

        if not path:
            return jsonify({"error": "No path provided"}) , 400

        # Flatten path if it's nested (client sends [[[waypoints]]])
        if path and isinstance(path, list) and len(path) > 0:
            if isinstance(path[0], list) and len(path[0]) > 0 and isinstance(path[0][0], list):
                # Path is nested: [[[wp1], [wp2], ...]] -> flatten to [[wp1], [wp2], ...]
                path = path[0]
                logger.info(f"Flattened nested path, now has {len(path)} waypoints")

        # Ensure vel and acc are floats and normalize to 0.0-1.0 range
        vel = float(data.get("vel", config.DEFAULT_VEL_SCALING))
        acc = float(data.get("acc", config.DEFAULT_ACC_SCALING))

        # If values are > 1.0, assume they're percentages (0-100) and convert to scaling factors (0.0-1.0)
        if vel > 1.0:
            vel = vel / 100.0
        if acc > 1.0:
            acc = acc / 100.0
        robot.node.get_logger().info(f"Executing path with {len(path)} waypoints, vel={vel}, acc={acc}")

        try:
            result = robot.execute_path(
                path,
                rx=data.get("rx"),
                ry=data.get("ry"),
                rz=data.get("rz"),
                vel=vel,
                acc=acc,
                blocking=data.get("blocking", False),
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

        result = _validate_pose_from_start(
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

            if result == 0:
                return jsonify({"result": result, "success": True}), 200
            else:
                return motion_error_response(result)

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
