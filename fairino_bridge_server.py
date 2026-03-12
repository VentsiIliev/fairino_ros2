#!/usr/bin/env python3
"""
ROS2 Bridge Server - Exposes FairinoRos2Robot via REST API
Run this in your ROS2 environment, then connect from any Python project
"""
import json
import threading
from flask import Flask, request, jsonify
import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import os

# Add the scripts directory to path
sys.path.append('/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts')

from robot_controller import FairinoRos2Robot, RobotController
from utils.work_object import WorkObject

app = Flask(__name__)
robot = None
executor = None
ros_thread = None

def init_ros2():
    """Initialize ROS2 and robot controller in background thread"""
    global robot, executor

    rclpy.init()
    node = RobotController()

    # Wait for TCP transform to load
    node.wait_for_monitor(timeout_sec=10.0)

    robot = FairinoRos2Robot(ip="0.0.0.0", node=node, workobject=None)

    # Spin in executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({"status": "ok", "ros2_active": robot is not None})

@app.route('/move/cartesian', methods=['POST'])
def move_cartesian():
    """Move robot in Cartesian space (PTP)"""
    data = request.json
    position = data.get('position')  # [x, y, z, rx, ry, rz]
    tool = data.get('tool', 0)
    user = data.get('user', 0)
    vel = data.get('vel', 30)
    acc = data.get('acc', 30)

    if not position or len(position) != 6:
        return jsonify({"error": "Invalid position format"}), 400

    result = robot.move_cartesian(position, tool=tool, user=user, vel=vel, acc=acc)
    if result > 0:
        return jsonify({"result": result, "success": True, "queued": True, "queue_position": result}), 202
    elif result == 0:
        return jsonify({"result": result, "success": True, "queued": False}), 200
    elif result == -5:
        return jsonify({"result": result, "success": False, "error": "Motion queue is full"}), 503
    elif result == -2:
        return jsonify({"result": result, "success": False, "error": "MoveIt service unavailable"}), 503
    elif result == -3:
        return jsonify({"result": result, "success": False, "error": "Safety violation"}), 400
    else:
        return jsonify({"result": result, "success": False, "error": f"Move failed with code {result}"}), 500

@app.route('/move/linear', methods=['POST'])
def move_linear():
    """Move robot in linear path (LIN)"""
    data = request.json
    position = data.get('position')
    tool = data.get('tool', 0)
    user = data.get('user', 0)
    vel = data.get('vel', 30)
    acc = data.get('acc', 30)

    if not position or len(position) != 6:
        return jsonify({"error": "Invalid position format"}), 400

    result = robot.move_liner(position, tool=tool, user=user, vel=vel, acc=acc)
    if result > 0:
        return jsonify({"result": result, "success": True, "queued": True, "queue_position": result}), 202
    elif result == 0:
        return jsonify({"result": result, "success": True, "queued": False}), 200
    elif result == -5:
        return jsonify({"result": result, "success": False, "error": "Motion queue is full"}), 503
    elif result == -2:
        return jsonify({"result": result, "success": False, "error": "MoveIt service unavailable"}), 503
    elif result == -3:
        return jsonify({"result": result, "success": False, "error": "Safety violation"}), 400
    else:
        return jsonify({"result": result, "success": False, "error": f"Move failed with code {result}"}), 500

@app.route('/execute/path', methods=['POST'])
def execute_path():
    """Execute a Cartesian path (multiple waypoints)"""
    data = request.json
    path = data.get('path')  # List of [x, y, z] or [x, y, z, rx, ry, rz]
    rx = data.get('rx')
    ry = data.get('ry')
    rz = data.get('rz')
    vel = data.get('vel', 0.6)
    acc = data.get('acc', 0.4)
    blocking = data.get('blocking', False)

    if not path:
        return jsonify({"error": "No path provided"}), 400

    result = robot.execute_path(path, rx=rx, ry=ry, rz=rz, vel=vel, acc=acc, blocking=blocking)
    if result > 0:
        return jsonify({"result": result, "success": True, "queued": True, "queue_position": result}), 202
    elif result == 0:
        return jsonify({"result": result, "success": True, "queued": False}), 200
    elif result == -5:
        return jsonify({"result": result, "success": False, "error": "Motion queue is full"}), 503
    elif result == -2:
        return jsonify({"result": result, "success": False, "error": "MoveIt service unavailable"}), 503
    elif result == -3:
        return jsonify({"result": result, "success": False, "error": "Safety violation"}), 400
    else:
        return jsonify({"result": result, "success": False, "error": f"Path execution failed with code {result}"}), 500

@app.route('/position/current', methods=['GET'])
def get_position():
    """Get current robot TCP position"""
    position = robot.get_current_position()
    if position is None:
        return jsonify({"error": "Failed to get position"}), 500
    return jsonify({"position": position})

@app.route('/velocity/current', methods=['GET'])
def get_velocity():
    """Get current Cartesian velocity"""
    velocity = robot.get_current_velocity()
    if velocity is None:
        return jsonify({"error": "Failed to get velocity"}), 500
    return jsonify({"velocity": velocity})

@app.route('/stop', methods=['POST'])
def stop_motion():
    """Stop all robot motion"""
    result = robot.stop_motion()
    success = (result == 0)
    return jsonify({"stopped": success, "result": result, "success": success})

@app.route('/workobject/set', methods=['POST'])
def set_workobject():
    """Set a work object frame"""
    data = request.json
    origin = data.get('origin')  # [x, y, z, rx, ry, rz]
    user_id = data.get('user_id', 0)

    if not origin or len(origin) != 6:
        return jsonify({"error": "Invalid origin format"}), 400

    workobject = WorkObject(origin=origin)
    robot.set_workobject(workobject, user_id=user_id)
    return jsonify({"success": True})

@app.route('/jog', methods=['POST'])
def jog():
    """Jog robot in specified axis"""
    data = request.json
    axis = data.get('axis', 0)  # 0=X, 1=Y, 2=Z
    direction = data.get('direction', 1)  # 1=PLUS, -1=MINUS
    step = data.get('step', 10.0)  # mm
    vel = data.get('vel', 10.0)
    acc = data.get('acc', 10.0)

    result = robot.start_jog(axis, direction, step, vel, acc)
    if result > 0:
        return jsonify({"result": result, "success": True, "queued": True, "queue_position": result}), 202
    elif result == 0:
        return jsonify({"result": result, "success": True}), 200
    elif result == -2:
        return jsonify({"result": result, "success": False, "error": "MoveIt service unavailable"}), 503
    elif result == -3:
        return jsonify({"result": result, "success": False, "error": "Safety violation"}), 400
    elif result == -5:
        return jsonify({"result": result, "success": False, "error": "Motion queue is full"}), 503
    else:
        return jsonify({"result": result, "success": False, "error": f"Jog failed with code {result}"}), 500

if __name__ == '__main__':
    # Start ROS2 in background thread
    ros_thread = threading.Thread(target=init_ros2, daemon=True)
    ros_thread.start()

    # Wait for initialization
    import time
    time.sleep(3)

    print("=" * 60)
    print("Fairino ROS2 Bridge Server")
    print("=" * 60)
    print("Server running on http://localhost:5000")
    print("\nAvailable endpoints:")
    print("  GET  /health           - Health check")
    print("  POST /move/cartesian   - Point-to-point move")
    print("  POST /move/linear      - Linear move")
    print("  POST /execute/path     - Execute multi-waypoint path")
    print("  GET  /position/current - Get current position")
    print("  GET  /velocity/current - Get current velocity")
    print("  POST /stop             - Stop motion")
    print("  POST /workobject/set   - Set work object frame")
    print("  POST /jog              - Jog in axis")
    print("=" * 60)

    # Run Flask server
    app.run(host='0.0.0.0', port=5000, threaded=True)