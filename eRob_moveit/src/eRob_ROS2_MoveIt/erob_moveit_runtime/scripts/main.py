#!/usr/bin/env python3
import atexit
import signal
import sys
import os

from threading import Thread

import rclpy
import subprocess

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from utils.work_object import WorkObject
from monitor_window import MonitorWindow
from robot_controller import RobotController
from backend.backend_factory import create_robot_backend
from rest_server import start_rest_server
import config


def _write_runtime_pid_file():
    pid_file = os.environ.get("ZEROERR_RUNTIME_PID_FILE", "").strip()
    if not pid_file:
        return None
    with open(pid_file, "w") as f:
        f.write(f"{os.getpid()}\n")
    return pid_file


def _remove_runtime_pid_file(pid_file):
    if not pid_file:
        return
    try:
        with open(pid_file) as f:
            recorded_pid = f.read().strip()
    except OSError:
        return
    if recorded_pid == str(os.getpid()):
        try:
            os.unlink(pid_file)
        except OSError:
            pass


def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

def open_rest_logs_terminal():
    subprocess.Popen([
        "gnome-terminal",
        "--",
        "bash",
        "-c",
        f"echo 'Robot REST Server Logs'; echo; tail -f {config.REST_LOG}",
    ])



def main():
    pid_file = _write_runtime_pid_file()
    atexit.register(_remove_runtime_pid_file, pid_file)

    def handle_shutdown_signal(signum, _frame):
        rclpy.try_shutdown()
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, handle_shutdown_signal)

    headless = os.environ.get("EROB_RUNTIME_HEADLESS", "0").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    app = None
    if not headless:
        from PyQt6.QtWidgets import QApplication
        app = QApplication(sys.argv)

    runtime = {}

    def initialize_runtime(update_status):
        update_status("initializing_ros", "ROS client library is initializing")
        rclpy.init()

        update_status("constructing_robot_controller", "Robot controller is initializing")
        ros_node = RobotController()
        runtime["ros_node"] = ros_node

        # Start spinning the node before monitor-dependent setup so timer
        # callbacks and TF transforms can be processed during startup.
        update_status("spinning_ros", "ROS node is spinning")
        thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
        thread.start()

        # Load default workobject from per-robot runtime.yaml
        # (DEFAULT_WORKOBJECT: [x,y,z,rx,ry,rz])
        update_status("creating_backend", "Robot backend is being created")
        wo_params = config.DEFAULT_WORKOBJECT
        work_object = WorkObject(*wo_params) if any(v != 0 for v in wo_params) else None
        robot_backend = create_robot_backend(node=ros_node, workobject=work_object, ip="192.168.58.2")
        runtime["robot_backend"] = robot_backend

        # win = MonitorWindow(ros_node, robot_backend)
        # ros_node.ui_callback = win.ros_update
        # win.show()
        return robot_backend, ros_node

    rest_kwargs = dict(
        start_ros=False,  # runtime_initializer owns ROS initialization
        host="0.0.0.0",
        port=5000,
        runtime_initializer=initialize_runtime,
        allow_starting_without_robot=True,
    )
    try:
        if app is not None:
            rest_thread = Thread(
                target=start_rest_server,
                kwargs=rest_kwargs,
                daemon=True,
            )
            rest_thread.start()
            sys.exit(app.exec())

        start_rest_server(**rest_kwargs)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node = runtime.get("ros_node")
        if ros_node is not None:
            ros_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
