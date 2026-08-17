#!/usr/bin/env python3
from setproctitle import setproctitle

setproctitle("zeroerr_runtime-main")
import atexit
import signal
import sys
import os
import time

from threading import Event, Thread

import rclpy
import subprocess

from monitor_window import MonitorWindow
from runtime_initializer import initialize_robot_runtime
from rest.server import start_rest_server
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






def main():
    pid_file = _write_runtime_pid_file()
    atexit.register(_remove_runtime_pid_file, pid_file)
    stop_event = Event()

    def handle_shutdown_signal(signum, _frame):
        stop_event.set()
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
        robot_backend, ros_node = initialize_robot_runtime(update_status)
        runtime["robot_backend"] = robot_backend
        runtime["ros_node"] = ros_node

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
        stop_event.set()
        ros_node = runtime.get("ros_node")
        if ros_node is not None:
            ros_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
