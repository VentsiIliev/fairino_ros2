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


def open_rest_logs_terminal():
    subprocess.Popen([
        "gnome-terminal",
        "--",
        "bash",
        "-c",
        f"echo 'Robot REST Server Logs'; echo; tail -f {config.REST_LOG}",
    ])


def _separate_rest_process_enabled():
    return os.environ.get("EROB_REST_SEPARATE_PROCESS", "1").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _restart_rest_process_enabled():
    return os.environ.get("EROB_REST_RESTART_ON_CRASH", "1").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _start_rest_server_process():
    rest_main = os.path.join(os.path.dirname(__file__), "rest", "main.py")
    env = dict(os.environ)
    env.setdefault("PYTHONFAULTHANDLER", "1")
    env.setdefault("PYTHONUNBUFFERED", "1")
    return subprocess.Popen([sys.executable, rest_main], env=env)


def _stop_rest_server_process(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def _supervise_rest_server_process(stop_event, process_holder):
    restart_delay_s = 2.0
    max_restart_delay_s = 30.0
    crash_window_s = 60.0
    crash_timestamps = []
    while not stop_event.is_set():
        started_at = time.monotonic()
        process = _start_rest_server_process()
        process_holder["process"] = process

        while process.poll() is None:
            if stop_event.wait(timeout=0.5):
                _stop_rest_server_process(process)
                break

        return_code = process.poll()
        process_holder["process"] = None
        if stop_event.is_set():
            break
        if return_code == 0 or not _restart_rest_process_enabled():
            break

        now = time.monotonic()
        crash_timestamps = [
            timestamp for timestamp in crash_timestamps
            if now - timestamp <= crash_window_s
        ]
        crash_timestamps.append(now)
        uptime_s = max(now - started_at, 0.0)
        if return_code is not None and return_code < 0:
            try:
                signal_name = signal.Signals(-return_code).name
            except ValueError:
                signal_name = f"signal {-return_code}"
            exit_reason = f"{return_code} ({signal_name})"
        else:
            exit_reason = str(return_code)

        print(
            "REST server exited with "
            f"code {exit_reason} after {uptime_s:.1f}s; "
            f"restart #{len(crash_timestamps)} in {restart_delay_s:.1f}s",
            file=sys.stderr,
        )
        stop_event.wait(timeout=restart_delay_s)
        if uptime_s < 30.0:
            restart_delay_s = min(restart_delay_s * 2.0, max_restart_delay_s)
        else:
            restart_delay_s = 2.0


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
    rest_process = {"process": None}

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
        if _separate_rest_process_enabled():
            if app is not None:
                Thread(
                    target=_supervise_rest_server_process,
                    args=(stop_event, rest_process),
                    daemon=True,
                ).start()
                sys.exit(app.exec())
            _supervise_rest_server_process(stop_event, rest_process)
            return

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
        _stop_rest_server_process(rest_process.get("process"))

if __name__ == '__main__':
    main()
