#!/usr/bin/env python3
"""Thread-safe store for planner-facing robot state snapshots."""

from threading import Lock


class RobotStateStore:
    def __init__(self):
        self._lock = Lock()
        self._prev_cartesian = None
        self._current_joint_state = None
        self._latest_data = None

    def set_prev_cartesian(self, value):
        with self._lock:
            self._prev_cartesian = value

    def get_prev_cartesian(self):
        with self._lock:
            return self._prev_cartesian

    def set_current_joint_state(self, value):
        with self._lock:
            self._current_joint_state = value

    def get_current_joint_state(self):
        with self._lock:
            return self._current_joint_state

    def set_latest_data(self, value):
        with self._lock:
            self._latest_data = value

    def get_latest_data(self):
        with self._lock:
            if self._latest_data is None:
                return None
            return self._latest_data.copy()
