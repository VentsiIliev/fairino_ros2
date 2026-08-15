#!/usr/bin/env python3
"""Thread-safe store for planner-facing robot state snapshots."""

from threading import Lock


class RobotStateStore:
    def __init__(self):
        self._lock = Lock()
        self._robot_context = None
        self._prev_cartesian = None
        self._current_joint_state = None
        self._latest_data = None

    def bind_robot_context(self, robot_context):
        """Bind this store to exactly one runtime robot.

        The ROS graph may publish a combined JointState for several robots, but
        planner-facing state stored here is always reduced to the joint set owned
        by this runtime instance. Downstream motion code therefore operates on a
        normal single-robot state and does not need to know about other robots.
        """
        with self._lock:
            self._robot_context = robot_context
            self._current_joint_state = None

    def set_prev_cartesian(self, value):
        with self._lock:
            self._prev_cartesian = value

    def get_prev_cartesian(self):
        with self._lock:
            return self._prev_cartesian

    def set_current_joint_state(self, value):
        scoped_value = self._scope_joint_state(value)
        with self._lock:
            self._current_joint_state = scoped_value
        return scoped_value is not None

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

    def _scope_joint_state(self, value):
        if value is None:
            return None

        robot_context = self._robot_context
        joint_names = list(getattr(robot_context, "joint_names", ()) or ())
        if not joint_names:
            return value

        names = list(getattr(value, "name", []) or [])
        positions = list(getattr(value, "position", []) or [])
        if not names or len(names) != len(positions):
            return None

        index_by_name = {name: index for index, name in enumerate(names)}
        if any(name not in index_by_name for name in joint_names):
            return None

        scoped = type(value)()
        if hasattr(value, "header"):
            scoped.header = value.header
        scoped.name = list(joint_names)
        scoped.position = [
            float(positions[index_by_name[name]])
            for name in joint_names
        ]

        velocities = list(getattr(value, "velocity", []) or [])
        if len(velocities) == len(names):
            scoped.velocity = [
                float(velocities[index_by_name[name]])
                for name in joint_names
            ]

        efforts = list(getattr(value, "effort", []) or [])
        if len(efforts) == len(names):
            scoped.effort = [
                float(efforts[index_by_name[name]])
                for name in joint_names
            ]

        return scoped
