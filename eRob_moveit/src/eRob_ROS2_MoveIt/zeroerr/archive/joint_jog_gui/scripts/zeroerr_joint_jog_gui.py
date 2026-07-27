#!/usr/bin/env python3

import math
import sys
import threading
from typing import Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]
DEFAULT_ACTION = "/manipulator_controller/follow_joint_trajectory"


def duration_from_seconds(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1_000_000_000))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


class JointJogNode(Node):
    def __init__(self) -> None:
        super().__init__("zeroerr_joint_jog_gui")
        self._lock = threading.Lock()
        self._positions_by_name: Dict[str, float] = {}
        self._goal_handle = None
        self._action_name = DEFAULT_ACTION
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                positions[name] = float(position)
        if positions:
            with self._lock:
                self._positions_by_name.update(positions)

    def set_action_name(self, action_name: str) -> None:
        action_name = action_name.strip() or DEFAULT_ACTION
        if action_name == self._action_name:
            return
        self._action_name = action_name
        self._action_client.destroy()
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

    def positions(self) -> Optional[List[float]]:
        with self._lock:
            if not all(name in self._positions_by_name for name in JOINT_NAMES):
                return None
            return [self._positions_by_name[name] for name in JOINT_NAMES]

    def send_relative_move(self, joint_name: str, delta_deg: float, duration_s: float) -> str:
        positions = self.positions()
        if positions is None:
            return "Waiting for /joint_states"
        if joint_name not in JOINT_NAMES:
            return f"Unknown joint: {joint_name}"
        if not self._action_client.wait_for_server(timeout_sec=0.2):
            return f"Action server unavailable: {self._action_name}"

        target = list(positions)
        index = JOINT_NAMES.index(joint_name)
        delta_rad = math.radians(float(delta_deg))
        target[index] += delta_rad

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        start = JointTrajectoryPoint()
        start.positions = list(positions)
        start.velocities = [0.0] * len(JOINT_NAMES)
        start.accelerations = [0.0] * len(JOINT_NAMES)
        start.time_from_start = Duration(sec=0, nanosec=0)

        end = JointTrajectoryPoint()
        end.positions = target
        end.velocities = [0.0] * len(JOINT_NAMES)
        end.accelerations = [0.0] * len(JOINT_NAMES)
        end.time_from_start = duration_from_seconds(duration_s)

        trajectory.points = [start, end]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = duration_from_seconds(max(2.0, duration_s))

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f"[JointJog] {joint_name}: {positions[index]:.6f} -> {target[index]:.6f} rad "
            f"({delta_deg:+.3f} deg over {duration_s:.2f}s)"
        )
        return f"Sent {joint_name} {delta_deg:+.3f} deg"

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"[JointJog] Goal send failed: {exc}")
            return
        if not goal_handle.accepted:
            self.get_logger().error("[JointJog] Goal rejected")
            return
        self._goal_handle = goal_handle
        self.get_logger().info("[JointJog] Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._goal_handle = None
        try:
            result = future.result().result
            code = getattr(getattr(result, "error_code", None), "val", result.error_code)
            text = getattr(result, "error_string", "")
            self.get_logger().info(f"[JointJog] Goal finished: error_code={code} {text}")
        except Exception as exc:
            self.get_logger().error(f"[JointJog] Goal result failed: {exc}")

    def cancel_active_goal(self) -> str:
        if self._goal_handle is None:
            return "No active goal"
        self._goal_handle.cancel_goal_async()
        return "Cancel requested"


class JointJogWindow(QWidget):
    def __init__(self, node: JointJogNode) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("ZeroErr Joint Jog Test")
        self.setMinimumWidth(520)

        root = QVBoxLayout(self)

        action_row = QHBoxLayout()
        action_row.addWidget(QLabel("Action"))
        self.action_input = QLineEdit(DEFAULT_ACTION)
        action_row.addWidget(self.action_input, 1)
        self.apply_action_button = QPushButton("Apply")
        self.apply_action_button.clicked.connect(self._apply_action)
        action_row.addWidget(self.apply_action_button)
        root.addLayout(action_row)

        controls = QGroupBox("Single Joint Move")
        controls_layout = QGridLayout(controls)

        self.joint_combo = QComboBox()
        self.joint_combo.addItems(JOINT_NAMES)
        controls_layout.addWidget(QLabel("Joint"), 0, 0)
        controls_layout.addWidget(self.joint_combo, 0, 1)

        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(0.01, 45.0)
        self.angle_spin.setDecimals(2)
        self.angle_spin.setSingleStep(0.5)
        self.angle_spin.setValue(1.0)
        self.angle_spin.setSuffix(" deg")
        controls_layout.addWidget(QLabel("Step"), 1, 0)
        controls_layout.addWidget(self.angle_spin, 1, 1)

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.2, 30.0)
        self.duration_spin.setDecimals(2)
        self.duration_spin.setSingleStep(0.5)
        self.duration_spin.setValue(3.0)
        self.duration_spin.setSuffix(" s")
        controls_layout.addWidget(QLabel("Duration"), 2, 0)
        controls_layout.addWidget(self.duration_spin, 2, 1)

        self.minus_button = QPushButton("Move -")
        self.minus_button.clicked.connect(lambda: self._send_move(-1.0))
        controls_layout.addWidget(self.minus_button, 3, 0)

        self.plus_button = QPushButton("Move +")
        self.plus_button.clicked.connect(lambda: self._send_move(1.0))
        controls_layout.addWidget(self.plus_button, 3, 1)

        self.cancel_button = QPushButton("Cancel Active Goal")
        self.cancel_button.clicked.connect(self._cancel)
        controls_layout.addWidget(self.cancel_button, 4, 0, 1, 2)

        root.addWidget(controls)

        positions_box = QGroupBox("Current Joint Positions")
        positions_layout = QGridLayout(positions_box)
        self.position_labels = {}
        for row, joint_name in enumerate(JOINT_NAMES):
            positions_layout.addWidget(QLabel(joint_name), row, 0)
            label = QLabel("waiting")
            label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            positions_layout.addWidget(label, row, 1)
            self.position_labels[joint_name] = label
        root.addWidget(positions_box)

        self.status_label = QLabel("Waiting for /joint_states")
        root.addWidget(self.status_label)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)

    def _apply_action(self) -> None:
        self.node.set_action_name(self.action_input.text())
        self.status_label.setText(f"Action set to {self.action_input.text().strip() or DEFAULT_ACTION}")

    def _send_move(self, sign: float) -> None:
        joint_name = self.joint_combo.currentText()
        delta_deg = sign * float(self.angle_spin.value())
        duration_s = float(self.duration_spin.value())
        self.status_label.setText(self.node.send_relative_move(joint_name, delta_deg, duration_s))

    def _cancel(self) -> None:
        self.status_label.setText(self.node.cancel_active_goal())

    def _tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        positions = self.node.positions()
        if positions is None:
            return
        for joint_name, position in zip(JOINT_NAMES, positions):
            self.position_labels[joint_name].setText(
                f"{position:+.6f} rad  {math.degrees(position):+.2f} deg"
            )
        if self.status_label.text() == "Waiting for /joint_states":
            self.status_label.setText("Ready")


def main() -> None:
    rclpy.init(args=None)
    node = JointJogNode()
    app = QApplication(sys.argv)
    window = JointJogWindow(node)
    window.show()
    try:
        app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
