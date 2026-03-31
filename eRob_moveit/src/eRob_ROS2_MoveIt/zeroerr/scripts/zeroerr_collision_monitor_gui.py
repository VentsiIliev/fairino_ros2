#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Dict
from urllib.error import URLError
from urllib.request import Request, urlopen

import rclpy
from control_msgs.msg import DynamicJointState
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QLineEdit,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


INTERFACES = [
    "drive_output_torque",
    "motor_current_a",
    "current_based_output_torque",
    "friction_torque",
    "measured_torque",
    "expected_torque",
    "torque_difference",
    "external_torque",
    "following_error_actual",
    "contact_active",
    "contact_cycles",
    "contact_latched",
    "dynamics_active",
    "dynamics_cycles",
    "dynamics_latched",
]

TABLE_COLUMNS = [
    "Joint",
    "DrvTau Nm",
    "Cur A",
    "CurTau Nm",
    "FricTau Nm",
    "MeasTau Nm",
    "ExpTau Nm",
    "DiffTau Nm",
    "ExtTau Nm",
    "FollowErr",
    "Contact",
    "Dyn",
    "Reason",
]

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
COLLISION_CONFIG_PATH = CONFIG_DIR / "collision_monitor_config.json"
DRAG_CONFIG_PATH = CONFIG_DIR / "drag_mode_config.json"


class CollisionMonitorGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("zeroerr_collision_monitor_gui")
        self.latest: Dict[str, Dict[str, float]] = {}
        self.stamp_text = "No data"
        self.config: Dict[str, object] = {
            "confirm_cycles": 1,
            "effort_thresholds": [float("nan")] * 6,
            "following_error_thresholds": [float("nan")] * 6,
            "external_torque_thresholds": [float("nan")] * 6,
            "dynamics_estimator_mode": "momentum_observer",
            "measured_torque_source": "drive_torque",
            "joint_models": ["eRob80H100T"] * 3 + ["eRob70H100T"] * 3,
            "friction_coulomb_nm": [0.0] * 6,
            "friction_viscous_nm_per_rad_s": [0.0] * 6,
            "friction_velocity_deadband_rad_s": 0.01,
            "model_names": ["eRob70H100T", "eRob80H100T"],
            "model_rated_current_ma": [3500.0, 5500.0],
            "model_output_torque_constant_nm_per_a": [4.76, 8.475],
        }
        self.config_stamp = "No config"
        self.config_pub = self.create_publisher(String, "/zeroerr/collision_monitor/config", 10)
        self.create_subscription(
            DynamicJointState,
            "/zeroerr/collision_monitor/state",
            self._on_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/zeroerr/collision_monitor/config_state",
            self._on_config_state,
            10,
        )

    def _on_state(self, msg: DynamicJointState) -> None:
        latest: Dict[str, Dict[str, float]] = {}
        for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
            joint_state: Dict[str, float] = {}
            for name, value in zip(interface_value.interface_names, interface_value.values):
                joint_state[name] = float(value)
            latest[joint_name] = joint_state
        self.latest = latest
        self.stamp_text = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

    def _on_config_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.config = payload
        self.config_stamp = "synced"

    def publish_config(self, config: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(config, sort_keys=True)
        self.config_pub.publish(msg)


class CollisionMonitorWindow(QMainWindow):
    def __init__(self, ros_node: CollisionMonitorGuiNode) -> None:
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("ZeroErr Collision Monitor")
        self.resize(980, 420)

        central = QWidget()
        layout = QVBoxLayout(central)

        header = QHBoxLayout()
        self.status_label = QLabel("Waiting for /zeroerr/collision_monitor/state")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.status_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        header.addWidget(self.status_label)
        header.addStretch()
        self.stamp_label = QLabel("No data")
        self.stamp_label.setStyleSheet("font-family: monospace; color: #555;")
        header.addWidget(self.stamp_label)
        layout.addLayout(header)

        tabs = QTabWidget()
        collision_tab = QWidget()
        collision_layout = QVBoxLayout(collision_tab)
        drag_tab = QWidget()
        drag_layout_root = QVBoxLayout(drag_tab)

        threshold_group = QGroupBox("Detector Config")
        threshold_layout = QGridLayout(threshold_group)
        threshold_layout.addWidget(QLabel("Joint"), 0, 0)
        threshold_layout.addWidget(QLabel("ContactTau"), 0, 1)
        threshold_layout.addWidget(QLabel("FollowErr"), 0, 2)
        threshold_layout.addWidget(QLabel("ExtTau"), 0, 3)
        threshold_layout.addWidget(QLabel("FricC"), 0, 4)
        threshold_layout.addWidget(QLabel("FricV"), 0, 5)
        threshold_layout.addWidget(QLabel("Model"), 0, 6)
        self.threshold_inputs: Dict[str, Dict[str, QLineEdit]] = {}
        for row in range(6):
            joint_name = f"Joint_{row + 1}"
            threshold_layout.addWidget(QLabel(joint_name), row + 1, 0)
            self.threshold_inputs[joint_name] = {}
            for column, key in enumerate(
                [
                    "effort",
                    "following_error",
                    "external_torque",
                    "friction_coulomb",
                    "friction_viscous",
                    "joint_model",
                ],
                start=1,
            ):
                edit = QLineEdit()
                edit.setMaximumWidth(80)
                threshold_layout.addWidget(edit, row + 1, column)
                self.threshold_inputs[joint_name][key] = edit

        threshold_layout.addWidget(QLabel("Confirm"), 7, 0)
        self.confirm_cycles_input = QLineEdit()
        self.confirm_cycles_input.setMaximumWidth(80)
        threshold_layout.addWidget(self.confirm_cycles_input, 7, 1)
        threshold_layout.addWidget(QLabel("Estimator"), 7, 2)
        self.dynamics_estimator_mode_input = QLineEdit()
        self.dynamics_estimator_mode_input.setMaximumWidth(120)
        threshold_layout.addWidget(self.dynamics_estimator_mode_input, 7, 3)
        threshold_layout.addWidget(QLabel("TorqueSrc"), 7, 4)
        self.measured_torque_source_input = QLineEdit()
        self.measured_torque_source_input.setMaximumWidth(140)
        threshold_layout.addWidget(self.measured_torque_source_input, 7, 5)
        threshold_layout.addWidget(QLabel("Deadband"), 7, 6)
        self.friction_deadband_input = QLineEdit()
        self.friction_deadband_input.setMaximumWidth(80)
        threshold_layout.addWidget(self.friction_deadband_input, 7, 7)
        self.apply_button = QPushButton("Apply Config")
        self.apply_button.clicked.connect(self._apply_config)
        threshold_layout.addWidget(self.apply_button, 8, 4, 1, 2)
        self.capture_baseline_button = QPushButton("Capture Baseline")
        self.capture_baseline_button.clicked.connect(self._start_baseline_capture)
        threshold_layout.addWidget(self.capture_baseline_button, 8, 0, 1, 2)
        self.baseline_status_label = QLabel("Baseline idle")
        self.baseline_status_label.setStyleSheet("color: #555;")
        threshold_layout.addWidget(self.baseline_status_label, 8, 2, 1, 2)
        collision_layout.addWidget(threshold_group)

        model_group = QGroupBox("Model Constants")
        model_layout = QGridLayout(model_group)
        model_layout.addWidget(QLabel("Model"), 0, 0)
        model_layout.addWidget(QLabel("Rated Current mA"), 0, 1)
        model_layout.addWidget(QLabel("Output Kt Nm/A"), 0, 2)
        self.model_inputs: Dict[str, Dict[str, QLineEdit]] = {}
        for row, model_name in enumerate(["eRob70H100T", "eRob80H100T"], start=1):
            model_layout.addWidget(QLabel(model_name), row, 0)
            self.model_inputs[model_name] = {}
            for column, key in enumerate(["rated_current_ma", "output_torque_constant"], start=1):
                edit = QLineEdit()
                edit.setMaximumWidth(120)
                model_layout.addWidget(edit, row, column)
                self.model_inputs[model_name][key] = edit
        collision_layout.addWidget(model_group)

        summary_group = QGroupBox("Detector Summary")
        summary_layout = QGridLayout(summary_group)
        self.summary_labels: Dict[str, QLabel] = {}
        for column, key in enumerate(["contact_active", "contact_latched", "dynamics_active", "dynamics_latched"]):
            title = key.replace("_", " ").title()
            label = QLabel("0")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("font-size: 20px; font-weight: bold; padding: 6px;")
            summary_layout.addWidget(QLabel(title), 0, column)
            summary_layout.addWidget(label, 1, column)
            self.summary_labels[key] = label
        collision_layout.addWidget(summary_group)

        self.table = QTableWidget(6, len(TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels(TABLE_COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.table.setAlternatingRowColors(True)
        collision_layout.addWidget(self.table)

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setPlaceholderText("Collision and drag event log")
        self.event_log.setMinimumHeight(140)
        collision_layout.addWidget(self.event_log)

        drag_group = QGroupBox("Drag Mode Control")
        drag_layout = QHBoxLayout(drag_group)
        self.drag_enable_button = QPushButton("Enable Drag")
        self.drag_enable_button.clicked.connect(self._enable_drag_mode)
        drag_layout.addWidget(self.drag_enable_button)
        self.drag_disable_button = QPushButton("Disable Drag")
        self.drag_disable_button.clicked.connect(self._disable_drag_mode)
        drag_layout.addWidget(self.drag_disable_button)
        self.drag_status_label = QLabel("Unknown")
        self.drag_status_label.setStyleSheet("font-weight: bold; color: #555;")
        drag_layout.addWidget(self.drag_status_label)
        drag_layout.addStretch()
        drag_layout_root.addWidget(drag_group)

        drag_config_group = QGroupBox("Drag Runtime Config")
        drag_config_layout = QGridLayout(drag_config_group)
        drag_config_layout.addWidget(QLabel("Joint"), 0, 0)
        drag_config_layout.addWidget(QLabel("JointScale"), 0, 1)
        drag_config_layout.addWidget(QLabel("Damping"), 0, 2)
        drag_config_layout.addWidget(QLabel("MaxEffort"), 0, 3)
        drag_config_layout.addWidget(QLabel("MaxOffset"), 0, 4)
        self.drag_inputs: Dict[str, Dict[str, QLineEdit]] = {}
        for row in range(6):
            joint_name = f"Joint_{row + 1}"
            drag_config_layout.addWidget(QLabel(joint_name), row + 1, 0)
            self.drag_inputs[joint_name] = {}
            for column, key in enumerate(["joint_scale", "damping", "max_effort", "max_offset"], start=1):
                edit = QLineEdit()
                edit.setMaximumWidth(90)
                drag_config_layout.addWidget(edit, row + 1, column)
                self.drag_inputs[joint_name][key] = edit

        drag_config_layout.addWidget(QLabel("CompScale"), 7, 0)
        self.drag_compensation_scale_input = QLineEdit()
        self.drag_compensation_scale_input.setMaximumWidth(90)
        drag_config_layout.addWidget(self.drag_compensation_scale_input, 7, 1)

        drag_config_layout.addWidget(QLabel("Settle s"), 7, 3)
        self.drag_settle_timeout_input = QLineEdit()
        self.drag_settle_timeout_input.setMaximumWidth(90)
        drag_config_layout.addWidget(self.drag_settle_timeout_input, 7, 4)

        drag_config_layout.addWidget(QLabel("DisablePulse s"), 8, 0)
        self.drag_disable_pulse_input = QLineEdit()
        self.drag_disable_pulse_input.setMaximumWidth(90)
        drag_config_layout.addWidget(self.drag_disable_pulse_input, 8, 1)

        drag_config_layout.addWidget(QLabel("EnablePulse s"), 8, 3)
        self.drag_enable_pulse_input = QLineEdit()
        self.drag_enable_pulse_input.setMaximumWidth(90)
        drag_config_layout.addWidget(self.drag_enable_pulse_input, 8, 4)

        self.apply_drag_button = QPushButton("Apply Drag Config")
        self.apply_drag_button.clicked.connect(self._apply_drag_config)
        drag_config_layout.addWidget(self.apply_drag_button, 9, 3, 1, 2)
        drag_layout_root.addWidget(drag_config_group)

        drag_help = QLabel(
            "Use this tab for CST drag tuning. CompensationScale and MaxOffset affect support against gravity first."
        )
        drag_help.setWordWrap(True)
        drag_help.setStyleSheet("color: #555;")
        drag_layout_root.addWidget(drag_help)
        drag_layout_root.addStretch()

        tabs.addTab(collision_tab, "Collision")
        tabs.addTab(drag_tab, "Drag")
        layout.addWidget(tabs)

        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        self.drag_status_timer = QTimer(self)
        self.drag_status_timer.timeout.connect(self._poll_drag_status)
        self.drag_status_timer.start(1000)
        self._config_loaded = False
        self._last_joint_states: Dict[str, str] = {}
        self._max_log_lines = 250
        self._baseline_capture_active = False
        self._baseline_capture_started_at = 0.0
        self._baseline_capture_duration_s = 5.0
        self._baseline_ext_tau_max = {f"Joint_{index}": 0.0 for index in range(1, 7)}
        self._drag_endpoint = "http://localhost:5000"
        self._drag_enabled = None
        self._drag_config_loaded = False
        self._collision_persisted_loaded = False

    def _refresh(self) -> None:
        data = self.ros_node.latest
        self._refresh_config_inputs()
        if not data:
            return

        self._update_baseline_capture(data)

        joints = sorted(data.keys(), key=lambda name: int(name.split("_")[1]))
        self.table.setRowCount(len(joints))

        summary_counts = {
            "contact_active": 0,
            "contact_latched": 0,
            "dynamics_active": 0,
            "dynamics_latched": 0,
        }

        for row, joint_name in enumerate(joints):
            joint = data[joint_name]
            self._update_event_log(joint_name, joint)
            row_values = [
                joint_name,
                self._fmt(joint.get("drive_output_torque"), 2),
                self._fmt(joint.get("motor_current_a"), 3),
                self._fmt(joint.get("current_based_output_torque"), 2),
                self._fmt(joint.get("friction_torque"), 2),
                self._fmt(joint.get("measured_torque"), 2),
                self._fmt(joint.get("expected_torque"), 2),
                self._fmt(joint.get("torque_difference"), 2),
                self._fmt(joint.get("external_torque"), 2),
                self._fmt(joint.get("following_error_actual"), 0),
                self._state_text(joint.get("contact_active"), joint.get("contact_cycles"), joint.get("contact_latched")),
                self._state_text(joint.get("dynamics_active"), joint.get("dynamics_cycles"), joint.get("dynamics_latched")),
                self._reason_text(joint_name, joint),
            ]

            row_color = self._row_color(joint)
            for column, value in enumerate(row_values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if row_color is not None:
                    item.setBackground(row_color)
                self.table.setItem(row, column, item)

            for key in summary_counts:
                if self._is_true(joint.get(key)):
                    summary_counts[key] += 1

        for key, label in self.summary_labels.items():
            count = summary_counts[key]
            label.setText(str(count))
            if count > 0:
                label.setStyleSheet(
                    "font-size: 20px; font-weight: bold; padding: 6px; color: white; background: #c62828;"
                )
            else:
                label.setStyleSheet(
                    "font-size: 20px; font-weight: bold; padding: 6px; color: #1b5e20; background: #e8f5e9;"
                )

        if summary_counts["contact_latched"] or summary_counts["dynamics_latched"]:
            self.status_label.setText("Collision detector latched")
            self.status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #b71c1c;")
        elif summary_counts["contact_active"] or summary_counts["dynamics_active"]:
            self.status_label.setText("Collision suspect active")
            self.status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #ef6c00;")
        else:
            self.status_label.setText("Collision detector clear")
            self.status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #1b5e20;")

        self.stamp_label.setText(self.ros_node.stamp_text)
        self.table.resizeColumnsToContents()

    def _start_baseline_capture(self) -> None:
        self._baseline_capture_active = True
        self._baseline_capture_started_at = time.time()
        self._baseline_ext_tau_max = {f"Joint_{index}": 0.0 for index in range(1, 7)}
        self.baseline_status_label.setText("Capturing baseline...")
        self.baseline_status_label.setStyleSheet("color: #1565c0; font-weight: bold;")
        self.event_log.append(
            f"[{time.strftime('%H:%M:%S')}] Baseline capture started ({self._baseline_capture_duration_s:.0f}s)"
        )

    def _update_baseline_capture(self, data: Dict[str, Dict[str, float]]) -> None:
        if not self._baseline_capture_active:
            return

        elapsed = time.time() - self._baseline_capture_started_at
        remaining = max(0.0, self._baseline_capture_duration_s - elapsed)
        self.baseline_status_label.setText(f"Capturing baseline... {remaining:.1f}s")

        for joint_name, joint in data.items():
            ext_tau = abs(joint.get("external_torque", 0.0))
            if not math.isnan(ext_tau):
                self._baseline_ext_tau_max[joint_name] = max(self._baseline_ext_tau_max[joint_name], ext_tau)

        if elapsed < self._baseline_capture_duration_s:
            return

        self._baseline_capture_active = False
        suggestions = []
        for joint_name in [f"Joint_{index}" for index in range(1, 7)]:
            baseline = self._baseline_ext_tau_max.get(joint_name, 0.0)
            suggested = max(1.0, baseline * 1.5 + 1.0)
            self.threshold_inputs[joint_name]["external_torque"].setText(f"{suggested:.2f}")
            suggestions.append(f"{joint_name}={suggested:.2f}")

        self.baseline_status_label.setText("Baseline applied to ExtTau fields")
        self.baseline_status_label.setStyleSheet("color: #1b5e20; font-weight: bold;")
        self.event_log.append(
            f"[{time.strftime('%H:%M:%S')}] Baseline capture finished | "
            + ", ".join(suggestions)
        )
        self._trim_event_log()

    def _drag_request(self, path: str, method: str = "GET") -> tuple[dict | None, str | None]:
        request = Request(f"{self._drag_endpoint}{path}", method=method)
        try:
            with urlopen(request, timeout=5.0) as response:
                payload = response.read().decode("utf-8")
                return (json.loads(payload) if payload else {}), None
        except (URLError, TimeoutError, json.JSONDecodeError) as exc:
            return None, str(exc)

    def _enable_drag_mode(self) -> None:
        response, error = self._drag_request("/drag/enable", method="POST")
        timestamp = time.strftime("%H:%M:%S")
        if response is None:
            suffix = f" | {error}" if error else ""
            self.event_log.append(f"[{timestamp}] Drag enable failed{suffix}")
            self.drag_status_label.setText("Unavailable")
            self.drag_status_label.setStyleSheet("font-weight: bold; color: #b71c1c;")
        else:
            self.event_log.append(f"[{timestamp}] Drag enable requested")
            self._update_drag_status_from_payload(response)
        self._trim_event_log()

    def _disable_drag_mode(self) -> None:
        response, error = self._drag_request("/drag/disable", method="POST")
        timestamp = time.strftime("%H:%M:%S")
        if response is None:
            suffix = f" | {error}" if error else ""
            self.event_log.append(f"[{timestamp}] Drag disable failed{suffix}")
            self.drag_status_label.setText("Unavailable")
            self.drag_status_label.setStyleSheet("font-weight: bold; color: #b71c1c;")
        else:
            self.event_log.append(f"[{timestamp}] Drag disable requested")
            self._update_drag_status_from_payload(response)
        self._trim_event_log()

    def _poll_drag_status(self) -> None:
        response, error = self._drag_request("/drag/status", method="GET")
        if response is None:
            self.drag_status_label.setText("Unavailable")
            self.drag_status_label.setStyleSheet("font-weight: bold; color: #b71c1c;")
            if error:
                self.event_log.append(f"[{time.strftime('%H:%M:%S')}] Drag status unavailable | {error}")
                self._trim_event_log()
            self._drag_enabled = None
            return
        self._update_drag_status_from_payload(response)
        self._poll_drag_config()

    def _update_drag_status_from_payload(self, payload: dict) -> None:
        enabled = bool(payload.get("enabled", False))
        if enabled:
            self.drag_status_label.setText("Enabled")
            self.drag_status_label.setStyleSheet("font-weight: bold; color: #1b5e20;")
        else:
            self.drag_status_label.setText("Disabled")
            self.drag_status_label.setStyleSheet("font-weight: bold; color: #555;")
        self._drag_enabled = enabled

    def _poll_drag_config(self) -> None:
        response, error = self._drag_request("/drag/config", method="GET")
        if response is None:
            return
        self._load_drag_config_inputs(response)

    def _load_drag_config_inputs(self, payload: dict) -> None:
        if self._drag_config_loaded:
            return
        joint_scale = payload.get("joint_compensation_scale", [])
        damping = payload.get("damping_nm_per_rad_s", [])
        max_effort = payload.get("max_effort_nm", [])
        max_offset = payload.get("max_torque_offset_nm", [])
        for row, joint_name in enumerate([f"Joint_{index}" for index in range(1, 7)]):
            if row < len(joint_scale):
                self.drag_inputs[joint_name]["joint_scale"].setText(str(joint_scale[row]))
            if row < len(damping):
                self.drag_inputs[joint_name]["damping"].setText(str(damping[row]))
            if row < len(max_effort):
                self.drag_inputs[joint_name]["max_effort"].setText(str(max_effort[row]))
            if row < len(max_offset):
                self.drag_inputs[joint_name]["max_offset"].setText(str(max_offset[row]))
        self.drag_compensation_scale_input.setText(str(payload.get("compensation_scale", 1.0)))
        self.drag_settle_timeout_input.setText(str(payload.get("settle_timeout_s", 2.0)))
        self.drag_disable_pulse_input.setText(str(payload.get("disable_pulse_s", 0.1)))
        self.drag_enable_pulse_input.setText(str(payload.get("enable_pulse_s", 0.1)))
        self._drag_config_loaded = True

    def _save_json(self, path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

    def _apply_drag_config(self) -> None:
        try:
            payload = {
                "compensation_scale": float(self.drag_compensation_scale_input.text()),
                "settle_timeout_s": float(self.drag_settle_timeout_input.text()),
                "disable_pulse_s": float(self.drag_disable_pulse_input.text()),
                "enable_pulse_s": float(self.drag_enable_pulse_input.text()),
                "joint_compensation_scale": [],
                "damping_nm_per_rad_s": [],
                "max_effort_nm": [],
                "max_torque_offset_nm": [],
            }
            for joint_name in [f"Joint_{index}" for index in range(1, 7)]:
                payload["joint_compensation_scale"].append(float(self.drag_inputs[joint_name]["joint_scale"].text()))
                payload["damping_nm_per_rad_s"].append(float(self.drag_inputs[joint_name]["damping"].text()))
                payload["max_effort_nm"].append(float(self.drag_inputs[joint_name]["max_effort"].text()))
                payload["max_torque_offset_nm"].append(float(self.drag_inputs[joint_name]["max_offset"].text()))
        except ValueError:
            self.event_log.append(f"[{time.strftime('%H:%M:%S')}] Drag config invalid")
            self._trim_event_log()
            return

        request = Request(
            f"{self._drag_endpoint}/drag/config",
            data=json.dumps(payload).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with urlopen(request, timeout=5.0) as resp:
                payload_response = json.loads(resp.read().decode("utf-8") or "{}")
                self._drag_config_loaded = False
                self._load_drag_config_inputs(payload_response)
                self._save_json(DRAG_CONFIG_PATH, payload)
                self.event_log.append(f"[{time.strftime('%H:%M:%S')}] Drag config applied")
        except Exception as exc:
            self.event_log.append(f"[{time.strftime('%H:%M:%S')}] Drag config apply failed | {exc}")
        self._trim_event_log()

    def _update_event_log(self, joint_name: str, joint: Dict[str, float]) -> None:
        current_state = self._event_state(joint)
        previous_state = self._last_joint_states.get(joint_name, "")
        if current_state == previous_state:
            return

        self._last_joint_states[joint_name] = current_state
        timestamp = time.strftime("%H:%M:%S")
        reason = self._reason_text(joint_name, joint)
        if not current_state:
            message = f"[{timestamp}] {joint_name} CLEAR"
        else:
            message = f"[{timestamp}] {joint_name} {current_state}"
            if reason:
                message += f" | {reason}"
            details = self._detail_text(joint_name, joint)
            if details:
                message += f" | {details}"

        self.event_log.append(message)
        self._trim_event_log()

    def _trim_event_log(self) -> None:
        lines = self.event_log.toPlainText().splitlines()
        if len(lines) <= self._max_log_lines:
            return
        self.event_log.setPlainText("\n".join(lines[-self._max_log_lines:]))
        cursor = self.event_log.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        self.event_log.setTextCursor(cursor)

    def _event_state(self, joint: Dict[str, float]) -> str:
        if self._is_true(joint.get("dynamics_latched")):
            return "DYNAMICS LATCHED"
        if self._is_true(joint.get("contact_latched")):
            return "CONTACT LATCHED"
        if self._is_true(joint.get("dynamics_active")):
            return "DYNAMICS ACTIVE"
        if self._is_true(joint.get("contact_active")):
            return "CONTACT ACTIVE"
        return ""

    def _refresh_config_inputs(self) -> None:
        if not self._collision_persisted_loaded:
            self._load_collision_persisted_inputs()
        config = self.ros_node.config
        if not config:
            return
        if self._config_loaded:
            return

        for row, joint_name in enumerate([f"Joint_{index}" for index in range(1, 7)]):
            effort_values = config.get("effort_thresholds", [])
            follow_values = config.get("following_error_thresholds", [])
            ext_values = config.get("external_torque_thresholds", [])
            friction_c_values = config.get("friction_coulomb_nm", [])
            friction_v_values = config.get("friction_viscous_nm_per_rad_s", [])
            joint_models = config.get("joint_models", [])
            if row < len(effort_values):
                self.threshold_inputs[joint_name]["effort"].setText(str(effort_values[row]))
            if row < len(follow_values):
                self.threshold_inputs[joint_name]["following_error"].setText(str(follow_values[row]))
            if row < len(ext_values):
                self.threshold_inputs[joint_name]["external_torque"].setText(str(ext_values[row]))
            if row < len(friction_c_values):
                self.threshold_inputs[joint_name]["friction_coulomb"].setText(str(friction_c_values[row]))
            if row < len(friction_v_values):
                self.threshold_inputs[joint_name]["friction_viscous"].setText(str(friction_v_values[row]))
            if row < len(joint_models):
                self.threshold_inputs[joint_name]["joint_model"].setText(str(joint_models[row]))

        self.confirm_cycles_input.setText(str(config.get("confirm_cycles", 1)))
        self.dynamics_estimator_mode_input.setText(str(config.get("dynamics_estimator_mode", "momentum_observer")))
        self.measured_torque_source_input.setText(str(config.get("measured_torque_source", "drive_torque")))
        self.friction_deadband_input.setText(str(config.get("friction_velocity_deadband_rad_s", 0.01)))
        model_names = config.get("model_names", [])
        rated_currents = config.get("model_rated_current_ma", [])
        output_constants = config.get("model_output_torque_constant_nm_per_a", [])
        for row, model_name in enumerate(model_names):
            if model_name not in self.model_inputs:
                continue
            if row < len(rated_currents):
                self.model_inputs[model_name]["rated_current_ma"].setText(str(rated_currents[row]))
            if row < len(output_constants):
                self.model_inputs[model_name]["output_torque_constant"].setText(str(output_constants[row]))
        self._config_loaded = True

    def _load_collision_persisted_inputs(self) -> None:
        self._collision_persisted_loaded = True
        if not COLLISION_CONFIG_PATH.exists():
            return
        try:
            config = json.loads(COLLISION_CONFIG_PATH.read_text(encoding="utf-8"))
        except Exception:
            return

        for row, joint_name in enumerate([f"Joint_{index}" for index in range(1, 7)]):
            effort_values = config.get("effort_thresholds", [])
            follow_values = config.get("following_error_thresholds", [])
            ext_values = config.get("external_torque_thresholds", [])
            friction_c_values = config.get("friction_coulomb_nm", [])
            friction_v_values = config.get("friction_viscous_nm_per_rad_s", [])
            joint_models = config.get("joint_models", [])
            if row < len(effort_values):
                self.threshold_inputs[joint_name]["effort"].setText(str(effort_values[row]))
            if row < len(follow_values):
                self.threshold_inputs[joint_name]["following_error"].setText(str(follow_values[row]))
            if row < len(ext_values):
                self.threshold_inputs[joint_name]["external_torque"].setText(str(ext_values[row]))
            if row < len(friction_c_values):
                self.threshold_inputs[joint_name]["friction_coulomb"].setText(str(friction_c_values[row]))
            if row < len(friction_v_values):
                self.threshold_inputs[joint_name]["friction_viscous"].setText(str(friction_v_values[row]))
            if row < len(joint_models):
                self.threshold_inputs[joint_name]["joint_model"].setText(str(joint_models[row]))

        self.confirm_cycles_input.setText(str(config.get("confirm_cycles", 1)))
        self.dynamics_estimator_mode_input.setText(str(config.get("dynamics_estimator_mode", "momentum_observer")))
        self.measured_torque_source_input.setText(str(config.get("measured_torque_source", "drive_torque")))
        self.friction_deadband_input.setText(str(config.get("friction_velocity_deadband_rad_s", 0.01)))
        model_names = config.get("model_names", [])
        rated_currents = config.get("model_rated_current_ma", [])
        output_constants = config.get("model_output_torque_constant_nm_per_a", [])
        for row, model_name in enumerate(model_names):
            if model_name not in self.model_inputs:
                continue
            if row < len(rated_currents):
                self.model_inputs[model_name]["rated_current_ma"].setText(str(rated_currents[row]))
            if row < len(output_constants):
                self.model_inputs[model_name]["output_torque_constant"].setText(str(output_constants[row]))

    def _apply_config(self) -> None:
        try:
            config = {
                "confirm_cycles": int(float(self.confirm_cycles_input.text())),
                "effort_thresholds": [],
                "following_error_thresholds": [],
                "external_torque_thresholds": [],
                "friction_coulomb_nm": [],
                "friction_viscous_nm_per_rad_s": [],
                "joint_models": [],
                "dynamics_estimator_mode": self.dynamics_estimator_mode_input.text().strip(),
                "measured_torque_source": self.measured_torque_source_input.text().strip(),
                "friction_velocity_deadband_rad_s": float(self.friction_deadband_input.text()),
                "model_names": [],
                "model_rated_current_ma": [],
                "model_output_torque_constant_nm_per_a": [],
            }
            for joint_name in [f"Joint_{index}" for index in range(1, 7)]:
                config["effort_thresholds"].append(
                    float(self.threshold_inputs[joint_name]["effort"].text())
                )
                config["following_error_thresholds"].append(
                    float(self.threshold_inputs[joint_name]["following_error"].text())
                )
                config["external_torque_thresholds"].append(
                    float(self.threshold_inputs[joint_name]["external_torque"].text())
                )
                config["friction_coulomb_nm"].append(
                    float(self.threshold_inputs[joint_name]["friction_coulomb"].text())
                )
                config["friction_viscous_nm_per_rad_s"].append(
                    float(self.threshold_inputs[joint_name]["friction_viscous"].text())
                )
                config["joint_models"].append(
                    self.threshold_inputs[joint_name]["joint_model"].text().strip()
                )
            for model_name in ["eRob70H100T", "eRob80H100T"]:
                config["model_names"].append(model_name)
                config["model_rated_current_ma"].append(
                    float(self.model_inputs[model_name]["rated_current_ma"].text())
                )
                config["model_output_torque_constant_nm_per_a"].append(
                    float(self.model_inputs[model_name]["output_torque_constant"].text())
                )
        except ValueError:
            self.status_label.setText("Invalid config value")
            self.status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #b71c1c;")
            return

        self.ros_node.publish_config(config)
        self._save_json(COLLISION_CONFIG_PATH, config)
        self.status_label.setText("Runtime config update sent")
        self.status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #1565c0;")

    def _row_color(self, joint: Dict[str, float]) -> QColor | None:
        if self._is_true(joint.get("contact_latched")) or self._is_true(joint.get("dynamics_latched")):
            return QColor("#ffcdd2")
        if self._is_true(joint.get("contact_active")) or self._is_true(joint.get("dynamics_active")):
            return QColor("#ffe0b2")
        return None

    def _fmt(self, value: float | None, decimals: int) -> str:
        if value is None or (isinstance(value, float) and math.isnan(value)):
            return "NA"
        return f"{value:.{decimals}f}"

    def _fmt_hex(self, value: float | None) -> str:
        if value is None or (isinstance(value, float) and math.isnan(value)):
            return "NA"
        return f"0x{int(value):04X}"

    def _is_true(self, value: float | None) -> bool:
        return value is not None and not math.isnan(value) and value >= 0.5

    def _state_text(self, active: float | None, cycles: float | None, latched: float | None) -> str:
        if self._is_true(latched):
            return f"LATCH {int(cycles or 0)}"
        if self._is_true(active):
            return f"ACTIVE {int(cycles or 0)}"
        return "-"

    def _reason_text(self, joint_name: str, joint: Dict[str, float]) -> str:
        joint_index = int(joint_name.split("_")[1]) - 1
        effort_thresholds = self.ros_node.config.get("effort_thresholds", [])
        following_thresholds = self.ros_node.config.get("following_error_thresholds", [])
        external_thresholds = self.ros_node.config.get("external_torque_thresholds", [])
        effort_threshold = effort_thresholds[joint_index] if joint_index < len(effort_thresholds) else float("nan")
        following_threshold = following_thresholds[joint_index] if joint_index < len(following_thresholds) else float("nan")
        external_threshold = external_thresholds[joint_index] if joint_index < len(external_thresholds) else float("nan")
        effort = abs(joint.get("measured_torque", float("nan")))
        following_error = abs(joint.get("following_error_actual", float("nan")))
        external_torque = abs(joint.get("external_torque", float("nan")))

        if self._is_true(joint.get("dynamics_latched")) or self._is_true(joint.get("dynamics_active")):
            return f"ext_tau {external_torque:.2f} > {float(external_threshold):.2f}"
        if self._is_true(joint.get("contact_latched")) or self._is_true(joint.get("contact_active")):
            return (
                f"meas_tau {effort:.1f} > {float(effort_threshold):.1f} and "
                f"foll_err {following_error:.0f} > {float(following_threshold):.0f}"
            )
        return (
            f"ext_tau {external_torque:.2f} / {float(external_threshold):.2f}, "
            f"meas_tau {effort:.1f} / {float(effort_threshold):.1f}, "
            f"foll_err {following_error:.0f} / {float(following_threshold):.0f}"
        )

    def _detail_text(self, joint_name: str, joint: Dict[str, float]) -> str:
        joint_index = int(joint_name.split("_")[1]) - 1
        external_thresholds = self.ros_node.config.get("external_torque_thresholds", [])
        effort_thresholds = self.ros_node.config.get("effort_thresholds", [])
        joint_models = self.ros_node.config.get("joint_models", [])
        external_threshold = external_thresholds[joint_index] if joint_index < len(external_thresholds) else float("nan")
        effort_threshold = effort_thresholds[joint_index] if joint_index < len(effort_thresholds) else float("nan")
        source = str(self.ros_node.config.get("measured_torque_source", "drive_torque"))
        model = str(joint_models[joint_index]) if joint_index < len(joint_models) else "?"
        return (
            f"meas={self._fmt(joint.get('measured_torque'), 2)}Nm "
            f"exp={self._fmt(joint.get('expected_torque'), 2)}Nm "
            f"diff={self._fmt(joint.get('torque_difference'), 2)}Nm "
            f"ext={self._fmt(joint.get('external_torque'), 2)}Nm/{float(external_threshold):.2f} "
            f"drv={self._fmt(joint.get('drive_output_torque'), 2)}Nm "
            f"cur={self._fmt(joint.get('motor_current_a'), 3)}A "
            f"cur_tau={self._fmt(joint.get('current_based_output_torque'), 2)}Nm "
            f"fric={self._fmt(joint.get('friction_torque'), 2)}Nm "
            f"contact_thr={float(effort_threshold):.2f} "
            f"src={source} model={model}"
        )


def main() -> None:
    rclpy.init()
    node = CollisionMonitorGuiNode()
    app = QApplication(sys.argv)
    window = CollisionMonitorWindow(node)
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(20)

    exit_code = app.exec()
    spin_timer.stop()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
