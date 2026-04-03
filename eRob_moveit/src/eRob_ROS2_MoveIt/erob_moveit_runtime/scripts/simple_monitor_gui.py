#!/usr/bin/env python3
"""
Compact motion oscilloscope GUI with trajectory test controls.
Two-column layout to fit on screen.
"""
import sys
import time
import json
import requests
from threading import Thread
import numpy as np
import pyqtgraph as pg
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                              QGroupBox, QGridLayout, QPushButton, QLineEdit, QMessageBox,
                              QFrame, QSplitter, QScrollArea, QComboBox)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QFont
import config
from status.robot_monitor import RobotMonitor
from utils.work_object import WorkObject


class SimpleMonitorGUI(QWidget):
    plot_ready_signal = pyqtSignal()

    def __init__(self, ros_node, robot_monitor):
        super().__init__()
        self.ros_node = ros_node
        self.robot_monitor = robot_monitor

        # Robot status tracking
        self.robot_status = {
            'is_executing': False,
            'is_available': True,
            'queue_size': 0,
            'current_task_id': None,
            'collision_detected': False,
            'collision_state': 'CLEAR',
            'collision_armed': False,
            'external_torque': [0.0] * 6,
            'expected_torque': [0.0] * 6,
            'measured_torque': [0.0] * 6,
            'current_rate': [0.0] * 6,
            'effective_rate_thresholds': [5.0, 5.0, 4.0, 2.7, 2.0, 1.7],  # Match detector defaults
            'use_dynamics': False
        }

        # Data collection state
        self.is_collecting = False
        self.collected_data = []
        self.collection_start_time = None
        self.was_executing = False
        self.command_start_wall_time = None
        self.execution_start_wall_time = None
        self.execution_end_wall_time = None
        self.last_trace_summary = None
        self.default_workobject = self._build_default_workobject()

        # Timing tracking
        self.last_update_time = time.time()
        self.update_times = []
        self.max_timing_samples = 30

        self.plot_ready_signal.connect(self._show_plot)
        self.motion_buttons = []
        self._last_live_trace_key = None

        self.setWindowTitle('Robot Motion Oscilloscope')
        self.setMinimumSize(1280, 820)

        # Main horizontal layout (two columns)
        main_layout = QHBoxLayout()

        # ========== LEFT COLUMN ==========
        left_column = QVBoxLayout()

        # Title and update rate (compact)
        header_layout = QHBoxLayout()
        title = QLabel('Robot Motion Oscilloscope')
        title.setStyleSheet('font-size: 14pt; font-weight: bold;')
        header_layout.addWidget(title)
        header_layout.addStretch()
        self.update_rate_label = QLabel('-- Hz')
        self.update_rate_label.setStyleSheet('font-family: monospace; font-size: 10pt; color: #0066cc;')
        header_layout.addWidget(self.update_rate_label)
        left_column.addLayout(header_layout)

        # Cartesian Position (compact horizontal)
        cart_group = QGroupBox('TCP Position')
        cart_layout = QGridLayout()
        cart_layout.setSpacing(2)
        self.cart_pos_labels = []
        axes = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
        units = ['mm', 'mm', 'mm', '°', '°', '°']
        for i, (axis, unit) in enumerate(zip(axes, units)):
            col = i % 3
            row = i // 3
            lbl = QLabel(f'{axis}:')
            lbl.setStyleSheet('font-size: 9pt;')
            val = QLabel('0.0')
            val.setStyleSheet('font-family: monospace; font-size: 10pt; font-weight: bold;')
            cart_layout.addWidget(lbl, row, col * 2)
            cart_layout.addWidget(val, row, col * 2 + 1)
            self.cart_pos_labels.append(val)
        cart_group.setLayout(cart_layout)
        left_column.addWidget(cart_group)

        # Cartesian Velocity/Acceleration (compact)
        vel_group = QGroupBox('TCP Velocity / Acceleration')
        vel_layout = QGridLayout()
        vel_layout.setSpacing(2)
        vel_layout.addWidget(QLabel('Vel:'), 0, 0)
        self.cart_vel_mag_label = QLabel('0.0 mm/s')
        self.cart_vel_mag_label.setStyleSheet('font-family: monospace; font-size: 10pt; font-weight: bold;')
        vel_layout.addWidget(self.cart_vel_mag_label, 0, 1)
        vel_layout.addWidget(QLabel('Acc:'), 0, 2)
        self.cart_acc_mag_label = QLabel('0.0 mm/s²')
        self.cart_acc_mag_label.setStyleSheet('font-family: monospace; font-size: 10pt; font-weight: bold;')
        vel_layout.addWidget(self.cart_acc_mag_label, 0, 3)
        vel_group.setLayout(vel_layout)
        left_column.addWidget(vel_group)

        # Robot Status (compact)
        status_group = QGroupBox('Status')
        status_layout = QGridLayout()
        status_layout.setSpacing(2)

        self.execution_state_label = QLabel('IDLE')
        self.execution_state_label.setStyleSheet(
            'font-family: monospace; font-size: 12pt; font-weight: bold; color: #00cc00;'
        )
        status_layout.addWidget(QLabel('State:'), 0, 0)
        status_layout.addWidget(self.execution_state_label, 0, 1)

        self.queue_size_label = QLabel('0')
        self.queue_size_label.setStyleSheet('font-family: monospace;')
        status_layout.addWidget(QLabel('Queue:'), 0, 2)
        status_layout.addWidget(self.queue_size_label, 0, 3)

        status_group.setLayout(status_layout)
        left_column.addWidget(status_group)

        # Joint Velocities (compact horizontal)
        jvel_group = QGroupBox('Joint Velocities (rad/s)')
        jvel_layout = QHBoxLayout()
        self.joint_vel_labels = []
        for i in range(6):
            frame = QVBoxLayout()
            lbl = QLabel(f'J{i+1}')
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet('font-size: 8pt;')
            val = QLabel('0.00')
            val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val.setStyleSheet('font-family: monospace; font-size: 9pt;')
            frame.addWidget(lbl)
            frame.addWidget(val)
            jvel_layout.addLayout(frame)
            self.joint_vel_labels.append(val)
        jvel_group.setLayout(jvel_layout)
        left_column.addWidget(jvel_group)

        # Trajectory Test
        collect_group = QGroupBox('Linear Trajectory Test')
        collect_layout = QGridLayout()
        collect_layout.setHorizontalSpacing(8)
        collect_layout.setVerticalSpacing(6)
        collect_layout.addWidget(QLabel('Dist (mm):'), 0, 0)
        self.jog_distance_input = QLineEdit('-200')
        self.jog_distance_input.setFixedWidth(70)
        collect_layout.addWidget(self.jog_distance_input, 0, 1)
        collect_layout.addWidget(QLabel('V%:'), 0, 2)
        self.jog_vel_input = QLineEdit('60')
        self.jog_vel_input.setFixedWidth(55)
        collect_layout.addWidget(self.jog_vel_input, 0, 3)
        collect_layout.addWidget(QLabel('A%:'), 0, 4)
        self.jog_acc_input = QLineEdit('40')
        self.jog_acc_input.setFixedWidth(55)
        collect_layout.addWidget(self.jog_acc_input, 0, 5)
        collect_layout.addWidget(QLabel('Opt:'), 0, 6)
        self.optimizer_selector = QComboBox()
        self.optimizer_selector.addItems(['TOTG', 'RUCKIG'])
        self.optimizer_selector.setCurrentText('TOTG')
        self.optimizer_selector.setFixedWidth(90)
        collect_layout.addWidget(self.optimizer_selector, 0, 7)
        collect_layout.addWidget(QLabel('Tool:'), 0, 8)
        self.tool_selector = QComboBox()
        self.tool_selector.addItems(['0', '1'])
        self.tool_selector.setCurrentText('1' if getattr(config, 'ROBOT_BACKEND', '').lower() == 'fairino' else '0')
        self.tool_selector.setFixedWidth(55)
        collect_layout.addWidget(self.tool_selector, 0, 9)
        collect_layout.addWidget(QLabel('User:'), 0, 10)
        self.user_selector = QComboBox()
        self.user_selector.addItems(['0'])
        self.user_selector.setCurrentText('0')
        self.user_selector.setFixedWidth(55)
        collect_layout.addWidget(self.user_selector, 0, 11)

        self.collect_button = QPushButton('Run X+')
        self.collect_button.clicked.connect(lambda: self.start_data_collection('x', 1))
        collect_layout.addWidget(self.collect_button, 1, 0, 1, 2)
        self.motion_buttons.append(self.collect_button)

        test_buttons = [
            ('X-', 'x', -1, 1, 2),
            ('X+', 'x', 1, 1, 3),
            ('Y-', 'y', -1, 1, 4),
            ('Y+', 'y', 1, 1, 5),
            ('Z-', 'z', -1, 1, 6),
            ('Z+', 'z', 1, 1, 7),
        ]
        for label, axis, direction, row, col in test_buttons:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _checked=False, a=axis, d=direction: self.start_data_collection(a, d))
            collect_layout.addWidget(btn, row, col)
            self.motion_buttons.append(btn)

        self.trace_status_label = QLabel('No trace recorded')
        self.trace_status_label.setStyleSheet('font-family: monospace; color: #666;')
        self.trace_status_label.setWordWrap(True)
        collect_layout.addWidget(self.trace_status_label, 2, 0, 1, 12)

        self.trace_timing_label = QLabel('Timing: n/a')
        self.trace_timing_label.setStyleSheet('font-family: monospace; color: #0066cc;')
        self.trace_timing_label.setWordWrap(True)
        collect_layout.addWidget(self.trace_timing_label, 3, 0, 1, 12)
        collect_group.setLayout(collect_layout)
        left_column.addWidget(collect_group)

        live_group = QGroupBox('Live Motion Trace')
        live_layout = QVBoxLayout()
        self.live_plot_widget = pg.GraphicsLayoutWidget()
        self.live_plot_widget.setBackground('w')
        self.live_progress_plot = self.live_plot_widget.addPlot(row=0, col=0)
        self.live_velocity_plot = self.live_plot_widget.addPlot(row=1, col=0)
        self.live_accel_plot = self.live_plot_widget.addPlot(row=2, col=0)
        self.live_jerk_plot = self.live_plot_widget.addPlot(row=3, col=0)

        live_specs = [
            (self.live_progress_plot, 'Progress (mm)', '#1f77b4'),
            (self.live_velocity_plot, 'Velocity (mm/s)', '#2ca02c'),
            (self.live_accel_plot, 'Accel (mm/s²)', '#d62728'),
            (self.live_jerk_plot, 'Jerk (mm/s³)', '#9467bd'),
        ]
        self.live_curves = {}
        for plot, label, color in live_specs:
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel('left', label)
            plot.getAxis('left').setWidth(80)
            plot.setMenuEnabled(False)
            plot.setMouseEnabled(x=False, y=False)
            self.live_curves[label] = plot.plot(pen=pg.mkPen(color=color, width=2))
        self.live_progress_plot.hideAxis('bottom')
        self.live_velocity_plot.hideAxis('bottom')
        self.live_accel_plot.hideAxis('bottom')
        self.live_jerk_plot.setLabel('bottom', 'Time (s)')
        self.live_velocity_plot.setXLink(self.live_progress_plot)
        self.live_accel_plot.setXLink(self.live_progress_plot)
        self.live_jerk_plot.setXLink(self.live_progress_plot)
        live_layout.addWidget(self.live_plot_widget)
        live_group.setLayout(live_layout)
        left_column.addWidget(live_group)

        # Digital Output
        do_group = QGroupBox('DO3')
        do_layout = QHBoxLayout()
        self.do3_on_button = QPushButton('ON')
        self.do3_on_button.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold;')
        self.do3_on_button.clicked.connect(lambda: self.set_digital_output(3, 1))
        do_layout.addWidget(self.do3_on_button)
        self.do3_off_button = QPushButton('OFF')
        self.do3_off_button.setStyleSheet('background-color: #f44336; color: white; font-weight: bold;')
        self.do3_off_button.clicked.connect(lambda: self.set_digital_output(3, 0))
        do_layout.addWidget(self.do3_off_button)
        do_group.setLayout(do_layout)
        left_column.addWidget(do_group)

        left_column.addStretch()

        # ========== RIGHT COLUMN (Torque Monitoring) ==========
        right_column = QVBoxLayout()

        # Collision Status (prominent)
        collision_group = QGroupBox('Collision Detection')
        collision_layout = QVBoxLayout()

        # Big collision indicator
        self.collision_label = QLabel('CLEAR')
        self.collision_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.collision_label.setStyleSheet(
            'font-family: monospace; font-size: 18pt; font-weight: bold; '
            'color: #00cc00; background-color: #e8f5e9; padding: 10px; border-radius: 5px;'
        )
        collision_layout.addWidget(self.collision_label)

        # Armed status
        armed_layout = QHBoxLayout()
        armed_layout.addWidget(QLabel('Detector:'))
        self.collision_armed_label = QLabel('ARMED')
        self.collision_armed_label.setStyleSheet('font-family: monospace; font-weight: bold; color: #0066cc;')
        armed_layout.addWidget(self.collision_armed_label)

        # Baseline calibration status
        armed_layout.addWidget(QLabel('  |  Baseline:'))
        self.baseline_status_label = QLabel('CALIBRATING...')
        self.baseline_status_label.setStyleSheet('font-family: monospace; color: #ff9800;')
        armed_layout.addWidget(self.baseline_status_label)

        armed_layout.addStretch()
        collision_layout.addLayout(armed_layout)


        collision_group.setLayout(collision_layout)
        right_column.addWidget(collision_group)

        # Torque Table (measured, expected, external, rate, threshold)
        torque_group = QGroupBox('Joint Torques (N·m)')
        torque_layout = QGridLayout()
        torque_layout.setSpacing(3)

        # Headers
        headers = ['Joint', 'Measured', 'Expected', 'External', 'Rate', 'Thresh']
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setStyleSheet('font-weight: bold; font-size: 9pt;')
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            torque_layout.addWidget(lbl, 0, col)

        # Data rows
        self.measured_torque_labels = []
        self.expected_torque_labels = []
        self.ext_torque_labels = []
        self.rate_labels = []  # Store rate labels for updates
        self.threshold_labels = []  # Store threshold labels for updates

        # Default rate thresholds from detector (dynamics_collision_detector.py line 86)
        default_rate_thresholds = [5.0, 5.0, 4.0, 2.7, 2.0, 1.7]

        for i in range(6):
            # Joint label
            jlbl = QLabel(f'J{i+1}')
            jlbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            torque_layout.addWidget(jlbl, i + 1, 0)

            # Measured
            m_lbl = QLabel('0.00')
            m_lbl.setStyleSheet('font-family: monospace; font-size: 10pt;')
            m_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            torque_layout.addWidget(m_lbl, i + 1, 1)
            self.measured_torque_labels.append(m_lbl)

            # Expected
            e_lbl = QLabel('0.00')
            e_lbl.setStyleSheet('font-family: monospace; font-size: 10pt; color: #666;')
            e_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            torque_layout.addWidget(e_lbl, i + 1, 2)
            self.expected_torque_labels.append(e_lbl)

            # External (colored)
            x_lbl = QLabel('0.00')
            x_lbl.setStyleSheet('font-family: monospace; font-size: 10pt; font-weight: bold;')
            x_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            torque_layout.addWidget(x_lbl, i + 1, 3)
            self.ext_torque_labels.append(x_lbl)

            # Rate (current calculated rate)
            r_lbl = QLabel('0.00')
            r_lbl.setStyleSheet('font-family: monospace; font-size: 9pt; color: #0066cc;')
            r_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            torque_layout.addWidget(r_lbl, i + 1, 4)
            self.rate_labels.append(r_lbl)

            # Threshold (rate threshold from settings)
            t_lbl = QLabel(f'{default_rate_thresholds[i]:.1f}')
            t_lbl.setStyleSheet('font-family: monospace; font-size: 9pt; color: #999;')
            t_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            torque_layout.addWidget(t_lbl, i + 1, 5)
            self.threshold_labels.append(t_lbl)

        torque_group.setLayout(torque_layout)
        right_column.addWidget(torque_group)

        # Formula reminder - shows baseline-relative detection
        # formula_label = QLabel('External = (τ_measured - τ_expected) - τ_baseline')
        # formula_label.setStyleSheet('font-family: monospace; font-size: 9pt; color: #666; padding: 5px;')
        # formula_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # right_column.addWidget(formula_label)

        right_column.addStretch()

        # Add columns to main layout
        left_widget = QWidget()
        left_widget.setLayout(left_column)
        left_widget.setMaximumWidth(450)

        right_widget = QWidget()
        right_widget.setLayout(right_column)

        main_layout.addWidget(left_widget)
        main_layout.addWidget(right_widget, stretch=1)

        self.setLayout(main_layout)

        # Create publisher for digital output
        self.do_publisher = self.ros_node.create_publisher(Int32MultiArray, '/set_do', 10)

        # Update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)

        # Subscribe to cartesian position
        self.ros_node.create_subscription(
            PoseStamped, '/cartesian_position',
            self._cartesian_position_callback, 10
        )

        # Store velocity/acceleration labels (hidden but used for data)
        self.cart_vel_labels = [QLabel() for _ in range(3)]
        self.cart_acc_labels = [QLabel() for _ in range(3)]
        self.joint_acc_labels = [QLabel() for _ in range(6)]
        self.joint_vel_mag_label = QLabel()
        self.joint_acc_mag_label = QLabel()
        self.available_label = QLabel()
        self.task_id_label = QLabel()
        self.collection_status_label = QLabel()

    def _cartesian_position_callback(self, msg):
        if not self.is_collecting:
            return
        stamp = msg.header.stamp
        time_sec = stamp.sec + stamp.nanosec * 1e-9
        if self.collection_start_time is None:
            self.collection_start_time = time_sec
        relative_time = time_sec - self.collection_start_time
        stable = self.robot_monitor.get_stable_data()
        if stable is None:
            return
        cart = np.array(stable['cartesian'], dtype=float)
        cart_vel = np.array(stable['cart_velocity'], dtype=float)
        cart_acc = np.array(stable['cart_acceleration'], dtype=float)
        joint_vel = np.array(stable['velocities'], dtype=float)
        joint_acc = np.array(stable['accelerations'], dtype=float)
        self.collected_data.append({
            't': relative_time,
            'cartesian': cart,
            'cart_velocity': cart_vel,
            'cart_acceleration': cart_acc,
            'cart_vel_mag': float(stable['cart_vel_magnitude']),
            'cart_acc_mag': float(stable['cart_acc_magnitude']),
            'joint_vel_mag': float(stable['vel_magnitude']),
            'joint_acc_mag': float(stable['acc_magnitude']),
            'joint_velocities': joint_vel,
            'joint_accelerations': joint_acc,
            'is_executing': bool(self.robot_status.get('is_executing', False)),
        })

    def _build_default_workobject(self):
        values = list(getattr(config, 'DEFAULT_WORKOBJECT', [0, 0, 0, 0, 0, 0]) or [0, 0, 0, 0, 0, 0])
        if len(values) != 6 or not any(abs(float(v)) > 1e-9 for v in values):
            return None
        return WorkObject(*values)

    def _current_pose_in_user_frame(self, user_id):
        try:
            response = requests.get('http://localhost:5000/position/current', timeout=2.0)
            response.raise_for_status()
            payload = response.json()
            position = payload.get('position')
            if position and len(position) == 6:
                return np.array(position, dtype=float)
        except Exception as exc:
            self.ros_node.get_logger().warning(
                f'Falling back to local monitor pose because /position/current failed: {exc}'
            )

        stable = self.robot_monitor.get_stable_data()
        if stable is None:
            return None
        pose = np.array(stable['cartesian'], dtype=float)
        if user_id == 0 and self.default_workobject is not None:
            pose = np.array(self.default_workobject.apply(pose.tolist(), inverse=True), dtype=float)
        return pose

    def _compute_motion_profile(self):
        if len(self.collected_data) < 3:
            return None

        from scipy.ndimage import uniform_filter1d

        samples = sorted(self.collected_data, key=lambda sample: sample['t'])
        t = np.array([sample['t'] for sample in samples], dtype=float)
        cartesian = np.array([sample['cartesian'] for sample in samples], dtype=float)
        cart_vel_mag = np.array([sample['cart_vel_mag'] for sample in samples], dtype=float)
        cart_acc_mag = np.array([sample['cart_acc_mag'] for sample in samples], dtype=float)

        unique_mask = np.diff(t, prepend=-1.0) > 0
        t = t[unique_mask]
        cartesian = cartesian[unique_mask]
        cart_vel_mag = cart_vel_mag[unique_mask]
        cart_acc_mag = cart_acc_mag[unique_mask]

        if len(t) < 3:
            return None

        start_xyz = cartesian[0, :3]
        progress = uniform_filter1d(np.linalg.norm(cartesian[:, :3] - start_xyz, axis=1), size=3)
        speed = uniform_filter1d(cart_vel_mag, size=3)
        acceleration = uniform_filter1d(cart_acc_mag, size=3)
        jerk = uniform_filter1d(np.gradient(acceleration, t), size=3)

        return {
            't': t,
            'progress': progress,
            'speed': speed,
            'acceleration': acceleration,
            'jerk': jerk,
            'cartesian': cartesian,
        }

    def _refresh_live_plot(self):
        profile = self._compute_motion_profile()
        if profile is None:
            return

        t = profile['t']
        self.live_curves['Progress (mm)'].setData(t, profile['progress'])
        self.live_curves['Velocity (mm/s)'].setData(t, profile['speed'])
        self.live_curves['Accel (mm/s²)'].setData(t, profile['acceleration'])
        self.live_curves['Jerk (mm/s³)'].setData(t, profile['jerk'])

        self.live_progress_plot.enableAutoRange(axis='xy', enable=True)
        self.live_velocity_plot.enableAutoRange(axis='y', enable=True)
        self.live_accel_plot.enableAutoRange(axis='y', enable=True)
        self.live_jerk_plot.enableAutoRange(axis='y', enable=True)

    def start_data_collection(self, axis='x', direction=1):
        if self.is_collecting:
            return
        try:
            jog_distance = float(self.jog_distance_input.text())
            vel = float(self.jog_vel_input.text())
            acc = float(self.jog_acc_input.text())
            trajectory_optimizer = self.optimizer_selector.currentText().strip().upper()
            tool_id = int(self.tool_selector.currentText())
            user_id = int(self.user_selector.currentText())
        except ValueError:
            QMessageBox.warning(self, 'Invalid Input', 'Please enter valid numbers.')
            return

        current_pose = self._current_pose_in_user_frame(user_id)
        if current_pose is None:
            QMessageBox.warning(self, 'No Robot Data', 'No current robot state available yet.')
            return

        axis_to_index = {'x': 0, 'y': 1, 'z': 2}
        if axis not in axis_to_index:
            QMessageBox.warning(self, 'Invalid Axis', f'Unsupported axis: {axis}')
            return

        target = current_pose.copy()
        target[axis_to_index[axis]] += abs(jog_distance) * direction

        self.collected_data = []
        self.collection_start_time = None
        self.is_collecting = True
        self.was_executing = False
        self.command_start_wall_time = time.time()
        self.execution_start_wall_time = None
        self.execution_end_wall_time = None
        self.last_trace_summary = None
        self._set_motion_buttons_enabled(False)
        self.trace_status_label.setText(
            f'Collecting {axis.upper()} {"+" if direction > 0 else "-"} {abs(jog_distance):.1f} mm '
            f'({trajectory_optimizer}, tool={tool_id}, user={user_id})'
        )
        self.trace_timing_label.setText(
            'Target: '
            f'[{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}, '
            f'RZ={target[5]:.1f}°]  waiting for motion start'
        )

        def send_linear():
            try:
                response = requests.post(
                    'http://localhost:5000/move/linear',
                    json={
                        'position': target.tolist(),
                        'tool': tool_id,
                        'user': user_id,
                        'vel': vel,
                        'acc': acc,
                        'blocking': True,
                        'trajectory_optimizer': trajectory_optimizer,
                    },
                    timeout=30.0,
                )
                if response.status_code >= 400:
                    raise RuntimeError(f'HTTP {response.status_code}: {response.text}')
            except Exception as e:
                self.is_collecting = False
                self._set_motion_buttons_enabled(True)
                self.trace_status_label.setText(f'Command failed: {e}')
                self.trace_timing_label.setText('Timing: n/a')
                self.ros_node.get_logger().error(f'Linear move failed: {e}')

        Thread(target=send_linear, daemon=True).start()

    def set_digital_output(self, do_id, status):
        msg = Int32MultiArray()
        msg.data = [do_id, status]
        self.do_publisher.publish(msg)

    def _set_motion_buttons_enabled(self, enabled):
        for button in self.motion_buttons:
            button.setEnabled(enabled)

    def update_display(self):
        # Update rate
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        self.update_times.append(dt)
        if len(self.update_times) > self.max_timing_samples:
            self.update_times.pop(0)
        if self.update_times:
            avg_dt = sum(self.update_times) / len(self.update_times)
            hz = 1.0 / avg_dt if avg_dt > 0 else 0
            self.update_rate_label.setText(f'{hz:.0f} Hz')

        data = self.robot_monitor.get_stable_data()
        if data is None:
            return

        # Cartesian Position
        cart_pos = data['cartesian']
        for i in range(6):
            fmt = '.1f' if i < 3 else '.2f'
            self.cart_pos_labels[i].setText(f'{cart_pos[i]:{fmt}}')

        # Velocity magnitudes
        self.cart_vel_mag_label.setText(f'{data["cart_vel_magnitude"]:.1f} mm/s')
        self.cart_acc_mag_label.setText(f'{data["cart_acc_magnitude"]:.1f} mm/s²')

        # Joint velocities
        joint_vel = data['velocities']
        for i in range(6):
            self.joint_vel_labels[i].setText(f'{joint_vel[i]:.2f}')

        # Robot Status
        is_executing = self.robot_status['is_executing']
        if is_executing:
            self.execution_state_label.setText('EXEC')
            self.execution_state_label.setStyleSheet(
                'font-family: monospace; font-size: 12pt; font-weight: bold; color: #ff6600;'
            )
        else:
            self.execution_state_label.setText('IDLE')
            self.execution_state_label.setStyleSheet(
                'font-family: monospace; font-size: 12pt; font-weight: bold; color: #00cc00;'
            )

        self.queue_size_label.setText(str(self.robot_status['queue_size']))

        # Collision Status
        collision_state = self.robot_status.get('collision_state', 'CLEAR')
        collision_detected = self.robot_status.get('collision_detected', False)

        if collision_detected:
            self.collision_label.setText('⚠ COLLISION!')
            self.collision_label.setStyleSheet(
                'font-family: monospace; font-size: 18pt; font-weight: bold; '
                'color: #ffffff; background-color: #f44336; padding: 10px; border-radius: 5px;'
            )
        elif collision_state == 'RECOVERING':
            self.collision_label.setText('RECOVERING...')
            self.collision_label.setStyleSheet(
                'font-family: monospace; font-size: 18pt; font-weight: bold; '
                'color: #ff6600; background-color: #fff3e0; padding: 10px; border-radius: 5px;'
            )
        else:
            self.collision_label.setText('CLEAR')
            self.collision_label.setStyleSheet(
                'font-family: monospace; font-size: 18pt; font-weight: bold; '
                'color: #00cc00; background-color: #e8f5e9; padding: 10px; border-radius: 5px;'
            )

        # Armed status
        if self.robot_status.get('collision_armed', False):
            self.collision_armed_label.setText('ARMED')
            self.collision_armed_label.setStyleSheet('font-family: monospace; font-weight: bold; color: #0066cc;')
        else:
            self.collision_armed_label.setText('DISARMED')
            self.collision_armed_label.setStyleSheet('font-family: monospace; color: #999;')

        # Rate-only detection mode
        self.baseline_status_label.setText('RATE-ONLY')
        self.baseline_status_label.setStyleSheet('font-family: monospace; font-weight: bold; color: #0066cc;')

        # Torque values
        measured = self.robot_status.get('measured_torque', [0.0] * 6)
        expected = self.robot_status.get('expected_torque', [0.0] * 6)
        external = self.robot_status.get('external_torque', [0.0] * 6)
        current_rate = self.robot_status.get('current_rate', [0.0] * 6)
        effective_rate_thresholds = self.robot_status.get('effective_rate_thresholds', [5.0, 5.0, 4.0, 2.7, 2.0, 1.7])

        for i in range(6):
            # Measured
            self.measured_torque_labels[i].setText(f'{measured[i]:+.2f}')

            # Expected
            self.expected_torque_labels[i].setText(f'{expected[i]:+.2f}')

            # External (with color coding based on rate vs threshold)
            ext_val = external[i]
            self.ext_torque_labels[i].setText(f'{ext_val:+.2f}')

            # Rate (current calculated rate)
            rate_val = current_rate[i]
            self.rate_labels[i].setText(f'{rate_val:.2f}')

            # Threshold (current effective rate threshold - dynamically scaled)
            thresh_val = effective_rate_thresholds[i]
            self.threshold_labels[i].setText(f'{thresh_val:.1f}')

            # Color code external torque based on rate vs effective threshold
            ratio = abs(rate_val) / effective_rate_thresholds[i] if effective_rate_thresholds[i] > 0 else 0
            if ratio > 1.0:
                self.ext_torque_labels[i].setStyleSheet(
                    'font-family: monospace; font-size: 10pt; font-weight: bold; '
                    'color: #fff; background-color: #f44336;'
                )
            elif ratio > 0.7:
                self.ext_torque_labels[i].setStyleSheet(
                    'font-family: monospace; font-size: 10pt; font-weight: bold; color: #ff6600;'
                )
            elif ratio > 0.4:
                self.ext_torque_labels[i].setStyleSheet(
                    'font-family: monospace; font-size: 10pt; font-weight: bold; color: #cc9900;'
                )
            else:
                self.ext_torque_labels[i].setStyleSheet(
                    'font-family: monospace; font-size: 10pt; font-weight: bold; color: #00aa00;'
                )

        # Data collection
        if self.is_collecting:
            self._refresh_live_plot()
            if is_executing:
                if self.execution_start_wall_time is None:
                    self.execution_start_wall_time = time.time()
                    planning_delay = self.execution_start_wall_time - self.command_start_wall_time
                    self.trace_timing_label.setText(
                        f'Timing: planning {planning_delay:.3f}s, executing...'
                    )
                self.was_executing = True
            if self.was_executing and not is_executing:
                self.is_collecting = False
                self.execution_end_wall_time = time.time()
                self._set_motion_buttons_enabled(True)
                self.last_trace_summary = self._build_trace_summary()
                self._update_trace_summary_labels()
                self._refresh_live_plot()
                self.plot_ready_signal.emit()

    def _show_plot(self):
        if len(self.collected_data) < 10:
            QMessageBox.warning(self, 'Insufficient Data', f'Only {len(self.collected_data)} samples.')
            return
        self._generate_analysis_plot()

    def _build_trace_summary(self):
        if self.command_start_wall_time is None:
            return None

        summary = {
            'samples': len(self.collected_data),
            'planning_delay_s': None,
            'execution_s': None,
            'total_s': None,
        }

        if self.execution_start_wall_time is not None:
            summary['planning_delay_s'] = self.execution_start_wall_time - self.command_start_wall_time
        if self.execution_start_wall_time is not None and self.execution_end_wall_time is not None:
            summary['execution_s'] = self.execution_end_wall_time - self.execution_start_wall_time
        if self.execution_end_wall_time is not None:
            summary['total_s'] = self.execution_end_wall_time - self.command_start_wall_time
        return summary

    def _update_trace_summary_labels(self):
        summary = self.last_trace_summary
        if not summary:
            return
        self.trace_status_label.setText(f'Trace ready: {summary["samples"]} samples')
        planning = summary['planning_delay_s']
        execution = summary['execution_s']
        total = summary['total_s']
        self.trace_timing_label.setText(
            'Timing: '
            f'plan={planning:.3f}s ' if planning is not None else 'Timing: plan=n/a '
            + (f'exec={execution:.3f}s ' if execution is not None else 'exec=n/a ')
            + (f'total={total:.3f}s' if total is not None else 'total=n/a')
        )

    def _generate_analysis_plot(self):
        import matplotlib.pyplot as plt
        profile = self._compute_motion_profile()
        if profile is None or len(profile['t']) < 10:
            return

        t = profile['t']
        cartesian = profile['cartesian']
        progress = profile['progress']
        speed = profile['speed']
        acceleration = profile['acceleration']
        jerk = profile['jerk']

        peak_speed = float(np.max(np.abs(speed))) if len(speed) else 0.0
        peak_acc = float(np.max(np.abs(acceleration))) if len(acceleration) else 0.0
        peak_jerk = float(np.max(np.abs(jerk))) if len(jerk) else 0.0

        fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)

        axes[0].plot(t, progress, color='b', linewidth=2)
        axes[0].set_ylabel('Progress (mm)')
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t, speed, color='g', linewidth=2)
        axes[1].set_ylabel('Velocity (mm/s)')
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(t, acceleration, color='r', linewidth=2)
        axes[2].set_ylabel('Accel (mm/s²)')
        axes[2].grid(True, alpha=0.3)

        axes[3].plot(t, jerk, color='m', linewidth=2)
        axes[3].set_ylabel('Jerk (mm/s³)')
        axes[3].set_xlabel('Time (s)')
        axes[3].grid(True, alpha=0.3)

        title = 'Motion Oscilloscope Trace'
        if self.last_trace_summary and None not in (
            self.last_trace_summary["planning_delay_s"],
            self.last_trace_summary["execution_s"],
            self.last_trace_summary["total_s"],
        ):
            summary = self.last_trace_summary
            title += (
                f'  plan={summary["planning_delay_s"]:.3f}s'
                f'  exec={summary["execution_s"]:.3f}s'
                f'  total={summary["total_s"]:.3f}s'
            )
        title += (
            f'  peak_v={peak_speed:.1f}mm/s'
            f'  peak_a={peak_acc:.1f}mm/s²'
            f'  peak_j={peak_jerk:.1f}mm/s³'
        )
        fig.suptitle(title)

        if len(cartesian) >= 2:
            move_delta = cartesian[-1, :3] - cartesian[0, :3]
            axes[0].text(
                0.01,
                0.95,
                f'ΔXYZ = [{move_delta[0]:.1f}, {move_delta[1]:.1f}, {move_delta[2]:.1f}] mm',
                transform=axes[0].transAxes,
                verticalalignment='top',
                fontsize=9,
                family='monospace',
                bbox={'boxstyle': 'round', 'facecolor': 'white', 'alpha': 0.8, 'edgecolor': '#cccccc'},
            )

        axes[2].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.show()

    def robot_status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.robot_status.update(status_data)
        except json.JSONDecodeError:
            pass



def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    ros_node = Node('simple_monitor_gui')
    robot_monitor = RobotMonitor(ros_node, stable_update_rate_hz=50.0)

    app = QApplication(sys.argv)
    gui = SimpleMonitorGUI(ros_node, robot_monitor)

    ros_node.create_subscription(String, '/robot_status', gui.robot_status_callback, 10)

    thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    thread.start()

    gui.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
