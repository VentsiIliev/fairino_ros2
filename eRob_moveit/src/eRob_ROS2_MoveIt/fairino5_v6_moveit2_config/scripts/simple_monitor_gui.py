#!/usr/bin/env python3
"""
Compact Robot Monitor GUI with collision detection visualization.
Two-column layout to fit on screen.
"""
import sys
import time
import json
import requests
from threading import Thread
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                              QGroupBox, QGridLayout, QPushButton, QLineEdit, QMessageBox,
                              QFrame, QSplitter, QScrollArea, QComboBox)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QFont
from status.robot_monitor import RobotMonitor
from safety import SensitivityPreset


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
            'use_dynamics': False
        }

        # Data collection state
        self.is_collecting = False
        self.collected_data = []
        self.collection_start_time = None
        self.was_executing = False

        # Timing tracking
        self.last_update_time = time.time()
        self.update_times = []
        self.max_timing_samples = 30

        self.plot_ready_signal.connect(self._show_plot)

        self.setWindowTitle('Robot Monitor - Compact View')
        self.setMinimumSize(1200, 600)

        # Main horizontal layout (two columns)
        main_layout = QHBoxLayout()

        # ========== LEFT COLUMN ==========
        left_column = QVBoxLayout()

        # Title and update rate (compact)
        header_layout = QHBoxLayout()
        title = QLabel('Robot Monitor')
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

        # Data Collection (compact)
        collect_group = QGroupBox('Data Collection')
        collect_layout = QHBoxLayout()
        collect_layout.addWidget(QLabel('Dist:'))
        self.jog_distance_input = QLineEdit('-200')
        self.jog_distance_input.setFixedWidth(60)
        collect_layout.addWidget(self.jog_distance_input)
        collect_layout.addWidget(QLabel('V%:'))
        self.jog_vel_input = QLineEdit('60')
        self.jog_vel_input.setFixedWidth(40)
        collect_layout.addWidget(self.jog_vel_input)
        collect_layout.addWidget(QLabel('A%:'))
        self.jog_acc_input = QLineEdit('40')
        self.jog_acc_input.setFixedWidth(40)
        collect_layout.addWidget(self.jog_acc_input)
        self.collect_button = QPushButton('Go')
        self.collect_button.setFixedWidth(50)
        self.collect_button.clicked.connect(self.start_data_collection)
        collect_layout.addWidget(self.collect_button)
        collect_group.setLayout(collect_layout)
        left_column.addWidget(collect_group)

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

        # Sensitivity preset selector
        sensitivity_layout = QHBoxLayout()
        sensitivity_layout.addWidget(QLabel('Sensitivity:'))

        self.sensitivity_combo = QComboBox()
        self.sensitivity_combo.addItems([
            'ULTRA_SENSITIVE_2KG',
            'HIGH_SENSITIVE_4KG',
            'MEDIUM_SENSITIVE_5KG',
            'STANDARD_6KG',
            'LOW_SENSITIVE_8KG'
        ])
        self.sensitivity_combo.setCurrentText('STANDARD_6KG')
        self.sensitivity_combo.currentTextChanged.connect(self.on_sensitivity_changed)
        self.sensitivity_combo.setStyleSheet('font-family: monospace; padding: 3px;')
        sensitivity_layout.addWidget(self.sensitivity_combo)

        # Info button to show preset details
        self.sensitivity_info_button = QPushButton('ℹ')
        self.sensitivity_info_button.setMaximumWidth(30)
        self.sensitivity_info_button.clicked.connect(self.show_sensitivity_info)
        self.sensitivity_info_button.setStyleSheet('font-weight: bold;')
        sensitivity_layout.addWidget(self.sensitivity_info_button)

        sensitivity_layout.addStretch()
        collision_layout.addLayout(sensitivity_layout)

        collision_group.setLayout(collision_layout)
        right_column.addWidget(collision_group)

        # Torque Table (measured, expected, external)
        torque_group = QGroupBox('Joint Torques (N·m)')
        torque_layout = QGridLayout()
        torque_layout.setSpacing(3)

        # Headers
        headers = ['Joint', 'Measured', 'Expected', 'External', 'Thresh']
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setStyleSheet('font-weight: bold; font-size: 9pt;')
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            torque_layout.addWidget(lbl, 0, col)

        # Data rows
        self.measured_torque_labels = []
        self.expected_torque_labels = []
        self.ext_torque_labels = []
        self.threshold_labels = []  # Store threshold labels for updates
        # Default thresholds match STANDARD_6KG preset
        thresholds = [18.0, 18.0, 12.0, 10.0, 6.0, 5.0]

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

            # Threshold (store in list for updates)
            t_lbl = QLabel(f'±{thresholds[i]:.0f}')
            t_lbl.setStyleSheet('font-family: monospace; font-size: 9pt; color: #999;')
            t_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            torque_layout.addWidget(t_lbl, i + 1, 4)
            self.threshold_labels.append(t_lbl)

        torque_group.setLayout(torque_layout)
        right_column.addWidget(torque_group)

        # Formula reminder - shows baseline-relative detection
        formula_label = QLabel('External = (τ_measured - τ_expected) - τ_baseline')
        formula_label.setStyleSheet('font-family: monospace; font-size: 9pt; color: #666; padding: 5px;')
        formula_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_column.addWidget(formula_label)

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
        x_mm = msg.pose.position.x * 1000.0
        if self.collection_start_time is None:
            self.collection_start_time = time_sec
        relative_time = time_sec - self.collection_start_time
        self.collected_data.append((relative_time, x_mm))

    def start_data_collection(self):
        if self.is_collecting:
            return
        try:
            jog_distance = float(self.jog_distance_input.text())
            vel = float(self.jog_vel_input.text())
            acc = float(self.jog_acc_input.text())
        except ValueError:
            QMessageBox.warning(self, 'Invalid Input', 'Please enter valid numbers.')
            return

        self.collected_data = []
        self.collection_start_time = None
        self.is_collecting = True
        self.was_executing = False
        self.collect_button.setEnabled(False)

        def send_jog():
            try:
                direction = 1 if jog_distance >= 0 else -1
                step = abs(jog_distance)
                requests.post(
                    'http://localhost:5000/jog',
                    json={'axis': 1, 'direction': direction, 'step': step, 'vel': vel, 'acc': acc},
                    timeout=5.0
                )
            except Exception as e:
                self.ros_node.get_logger().error(f'Jog failed: {e}')

        Thread(target=send_jog, daemon=True).start()

    def set_digital_output(self, do_id, status):
        msg = Int32MultiArray()
        msg.data = [do_id, status]
        self.do_publisher.publish(msg)

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

        # Get current thresholds based on selected sensitivity preset
        current_preset = self.sensitivity_combo.currentText()
        thresholds = self.get_preset_thresholds(current_preset)

        for i in range(6):
            # Measured
            self.measured_torque_labels[i].setText(f'{measured[i]:+.2f}')

            # Expected
            self.expected_torque_labels[i].setText(f'{expected[i]:+.2f}')

            # External (with color coding)
            ext_val = external[i]
            self.ext_torque_labels[i].setText(f'{ext_val:+.2f}')

            ratio = abs(ext_val) / thresholds[i] if thresholds[i] > 0 else 0
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
            if is_executing:
                self.was_executing = True
            if self.was_executing and not is_executing:
                self.is_collecting = False
                self.collect_button.setEnabled(True)
                self.plot_ready_signal.emit()

    def _show_plot(self):
        if len(self.collected_data) < 10:
            QMessageBox.warning(self, 'Insufficient Data', f'Only {len(self.collected_data)} samples.')
            return
        self._generate_analysis_plot()

    def _generate_analysis_plot(self):
        import matplotlib.pyplot as plt
        from scipy.ndimage import uniform_filter1d

        data = np.array(self.collected_data)
        t = data[:, 0]
        x = data[:, 1]

        sort_idx = np.argsort(t)
        t, x = t[sort_idx], x[sort_idx]

        unique_mask = np.diff(t, prepend=-1) > 0
        t, x = t[unique_mask], x[unique_mask]

        if len(t) < 10:
            return

        x_smooth = uniform_filter1d(x, size=3)
        dt = np.diff(t)
        dt[dt == 0] = 1e-6

        velocity = np.diff(x_smooth) / dt
        t_vel = t[:-1] + dt / 2
        velocity_smooth = uniform_filter1d(velocity, size=5)

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        axes[0].plot(t, x_smooth, 'b-', linewidth=2)
        axes[0].set_ylabel('Position (mm)')
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t_vel, velocity_smooth, 'g-', linewidth=2)
        axes[1].set_ylabel('Velocity (mm/s)')
        axes[1].set_xlabel('Time (s)')
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def robot_status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.robot_status.update(status_data)
        except json.JSONDecodeError:
            pass

    def on_sensitivity_changed(self, preset_name):
        """Called when user selects a new sensitivity preset."""
        try:
            # Send request to robot controller via HTTP API
            url = 'http://localhost:5000/set_sensitivity'
            data = {'preset': preset_name}
            response = requests.post(url, json=data, timeout=2)

            if response.status_code == 200:
                # Update threshold labels in the GUI
                self.update_threshold_labels(preset_name)

                self.ros_node.get_logger().info(f'✓ Sensitivity changed to: {preset_name}')
                QMessageBox.information(
                    self,
                    'Sensitivity Changed',
                    f'Collision sensitivity set to:\n{preset_name}\n\n{self.get_preset_description(preset_name)}'
                )
            else:
                error_msg = response.json().get('error', 'Unknown error')
                QMessageBox.warning(self, 'Error', f'Failed to change sensitivity:\n{error_msg}')

        except requests.exceptions.ConnectionError:
            QMessageBox.warning(
                self,
                'Connection Error',
                'Could not connect to robot controller.\nMake sure the robot controller is running.'
            )
        except Exception as e:
            QMessageBox.warning(self, 'Error', f'Failed to change sensitivity:\n{str(e)}')

    def update_threshold_labels(self, preset_name):
        """Update the threshold column in the torque table based on selected preset."""
        thresholds = self.get_preset_thresholds(preset_name)
        for i in range(6):
            self.threshold_labels[i].setText(f'±{thresholds[i]:.1f}')

    def get_preset_thresholds(self, preset_name):
        """Get the threshold values for a given preset."""
        # Use actual SensitivityPreset data instead of duplicating
        preset_data = SensitivityPreset.get_preset(preset_name)
        return preset_data['external_torque'].tolist() if hasattr(preset_data['external_torque'], 'tolist') else preset_data['external_torque']

    def show_sensitivity_info(self):
        """Show information about all sensitivity presets."""
        # Dynamically generate info from actual SensitivityPreset data
        info_text = "<h3>Collision Sensitivity Presets</h3>\n"

        for preset_name in SensitivityPreset.list_presets():
            if preset_name == 'CUSTOM':
                continue  # Skip custom preset in info dialog

            preset_data = SensitivityPreset.get_preset(preset_name)
            thresholds = preset_data['external_torque']
            description = preset_data['description']

            info_text += f"\n<p><b>{preset_name}</b><br/>"
            info_text += f"External Torque: {list(thresholds)} N·m<br/>"
            info_text += f"Use for: {description}</p>"

        info_text += "\n<p><i>Force ≈ Torque / (0.3m × 9.81 m/s²)</i></p>"

        msg = QMessageBox(self)
        msg.setWindowTitle('Sensitivity Presets Information')
        msg.setTextFormat(Qt.TextFormat.RichText)
        msg.setText(info_text)
        msg.exec()

    def get_preset_description(self, preset_name):
        """Get human-readable description of a preset."""
        # Use actual SensitivityPreset description
        return SensitivityPreset.get_description(preset_name)


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