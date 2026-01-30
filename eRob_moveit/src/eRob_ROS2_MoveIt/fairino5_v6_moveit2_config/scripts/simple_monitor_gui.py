#!/usr/bin/env python3
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
                              QGroupBox, QGridLayout, QPushButton, QLineEdit, QMessageBox)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QFont
from status.robot_monitor import RobotMonitor


class SimpleMonitorGUI(QWidget):
    # Signal to trigger plot from main thread
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
            'current_task_id': None
        }

        # Data collection state
        self.is_collecting = False
        self.collected_data = []  # List of (time_sec, x_mm)
        self.collection_start_time = None
        self.was_executing = False

        # Timing tracking for update rate
        self.last_update_time = time.time()
        self.update_times = []
        self.max_timing_samples = 30

        # Connect signal to slot
        self.plot_ready_signal.connect(self._show_plot)

        self.setWindowTitle('ROS2 Robot Monitor - Topic Data Display')
        self.setMinimumSize(800, 700)
        main_layout = QVBoxLayout()

        # Title
        title = QLabel('Robot State Monitor')
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title)

        # Update rate display
        update_rate_layout = QHBoxLayout()
        update_rate_layout.addStretch()
        self.update_rate_label = QLabel('Update Rate: -- Hz (-- ms)')
        self.update_rate_label.setStyleSheet('font-family: monospace; font-size: 11pt; color: #0066cc;')
        update_rate_layout.addWidget(self.update_rate_label)
        update_rate_layout.addStretch()
        main_layout.addLayout(update_rate_layout)

        # Cartesian Position Group
        cart_pos_group = QGroupBox('Cartesian Position (from /cartesian_position)')
        cart_pos_layout = QGridLayout()
        cart_pos_layout.addWidget(QLabel('Axis'), 0, 0)
        cart_pos_layout.addWidget(QLabel('Position'), 0, 1)
        self.cart_pos_labels = []
        for i, axis in enumerate(['X (mm)', 'Y (mm)', 'Z (mm)', 'RX (deg)', 'RY (deg)', 'RZ (deg)']):
            label_axis = QLabel(axis)
            label_value = QLabel('0.000')
            label_value.setStyleSheet('font-family: monospace; font-size: 12pt;')
            cart_pos_layout.addWidget(label_axis, i + 1, 0)
            cart_pos_layout.addWidget(label_value, i + 1, 1)
            self.cart_pos_labels.append(label_value)
        cart_pos_group.setLayout(cart_pos_layout)
        main_layout.addWidget(cart_pos_group)

        # Cartesian Velocity/Acceleration Group
        cart_vel_group = QGroupBox('Cartesian Velocity & Acceleration')
        cart_vel_layout = QGridLayout()
        cart_vel_layout.addWidget(QLabel('Topic'), 0, 0)
        cart_vel_layout.addWidget(QLabel('X'), 0, 1)
        cart_vel_layout.addWidget(QLabel('Y'), 0, 2)
        cart_vel_layout.addWidget(QLabel('Z'), 0, 3)
        cart_vel_layout.addWidget(QLabel('Magnitude'), 0, 4)
        cart_vel_layout.addWidget(QLabel('/cartesian_velocity (mm/s)'), 1, 0)
        self.cart_vel_labels = []
        for i in range(3):
            label = QLabel('0.00')
            label.setStyleSheet('font-family: monospace;')
            cart_vel_layout.addWidget(label, 1, i + 1)
            self.cart_vel_labels.append(label)
        self.cart_vel_mag_label = QLabel('0.00')
        self.cart_vel_mag_label.setStyleSheet('font-family: monospace; font-weight: bold;')
        cart_vel_layout.addWidget(self.cart_vel_mag_label, 1, 4)
        cart_vel_layout.addWidget(QLabel('/cartesian_acceleration (mm/s^2)'), 2, 0)
        self.cart_acc_labels = []
        for i in range(3):
            label = QLabel('0.00')
            label.setStyleSheet('font-family: monospace;')
            cart_vel_layout.addWidget(label, 2, i + 1)
            self.cart_acc_labels.append(label)
        self.cart_acc_mag_label = QLabel('0.00')
        self.cart_acc_mag_label.setStyleSheet('font-family: monospace; font-weight: bold;')
        cart_vel_layout.addWidget(self.cart_acc_mag_label, 2, 4)
        cart_vel_group.setLayout(cart_vel_layout)
        main_layout.addWidget(cart_vel_group)

        # Joint Velocities Group
        joint_vel_group = QGroupBox('Joint Velocities (from /joint_velocity)')
        joint_vel_layout = QGridLayout()
        joint_vel_layout.addWidget(QLabel('Joint'), 0, 0)
        joint_vel_layout.addWidget(QLabel('Velocity (rad/s)'), 0, 1)
        self.joint_vel_labels = []
        for i in range(6):
            label_joint = QLabel(f'J{i + 1}')
            label_value = QLabel('0.0000')
            label_value.setStyleSheet('font-family: monospace;')
            joint_vel_layout.addWidget(label_joint, i + 1, 0)
            joint_vel_layout.addWidget(label_value, i + 1, 1)
            self.joint_vel_labels.append(label_value)
        label_mag = QLabel('Magnitude:')
        label_mag.setStyleSheet('font-weight: bold;')
        self.joint_vel_mag_label = QLabel('0.0000 rad/s')
        self.joint_vel_mag_label.setStyleSheet('font-family: monospace; font-weight: bold;')
        joint_vel_layout.addWidget(label_mag, 7, 0)
        joint_vel_layout.addWidget(self.joint_vel_mag_label, 7, 1)
        joint_vel_group.setLayout(joint_vel_layout)
        main_layout.addWidget(joint_vel_group)

        # Robot Status Group (Execution State & Queue)
        status_group = QGroupBox('Robot Status (from /robot_status)')
        status_layout = QGridLayout()

        status_layout.addWidget(QLabel('Execution State:'), 0, 0)
        self.execution_state_label = QLabel('UNKNOWN')
        self.execution_state_label.setStyleSheet('font-family: monospace; font-size: 14pt; font-weight: bold;')
        status_layout.addWidget(self.execution_state_label, 0, 1)

        status_layout.addWidget(QLabel('Available:'), 1, 0)
        self.available_label = QLabel('--')
        self.available_label.setStyleSheet('font-family: monospace; font-size: 12pt;')
        status_layout.addWidget(self.available_label, 1, 1)

        status_layout.addWidget(QLabel('Queue Size:'), 2, 0)
        self.queue_size_label = QLabel('0')
        self.queue_size_label.setStyleSheet('font-family: monospace; font-size: 12pt;')
        status_layout.addWidget(self.queue_size_label, 2, 1)

        status_layout.addWidget(QLabel('Current Task ID:'), 3, 0)
        self.task_id_label = QLabel('None')
        self.task_id_label.setStyleSheet('font-family: monospace; font-size: 12pt;')
        status_layout.addWidget(self.task_id_label, 3, 1)

        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # Joint Accelerations Group
        joint_acc_group = QGroupBox('Joint Accelerations (from /joint_acceleration)')
        joint_acc_layout = QGridLayout()
        joint_acc_layout.addWidget(QLabel('Joint'), 0, 0)
        joint_acc_layout.addWidget(QLabel('Acceleration (rad/s^2)'), 0, 1)
        self.joint_acc_labels = []
        for i in range(6):
            label_joint = QLabel(f'J{i + 1}')
            label_value = QLabel('0.0000')
            label_value.setStyleSheet('font-family: monospace;')
            joint_acc_layout.addWidget(label_joint, i + 1, 0)
            joint_acc_layout.addWidget(label_value, i + 1, 1)
            self.joint_acc_labels.append(label_value)
        label_mag = QLabel('Magnitude:')
        label_mag.setStyleSheet('font-weight: bold;')
        self.joint_acc_mag_label = QLabel('0.0000 rad/s^2')
        self.joint_acc_mag_label.setStyleSheet('font-family: monospace; font-weight: bold;')
        joint_acc_layout.addWidget(label_mag, 7, 0)
        joint_acc_layout.addWidget(self.joint_acc_mag_label, 7, 1)
        joint_acc_group.setLayout(joint_acc_layout)
        main_layout.addWidget(joint_acc_group)

        # Data Collection Group
        collect_group = QGroupBox('Data Collection & Analysis')
        collect_layout = QGridLayout()

        collect_layout.addWidget(QLabel('Jog Distance (mm):'), 0, 0)
        self.jog_distance_input = QLineEdit('-200')
        self.jog_distance_input.setFixedWidth(100)
        collect_layout.addWidget(self.jog_distance_input, 0, 1)

        collect_layout.addWidget(QLabel('Velocity (%):'), 0, 2)
        self.jog_vel_input = QLineEdit('60')
        self.jog_vel_input.setFixedWidth(60)
        collect_layout.addWidget(self.jog_vel_input, 0, 3)

        collect_layout.addWidget(QLabel('Acceleration (%):'), 0, 4)
        self.jog_acc_input = QLineEdit('40')
        self.jog_acc_input.setFixedWidth(60)
        collect_layout.addWidget(self.jog_acc_input, 0, 5)

        self.collect_button = QPushButton('Execute and Collect Data')
        self.collect_button.setStyleSheet('font-size: 12pt; font-weight: bold; padding: 10px;')
        self.collect_button.clicked.connect(self.start_data_collection)
        collect_layout.addWidget(self.collect_button, 1, 0, 1, 6)

        self.collection_status_label = QLabel('Status: Ready')
        self.collection_status_label.setStyleSheet('font-family: monospace; color: #666;')
        collect_layout.addWidget(self.collection_status_label, 2, 0, 1, 6)

        collect_group.setLayout(collect_layout)
        main_layout.addWidget(collect_group)

        # Digital Output Control Group
        do_group = QGroupBox('Digital Output Control')
        do_layout = QHBoxLayout()

        do_layout.addWidget(QLabel('DO3:'))

        self.do3_on_button = QPushButton('ON')
        self.do3_on_button.setStyleSheet('font-size: 12pt; font-weight: bold; padding: 8px 20px; background-color: #4CAF50; color: white;')
        self.do3_on_button.clicked.connect(lambda: self.set_digital_output(3, 1))
        do_layout.addWidget(self.do3_on_button)

        self.do3_off_button = QPushButton('OFF')
        self.do3_off_button.setStyleSheet('font-size: 12pt; font-weight: bold; padding: 8px 20px; background-color: #f44336; color: white;')
        self.do3_off_button.clicked.connect(lambda: self.set_digital_output(3, 0))
        do_layout.addWidget(self.do3_off_button)

        do_layout.addStretch()

        do_group.setLayout(do_layout)
        main_layout.addWidget(do_group)

        # Create publisher for digital output control
        self.do_publisher = self.ros_node.create_publisher(Int32MultiArray, '/set_do', 10)

        self.setLayout(main_layout)

        # Update timer - 30ms (33 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)

        # Subscribe to cartesian position for data collection
        self.ros_node.create_subscription(
            PoseStamped,
            '/cartesian_position',
            self._cartesian_position_callback,
            10
        )

    def _cartesian_position_callback(self, msg):
        """Callback for /cartesian_position topic - collects data during motion."""
        if not self.is_collecting:
            return

        # Get timestamp from message header
        stamp = msg.header.stamp
        time_sec = stamp.sec + stamp.nanosec * 1e-9

        # Get X position in mm
        x_mm = msg.pose.position.x * 1000.0

        # Calculate relative time from start
        if self.collection_start_time is None:
            self.collection_start_time = time_sec

        relative_time = time_sec - self.collection_start_time

        self.collected_data.append((relative_time, x_mm))

    def start_data_collection(self):
        """Start data collection and execute jog motion."""
        if self.is_collecting:
            return

        try:
            jog_distance = float(self.jog_distance_input.text())
            vel = float(self.jog_vel_input.text())
            acc = float(self.jog_acc_input.text())
        except ValueError:
            QMessageBox.warning(self, 'Invalid Input', 'Please enter valid numbers for distance, velocity, and acceleration.')
            return

        # Reset collection state
        self.collected_data = []
        self.collection_start_time = None
        self.is_collecting = True
        self.was_executing = False

        self.collect_button.setEnabled(False)
        self.collection_status_label.setText('Status: Starting motion and collecting data...')
        self.collection_status_label.setStyleSheet('font-family: monospace; color: #ff6600;')

        # Send jog command via REST API in background thread
        def send_jog():
            try:
                # Determine direction based on sign
                if jog_distance >= 0:
                    direction = 1  # PLUS
                    step = abs(jog_distance)
                else:
                    direction = -1  # MINUS (not 2!)
                    step = abs(jog_distance)

                response = requests.post(
                    'http://localhost:5000/jog',
                    json={
                        'axis': 1,  # X axis
                        'direction': direction,
                        'step': step,
                        'vel': vel,
                        'acc': acc
                    },
                    timeout=5.0
                )

                if response.status_code != 200:
                    self.ros_node.get_logger().error(f'Jog command failed: {response.text}')

            except Exception as e:
                self.ros_node.get_logger().error(f'Failed to send jog command: {e}')

        Thread(target=send_jog, daemon=True).start()

    def set_digital_output(self, do_id, status):
        """Set digital output via ROS2 topic."""
        msg = Int32MultiArray()
        msg.data = [do_id, status]
        self.do_publisher.publish(msg)
        self.ros_node.get_logger().info(f'Published DO{do_id} = {status} ({"ON" if status else "OFF"})')

    def update_display(self):
        # Calculate update rate
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Track timing samples
        self.update_times.append(dt)
        if len(self.update_times) > self.max_timing_samples:
            self.update_times.pop(0)

        # Calculate average Hz and ms
        if len(self.update_times) > 0:
            avg_dt = sum(self.update_times) / len(self.update_times)
            hz = 1.0 / avg_dt if avg_dt > 0 else 0
            ms = avg_dt * 1000.0
            self.update_rate_label.setText(f'Update Rate: {hz:.1f} Hz ({ms:.1f} ms)')

        data = self.robot_monitor.get_stable_data()
        if data is None:
            return

        # Update Cartesian Position
        cart_pos = data['cartesian']
        for i in range(6):
            self.cart_pos_labels[i].setText(f'{cart_pos[i]:.3f}')

        # Update Cartesian Velocity
        cart_vel = data['cart_velocity']
        for i in range(3):
            self.cart_vel_labels[i].setText(f'{cart_vel[i]:.2f}')
        self.cart_vel_mag_label.setText(f'{data["cart_vel_magnitude"]:.2f}')

        # Update Cartesian Acceleration
        cart_acc = data['cart_acceleration']
        for i in range(3):
            self.cart_acc_labels[i].setText(f'{cart_acc[i]:.2f}')
        self.cart_acc_mag_label.setText(f'{data["cart_acc_magnitude"]:.2f}')

        # Update Joint Velocities
        joint_vel = data['velocities']
        for i in range(6):
            self.joint_vel_labels[i].setText(f'{joint_vel[i]:.4f}')
        self.joint_vel_mag_label.setText(f'{data["vel_magnitude"]:.4f} rad/s')

        # Update Joint Accelerations
        joint_acc = data['accelerations']
        for i in range(6):
            self.joint_acc_labels[i].setText(f'{joint_acc[i]:.4f}')
        self.joint_acc_mag_label.setText(f'{data["acc_magnitude"]:.4f} rad/s^2')

        # Update Robot Status
        is_executing = self.robot_status['is_executing']

        if is_executing:
            self.execution_state_label.setText('EXECUTING')
            self.execution_state_label.setStyleSheet('font-family: monospace; font-size: 14pt; font-weight: bold; color: #ff6600;')
        else:
            self.execution_state_label.setText('IDLE')
            self.execution_state_label.setStyleSheet('font-family: monospace; font-size: 14pt; font-weight: bold; color: #00cc00;')

        if self.robot_status['is_available']:
            self.available_label.setText('YES')
            self.available_label.setStyleSheet('font-family: monospace; font-size: 12pt; color: #00cc00;')
        else:
            self.available_label.setText('NO')
            self.available_label.setStyleSheet('font-family: monospace; font-size: 12pt; color: #ff0000;')

        queue_size = self.robot_status['queue_size']
        self.queue_size_label.setText(f'{queue_size}')
        if queue_size > 0:
            self.queue_size_label.setStyleSheet('font-family: monospace; font-size: 12pt; color: #ff6600; font-weight: bold;')
        else:
            self.queue_size_label.setStyleSheet('font-family: monospace; font-size: 12pt;')

        task_id = self.robot_status['current_task_id']
        self.task_id_label.setText(f'{task_id}' if task_id is not None else 'None')

        # Check if data collection should stop
        if self.is_collecting:
            self.collection_status_label.setText(f'Status: Collecting... ({len(self.collected_data)} samples)')

            # Track execution state
            if is_executing:
                self.was_executing = True

            # Stop collecting when motion completes (was executing, now idle)
            if self.was_executing and not is_executing:
                self.is_collecting = False
                self.collect_button.setEnabled(True)
                self.collection_status_label.setText(f'Status: Collection complete! ({len(self.collected_data)} samples)')
                self.collection_status_label.setStyleSheet('font-family: monospace; color: #00cc00;')

                # Trigger plot generation from main thread
                self.plot_ready_signal.emit()

    def _show_plot(self):
        """Generate and show the analysis plot (called from main thread via signal)."""
        if len(self.collected_data) < 10:
            QMessageBox.warning(self, 'Insufficient Data', f'Only collected {len(self.collected_data)} samples. Need at least 10.')
            return

        self._generate_analysis_plot()

    def _generate_analysis_plot(self):
        """Generate matplotlib plot with position, velocity, acceleration, and jerk."""
        import matplotlib.pyplot as plt
        from scipy.ndimage import uniform_filter1d

        # Convert data to numpy arrays
        data = np.array(self.collected_data)
        t = data[:, 0]
        x = data[:, 1]

        # Sort by time (in case of out-of-order messages)
        sort_idx = np.argsort(t)
        t = t[sort_idx]
        x = x[sort_idx]

        # Remove duplicate times
        unique_mask = np.diff(t, prepend=-1) > 0
        t = t[unique_mask]
        x = x[unique_mask]

        if len(t) < 10:
            QMessageBox.warning(self, 'Insufficient Data', 'Not enough unique time samples after filtering.')
            return

        # Smooth position data slightly to reduce noise
        x_smooth = uniform_filter1d(x, size=3)

        # Calculate derivatives
        dt = np.diff(t)
        dt[dt == 0] = 1e-6  # Avoid division by zero

        # Velocity (first derivative of position)
        velocity = np.diff(x_smooth) / dt
        t_vel = t[:-1] + dt / 2  # Midpoint times

        # Smooth velocity
        velocity_smooth = uniform_filter1d(velocity, size=5)

        # Acceleration (derivative of velocity)
        dt_vel = np.diff(t_vel)
        dt_vel[dt_vel == 0] = 1e-6
        acceleration = np.diff(velocity_smooth) / dt_vel
        t_acc = t_vel[:-1] + dt_vel / 2

        # Smooth acceleration
        acceleration_smooth = uniform_filter1d(acceleration, size=5)

        # Jerk (derivative of acceleration)
        dt_acc = np.diff(t_acc)
        dt_acc[dt_acc == 0] = 1e-6
        jerk = np.diff(acceleration_smooth) / dt_acc
        t_jerk = t_acc[:-1] + dt_acc / 2

        # Calculate statistics
        max_vel = np.max(np.abs(velocity_smooth))
        max_acc = np.max(np.abs(acceleration_smooth))
        max_jerk = np.max(np.abs(jerk))

        # Create plot
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(f'Motion Analysis - X Axis\n'
                     f'Max Velocity: {max_vel:.1f} mm/s | '
                     f'Max Acceleration: {max_acc:.1f} mm/s^2 | '
                     f'Max Jerk: {max_jerk:.1f} mm/s^3',
                     fontsize=12, fontweight='bold')

        # Position plot
        axes[0].plot(t, x, 'b-', linewidth=1, alpha=0.5, label='Raw')
        axes[0].plot(t, x_smooth, 'b-', linewidth=2, label='Smoothed')
        axes[0].set_ylabel('Position (mm)')
        axes[0].legend(loc='upper right')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('Position')

        # Velocity plot
        axes[1].plot(t_vel, velocity, 'g-', linewidth=1, alpha=0.5, label='Raw')
        axes[1].plot(t_vel, velocity_smooth, 'g-', linewidth=2, label='Smoothed')
        axes[1].axhline(y=max_vel, color='r', linestyle='--', alpha=0.5, label=f'Max: {max_vel:.1f}')
        axes[1].axhline(y=-max_vel, color='r', linestyle='--', alpha=0.5)
        axes[1].set_ylabel('Velocity (mm/s)')
        axes[1].legend(loc='upper right')
        axes[1].grid(True, alpha=0.3)
        axes[1].set_title('Velocity')

        # Acceleration plot
        axes[2].plot(t_acc, acceleration, 'orange', linewidth=1, alpha=0.5, label='Raw')
        axes[2].plot(t_acc, acceleration_smooth, 'orange', linewidth=2, label='Smoothed')
        axes[2].axhline(y=max_acc, color='r', linestyle='--', alpha=0.5, label=f'Max: {max_acc:.1f}')
        axes[2].axhline(y=-max_acc, color='r', linestyle='--', alpha=0.5)
        axes[2].set_ylabel('Acceleration (mm/s^2)')
        axes[2].legend(loc='upper right')
        axes[2].grid(True, alpha=0.3)
        axes[2].set_title('Acceleration')

        # Jerk plot
        axes[3].plot(t_jerk, jerk, 'r-', linewidth=1.5)
        axes[3].axhline(y=max_jerk, color='purple', linestyle='--', alpha=0.5, label=f'Max: {max_jerk:.1f}')
        axes[3].axhline(y=-max_jerk, color='purple', linestyle='--', alpha=0.5)
        axes[3].set_ylabel('Jerk (mm/s^3)')
        axes[3].set_xlabel('Time (s)')
        axes[3].legend(loc='upper right')
        axes[3].grid(True, alpha=0.3)
        axes[3].set_title('Jerk')

        plt.tight_layout()
        plt.show()

        # Log statistics
        self.ros_node.get_logger().info(f'Motion Analysis Complete:')
        self.ros_node.get_logger().info(f'  Samples: {len(self.collected_data)}')
        self.ros_node.get_logger().info(f'  Duration: {t[-1] - t[0]:.3f} s')
        self.ros_node.get_logger().info(f'  Max Velocity: {max_vel:.1f} mm/s')
        self.ros_node.get_logger().info(f'  Max Acceleration: {max_acc:.1f} mm/s^2')
        self.ros_node.get_logger().info(f'  Max Jerk: {max_jerk:.1f} mm/s^3')

    def robot_status_callback(self, msg):
        """Callback for /robot_status topic."""
        try:
            status_data = json.loads(msg.data)
            self.robot_status.update(status_data)
        except json.JSONDecodeError as e:
            self.ros_node.get_logger().error(f'Failed to parse robot status: {e}')


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    ros_node = Node('simple_monitor_gui')

    robot_monitor = RobotMonitor(ros_node, stable_update_rate_hz=50.0)

    # Create GUI first
    app = QApplication(sys.argv)
    gui = SimpleMonitorGUI(ros_node, robot_monitor)

    # Subscribe to robot status topic
    ros_node.create_subscription(
        String,
        '/robot_status',
        gui.robot_status_callback,
        10
    )
    ros_node.get_logger().info('[SimpleMonitor] Subscribed to /robot_status')

    # Start ROS spinning in background thread
    thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    thread.start()

    # Show GUI and run
    gui.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()