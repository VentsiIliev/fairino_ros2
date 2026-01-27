#!/usr/bin/env python3
import sys
import time
from threading import Thread
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox, QGridLayout
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
from status.robot_monitor import RobotMonitor
class SimpleMonitorGUI(QWidget):
    def __init__(self, ros_node, robot_monitor):
        super().__init__()
        self.ros_node = ros_node
        self.robot_monitor = robot_monitor

        # Timing tracking for update rate
        self.last_update_time = time.time()
        self.update_times = []
        self.max_timing_samples = 30

        self.setWindowTitle('ROS2 Robot Monitor - Topic Data Display')
        self.setMinimumSize(800, 600)
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
        for i, axis in enumerate(['X (mm)', 'Y (mm)', 'Z (mm)', 'RX (°)', 'RY (°)', 'RZ (°)']):
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
        cart_vel_layout.addWidget(QLabel('/cartesian_acceleration (mm/s²)'), 2, 0)
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
        # Joint Accelerations Group
        joint_acc_group = QGroupBox('Joint Accelerations (from /joint_acceleration)')
        joint_acc_layout = QGridLayout()
        joint_acc_layout.addWidget(QLabel('Joint'), 0, 0)
        joint_acc_layout.addWidget(QLabel('Acceleration (rad/s²)'), 0, 1)
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
        self.joint_acc_mag_label = QLabel('0.0000 rad/s²')
        self.joint_acc_mag_label.setStyleSheet('font-family: monospace; font-weight: bold;')
        joint_acc_layout.addWidget(label_mag, 7, 0)
        joint_acc_layout.addWidget(self.joint_acc_mag_label, 7, 1)
        joint_acc_group.setLayout(joint_acc_layout)
        main_layout.addWidget(joint_acc_group)
        self.setLayout(main_layout)
        # Update timer - 30ms (33 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)
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

        data = self.robot_monitor.get_latest_data()
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
        self.joint_acc_mag_label.setText(f'{data["acc_magnitude"]:.4f} rad/s²')
def ros_spin_thread(node):
    rclpy.spin(node)
def main():
    rclpy.init()
    # Create ROS node
    ros_node = Node('simple_monitor_gui')
    # Create RobotMonitor - subscribes to all topics from robot_state_publisher.cpp
    robot_monitor = RobotMonitor(ros_node)
    # Start ROS spinning in background thread
    thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    thread.start()
    # Create and run PyQt6 GUI
    app = QApplication(sys.argv)
    gui = SimpleMonitorGUI(ros_node, robot_monitor)
    gui.show()
    sys.exit(app.exec())
if __name__ == '__main__':
    main()
