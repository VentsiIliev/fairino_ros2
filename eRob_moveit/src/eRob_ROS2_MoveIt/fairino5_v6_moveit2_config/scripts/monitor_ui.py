from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QGridLayout
from PyQt6.QtCore import QTimer
import sys
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float64

class ROS2Listener(Node):
    def __init__(self, ui_callback):
        super().__init__('monitor_ui_node')
        self.ui_callback = ui_callback
        self.create_subscription(Float64, 'fairino/current_velocity', self.vel_cb, 10)
        self.create_subscription(Float64, 'fairino/current_acceleration', self.acc_cb, 10)

    def vel_cb(self, msg):
        self.ui_callback('vel', msg.data)

    def acc_cb(self, msg):
        self.ui_callback('acc', msg.data)

class MonitorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Velocity & Acceleration Monitor')
        layout = QGridLayout()
        self.vel_label = QLabel('Velocity: 0.0')
        self.acc_label = QLabel('Acceleration: 0.0')
        layout.addWidget(self.vel_label, 0, 0)
        layout.addWidget(self.acc_label, 1, 0)
        self.setLayout(layout)
        self.vel = 0.0
        self.acc = 0.0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(200)

    def ros_update(self, key, value):
        if key == 'vel':
            self.vel = value
        elif key == 'acc':
            self.acc = value

    def update_labels(self):
        self.vel_label.setText(f'Velocity: {self.vel:.2f}')
        self.acc_label.setText(f'Acceleration: {self.acc:.2f}')

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    win = MonitorWindow()
    ros_node = ROS2Listener(win.ros_update)
    thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    thread.start()
    win.show()
    app.exec()
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
