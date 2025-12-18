from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QGridLayout
from PyQt6.QtCore import QTimer
import sys
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Float64, Float64MultiArray
from fairino_msgs.msg import RobotNonrtState

class ROS2Listener(Node):
    def __init__(self, ui_callback):
        super().__init__('monitor_ui_node')
        self.ui_callback = ui_callback
        self.create_subscription(RobotNonrtState, 'nonrt_state_data', self.state_cb, 10)
    def state_cb(self, msg):
        joint = [msg.j1_cur_pos, msg.j2_cur_pos, msg.j3_cur_pos, msg.j4_cur_pos, msg.j5_cur_pos, msg.j6_cur_pos]
        cart = [msg.cart_x_cur_pos, msg.cart_y_cur_pos, msg.cart_z_cur_pos]
        self.ui_callback('joint', joint)
        self.ui_callback('cart', cart)
        # velocity/acceleration not directly available, set to 0.0 or extract if present
        self.ui_callback('vel', 0.0)
        self.ui_callback('acc', 0.0)

class MonitorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Robot State Monitor')
        layout = QGridLayout()
        self.vel_label = QLabel('Velocity: 0.0')
        self.acc_label = QLabel('Acceleration: 0.0')
        self.cart_label = QLabel('Cartesian Coords: (0.0, 0.0, 0.0)')
        self.joint_label = QLabel('Joint States: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)')
        layout.addWidget(self.vel_label, 0, 0)
        layout.addWidget(self.acc_label, 1, 0)
        layout.addWidget(self.cart_label, 2, 0)
        layout.addWidget(self.joint_label, 3, 0)
        self.setLayout(layout)
        self.vel = 0.0
        self.acc = 0.0
        self.cart = (0.0, 0.0, 0.0)
        self.joint = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(200)
    def ros_update(self, key, value):
        if key == 'vel':
            self.vel = value
        elif key == 'acc':
            self.acc = value
        elif key == 'cart':
            if isinstance(value, (list, tuple)) and len(value) >= 3:
                self.cart = tuple(value[:3])
        elif key == 'joint':
            if isinstance(value, (list, tuple)) and len(value) >= 6:
                self.joint = tuple(value[:6])
    def update_labels(self):
        self.vel_label.setText(f'Velocity: {self.vel:.2f}')
        self.acc_label.setText(f'Acceleration: {self.acc:.2f}')
        self.cart_label.setText(f'Cartesian Coords: ({self.cart[0]:.2f}, {self.cart[1]:.2f}, {self.cart[2]:.2f})')
        self.joint_label.setText(f'Joint States: {self.joint}')

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
