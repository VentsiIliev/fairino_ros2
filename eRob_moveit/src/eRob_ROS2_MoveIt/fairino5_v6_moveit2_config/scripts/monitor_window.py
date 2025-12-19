import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox, QGridLayout, QLineEdit, QHBoxLayout, QPushButton, \
    QListWidget, QComboBox
from enums import RobotAxis,Direction

class MonitorWindow(QWidget):
    def __init__(self, ros_node, robot):
        super().__init__()
        self.selected_frame = "base_link"
        self.ros_node = ros_node
        self.robot = robot  # Use the wrapper, not the raw ROS node
        self.setWindowTitle('Fairino Robot Velocity & Acceleration Monitor')
        self.resize(850, 900)

        main_layout = QVBoxLayout()

        title_label = QLabel('Real-Time Robot Monitor')
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title_label)

        # Tool and WorkObject Configuration Group
        config_group = QGroupBox('Tool & WorkObject Configuration')
        config_layout = QGridLayout()

        # Tool Selector
        config_layout.addWidget(QLabel('Active Tool:'), 0, 0)
        self.tool_selector = QComboBox()
        self.tool_selector.addItem('TOOL_0 (No Offset)')
        self.tool_selector.addItem('TOOL_1 (0.081, -7.250, 0)')
        self.tool_selector.setCurrentIndex(0)
        config_layout.addWidget(self.tool_selector, 0, 1, 1, 2)

        self.apply_tool_btn = QPushButton('Apply Tool')
        self.apply_tool_btn.setStyleSheet(
            'QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 8px; }')
        self.apply_tool_btn.clicked.connect(self.apply_tool)
        config_layout.addWidget(self.apply_tool_btn, 0, 3)

        # WorkObject Configuration
        config_layout.addWidget(QLabel('WorkObject:'), 1, 0)

        # WorkObject position inputs
        config_layout.addWidget(QLabel('X (mm):'), 2, 0)
        self.wo_x_input = QLineEdit('0.0')
        self.wo_x_input.setMaximumWidth(80)
        config_layout.addWidget(self.wo_x_input, 2, 1)

        config_layout.addWidget(QLabel('Y (mm):'), 2, 2)
        self.wo_y_input = QLineEdit('0.0')
        self.wo_y_input.setMaximumWidth(80)
        config_layout.addWidget(self.wo_y_input, 2, 3)

        config_layout.addWidget(QLabel('Z (mm):'), 2, 4)
        self.wo_z_input = QLineEdit('0.0')
        self.wo_z_input.setMaximumWidth(80)
        config_layout.addWidget(self.wo_z_input, 2, 5)

        # WorkObject orientation inputs
        config_layout.addWidget(QLabel('RX (°):'), 3, 0)
        self.wo_rx_input = QLineEdit('0.0')
        self.wo_rx_input.setMaximumWidth(80)
        config_layout.addWidget(self.wo_rx_input, 3, 1)

        config_layout.addWidget(QLabel('RY (°):'), 3, 2)
        self.wo_ry_input = QLineEdit('0.0')
        self.wo_ry_input.setMaximumWidth(80)
        config_layout.addWidget(self.wo_ry_input, 3, 3)

        config_layout.addWidget(QLabel('RZ (°):'), 3, 4)
        self.wo_rz_input = QLineEdit('-10.0')
        self.wo_rz_input.setMaximumWidth(80)
        config_layout.addWidget(self.wo_rz_input, 3, 5)

        # WorkObject action buttons
        wo_btn_layout = QHBoxLayout()

        self.apply_wo_btn = QPushButton('Apply WorkObject')
        self.apply_wo_btn.setStyleSheet(
            'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }')
        self.apply_wo_btn.clicked.connect(self.apply_workobject)
        wo_btn_layout.addWidget(self.apply_wo_btn)

        self.clear_wo_btn = QPushButton('Clear WorkObject (Identity)')
        self.clear_wo_btn.setStyleSheet(
            'QPushButton { background-color: #9E9E9E; color: white; font-weight: bold; padding: 8px; }')
        self.clear_wo_btn.clicked.connect(self.clear_workobject)
        wo_btn_layout.addWidget(self.clear_wo_btn)

        config_layout.addLayout(wo_btn_layout, 4, 0, 1, 6)

        # WorkObject status label
        self.wo_status_label = QLabel('Active WorkObject: Loading...')
        self.wo_status_label.setStyleSheet('color: #666; font-style: italic;')
        config_layout.addWidget(self.wo_status_label, 5, 0, 1, 6)

        config_group.setLayout(config_layout)
        main_layout.addWidget(config_group)

        control_group = QGroupBox('Cartesian Position Control')
        control_layout = QGridLayout()

        # Reference frame selector
        control_layout.addWidget(QLabel('Frame:'), 0, 0)
        self.frame_selector = QComboBox()
        self.frame_selector.addItem('Base Frame')
        self.frame_selector.addItem('Tool Frame (ee_link)')
        self.frame_selector.setCurrentIndex(0)
        self.frame_selector.currentIndexChanged.connect(self.on_frame_changed)
        control_layout.addWidget(self.frame_selector, 0, 1)

        control_layout.addWidget(QLabel('X (mm):'), 1, 0)
        self.x_input = QLineEdit('300.0')
        control_layout.addWidget(self.x_input, 1, 1)

        control_layout.addWidget(QLabel('Y (mm):'), 1, 2)
        self.y_input = QLineEdit('0.0')
        control_layout.addWidget(self.y_input, 1, 3)

        control_layout.addWidget(QLabel('Z (mm):'), 1, 4)
        self.z_input = QLineEdit('400.0')
        control_layout.addWidget(self.z_input, 1, 5)

        control_layout.addWidget(QLabel('rX (°):'), 2, 0)
        self.rx_input = QLineEdit('180.0')
        control_layout.addWidget(self.rx_input, 2, 1)

        control_layout.addWidget(QLabel('rY (°):'), 2, 2)
        self.ry_input = QLineEdit('0.0')
        control_layout.addWidget(self.ry_input, 2, 3)

        control_layout.addWidget(QLabel('rZ (°):'), 2, 4)
        self.rz_input = QLineEdit('0.0')
        control_layout.addWidget(self.rz_input, 2, 5)

        # Auto-update toggle
        self.auto_update_position = True  # Enable auto-update by default

        # Connect textChanged signals to disable auto-update when user edits
        self.x_input.textEdited.connect(self.on_position_manually_edited)
        self.y_input.textEdited.connect(self.on_position_manually_edited)
        self.z_input.textEdited.connect(self.on_position_manually_edited)
        self.rx_input.textEdited.connect(self.on_position_manually_edited)
        self.ry_input.textEdited.connect(self.on_position_manually_edited)
        self.rz_input.textEdited.connect(self.on_position_manually_edited)

        button_layout = QHBoxLayout()

        # Linear motion button
        self.move_linear_button = QPushButton('Move Linear (LIN)')
        self.move_linear_button.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 10px; }')
        self.move_linear_button.clicked.connect(self.send_move_linear_command)
        button_layout.addWidget(self.move_linear_button)

        # Cartesian (PTP) motion button
        self.move_cartesian_button = QPushButton('Move Cartesian (PTP)')
        self.move_cartesian_button.setStyleSheet(
            'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }')
        self.move_cartesian_button.clicked.connect(self.send_move_cartesian_command)
        button_layout.addWidget(self.move_cartesian_button)

        control_layout.addLayout(button_layout, 3, 0, 1, 6)

        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)

        # Jog Control Group
        jog_group = QGroupBox('Jog Control (Base Frame)')
        jog_layout = QGridLayout()

        # Step size input
        jog_layout.addWidget(QLabel('Step (mm):'), 0, 1)
        self.jog_step_input = QLineEdit('5.0')
        jog_layout.addWidget(self.jog_step_input, 0, 2)

        # X axis buttons
        btn_x_minus = QPushButton('-X')
        btn_x_plus = QPushButton('+X')
        btn_x_minus.clicked.connect(lambda: self.robot.start_jog(axis=RobotAxis.X, direction=Direction.MINUS,
                                                                 step=float(self.jog_step_input.text()),
                                                                 vel=float(self.path_vel_input.text()) * 100,
                                                                 acc=float(self.path_acc_input.text()) * 100))
        btn_x_plus.clicked.connect(lambda: self.robot.start_jog(RobotAxis.X, direction=Direction.PLUS,
                                                                step=float(self.jog_step_input.text()),
                                                                vel=float(self.path_vel_input.text()) * 100,
                                                                acc=float(self.path_acc_input.text()) * 100))

        jog_layout.addWidget(btn_x_minus, 1, 0)
        jog_layout.addWidget(QLabel('X'), 1, 1)
        jog_layout.addWidget(btn_x_plus, 1, 2)

        # Y axis buttons
        btn_y_minus = QPushButton('-Y')
        btn_y_plus = QPushButton('+Y')
        btn_y_minus.clicked.connect(lambda: self.robot.start_jog(axis=RobotAxis.Y, direction=Direction.MINUS,
                                                                 step=float(self.jog_step_input.text()),
                                                                 vel=float(self.path_vel_input.text()) * 100,
                                                                 acc=float(self.path_acc_input.text()) * 100))
        btn_y_plus.clicked.connect(lambda: self.robot.start_jog(axis=RobotAxis.Y, direction=Direction.PLUS,
                                                                step=float(self.jog_step_input.text()),
                                                                vel=float(self.path_vel_input.text()) * 100,
                                                                acc=float(self.path_acc_input.text()) * 100))

        jog_layout.addWidget(btn_y_minus, 2, 0)
        jog_layout.addWidget(QLabel('Y'), 2, 1)
        jog_layout.addWidget(btn_y_plus, 2, 2)

        # Z axis buttons
        btn_z_minus = QPushButton('-Z')
        btn_z_plus = QPushButton('+Z')
        btn_z_minus.clicked.connect(lambda: self.robot.start_jog(axis=RobotAxis.Z, direction=Direction.MINUS,
                                                                 step=float(self.jog_step_input.text()),
                                                                 vel=float(self.path_vel_input.text()) * 100,
                                                                 acc=float(self.path_acc_input.text()) * 100))
        btn_z_plus.clicked.connect(lambda: self.robot.start_jog(axis=RobotAxis.Z, direction=Direction.PLUS,
                                                                step=float(self.jog_step_input.text()),
                                                                vel=float(self.path_vel_input.text()) * 100,
                                                                acc=float(self.path_acc_input.text()) * 100))

        jog_layout.addWidget(btn_z_minus, 3, 0)
        jog_layout.addWidget(QLabel('Z'), 3, 1)
        jog_layout.addWidget(btn_z_plus, 3, 2)

        jog_group.setLayout(jog_layout)
        main_layout.addWidget(jog_group)

        # Path Planning Group
        path_group = QGroupBox('Path Planning - Multiple Waypoints')
        path_layout = QGridLayout()

        # Waypoint input fields
        path_layout.addWidget(QLabel('X (mm):'), 0, 0)
        self.path_x_input = QLineEdit('300.0')
        path_layout.addWidget(self.path_x_input, 0, 1)

        path_layout.addWidget(QLabel('Y (mm):'), 0, 2)
        self.path_y_input = QLineEdit('0.0')
        path_layout.addWidget(self.path_y_input, 0, 3)

        path_layout.addWidget(QLabel('Z (mm):'), 0, 4)
        self.path_z_input = QLineEdit('400.0')
        path_layout.addWidget(self.path_z_input, 0, 5)

        # Waypoint management buttons
        waypoint_btn_layout = QHBoxLayout()

        self.add_waypoint_btn = QPushButton('Add Waypoint')
        self.add_waypoint_btn.clicked.connect(self.add_waypoint)
        waypoint_btn_layout.addWidget(self.add_waypoint_btn)

        self.add_current_waypoint_btn = QPushButton('Add Current Position')
        self.add_current_waypoint_btn.clicked.connect(self.add_current_waypoint)
        waypoint_btn_layout.addWidget(self.add_current_waypoint_btn)

        self.remove_waypoint_btn = QPushButton('Remove Selected')
        self.remove_waypoint_btn.clicked.connect(self.remove_waypoint)
        waypoint_btn_layout.addWidget(self.remove_waypoint_btn)

        self.clear_waypoints_btn = QPushButton('Clear All')
        self.clear_waypoints_btn.clicked.connect(self.clear_waypoints)
        waypoint_btn_layout.addWidget(self.clear_waypoints_btn)

        path_layout.addLayout(waypoint_btn_layout, 1, 0, 1, 6)

        # Circle generation controls (separate row)
        circle_btn_layout = QHBoxLayout()
        self.gen_circle_btn = QPushButton('Generate Circle')
        self.gen_circle_btn.setStyleSheet(
            'QPushButton { background-color: #9C27B0; color: white; font-weight: bold; padding: 8px; }')
        self.gen_circle_btn.clicked.connect(self.generate_circle_path)
        circle_btn_layout.addWidget(self.gen_circle_btn)

        # Number of points input
        circle_btn_layout.addWidget(QLabel('Points:'))
        self.circle_points_input = QLineEdit('16')
        self.circle_points_input.setMaximumWidth(40)
        circle_btn_layout.addWidget(self.circle_points_input)

        # Radius input
        circle_btn_layout.addWidget(QLabel('Radius (mm):'))
        self.circle_radius_input = QLineEdit('50.0')
        self.circle_radius_input.setMaximumWidth(60)
        circle_btn_layout.addWidget(self.circle_radius_input)

        circle_btn_layout.addStretch()
        path_layout.addLayout(circle_btn_layout, 2, 0, 1, 6)

        # Rectangle generation controls (separate row)
        rectangle_btn_layout = QHBoxLayout()
        self.gen_rectangle_btn = QPushButton('Generate Rectangle')
        self.gen_rectangle_btn.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; }')
        self.gen_rectangle_btn.clicked.connect(self.generate_rectangle_path)
        rectangle_btn_layout.addWidget(self.gen_rectangle_btn)

        # Rectangle width and height inputs
        rectangle_btn_layout.addWidget(QLabel('Width (mm):'))
        self.rect_width_input = QLineEdit('100.0')
        self.rect_width_input.setMaximumWidth(60)
        rectangle_btn_layout.addWidget(self.rect_width_input)

        rectangle_btn_layout.addWidget(QLabel('Height (mm):'))
        self.rect_height_input = QLineEdit('50.0')
        self.rect_height_input.setMaximumWidth(60)
        rectangle_btn_layout.addWidget(self.rect_height_input)

        rectangle_btn_layout.addStretch()
        path_layout.addLayout(rectangle_btn_layout, 3, 0, 1, 6)

        # Waypoint list display
        self.waypoint_list = QListWidget()
        self.waypoint_list.setMaximumHeight(120)
        path_layout.addWidget(QLabel('Waypoints:'), 4, 0, 1, 6)
        path_layout.addWidget(self.waypoint_list, 5, 0, 1, 6)

        # Planning method selector
        path_layout.addWidget(QLabel('Planner:'), 6, 0)
        self.planning_method_combo = QComboBox()
        self.planning_method_combo.addItem('Cartesian Path (MoveIt) + TOTG')
        self.planning_method_combo.setMaximumWidth(250)
        path_layout.addWidget(self.planning_method_combo, 6, 1, 1, 2)

        # Velocity and acceleration scaling (optimized for smooth motion)
        path_layout.addWidget(QLabel('Velocity:'), 7, 0)
        self.path_vel_input = QLineEdit('0.6')  # Optimized default
        self.path_vel_input.setMaximumWidth(60)
        path_layout.addWidget(self.path_vel_input, 7, 1)

        path_layout.addWidget(QLabel('Accel:'), 7, 2)
        self.path_acc_input = QLineEdit('0.4')  # Optimized default
        self.path_acc_input.setMaximumWidth(60)
        path_layout.addWidget(self.path_acc_input, 7, 3)

        # Execute path button
        self.execute_path_btn = QPushButton('Execute Path')
        self.execute_path_btn.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 10px; }')
        self.execute_path_btn.clicked.connect(self.execute_path)
        path_layout.addWidget(self.execute_path_btn, 7, 4, 1, 1)

        # Stop motion button
        self.stop_motion_btn = QPushButton('STOP')
        self.stop_motion_btn.setStyleSheet(
            'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; font-size: 14px; }')
        self.stop_motion_btn.clicked.connect(self.stop_motion)
        path_layout.addWidget(self.stop_motion_btn, 7, 5, 1, 1)

        path_group.setLayout(path_layout)
        main_layout.addWidget(path_group)

        # Initialize waypoints storage
        self.waypoints = []

        cartesian_group = QGroupBox('Cartesian End-Effector')
        cartesian_layout = QGridLayout()

        cartesian_layout.addWidget(QLabel('Axis'), 0, 0)
        cartesian_layout.addWidget(QLabel('Position (mm)'), 0, 1)
        cartesian_layout.addWidget(QLabel('Velocity (mm/s)'), 0, 2)
        cartesian_layout.addWidget(QLabel('Acceleration (mm/s²)'), 0, 3)

        self.cart_labels = []
        for i, axis in enumerate(['X', 'Y', 'Z']):
            label_axis = QLabel(axis)
            label_pos = QLabel('0.000')
            label_vel = QLabel('0.000')
            label_acc = QLabel('0.000')

            cartesian_layout.addWidget(label_axis, i + 1, 0)
            cartesian_layout.addWidget(label_pos, i + 1, 1)
            cartesian_layout.addWidget(label_vel, i + 1, 2)
            cartesian_layout.addWidget(label_acc, i + 1, 3)

            self.cart_labels.append((label_pos, label_vel, label_acc))

        cartesian_layout.addWidget(QLabel(''), 4, 0)
        cartesian_layout.addWidget(QLabel('Magnitude:'), 5, 0)
        self.cart_vel_mag_label = QLabel('0.000 mm/s')
        self.cart_acc_mag_label = QLabel('0.000 mm/s²')
        cartesian_layout.addWidget(QLabel(''), 5, 1)
        cartesian_layout.addWidget(self.cart_vel_mag_label, 5, 2)
        cartesian_layout.addWidget(self.cart_acc_mag_label, 5, 3)

        cartesian_group.setLayout(cartesian_layout)
        main_layout.addWidget(cartesian_group)

        magnitude_group = QGroupBox('Joint Space Magnitudes')
        magnitude_layout = QGridLayout()
        self.vel_mag_label = QLabel('Velocity: 0.000 rad/s')
        self.acc_mag_label = QLabel('Acceleration: 0.000 rad/s²')
        magnitude_layout.addWidget(self.vel_mag_label, 0, 0)
        magnitude_layout.addWidget(self.acc_mag_label, 1, 0)
        magnitude_group.setLayout(magnitude_layout)
        main_layout.addWidget(magnitude_group)

        joint_group = QGroupBox('Joint Details')
        joint_layout = QGridLayout()

        header_labels = ['Joint', 'Position (rad)', 'Velocity (rad/s)', 'Acceleration (rad/s²)']
        for col, text in enumerate(header_labels):
            label = QLabel(text)
            label.setStyleSheet('font-weight: bold;')
            joint_layout.addWidget(label, 0, col)

        self.joint_labels = []
        for i in range(6):
            row_labels = []
            label_joint = QLabel(f'J{i + 1}')
            label_pos = QLabel('0.000')
            label_vel = QLabel('0.000')
            label_acc = QLabel('0.000')

            joint_layout.addWidget(label_joint, i + 1, 0)
            joint_layout.addWidget(label_pos, i + 1, 1)
            joint_layout.addWidget(label_vel, i + 1, 2)
            joint_layout.addWidget(label_acc, i + 1, 3)

            row_labels.append((label_pos, label_vel, label_acc))
            self.joint_labels.append(row_labels)

        joint_group.setLayout(joint_layout)
        main_layout.addWidget(joint_group)

        self.setLayout(main_layout)

        self.data = {
            'positions': np.zeros(6),
            'velocities': np.zeros(6),
            'accelerations': np.zeros(6),
            'vel_magnitude': 0.0,
            'acc_magnitude': 0.0,
            'cartesian': np.zeros(3),
            'cart_velocity': np.zeros(3),
            'cart_acceleration': np.zeros(3),
            'cart_vel_magnitude': 0.0,
            'cart_acc_magnitude': 0.0
        }

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)

        # Initialize workobject fields from robot's current workobject
        self.update_workobject_display()

    def on_position_manually_edited(self):
        """
        Called when the user manually edits any Cartesian position field.
        Disables auto-update so values are not overwritten by feedback.
        """
        self.auto_update_position = False

    def send_move_linear_command(self):
        """Move robot using linear (LIN) motion - straight line in Cartesian space"""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
            rx = float(self.rx_input.text())
            ry = float(self.ry_input.text())
            rz = float(self.rz_input.text())

            vel_scaling = float(self.path_vel_input.text())
            acc_scaling = float(self.path_acc_input.text())

            vel_percent = max(0.0, min(1.0, vel_scaling)) * 100.0
            acc_percent = max(0.0, min(1.0, acc_scaling)) * 100.0

            self.robot.move_liner(
                position=[x, y, z, rx, ry, rz],
                vel=vel_percent,
                acc=acc_percent,
                blocking=True
            )

            self.move_linear_button.setStyleSheet(
                'QPushButton { background-color: #FFA500; color: white; font-weight: bold; padding: 10px; }')
            self.move_linear_button.setText('Moving Linear...')
            QTimer.singleShot(2000, self.reset_linear_button)
        except ValueError:
            self.move_linear_button.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }')
            self.move_linear_button.setText('Invalid Input!')
            QTimer.singleShot(1000, self.reset_linear_button)

    def send_move_cartesian_command(self):
        """Move robot using PTP (Point-to-Point) motion - joint space, fastest but curved path"""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
            rx = float(self.rx_input.text())
            ry = float(self.ry_input.text())
            rz = float(self.rz_input.text())

            vel_scaling = float(self.path_vel_input.text())
            acc_scaling = float(self.path_acc_input.text())

            vel_percent = max(0.0, min(1.0, vel_scaling)) * 100.0
            acc_percent = max(0.0, min(1.0, acc_scaling)) * 100.0

            self.robot.move_cartesian(
                position=[x, y, z, rx, ry, rz],
                vel=vel_percent,
                acc=acc_percent
            )

            self.move_cartesian_button.setStyleSheet(
                'QPushButton { background-color: #FFA500; color: white; font-weight: bold; padding: 10px; }')
            self.move_cartesian_button.setText('Moving PTP...')
            QTimer.singleShot(2000, self.reset_cartesian_button)
        except ValueError:
            self.move_cartesian_button.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }')
            self.move_cartesian_button.setText('Invalid Input!')
            QTimer.singleShot(1000, self.reset_cartesian_button)

    def reset_linear_button(self):
        """Reset the linear motion button to the default state"""
        self.move_linear_button.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 10px; }')
        self.move_linear_button.setText('Move Linear (LIN)')

    def reset_cartesian_button(self):
        """Reset the cartesian (PTP) motion button to the default state"""
        self.move_cartesian_button.setStyleSheet(
            'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }')
        self.move_cartesian_button.setText('Move Cartesian (PTP)')
        self.auto_update_position = True

    def add_waypoint(self):
        try:
            x = float(self.path_x_input.text())
            y = float(self.path_y_input.text())
            z = float(self.path_z_input.text())

            self.waypoints.append((x, y, z))
            self.waypoint_list.addItem(f'Waypoint {len(self.waypoints)}: X={x:.1f}, Y={y:.1f}, Z={z:.1f}')
        except ValueError:
            pass

    def add_current_waypoint(self):
        """Add current robot position as a waypoint"""
        pose = self.robot.get_current_position()
        if pose is None:
            return

        x, y, z, rx, ry, rz = pose
        self.waypoints.append((x, y, z, rx, ry, rz))
        self.waypoint_list.addItem(
            f'Waypoint {len(self.waypoints)}: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, RX={rx:.1f}, RY={ry:.1f}, RZ={rz:.1f}')

    def remove_waypoint(self):
        current_row = self.waypoint_list.currentRow()
        if current_row >= 0:
            self.waypoint_list.takeItem(current_row)
            self.waypoints.pop(current_row)
            self.update_waypoint_list()

    def clear_waypoints(self):
        self.waypoints = []
        self.waypoint_list.clear()

    def update_waypoint_list(self):
        self.waypoint_list.clear()
        for i, wp in enumerate(self.waypoints, 1):
            if len(wp) >= 6:
                # Display full 6D waypoint
                self.waypoint_list.addItem(
                    f'WP{i}: X={wp[0]:.1f} Y={wp[1]:.1f} Z={wp[2]:.1f} | RX={wp[3]:.1f}° RY={wp[4]:.1f}° RZ={wp[5]:.1f}°'
                )
            else:
                # Display 3D waypoint
                self.waypoint_list.addItem(f'WP{i}: X={wp[0]:.1f}, Y={wp[1]:.1f}, Z={wp[2]:.1f}')

    def generate_circle_path(self):
        """Generate a circular path around the current position in the XY plane"""
        try:
            # Get parameters from UI
            num_points = int(self.circle_points_input.text())
            radius_mm = float(self.circle_radius_input.text())

            # Get current TCP position and orientation
            cart = self.robot.get_current_position()
            if cart is None:
                raise ValueError("Could not get current robot position")

            center_x = cart[0]
            center_y = cart[1]
            center_z = cart[2]
            rx = cart[3]  # Get current orientation
            ry = cart[4]
            rz = cart[5]

            self.robot.node.get_logger().info(
                f"[CIRCLE] Center: X={center_x:.1f}, Y={center_y:.1f}, Z={center_z:.1f}, RX={rx:.1f}°, RY={ry:.1f}°, RZ={rz:.1f}°"
            )

            # Clear existing waypoints
            self.waypoints = []

            # Generate circle waypoints in XY plane with current orientation
            for i in range(num_points):
                angle = 2.0 * np.pi * i / num_points
                x = center_x + radius_mm * np.cos(angle)
                y = center_y + radius_mm * np.sin(angle)
                z = center_z
                self.waypoints.append((x, y, z, rx, ry, rz))

            # Add the first point again to close the circle
            if num_points > 0:
                self.waypoints.append(self.waypoints[0])

            # Update the waypoint list display
            self.update_waypoint_list()

            # Update button to show success
            self.gen_circle_btn.setStyleSheet(
                'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }')
            self.gen_circle_btn.setText(f'Circle Generated ({num_points + 1} points)')
            QTimer.singleShot(2000, self.reset_circle_button)

        except ValueError:
            self.gen_circle_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }')
            self.gen_circle_btn.setText('Invalid Input!')
            QTimer.singleShot(1000, self.reset_circle_button)

    def reset_circle_button(self):
        self.gen_circle_btn.setStyleSheet(
            'QPushButton { background-color: #9C27B0; color: white; font-weight: bold; padding: 8px; }')
        radius = self.circle_radius_input.text()
        self.gen_circle_btn.setText(f'Generate Circle ({radius}mm radius)')

    def generate_rectangle_path(self):
        """Generate a rectangular path around the current position in the XY plane"""
        try:
            # Get parameters from UI
            width_mm = float(self.rect_width_input.text())
            height_mm = float(self.rect_height_input.text())

            # Get current TCP position and orientation
            cart = self.robot.get_current_position()
            if cart is None:
                raise ValueError("Could not get current robot position")

            center_x = cart[0]
            center_y = cart[1]
            center_z = cart[2]
            rx = cart[3]  # Get current orientation
            ry = cart[4]
            rz = cart[5]

            # Calculate rectangle corners
            half_w = width_mm / 2.0
            half_h = height_mm / 2.0

            corners = [
                (center_x - half_w, center_y - half_h, center_z, rx, ry, rz),
                (center_x + half_w, center_y - half_h, center_z, rx, ry, rz),
                (center_x + half_w, center_y + half_h, center_z, rx, ry, rz),
                (center_x - half_w, center_y + half_h, center_z, rx, ry, rz),
                (center_x - half_w, center_y - half_h, center_z, rx, ry, rz)  # Close the rectangle
            ]

            # Clear existing waypoints and add rectangle corners
            self.waypoints = corners
            self.update_waypoint_list()

            # Update button to show success
            self.gen_rectangle_btn.setStyleSheet(
                'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }')
            self.gen_rectangle_btn.setText(f'Rectangle Generated (4 points)')
            QTimer.singleShot(2000, self.reset_rectangle_button)

        except ValueError:
            self.gen_rectangle_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }')
            self.gen_rectangle_btn.setText('Invalid Input!')
            QTimer.singleShot(1000, self.reset_rectangle_button)

    def reset_rectangle_button(self):
        self.gen_rectangle_btn.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; }')
        self.gen_rectangle_btn.setText('Generate Rectangle')

    def execute_path(self):
        """Execute a Cartesian path through multiple waypoints using robot interface"""
        if len(self.waypoints) < 2:
            self.execute_path_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }')
            self.execute_path_btn.setText('Need 2+ Waypoints!')
            QTimer.singleShot(1500, self.reset_path_button)
            return

        try:
            vel_scaling = float(self.path_vel_input.text())
            acc_scaling = float(self.path_acc_input.text())

            # Clamp values between 0.0 and 1.0
            vel_scaling = max(0.0, min(1.0, vel_scaling))
            acc_scaling = max(0.0, min(1.0, acc_scaling))

            # Get orientation from single position control inputs
            rx = float(self.rx_input.text())
            ry = float(self.ry_input.text())
            rz = float(self.rz_input.text())

            # Use a robot interface to execute a path

            self.robot.execute_path(
                path=self.waypoints,
                rx=rx, ry=ry, rz=rz,
                vel=vel_scaling,
                acc=acc_scaling,
                blocking=False
            )

            self.execute_path_btn.setStyleSheet(
                'QPushButton { background-color: #FFA500; color: white; font-weight: bold; padding: 10px; }')
            self.execute_path_btn.setText('Executing Path...')
            QTimer.singleShot(3000, self.reset_path_button)
        except ValueError:
            self.execute_path_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }')
            self.execute_path_btn.setText('Invalid Input!')
            QTimer.singleShot(1000, self.reset_path_button)

    def reset_path_button(self):
        self.execute_path_btn.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 10px; }')
        self.execute_path_btn.setText('Execute Path')

    def ros_update(self, data):
        self.data = data

    def update_display(self):

        data = self.robot.node.get_latest_data()

        if data is None:

            return
        self.data = data


        # Cartesian display
        for i in range(3):
            pos = data['cartesian'][i] * 1000.0
            vel = data['cart_velocity'][i]
            acc = data['cart_acceleration'][i]

            labels = self.cart_labels[i]
            labels[0].setText(f'{pos:.1f}')
            labels[1].setText(f'{vel:.1f}')
            labels[2].setText(f'{acc:.1f}')

        self.cart_vel_mag_label.setText(f'{data["cart_vel_magnitude"]:.1f} mm/s')
        self.cart_acc_mag_label.setText(f'{data["cart_acc_magnitude"]:.1f} mm/s²')

        # Joint magnitudes
        self.vel_mag_label.setText(f'Velocity: {data["vel_magnitude"]:.3f} rad/s')
        self.acc_mag_label.setText(f'Acceleration: {data["acc_magnitude"]:.3f} rad/s²')

        # Joint details
        for i in range(6):
            pos = data['positions'][i]
            vel = data['velocities'][i]
            acc = data['accelerations'][i]

            labels = self.joint_labels[i][0]
            labels[0].setText(f'{pos:.3f}')
            labels[1].setText(f'{vel:.3f}')
            labels[2].setText(f'{acc:.3f}')

        # Update Cartesian Position Control inputs with current robot position
        # Only update if user is not actively editing the fields AND auto-update is enabled
        if (hasattr(self, 'auto_update_position') and self.auto_update_position and
            not (self.x_input.hasFocus() or self.y_input.hasFocus() or self.z_input.hasFocus() or
                 self.rx_input.hasFocus() or self.ry_input.hasFocus() or self.rz_input.hasFocus())):
            pose = self.robot.get_current_position()
            if pose is not None:
                x, y, z, rx, ry, rz = pose
                self.x_input.setText(f'{x:.1f}')
                self.y_input.setText(f'{y:.1f}')
                self.z_input.setText(f'{z:.1f}')
                self.rx_input.setText(f'{rx:.1f}')
                self.ry_input.setText(f'{ry:.1f}')
                self.rz_input.setText(f'{rz:.1f}')

    def update_joint_state(self, joint_state):
        self.joint_state = joint_state
        self.data_ready = True
        self.refresh_ui()

    def refresh_ui(self):
        # Add logic to update all relevant UI elements with the latest joint_state
        # Example:
        if hasattr(self, 'joint_state') and self.data_ready:
            # Update UI widgets here
            pass

    def on_joint_state_received(self, msg):
        # Called when new joint state is received
        self.update_joint_state(msg)

    def on_frame_changed(self, index):
        """Callback when the reference frame is changed"""
        print(f"[MonitorWindow] Frame changed to index: {index}")
        if index == 0:
            # Base Frame selected
            self.selected_frame = 'base_link'
            print("Switched to Base Frame")
            # All positions are already in the base frame, no transformation needed
        elif index == 1:
            # Tool Frame selected
            print("Switched to Tool Frame (ee_link)")
            self.selected_frame = 'tool_link'
            # TODO: Add transformation logic to convert positions/movements to tool frame
            print("WARNING: Tool frame transformations not yet fully implemented!")
            print("Position commands will still be sent in base frame.")

    def apply_tool(self):
        """Apply the selected tool from the dropdown"""
        try:
            tool_index = self.tool_selector.currentIndex()
            tool_names = ["TOOL_0", "TOOL_1"]

            if tool_index < len(tool_names):
                tool_name = tool_names[tool_index]
                self.ros_node.set_tool(tool_name)

                # Visual feedback
                self.apply_tool_btn.setStyleSheet(
                    'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }')
                self.apply_tool_btn.setText(f'{tool_name} Applied!')
                QTimer.singleShot(2000, self.reset_tool_button)

                self.ros_node.get_logger().info(f"Tool switched to {tool_name}")
        except Exception as e:
            self.apply_tool_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }')
            self.apply_tool_btn.setText('Failed!')
            QTimer.singleShot(1000, self.reset_tool_button)
            print(f"Error applying tool: {e}")

    def reset_tool_button(self):
        """Reset the apply tool button to the default state"""
        self.apply_tool_btn.setStyleSheet(
            'QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 8px; }')
        self.apply_tool_btn.setText('Apply Tool')

    def apply_workobject(self):
        """Apply the workobject configuration from the input fields"""
        try:
            from robot_controller import WorkObject

            x = float(self.wo_x_input.text())
            y = float(self.wo_y_input.text())
            z = float(self.wo_z_input.text())
            rx = float(self.wo_rx_input.text())
            ry = float(self.wo_ry_input.text())
            rz = float(self.wo_rz_input.text())

            # Create and set workobject
            workobject = WorkObject(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
            self.robot.set_workobject(workobject)

            # Update status label
            self.wo_status_label.setText(f'WorkObject: [{x:.1f}, {y:.1f}, {z:.1f}, {rx:.1f}, {ry:.1f}, {rz:.1f}]')

            # Visual feedback
            self.apply_wo_btn.setStyleSheet(
                'QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; }')
            self.apply_wo_btn.setText('WorkObject Applied!')
            QTimer.singleShot(2000, self.reset_wo_apply_button)

            print(f"WorkObject set to: X={x}, Y={y}, Z={z}, RX={rx}, RY={ry}, RZ={rz}")
        except ValueError:
            self.apply_wo_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }')
            self.apply_wo_btn.setText('Invalid Input!')
            QTimer.singleShot(1000, self.reset_wo_apply_button)
        except Exception as e:
            self.apply_wo_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }')
            self.apply_wo_btn.setText('Failed!')
            QTimer.singleShot(1000, self.reset_wo_apply_button)
            print(f"Error applying work object: {e}")

    def reset_wo_apply_button(self):
        """Reset the apply work object button to the default state"""
        self.apply_wo_btn.setStyleSheet(
            'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }')
        self.apply_wo_btn.setText('Apply WorkObject')

    def update_workobject_display(self):
        """Update the UI to show the current active work object"""
        try:
            # Get current workobject from robot
            if hasattr(self.robot, 'work_object') and self.robot.work_object is not None:
                wo = self.robot.work_object

                # Update input fields with current workobject values
                self.wo_x_input.setText(f'{wo.x:.1f}')
                self.wo_y_input.setText(f'{wo.y:.1f}')
                self.wo_z_input.setText(f'{wo.z:.1f}')
                self.wo_rx_input.setText(f'{wo.rx:.1f}')
                self.wo_ry_input.setText(f'{wo.ry:.1f}')
                self.wo_rz_input.setText(f'{wo.rz:.1f}')

                # Update status label
                self.wo_status_label.setText(
                    f'Active WorkObject: [{wo.x:.1f}, {wo.y:.1f}, {wo.z:.1f}, {wo.rx:.1f}, {wo.ry:.1f}, {wo.rz:.1f}]')
            else:
                self.wo_status_label.setText('Active WorkObject: [0, 0, 0, 0, 0, 0] (Identity)')
        except Exception as e:
            self.wo_status_label.setText(f'Active WorkObject: Error reading - {str(e)[:30]}')
            print(f"Error reading work object: {e}")

    def clear_workobject(self):
        """Clear the workobject (set to identity/zero)"""
        try:
            from robot_controller import WorkObject

            # Set all fields to zero
            self.wo_x_input.setText('0.0')
            self.wo_y_input.setText('0.0')
            self.wo_z_input.setText('0.0')
            self.wo_rx_input.setText('0.0')
            self.wo_ry_input.setText('0.0')
            self.wo_rz_input.setText('0.0')

            # Create and set identity workobject
            workobject = WorkObject(x=0, y=0, z=0, rx=0, ry=0, rz=0)
            self.robot.set_workobject(workobject)

            # Update status label
            self.wo_status_label.setText('WorkObject: [0, 0, 0, 0, 0, 0] (Identity)')

            # Visual feedback
            self.clear_wo_btn.setStyleSheet(
                'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }')
            self.clear_wo_btn.setText('Cleared!')
            QTimer.singleShot(1500, self.reset_wo_clear_button)

            print("WorkObject cleared (set to identity)")
        except Exception as e:
            self.clear_wo_btn.setStyleSheet(
                'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }')
            self.clear_wo_btn.setText('Failed!')
            QTimer.singleShot(1000, self.reset_wo_clear_button)
            print(f"Error clearing work object: {e}")

    def stop_motion(self):
        """Stop all robot motion by cancelling active goals."""
        try:
            result = self.robot.stop_motion()

            if result == 0:
                # Motion was stopped successfully
                self.stop_motion_btn.setStyleSheet(
                    'QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; font-size: 14px; }')
                self.stop_motion_btn.setText('STOPPED!')
                print("Robot motion stopped successfully")
            else:
                # No active goals to stop
                self.stop_motion_btn.setStyleSheet(
                    'QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 10px; font-size: 14px; }')
                self.stop_motion_btn.setText('NO MOTION')
                print("No active motion to stop")

            # Reset button after 1.5 seconds
            QTimer.singleShot(1500, self.reset_stop_button)

        except Exception as e:
            self.stop_motion_btn.setStyleSheet(
                'QPushButton { background-color: #9E9E9E; color: white; font-weight: bold; padding: 10px; font-size: 14px; }')
            self.stop_motion_btn.setText('ERROR')
            QTimer.singleShot(1000, self.reset_stop_button)
            print(f"Error stopping motion: {e}")

    def reset_stop_button(self):
        """Reset the stop button to its original state."""
        self.stop_motion_btn.setStyleSheet(
            'QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; font-size: 14px; }')
        self.stop_motion_btn.setText('STOP')

    def reset_wo_clear_button(self):
        """Reset the clear work object button to the default state"""
        self.clear_wo_btn.setStyleSheet(
            'QPushButton { background-color: #9E9E9E; color: white; font-weight: bold; padding: 8px; }')
        self.clear_wo_btn.setText('Clear WorkObject')
