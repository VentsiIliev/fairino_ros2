#!/usr/bin/env python3
"""Interactive RViz pose tuner for the paint mounting surface mesh."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def euler_from_quaternion(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def axis_control(name: str, orientation, mode: int) -> InteractiveMarkerControl:
    control = InteractiveMarkerControl()
    control.name = name
    control.orientation.x = orientation[0]
    control.orientation.y = orientation[1]
    control.orientation.z = orientation[2]
    control.orientation.w = orientation[3]
    control.interaction_mode = mode
    control.orientation_mode = InteractiveMarkerControl.FIXED
    return control


class MountingSurfacePoseTuner(Node):
    def __init__(self):
        super().__init__("mounting_surface_pose_tuner")
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "tuned_mounting_surface")
        self.declare_parameter(
            "mesh_resource",
            "package://zeroerr/meshes/paint/latest_mounting.stl",
        )
        self.declare_parameter("mesh_scale", 0.001)
        self.declare_parameter("x", -0.015219)
        self.declare_parameter("y", 0.015023)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 2.856873)
        self.declare_parameter("lock_roll_pitch", True)

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.lock_roll_pitch = bool(self.get_parameter("lock_roll_pitch").value)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.server = InteractiveMarkerServer(self, "mounting_surface_pose_tuner")
        self.pose = None

        marker = self._make_interactive_marker()
        self.pose = marker.pose
        self.server.insert(marker, feedback_callback=self._process_feedback)
        self.server.applyChanges()

        self.create_timer(0.05, self._publish_tf)
        self._log_pose("initial")

    def _make_interactive_marker(self) -> InteractiveMarker:
        marker = InteractiveMarker()
        marker.header.frame_id = self.parent_frame
        marker.name = "mounting_surface"
        marker.description = "mounting surface"
        marker.scale = 0.25

        marker.pose.position.x = float(self.get_parameter("x").value)
        marker.pose.position.y = float(self.get_parameter("y").value)
        marker.pose.position.z = float(self.get_parameter("z").value)
        q = quaternion_from_euler(
            float(self.get_parameter("roll").value),
            float(self.get_parameter("pitch").value),
            float(self.get_parameter("yaw").value),
        )
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.mesh_resource = self.get_parameter("mesh_resource").value
        mesh_marker.mesh_use_embedded_materials = False
        scale = float(self.get_parameter("mesh_scale").value)
        mesh_marker.scale.x = scale
        mesh_marker.scale.y = scale
        mesh_marker.scale.z = scale
        mesh_marker.color.r = 0.0
        mesh_marker.color.g = 1.0
        mesh_marker.color.b = 1.0
        mesh_marker.color.a = 0.82

        mesh_control = InteractiveMarkerControl()
        mesh_control.name = "mesh"
        mesh_control.always_visible = True
        mesh_control.markers.append(mesh_marker)
        marker.controls.append(mesh_control)

        x_axis = (1.0, 0.0, 0.0, 1.0)
        y_axis = (0.0, 0.0, 1.0, 1.0)
        z_axis = (0.0, 1.0, 0.0, 1.0)
        marker.controls.extend(
            [
                axis_control("move_x", x_axis, InteractiveMarkerControl.MOVE_AXIS),
                axis_control("move_y", y_axis, InteractiveMarkerControl.MOVE_AXIS),
                axis_control("move_z", z_axis, InteractiveMarkerControl.MOVE_AXIS),
                axis_control("rotate_z", z_axis, InteractiveMarkerControl.ROTATE_AXIS),
            ]
        )
        if not bool(self.get_parameter("lock_roll_pitch").value):
            marker.controls.extend(
                [
                    axis_control("rotate_x", x_axis, InteractiveMarkerControl.ROTATE_AXIS),
                    axis_control("rotate_y", y_axis, InteractiveMarkerControl.ROTATE_AXIS),
                ]
            )
        return marker

    def _process_feedback(self, feedback):
        if feedback.event_type != feedback.POSE_UPDATE:
            return
        self.pose = feedback.pose
        if self.lock_roll_pitch:
            roll, pitch, yaw = euler_from_quaternion(
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z,
                self.pose.orientation.w,
            )
            q = quaternion_from_euler(0.0, 0.0, yaw)
            self.pose.orientation.x = q[0]
            self.pose.orientation.y = q[1]
            self.pose.orientation.z = q[2]
            self.pose.orientation.w = q[3]
            self.server.setPose(feedback.marker_name, self.pose)
            self.server.applyChanges()
        self._log_pose("updated")

    def _publish_tf(self):
        if self.pose is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.pose.position.x
        transform.transform.translation.y = self.pose.position.y
        transform.transform.translation.z = self.pose.position.z
        transform.transform.rotation = self.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _log_pose(self, label: str):
        p = self.pose.position
        q = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        origin = (
            f'<origin xyz="{p.x:.6f} {p.y:.6f} {p.z:.6f}" '
            f'rpy="{roll:.6f} {pitch:.6f} {yaw:.6f}"/>'
        )
        self.get_logger().info(f"{label}: {origin}")


def main():
    rclpy.init()
    node = MountingSurfacePoseTuner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
