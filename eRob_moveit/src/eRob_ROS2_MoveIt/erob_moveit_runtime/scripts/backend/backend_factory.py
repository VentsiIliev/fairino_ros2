from __future__ import annotations

import config
from backend.moveit_robot_backend import MoveItRobotBackend
from backend.runtime_unwind import unwind_joint6_with_rotational_path
from motion.servo.cartesian_servo.moveit_cartesian_servo import MoveItCartesianServo
from rclpy.duration import Duration
from rclpy.time import Time
from utils.transformation_utils import TransformationUtils


class _RobotScopedMoveItCartesianServo(MoveItCartesianServo):
    """MoveIt Servo bound to the RobotController's selected robot context."""

    def _lookup_base_to_ee_transform(self):
        robot_context = getattr(self._node, "robot_context", None)
        ee_link = str(
            getattr(robot_context, "ee_link", "")
            or getattr(config, "EE_LINK", "")
        ).strip()
        if not ee_link:
            raise RuntimeError("No EE link configured for Cartesian Servo")

        transform = self._node.tf_buffer.lookup_transform(
            self._base_frame,
            ee_link,
            Time(),
            timeout=Duration(seconds=self._tf_timeout_s),
        )
        return TransformationUtils.tf2_to_transform(transform)


class _RobotScopedMoveItRobotBackend(MoveItRobotBackend):
    """MoveIt backend with legacy unwind bound to the selected robot context."""

    def _unwind_joint6_with_rotational_path(
        self,
        vel=None,
        acc=None,
        queue_if_busy=True,
    ):
        return unwind_joint6_with_rotational_path(
            self,
            vel=vel,
            acc=acc,
            queue_if_busy=queue_if_busy,
        )


def create_robot_backend(node, workobject=None, ip: str = '0.0.0.0'):
    robot_context = getattr(node, "robot_context", None)
    base_frame = str(
        getattr(robot_context, "base_link", "")
        or getattr(config, "BASE_LINK", "base_link")
    )

    cartesian_servo = _RobotScopedMoveItCartesianServo(
        publish_rate_hz=float(getattr(config, "CARTESIAN_SERVO_PUBLISH_RATE_HZ", 100.0)),
        node=node,
        base_frame=base_frame,
        service_timeout_s=float(getattr(config, "CARTESIAN_SERVO_SERVICE_TIMEOUT_S", 5.0)),
        tf_timeout_s=float(getattr(config, "CARTESIAN_SERVO_TF_TIMEOUT_S", 0.25)),
    )

    backend_kind = str(getattr(config, 'ROBOT_BACKEND', 'moveit')).lower()
    if backend_kind in {'moveit', 'fairino', 'zeroerr'}:
        return _RobotScopedMoveItRobotBackend(
            ip=ip,
            node=node,
            workobject=workobject,
            cartesian_servo=cartesian_servo,
        )
    raise ValueError(f'Unsupported ROBOT_BACKEND={backend_kind!r}')
