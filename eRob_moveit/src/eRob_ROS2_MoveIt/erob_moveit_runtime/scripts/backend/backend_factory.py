from __future__ import annotations

import config
from backend.moveit_robot_backend import MoveItRobotBackend
from motion.servo.cartesian_servo.dummy_cartesian_servo import DummyCartesianServo
from motion.servo.cartesian_servo.moveit_cartesian_servo import MoveItCartesianServo


def create_robot_backend(node, workobject=None, ip: str = '0.0.0.0'):
    cartesian_servo = MoveItCartesianServo(
        publish_rate_hz=float(getattr(config, "CARTESIAN_SERVO_PUBLISH_RATE_HZ", 100.0)),
        node=node,
        base_frame="base_link",
        service_timeout_s=float(getattr(config, "CARTESIAN_SERVO_SERVICE_TIMEOUT_S", 5.0)),
        tf_timeout_s=float(getattr(config, "CARTESIAN_SERVO_TF_TIMEOUT_S", 0.25)),
    )

    backend_kind = str(getattr(config, 'ROBOT_BACKEND', 'moveit')).lower()
    if backend_kind in {'moveit', 'fairino', 'zeroerr'}:
        return MoveItRobotBackend(ip=ip, node=node, workobject=workobject,cartesian_servo=cartesian_servo)
    raise ValueError(f'Unsupported ROBOT_BACKEND={backend_kind!r}')
