from __future__ import annotations

import config
from backend.moveit_robot_backend import MoveItRobotBackend
from motion.servo.cartesian_servo.dummy_cartesian_servo import DummyCartesianServo

def create_robot_backend(node, workobject=None, ip: str = '0.0.0.0'):
    cartesian_servo = DummyCartesianServo(
        publish_rate_hz=100.0,
    )

    backend_kind = str(getattr(config, 'ROBOT_BACKEND', 'moveit')).lower()
    if backend_kind in {'moveit', 'fairino', 'zeroerr'}:
        return MoveItRobotBackend(ip=ip, node=node, workobject=workobject,cartesian_servo=cartesian_servo)
    raise ValueError(f'Unsupported ROBOT_BACKEND={backend_kind!r}')
