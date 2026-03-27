from __future__ import annotations

import config
from backend.moveit_robot_backend import MoveItRobotBackend


def create_robot_backend(node, workobject=None, ip: str = '0.0.0.0'):
    backend_kind = str(getattr(config, 'ROBOT_BACKEND', 'moveit')).lower()
    if backend_kind in {'moveit', 'fairino', 'zeroerr'}:
        return MoveItRobotBackend(ip=ip, node=node, workobject=workobject)
    raise ValueError(f'Unsupported ROBOT_BACKEND={backend_kind!r}')
