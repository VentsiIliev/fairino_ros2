#!/usr/bin/env python3
"""Planner support helpers that should not live on PlannerContext."""


class PlannerSupportService:
    def __init__(self, node):
        self._node = node
        self._fk_client = None
        self._ik_client = None
        self._state_validity_client = None

    def get_fk_client(self):
        if self._fk_client is None:
            from moveit_msgs.srv import GetPositionFK
            self._fk_client = self._node.create_client(GetPositionFK, '/compute_fk')
        return self._fk_client

    def get_ik_client(self):
        if self._ik_client is None:
            from moveit_msgs.srv import GetPositionIK
            self._ik_client = self._node.create_client(GetPositionIK, '/compute_ik')
        return self._ik_client

    def get_state_validity_client(self):
        if self._state_validity_client is None:
            from moveit_msgs.srv import GetStateValidity
            self._state_validity_client = self._node.create_client(
                GetStateValidity,
                '/check_state_validity',
            )
        return self._state_validity_client
