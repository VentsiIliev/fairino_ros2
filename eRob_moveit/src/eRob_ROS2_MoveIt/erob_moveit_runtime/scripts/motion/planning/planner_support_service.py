#!/usr/bin/env python3
"""Planner support helpers that should not live on PlannerContext."""


class PlannerSupportService:
    def __init__(self, node):
        self._node = node
        self._fk_client = None
        self._ik_client = None
        self._contour_ik_client = None
        self._linked_lin_client = None
        self._state_validity_client = None
        self._ptp_client = None

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

    def get_contour_ik_client(self):
        if self._contour_ik_client is None:
            import config
            from erob_moveit_runtime.srv import ComputeContourIK
            self._contour_ik_client = self._node.create_client(
                ComputeContourIK,
                getattr(config, 'SERVICE_CONTOUR_IK', '/compute_contour_ik'),
            )
        return self._contour_ik_client

    def get_linked_lin_client(self):
        if self._linked_lin_client is None:
            import config
            from erob_moveit_runtime.srv import ComputeLinkedLin

            self._linked_lin_client = self._node.create_client(
                ComputeLinkedLin,
                getattr(config, 'SERVICE_LINKED_LIN', '/compute_linked_lin'),
            )

        return self._linked_lin_client

    def get_state_validity_client(self):
        if self._state_validity_client is None:
            from moveit_msgs.srv import GetStateValidity
            self._state_validity_client = self._node.create_client(
                GetStateValidity,
                '/check_state_validity',
            )
        return self._state_validity_client

    def get_ptp_client(self):
        if self._ptp_client is None:
            import config
            from erob_moveit_runtime.srv import ComputePtp

            self._ptp_client = self._node.create_client(
                ComputePtp,
                getattr(
                    config,
                    "SERVICE_PTP",
                    "/compute_ptp",
                ),
            )

        return self._ptp_client
