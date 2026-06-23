from __future__ import annotations

import config
import numpy as np


class RuntimeBackendAdapter:
    """Backend-specific runtime policy hooks for the shared MoveIt runtime."""

    backend_name = "generic"
    supports_drag_mode = False

    def get_monitor_tcp_transform(self, robot_controller):
        """Return the transform RobotMonitor should apply on /cartesian_position."""
        return robot_controller.T_monitor_tool

    def get_planning_tool_transform(self, robot_controller, registry_tool_transform):
        """Return the ee_link -> TCP transform used by MoveIt planning."""
        return registry_tool_transform


class FairinoRuntimeAdapter(RuntimeBackendAdapter):
    backend_name = "fairino"

    def get_monitor_tcp_transform(self, robot_controller):
        # Fairino /cartesian_position is sourced from the mechanical flange.
        # The tool registry stores flange -> TCP transforms for calibrated tools.
        return robot_controller.T_monitor_tool

    def get_planning_tool_transform(self, robot_controller, registry_tool_transform):
        # MoveIt targets ee_link, so convert the flange -> TCP registry transform
        # into the equivalent ee_link -> TCP transform before removing TCP offset.
        if robot_controller.T_ee_link is None:
            return registry_tool_transform
        return np.linalg.inv(robot_controller.T_ee_link) @ registry_tool_transform


class ZeroErrRuntimeAdapter(RuntimeBackendAdapter):
    backend_name = "zeroerr"
    supports_drag_mode = True

    def get_planning_tool_transform(self, robot_controller, registry_tool_transform):
        # ZeroErr can publish /cartesian_position from the wrist/flange frame
        # while MoveIt still plans on ee_link. In that mode the registry stores
        # wrist/flange -> TCP and planning needs ee_link -> TCP.
        source_link = str(getattr(config, "CARTESIAN_SOURCE_LINK", config.EE_LINK))
        if source_link == config.WRIST_LINK and robot_controller.T_ee_link is not None:
            return np.linalg.inv(robot_controller.T_ee_link) @ registry_tool_transform
        return registry_tool_transform


def create_runtime_adapter():
    backend_name = str(getattr(config, "ROBOT_BACKEND", "generic")).lower()
    if backend_name == "fairino":
        return FairinoRuntimeAdapter()
    if backend_name == "zeroerr":
        return ZeroErrRuntimeAdapter()
    return RuntimeBackendAdapter()
