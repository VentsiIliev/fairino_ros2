from __future__ import annotations

import config


class RuntimeBackendAdapter:
    """Backend-specific runtime policy hooks for the shared MoveIt runtime."""

    backend_name = "generic"
    supports_drag_mode = False

    def get_monitor_tcp_transform(self, robot_controller):
        """Return the transform RobotMonitor should apply on /cartesian_position."""
        return robot_controller.T_tool


class FairinoRuntimeAdapter(RuntimeBackendAdapter):
    backend_name = "fairino"

    def get_monitor_tcp_transform(self, robot_controller):
        # Fairino /cartesian_position is sourced from flange pose. Reconstruct the
        # runtime TCP as flange -> ee_link -> selected tool.
        if robot_controller.T_ee_link is None:
            return robot_controller.T_tool
        return robot_controller.T_ee_link @ robot_controller.T_tool


class ZeroErrRuntimeAdapter(RuntimeBackendAdapter):
    backend_name = "zeroerr"
    supports_drag_mode = True


def create_runtime_adapter():
    backend_name = str(getattr(config, "ROBOT_BACKEND", "generic")).lower()
    if backend_name == "fairino":
        return FairinoRuntimeAdapter()
    if backend_name == "zeroerr":
        return ZeroErrRuntimeAdapter()
    return RuntimeBackendAdapter()
