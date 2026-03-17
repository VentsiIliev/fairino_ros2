#!/usr/bin/env python3


class MotionStrategy:
    queueable = True

    def execute(self, robot_controller) -> int:
        raise NotImplementedError


class SingleTargetStrategy(MotionStrategy):
    def __init__(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                 tool_transform=None):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.z_mm = z_mm
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self.tool_transform = tool_transform  # per-move TCP override; None = use robot_controller.T_tool

    def execute(self, robot_controller) -> int:
        from .planning.single_target import send_cartesian_goal
        planner_context = getattr(robot_controller, "planner_context", robot_controller)
        return send_cartesian_goal(
            planner_context,
            self.x_mm, self.y_mm, self.z_mm,
            self.rx, self.ry, self.rz,
            self.vel_scale, self.acc_scale,
            tool_transform=self.tool_transform)


class PathStrategy(MotionStrategy):
    def __init__(self, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
        self.waypoints_mm = waypoints_mm
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.vel_scaling = vel_scaling
        self.acc_scaling = acc_scaling

    def execute(self, robot_controller) -> int:
        from .planning.trajectory import send_path_cartesian
        planner_context = getattr(robot_controller, "planner_context", robot_controller)
        return send_path_cartesian(
            planner_context,
            self.waypoints_mm, self.rx, self.ry, self.rz,
            self.vel_scaling, self.acc_scaling)
