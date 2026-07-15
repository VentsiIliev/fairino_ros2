#!/usr/bin/env python3


class MotionStrategy:
    queueable = True

    def execute(self, robot_controller) -> int:
        raise NotImplementedError


class SingleTargetStrategy(MotionStrategy):
    def __init__(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                 tool_transform=None, avoid_collisions=None, trajectory_optimizer=None):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.z_mm = z_mm
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self.tool_transform = tool_transform  # per-move TCP override; None = use robot_controller.T_tool
        from config import resolve_avoid_collisions
        self.avoid_collisions = resolve_avoid_collisions(avoid_collisions)
        self.trajectory_optimizer = trajectory_optimizer

    def execute(self, robot_controller) -> int:
        from .planning.single_target import send_cartesian_goal
        planner_context = getattr(robot_controller, "planner_context", robot_controller)
        return send_cartesian_goal(
            planner_context,
            self.x_mm, self.y_mm, self.z_mm,
            self.rx, self.ry, self.rz,
            self.vel_scale, self.acc_scale,
            tool_transform=self.tool_transform,
            avoid_collisions=self.avoid_collisions,
            trajectory_optimizer=self.trajectory_optimizer)


class PtpTargetStrategy(MotionStrategy):
    def __init__(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                 tool_transform=None, trajectory_optimizer=None):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.z_mm = z_mm
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self.tool_transform = tool_transform
        self.trajectory_optimizer = trajectory_optimizer

    def execute(self, robot_controller) -> int:
        from .planning.ptp_target import send_ptp_goal
        planner_context = getattr(robot_controller, "planner_context", robot_controller)
        return send_ptp_goal(
            planner_context,
            self.x_mm, self.y_mm, self.z_mm,
            self.rx, self.ry, self.rz,
            self.vel_scale, self.acc_scale,
            tool_transform=self.tool_transform,
            trajectory_optimizer_name=self.trajectory_optimizer,
        )


class PathStrategy(MotionStrategy):
    def __init__(
        self,
        waypoints_mm,
        rx,
        ry,
        rz,
        vel_scaling,
        acc_scaling,
        trajectory_optimizer=None,
        orientation_mode="constant",
    ):
        self.waypoints_mm = waypoints_mm
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.vel_scaling = vel_scaling
        self.acc_scaling = acc_scaling
        self.trajectory_optimizer = trajectory_optimizer
        self.orientation_mode = orientation_mode

    def execute(self, robot_controller) -> int:
        from .planning.trajectory import send_path_cartesian
        planner_context = getattr(robot_controller, "planner_context", robot_controller)
        return send_path_cartesian(
            planner_context,
            self.waypoints_mm, self.rx, self.ry, self.rz,
            self.vel_scaling, self.acc_scaling,
            trajectory_optimizer_name=self.trajectory_optimizer,
            orientation_mode=self.orientation_mode)



class SequenceStrategy(MotionStrategy):
    def __init__(self, segments, tool_transform=None):
        self.segments = list(segments)
        self.tool_transform = tool_transform

    def execute(self, robot_controller) -> int:
        from .planning.sequence import send_motion_sequence
        planner_context = getattr(robot_controller, "planner_context", robot_controller)
        return send_motion_sequence(
            planner_context,
            self.segments,
            tool_transform=self.tool_transform,
        )
