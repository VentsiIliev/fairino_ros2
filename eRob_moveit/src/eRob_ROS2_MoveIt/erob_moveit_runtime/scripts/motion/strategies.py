#!/usr/bin/env python3


class MotionStrategy:
    queueable = True

    def execute(self, robot_controller) -> int:
        raise NotImplementedError


class _MoveLinStrategyBase(MotionStrategy):
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


class CartesianMoveLinStrategy(_MoveLinStrategyBase):
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


class PilzMoveLinStrategy(_MoveLinStrategyBase):
    """Plan a LIN motion through MoveIt's Pilz sequence action.

    Unlike :class:`CartesianMoveLinStrategy`, this path never calls
    ``/compute_cartesian_path``.  It is also exposed directly by the
    ``/move/fast_lin`` diagnostic endpoint so its planning latency can be
    compared without changing the configured default LIN strategy.
    """

    def execute(self, robot_controller) -> int:
        from .planning.sequence import send_motion_sequence
        planner_context = getattr(robot_controller, "planner_context", robot_controller)
        return send_motion_sequence(
            planner_context,
            [{
                "position": [self.x_mm, self.y_mm, self.z_mm, self.rx, self.ry, self.rz],
                "motion_type": "linear",
                "vel": self.vel_scale * 100.0,
                "acc": self.acc_scale * 100.0,
                "blend_radius": 0.0,
            }],
            tool_transform=self.tool_transform,
        )


class SingleTargetStrategy(_MoveLinStrategyBase):
    def execute(self, robot_controller) -> int:
        import config
        strategy_name = str(getattr(config, "MOVE_LIN_STRATEGY", "cartesian_path") or "cartesian_path").strip().lower()
        if strategy_name in {"pilz", "pilz_lin", "sequence_lin"}:
            selected = PilzMoveLinStrategy
            resolved_name = "pilz_lin"
        else:
            selected = CartesianMoveLinStrategy
            resolved_name = "cartesian_path"
        logger_getter = getattr(robot_controller, "get_logger", None)
        if callable(logger_getter):
            logger_getter().info(f"[MOVE_LIN] strategy={resolved_name} configured={strategy_name}")
        try:
            from .move_linear_timing import ensure as ensure_move_linear_timing, mark as mark_move_linear_timing
            planner_context = getattr(robot_controller, "planner_context", robot_controller)
            timing = ensure_move_linear_timing(robot_controller, source="SingleTargetStrategy")
            try:
                setattr(planner_context, "_move_linear_timing", timing)
            except Exception:
                pass
            mark_move_linear_timing(planner_context, "strategy_selected", strategy=resolved_name, configured=strategy_name)
        except Exception:
            pass
        return selected(
            self.x_mm, self.y_mm, self.z_mm,
            self.rx, self.ry, self.rz,
            self.vel_scale, self.acc_scale,
            tool_transform=self.tool_transform,
            avoid_collisions=self.avoid_collisions,
            trajectory_optimizer=self.trajectory_optimizer,
        ).execute(robot_controller)


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
