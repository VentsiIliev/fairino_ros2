#!/usr/bin/env python3
"""Offline-planned trajectory container for plan-now / execute-later moves."""

from dataclasses import dataclass, field
from trajectory_msgs.msg import JointTrajectory


@dataclass
class PreparedTrajectory:
    """A validated, time-parameterized trajectory produced offline by ``prepare_path``.

    The planner pipeline (safety check, MoveIt cartesian planning, trajectory
    optimization) runs to completion but the result is captured instead of being
    dispatched to the trajectory controller. Execution is deferred until
    ``execute_prepared`` is called.

    How the cached trajectory is treated at execution time depends on the
    ``start_policy`` (``metadata.get("start_policy")`` or the explicit argument):

    - ``live_anchor``: the first controller point is re-anchored to the live
      joint state at execution time. The cached trajectory is mutated then, so
      the cached first point is NOT the configuration that runs.
    - ``require_exact``: the cached trajectory is preserved exactly and its
      first point is verified against the live joint state within
      ``EXECUTOR_PREPARED_START_TOL_RAD``, failing with
      ``MOTION_ERROR_PREPARED_START_MISMATCH`` otherwise. The start
      configuration is therefore validated, not overwritten.

    ``metadata`` also carries loop-closure information for choreography:
    ``cyclic`` (last point matches first point within
    ``PREPARED_TRAJECTORY_CLOSURE_TOL_RAD``) and ``closure_error_rad``.

    ``noop=True`` marks a preparation that produced no actual motion (for
    example a target already reached). A no-op prepare is still a SUCCESS: the
    robot is considered ready and participates in any synchronization barrier.
    """

    trajectory: JointTrajectory | None
    joint_names: tuple[str, ...] = ()
    start_positions: tuple[float, ...] = ()
    end_positions: tuple[float, ...] = ()
    duration_s: float = 0.0
    source: str = ""
    metadata: dict = field(default_factory=dict)
    noop: bool = False
    result: int = 0

    @property
    def ok(self) -> bool:
        return self.result == 0

    @property
    def point_count(self) -> int:
        if self.trajectory is None:
            return 0
        return len(self.trajectory.points)
