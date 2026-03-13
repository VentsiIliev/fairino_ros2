"""
Planner Utilities
=================
Shared constants and exit-path helpers used by trajectory_planner.py and jacobian_move.py.

Both modules import from here to avoid circular dependencies.
"""

# ============ Trajectory Parameterization Selection ============
# Options:
#   "RUCKIG"  - Use Ruckig C++ service (jerk-limited, requires ruckig_helper node)
#   "TOTG"    - Use TOTG C++ service (time-optimal, requires ipp_helper node)
#
# Ruckig provides smoother motion with S-curve profiles (3rd order)
# TOTG is faster but has trapezoidal velocity profiles (2nd order)
TIME_PARAMETERIZATION = "TOTG"


def _set_result(rc, code):
    """
    Atomically clear the executing flag and record the final result code.

    Called at every early-exit point (planning failure, collision, timeout, etc.)
    and at the start of successful trajectory dispatch to record the outcome
    before _send_trajectory_to_controller takes ownership of is_executing.

    Args:
        rc:   RobotController node instance
        code: Integer result code (0 = success, negative = error — see error_codes.md)
    """
    with rc.lock:
        rc.is_executing = False
        rc.last_move_result = code


def _is_stale(rc, generation):
    """
    Return True if the incoming response belongs to a superseded plan.

    Each new move increments rc.plan_generation before submitting the async
    service call. If stop_motion() or a new move was issued between submission
    and response, the generation counter no longer matches and the response
    must be silently discarded to prevent a stale trajectory from executing.

    Args:
        rc:         RobotController node instance
        generation: The plan_generation value captured at submission time

    Returns:
        bool: True → discard response; False → response is current
    """
    with rc.lock:
        return generation is not None and rc.plan_generation != generation
