# PTP Wrist Minimization Policy

This runtime's PTP implementation keeps the public API unchanged:

- target pose remains `[x, y, z, rx, ry, rz]`
- callers still use the existing velocity, acceleration, tool, user, blocking,
  and trajectory optimizer parameters

The policy only changes how the backend interprets and validates the move.

## Goal

PTP must always aim for the requested final pose.

That includes:

- requested final position
- requested final orientation

At the same time, the planner should avoid unnecessary wrist rotation and
unnecessary orientation deviation during the move.

## Core Rules

1. The final target orientation is never replaced with the current orientation.
2. If current and requested orientations are already effectively the same,
   the planner treats the move as a "same-orientation" PTP:
   - it still reaches the requested final orientation
   - it rejects gratuitous wrist-branch flips
   - it enforces a tighter orientation-deviation threshold along the path
3. If current and requested orientations differ meaningfully, reorientation is
   allowed:
   - the planner still minimizes wrist motion where possible
   - the planner allows larger orientation change, but still validates the path
4. Every sampled joint state is checked for validity before execution.

## Wrist Minimization Logic

The runtime does not currently enumerate all IK branches from MoveIt.
Instead, it minimizes wrist motion in two practical stages:

1. IK is solved from the live joint state as the seed.
2. The resulting target joint solution is normalized onto the nearest
   equivalent angular branch for each joint.
3. For the last three joints, equivalent `+/- 2pi` wrist wraps are evaluated.
4. A joint-distance cost is computed with this priority:
   - minimize total joint rotation first
   - add extra penalty once wrist motion exceeds a configurable threshold
5. The lowest-cost equivalent wrist branch is selected.

This gives a stable "closest branch with strict wrist discipline" policy
without changing the client API or requiring a new planner stack.

If the resulting target joint state is already effectively the live joint state,
the move is treated as a successful no-op and is not sent to the trajectory
optimizer.  The threshold is `PTP_NOOP_JOINT_DELTA_RAD`.

## Same-Orientation Protection

When the angular difference between current and target TCP orientation is within
`PTP_LOCK_ORIENTATION_TOL_DEG`, the move is treated as same-orientation.

In this mode:

- target orientation is still the requested target orientation
- large wrist branch jumps are rejected using
  `PTP_LOCKED_MAX_WRIST_DELTA_DEG`
- sampled FK orientation is checked against the start/target attitude with the
  tighter `PTP_LOCKED_PATH_MAX_DRIFT_DEG` threshold

This is specifically meant to prevent cases where the robot could technically
reach the same attitude by flipping the wrist onto a different branch.

## Oriented PTP

If current and target orientations differ more than
`PTP_LOCK_ORIENTATION_TOL_DEG`, the planner allows reorientation.

In this mode:

- the final requested orientation is used directly
- total joint rotation is still minimized
- excessive wrist rotation is still rejected using `PTP_MAX_WRIST_DELTA_DEG`
- sampled FK orientation is compared against the expected orientation
  progression from start to target
- deviation is limited by `PTP_ORIENTED_PATH_MAX_DEVIATION_DEG`

## Current Limitation

This is not a full global joint-space planner over all IK branches.

It is:

- seeded by the live robot state
- branch-normalized near the current joints
- collision/state-validity checked on interpolated joint samples
- orientation-validated with FK along the path

That is materially safer than a pure Cartesian target call, but it is still not
the same as a full OMPL-style search over multiple IK families.

## Relevant Files

- `ptp_target.py`
- `strategies.py`
- `moveit_robot_backend.py`
- `config.py`
