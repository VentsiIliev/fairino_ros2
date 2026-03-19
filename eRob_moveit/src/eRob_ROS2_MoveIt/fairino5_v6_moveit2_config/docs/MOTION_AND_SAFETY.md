# Motion And Safety

## Motion Lifecycle

### 1. Request acceptance

A request enters through:
- REST
- or direct strategy execution on `RobotController`

`MotionCoordinator.execute()` decides whether the request:
- executes immediately
- is queued
- is rejected because a non-queueable command was sent while busy

### 2. Planning

Single targets and paths both use MoveIt Cartesian planning modules under `motion/planning/`.

Important planning helpers:
- `_build_cartesian_request()` in `trajectory_planner.py`
- `_cartesian_path_response()` in `trajectory_planner.py`
- staged path approach flow in `trajectory.py`
- Jacobian fallback in `jacobian_move.py`

### 3. Trajectory optimization

After raw MoveIt joint trajectories are produced, they are time-parameterized through:
- TOTG helper
- or Ruckig helper

Selection is controlled by:
- `config.TRAJECTORY_OPTIMIZER`

### 4. Controller execution

Timed trajectories are sent through the FollowJointTrajectory action client.

`TrajectoryExecutor` owns:
- goal submission
- result handling
- handoff to queued motion when one goal completes

## Safety Layers

### Local workspace safety

`SafetyWallManager.check_position_safety()` validates raw target XYZ against configured workspace bounds before planning.

Inputs come from:
- workspace extracted from URDF
- XY expansion configured by `WALL_XY_OFFSET_M`
- soft warning margin configured by `SAFETY_MARGIN_M`

### MoveIt planning-scene safety walls

`SafetyWallManager` also publishes wall collision objects into the MoveIt planning scene and matching RViz markers.

Capabilities:
- enable / disable walls
- republish on demand
- clear from planning scene
- expose status through REST

Allowed-collision support:
- `WALL_BYPASS_LINKS` can suppress selected link-wall contacts inside the ACM

### Collision diagnostics

Planner diagnostics distinguish:
- start-state collision
- FK mismatch / bad start-state assumptions
- IK/reachability failures

### Dynamics-based collision detector

`/scripts/safety/collision_detection/` contains a dynamics-based external torque detector.

Current status in `RobotController`:
- initialized
- callback wired
- presently disabled for automatic stopping unless explicitly enabled

## Reachability Precheck Versus Real Execution

### `POST /reachability/pose`

Intended for:
- UI prechecks
- simulation of planned measurement order
- quick “can I use this point?” validation

It now uses:
- IK for start
- IK for target with start seed
- `check_state_validity` on the target joint state

Pros:
- much faster than full Cartesian planning
- keeps endpoint collision detection
- keeps joint-limit and reachability filtering

Cons:
- does not validate the full path between the two poses

### Real execution

Actual motion commands still use the normal planner/executor path.
That remains the authority for:
- full path feasibility
- partial-cartesian-path rejection rules
- real controller execution success/failure

## Important Config Knobs

In `scripts/config.py`:
- `CARTESIAN_MIN_FRACTION`
- `WALL_XY_OFFSET_M`
- `SAFETY_MARGIN_M`
- `MOTION_QUEUE_MAX_SIZE`
- `BLOCKING_MOVE_TIMEOUT_S`
- `DEFAULT_VEL_PERCENT`
- `DEFAULT_ACC_PERCENT`
- `TRAJECTORY_OPTIMIZER`

## Operational Notes

- Safety walls and local workspace checks are separate layers.
- Queueing preserves mixed ordering between immediate point moves and path moves.
- Stopping motion can also clear queued tasks.
- Workobjects are applied before planning, so REST/user-frame targets are not always base-frame targets.
