# Architecture

## Package Layout

```text
fairino5_v6_moveit2_config/
├── scripts/
│   ├── main.py
│   ├── robot_controller.py
│   ├── config.py
│   ├── rest/
│   │   ├── main.py
│   │   ├── server.py
│   │   ├── api_support.py
│   │   └── openapi.py
│   ├── backend/
│   ├── motion/
│   │   ├── strategies.py
│   │   ├── planning/
│   │   └── execution/
│   ├── safety/
│   ├── status/
│   └── utils/
├── src_cpp/
│   ├── ipp_helper_node.cpp
│   ├── ruckig_helper_node.cpp
│   └── robot_state_publisher.cpp
├── srv/
│   └── ApplyIPP.srv
├── launch/
├── config/
└── meshes/
```

## Runtime Composition

### `main.py`

Typical dev/runtime composition:
1. initialize ROS2
2. construct `RobotController`
3. spin the node in a background thread
4. construct the configured robot backend through `backend_factory.py`
5. start the Flask REST server in another background thread

### `RobotController`

`RobotController` is the main owner of runtime state.

Responsibilities:
- create ROS clients and action clients
- extract workspace limits from URDF
- manage `SafetyWallManager`
- own `MotionQueue` and `MotionCoordinator`
- route strategies into planning/execution modules
- keep current joint/cartesian state through `RobotStateStore`
- host helper accessors used by planning, diagnostics, and REST

Important embedded components:
- `MotionQueue`
- `MotionCoordinator`
- `TrajectoryExecutor`
- `PlannerContext`
- `PlannerSupportService`
- `SafetyWallManager`
- `RobotMonitor`
- `RobotStateStore`
- `RobotStatusPublisher`

### `MoveItRobotBackend`

`MoveItRobotBackend` is the robot-wrapper facade used by the REST layer.

Responsibilities:
- apply workobject/user transforms
- convert tool/user/velocity inputs into execution requests
- expose robot-style methods like `move_liner`, `execute_path`, `get_current_position`
- proxy safety-wall control to `RobotController`

### REST Layer

`rest/server.py` exposes HTTP endpoints for:
- point moves
- path execution
- jog
- stop
- current position/velocity/status
- safety wall control
- reachability validation

The REST server can run in:
- standalone mode
  - creates its own ROS node
- embedded mode
  - reuses an existing `RobotController`

`rest/api_support.py` holds the non-routing REST helpers:
- motion error to HTTP mapping
- jog payload parsing / normalization
- pose reachability validation via IK + state validity

`rest/openapi.py` holds the OpenAPI spec and Swagger HTML served by the docs routes.

## Motion Flow

### Single target

```text
REST /move/linear
  -> MoveItRobotBackend.move_liner()
    -> SingleTargetStrategy
      -> motion.planning.single_target.send_cartesian_goal()
        -> MoveIt cartesian planning
        -> trajectory optimization (TOTG or Ruckig helper)
        -> FollowJointTrajectory execution
```

### Multi-waypoint path

```text
REST /execute/path
  -> MoveItRobotBackend.execute_path()
    -> PathStrategy
      -> motion.planning.trajectory.send_path_cartesian()
        -> path planning / optional staged approach
        -> trajectory optimization
        -> controller execution
```

### Queueing

`MotionCoordinator` decides whether a request:
- starts immediately
- is queued
- is rejected because it is non-queueable while busy

`MotionQueue` tracks:
- current task
- queued tasks
- completion result per task id

## Planning Modules

- `motion/strategies.py`
  - thin strategy wrappers for single-point and path requests
- `motion/planning/single_target.py`
  - planning entry for single Cartesian targets
  - true micro-moves use the Jacobian path
  - normal single-target linear moves currently send only start/end Cartesian poses into MoveIt and let `compute_cartesian_path` do the interpolation
  - structural pre-interpolation is intentionally disabled for normal single-target moves because exact intermediate waypoint anchors were found to over-constrain short Cartesian moves
- `motion/planning/trajectory.py`
  - planning entry for multi-waypoint Cartesian paths
- `motion/planning/trajectory_planner.py`
  - shared request builder and response handling
- `motion/planning/jacobian_move.py`
  - fallback for extremely small moves / collapsed trajectories
- `motion/planning/planner_diagnostics.py`
  - FK mismatch and collision diagnostics
- `motion/planning/planner_support_service.py`
  - lazy ROS service clients for FK, IK, and state validity
- `motion/planning/planner_context.py`
  - narrow context passed into planning code instead of direct god-node access

## Execution Modules

- `motion/execution/motion_coordinator.py`
  - execution ownership, queue arbitration, stop semantics
- `motion/execution/motion_queue.py`
  - queued task bookkeeping and wait-for-task support
- `motion/execution/trajectory_executor.py`
  - FollowJointTrajectory dispatch and result handling
- `motion/execution/trajectory_optimizer.py`
  - optimizer selection / helper dispatch

## Safety And Status Modules

- `safety/safety_wall_manager.py`
  - workspace walls, RViz markers, planning-scene collision objects
- `safety/collision_detection/`
  - dynamics-based external torque collision detection
- `status/robot_monitor.py`
  - live robot monitor / cartesian state pipeline
- `status/robot_state_store.py`
  - shared state cache for planning and REST
- `status/robot_status_publisher.py`
  - ROS status publication
