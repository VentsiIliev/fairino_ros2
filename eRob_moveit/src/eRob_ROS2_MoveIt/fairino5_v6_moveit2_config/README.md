# `fairino5_v6_moveit2_config`

ROS2 + MoveIt2 bridge package for the Fairino 5 v6 robot.

This package contains:
- the Python control node that owns planning, execution, queueing, safety walls, monitoring, and REST exposure
- helper Python modules for motion planning, execution, safety, status, and transforms
- C++ helper nodes for time-parameterization and robot-state publishing
- the custom `ApplyIPP.srv` service used by trajectory optimization helpers

## Main Entry Points

- `scripts/main.py`
  - desktop/dev entrypoint that starts `RobotController`, wraps it in `FairinoRos2Robot`, and launches the REST server in the same process
- `scripts/robot_controller.py`
  - the main ROS node (`velocity_monitor`) that owns MoveIt service clients, motion queueing, safety walls, and monitoring state
- `scripts/rest_server.py`
  - Flask REST bridge exposing motion, stop, status, safety-wall control, and reachability validation
- `scripts/rest_api_support.py`
  - REST-only helpers for motion error mapping, jog payload parsing, and pose reachability validation
- `scripts/fairino_ros2_robot.py`
  - robot-style wrapper used by the REST layer; converts workobject/user/tool inputs into node execution requests

## High-Level Architecture

```text
REST client
  -> rest_server.py
    -> rest_api_support.py for request parsing / validation helpers
    -> FairinoRos2Robot
      -> RobotController
        -> motion.strategies
          -> motion.planning
            -> MoveIt services
          -> motion.execution
            -> FollowJointTrajectory action
```

Supporting subsystems:
- `safety/`
  - safety walls, workspace validation, collision-object publishing
- `status/`
  - joint/cartesian state store, monitoring, ROS status publishing
- `utils/`
  - transforms, workobjects, workspace extraction

## Key Behaviors

- Point-to-point and path moves are planned in Cartesian space and then time-parameterized before controller execution.
- Motion requests can be queued; queued and immediate commands share one execution/ownership path.
- Workspace safety exists in two layers:
  - local precheck against extracted workspace bounds
  - remote MoveIt planning-scene safety walls
- The REST server exposes wall enable/disable/status APIs and a planning-only pose reachability check.

## Reachability Validation

`POST /reachability/pose` is intended for simulation/precheck workflows such as platform grid verification.

Current semantics:
- solve IK for the supplied start pose
- solve IK for the target pose, seeded from the start solution
- run `check_state_validity` on the target joint state

This keeps:
- target-pose reachability
- joint-limit checks
- endpoint collision checks

It does **not** guarantee that the full Cartesian path between start and target is collision-free. Actual robot execution still uses the full motion planning path.

## Docs

- [Architecture](docs/ARCHITECTURE.md)
- [REST API](docs/REST_API.md)
- [Motion And Safety](docs/MOTION_AND_SAFETY.md)
