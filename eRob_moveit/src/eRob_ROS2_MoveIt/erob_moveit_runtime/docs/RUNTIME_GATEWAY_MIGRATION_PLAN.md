# Runtime Gateway Migration Plan

## Goal

Prevent REST or WebSocket server failures from taking down the robot runtime, while avoiding a large one-shot rewrite.

The migration introduces a stable `RuntimeGateway` interface between API handlers and robot-control implementation. The current implementation stays local and direct. Later, the same gateway interface can be backed by ROS services, actions, and topic subscriptions.

## Target Shape

Current foundation:

```text
REST / WebSocket
      |
      v
RuntimeApi
      |
      v
RuntimeGateway
      |
      v
LocalRuntimeGateway
      |
      +-- RobotController
      +-- MoveItRobotBackend
```

Future process-isolated shape:

```text
REST / WebSocket process
      |
      v
RuntimeApi
      |
      v
RosRuntimeGateway
      |
      +-- ROS services
      +-- ROS actions
      +-- ROS topic subscriptions

Robot runtime process
      |
      +-- RobotController
      +-- MoveItRobotBackend
      +-- ROS service/action servers
      +-- ROS state publishers
```

## Module Location

Add the gateway modules under:

```text
scripts/runtime_gateway/
  __init__.py
  base.py
  local.py
  ros.py        # later
```

Install the directory from `CMakeLists.txt` by adding `runtime_gateway` to the existing Python module directory install loop.

## Responsibilities

### `RuntimeApi`

`RuntimeApi` owns API-level behavior:

- request validation
- response shape used by REST/WebSocket clients
- HTTP-oriented status mapping through `ApiResponse`
- backwards-compatible endpoint behavior

`RuntimeApi` should not care whether runtime calls are local Python calls or ROS IPC.

### `RuntimeGateway`

`RuntimeGateway` owns the stable runtime contract:

- motion commands
- stop/cancel commands
- runtime status queries
- state snapshot access
- readiness and fault checks

The gateway should not return Flask responses or HTTP-specific objects. It should return runtime-level dictionaries and primitive values.

### `LocalRuntimeGateway`

`LocalRuntimeGateway` preserves current behavior:

- holds `RobotController`
- holds `MoveItRobotBackend`
- calls backend/node methods directly
- has effectively no performance cost beyond one Python method call

### `RosRuntimeGateway`

`RosRuntimeGateway` is introduced later:

- calls ROS services for request/response commands
- calls ROS actions for long-running motion commands
- caches topic subscriptions for frequently-read state
- lets REST/WebSocket processes restart independently from the robot runtime process

## Migration Steps

### Phase 1: Add the Local Gateway

Create `RuntimeGateway` and `LocalRuntimeGateway`.

Start with low-risk methods:

```python
startup_status()
runtime_state_snapshot()
is_motion_stack_ready()
get_motion_stack_fault_reason()
status()
stop_motion()
```

Wire `rest_server.py` to construct a `LocalRuntimeGateway` from the existing `robot` and `node`.

Update `RuntimeApi` to accept a gateway while temporarily keeping `robot_getter` and `node_getter` for unmigrated calls.

### Phase 2: Migrate Motion Calls One at a Time

Move direct backend calls from `RuntimeApi` into `LocalRuntimeGateway`.

Suggested order:

```text
stop_motion
status
state_snapshot
current_position
current_velocity
move_linear
move_ptp
execute_path
execute_sequence
ordered_motion_chain
jog / servojog
cartesian_servo
drive enable / disable / status
safety wall operations
tool registry operations
```

After each migration, keep endpoint responses unchanged and run focused tests or manual smoke checks.

### Phase 3: Define ROS Runtime Contracts

Once gateway method shapes stabilize, define ROS service/action contracts for migrated methods.

Use:

- ROS services for short request/response operations
- ROS actions for motion commands that may block, queue, or report progress
- ROS topics for high-frequency state and status

Avoid service calls for high-rate state polling. The REST process should cache topic data and serve HTTP reads from memory.

### Phase 4: Add `RosRuntimeGateway`

Implement `RosRuntimeGateway` behind the same interface.

At this stage, the REST server can switch between gateway implementations through configuration:

```text
EROB_RUNTIME_GATEWAY=local
EROB_RUNTIME_GATEWAY=ros
```

Default to `local` until the ROS gateway covers enough endpoints.

### Phase 5: Split Robot Runtime Process

Add a robot runtime process that owns:

- ROS initialization
- `RobotController`
- `MoveItRobotBackend`
- ROS service/action servers
- state/status publishers

Then run REST/WebSocket as client-only processes using `RosRuntimeGateway`.

At this point, a hard REST/WebSocket crash should not destroy the robot runtime.

## Performance Notes

The local gateway adds negligible overhead.

The later ROS gateway adds IPC overhead, but this is small compared with MoveIt planning and physical robot execution for normal motion commands.

High-rate paths need special handling:

- status and state snapshots should use cached topic subscriptions
- servo/jog streaming should avoid one blocking service call per update
- long-running motion should use actions instead of synchronous services

## Compatibility Rules

- Keep existing REST endpoint URLs unchanged.
- Keep existing JSON request and response shapes unchanged unless a separate API migration is planned.
- Do not move all calls at once.
- Keep `LocalRuntimeGateway` as the reference behavior while developing `RosRuntimeGateway`.
- Do not let gateway implementations import Flask.

## First Implementation Checklist

- Add `scripts/runtime_gateway/__init__.py`.
- Add `scripts/runtime_gateway/base.py`.
- Add `scripts/runtime_gateway/local.py`.
- Add `runtime_gateway` to the CMake install loop.
- Construct `LocalRuntimeGateway` in `rest_server.py`.
- Pass the gateway into `RuntimeApi`.
- Migrate `status()` and `stop_motion()` first.
- Verify with `python3 -m py_compile`.
- Verify with `colcon build --packages-select erob_moveit_runtime --allow-overriding erob_moveit_runtime`.
