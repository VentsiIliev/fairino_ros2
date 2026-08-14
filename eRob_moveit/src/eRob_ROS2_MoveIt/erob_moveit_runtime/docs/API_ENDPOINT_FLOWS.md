# ZeroErr Runtime REST API Endpoint Flows

This document describes the REST control flow in `scripts/rest/server.py`.
It is written for frontend/backend integration work: each section explains what
the endpoint calls, what can fail, and what response shape to expect.

## Global Behavior

### Process Model

The HTTP server is not a separate process. `zeroerr_runtime.py` owns the process,
and Flask runs inside that process in a background thread. Killing
`zeroerr_runtime.py` kills the REST server.

### Startup Gate

All endpoints except these are blocked until both `robot` and `node` exist:

- `GET /health`
- `GET /startup/status`
- `GET /docs`
- `GET /api/docs`
- `GET /openapi.json`

If a blocked endpoint is called while runtime initialization is still in progress:

```json
{
  "success": false,
  "error": "robot runtime is still starting",
  "startup": {
    "phase": "...",
    "message": "...",
    "ready": false,
    "error": null,
    "ros2_active": false
  }
}
```

HTTP status: `503`.

### Motion Result Code Mapping

Motion endpoints call backend methods that return integer result codes. Negative
codes are converted by `motion_error_response(result)`.

| Code | Meaning | HTTP |
| --- | --- | --- |
| `-1` | Busy, invalid input, generic error | `500` |
| `-2` | MoveIt service unavailable | `503` |
| `-3` | Safety violation, outside workspace | `400` |
| `-4` | No current robot position | `500` |
| `-5` | Motion queue full | `503` |
| `-6` | Path planning failed, no trajectory | `500` |
| `-7` | Time parameterization failed | `500` |
| `-8` | Jacobian fallback failed | `500` |
| `-9` | Near singularity | `500` |
| `-10` | Collision detected | `500` |
| `-11` | Cartesian path unreachable/collision/joint limit | `400` |
| `-12` | Hardware not ready, EtherCAT not OP | `503` |
| `-13` | Drive operation not enabled | `409` |
| `-14` | Controller execution failed | `409` |

Error body:

```json
{
  "result": -13,
  "success": false,
  "error": "Drive operation is not enabled; call POST /drive/enable before motion"
}
```

## Documentation Endpoints

### GET /docs and GET /api/docs

Purpose:
Serve Swagger UI.

Flow:
1. Flask receives request.
2. Endpoint returns static `SWAGGER_HTML`.
3. Browser loads `/openapi.json`.
4. Browser loads Swagger UI assets from CDN.

Success:
HTTP `200`, content type `text/html`.

Failure:
No robot-runtime dependency. If CDN is unavailable, the HTML still loads but
Swagger UI assets may not render in the browser.

### GET /openapi.json

Purpose:
Serve the OpenAPI route/spec document used by Swagger.

Flow:
1. Flask receives request.
2. Endpoint returns `OPENAPI_SPEC`.

Success:
HTTP `200`, JSON OpenAPI document.

Failure:
No robot-runtime dependency.

## Startup Endpoints

### GET /health

Purpose:
Small health/status check for the HTTP server and runtime startup state.

Flow:
1. Calls `get_startup_status()`.
2. If `startup.error is None`, returns HTTP `200`.
3. If startup failed, returns HTTP `500`.

Success/starting:

```json
{
  "status": "http_ready",
  "phase": "http_ready",
  "ready": false,
  "error": null,
  "ros2_active": false
}
```

Ready:

```json
{
  "status": "ok",
  "phase": "ready",
  "ready": true,
  "error": null,
  "ros2_active": true
}
```

Startup failure:
HTTP `500`, same fields, `error` contains the failure text.

### GET /startup/status

Purpose:
Detailed startup polling endpoint for the frontend.

Flow:
1. Calls `get_startup_status()`.
2. Returns current phase/message/timestamps.
3. HTTP `200` unless startup has an error.
4. HTTP `500` if startup failed.

Response fields:
`phase`, `message`, `ready`, `error`, `started_at`, `updated_at`, `ros2_active`.

## Motion Endpoints

### POST /move/linear

Purpose:
Submit a linear TCP move.

Flow:
1. Parses JSON through `parse_move_linear_request(request.json)`.
2. Parser requires `position` with 6 values.
3. Parser accepts optional `tool`, `user`, `vel`, `acc`, `blocking`,
   `trajectory_optimizer`.
4. If `trajectory_optimizer` is present, it must be `TOTG` or `RUCKIG`.
5. Calls `robot.move_liner(...)`.
6. Backend checks drive enable with `_reject_if_drive_not_enabled("MOVE_LINER")`.
7. Backend checks `node.is_hardware_ready_for_motion()`.
8. Backend applies work object, gets tool transform, converts vel/acc percent to
   scaling, creates `SingleTargetStrategy`.
9. Calls `node.execute(strategy)`.
10. REST maps result:
    - `result > 0`: queued.
    - `result == 0`: accepted/non-queued.
    - `result < 0`: `motion_error_response(result)`.

Validation failure:
HTTP `400`.

```json
{"success": false, "error": "Invalid position format"}
```

Queued:
HTTP `202`.

```json
{
  "result": 1,
  "success": true,
  "queued": true,
  "queue_position": 1,
  "task_id": 42
}
```

Accepted:
HTTP `200`.

```json
{"result": 0, "success": true, "queued": false, "task_id": 42}
```

Motion failure:
Uses the global motion result mapping.

### POST /move/ptp

Purpose:
Submit a point-to-point move.

Flow:
1. Parses JSON through `parse_move_linear_request(request.json)`.
2. Same validation as `/move/linear`.
3. Calls `robot.move_ptp(...)`.
4. Backend checks drive enable and hardware readiness.
5. Backend applies work object, gets tool transform, converts vel/acc percent to
   scaling, creates `PtpTargetStrategy`.
6. Calls `node.execute(strategy)`.
7. REST maps `result > 0`, `result == 0`, and `result < 0` the same as
   `/move/linear`.

Responses:
Same shape as `/move/linear`.

### POST /execute/path

Purpose:
Execute a Cartesian waypoint path.

Flow:
1. Parses JSON through `parse_execute_path_request(request.json)`.
2. Parser requires `path`.
3. If path is wrapped one extra level, parser unwraps it.
4. Optional `trajectory_optimizer` must be `TOTG` or `RUCKIG`.
5. Optional `orientation_mode` must be `constant` or `per_waypoint`.
6. Calls `robot.execute_path(...)`.
7. Backend rejects if drives are not enabled.
8. Backend rejects if hardware is not ready.
9. Backend converts vel/acc percent to scaling and builds path waypoints.
10. Backend plans/executes through the MoveIt motion stack.
11. REST maps integer result.

Validation failure:
HTTP `400`, `success: false`.

Execution exception:
HTTP `500`.

```json
{"result": -1, "success": false, "error": "<exception text>"}
```

Queued/accepted/motion failure:
Same result mapping as `/move/linear`.

### POST /execute/sequence

Purpose:
Execute a list of linear/PTP segments.

Flow:
1. Flask receives `POST /execute/sequence`.
2. REST calls `parse_execute_sequence_request(request.json)`.
3. Parser normalizes `payload = data or {}`.
4. Parser reads `segments`.
5. If `segments` is not a non-empty list:
   - raises `ValueError("Missing non-empty 'segments'")`
   - REST returns HTTP `400`, `success: false`.
6. For each raw segment:
   - segment must be a JSON object
   - `position` must exist and contain 6 values
   - `motion_type` defaults to `linear`
   - `motion_type` must be `linear` or `ptp`
   - `vel`, `acc`, and `blend_radius` are converted to floats
7. Parser returns normalized payload:

```json
{
  "segments": [
    {
      "position": [300, 0, 300, 180, 0, 0],
      "vel": 20.0,
      "acc": 20.0,
      "motion_type": "linear",
      "blend_radius": 0.0
    }
  ],
  "tool": 0,
  "user": 0,
  "blocking": false
}
```

8. REST calls:

```python
robot.execute_sequence(
    payload["segments"],
    tool=payload["tool"],
    user=payload["user"],
    blocking=payload["blocking"],
)
```

9. `MoveItRobotBackend.execute_sequence(...)` starts timing.
10. If `segments` is empty, backend returns `-1`.
11. Backend calls `_reject_if_drive_not_enabled("EXECUTE_SEQUENCE")`.
12. `_reject_if_drive_not_enabled` calls:

```python
node.is_drive_operation_enabled_for_motion()
```

13. If drives are not operation-enabled:
    - logs `node.get_drive_enable_fault_reason()`
    - returns `config.MOTION_ERROR_DRIVE_NOT_ENABLED` (`-13`)
    - REST maps this to HTTP `409`.
14. Backend checks hardware readiness:

```python
node.is_hardware_ready_for_motion()
```

15. If hardware is not ready:
    - logs `node.get_hardware_fault_reason()`
    - returns `config.MOTION_ERROR_HARDWARE_NOT_READY` (`-12`)
    - REST maps this to HTTP `503`.
16. Backend gets active tool transform:

```python
tool_transform = node.get_tool_transform(tool)
```

17. Backend creates `sequence_segments`.
18. For each normalized segment:
    - calls `self.apply_workobject(segment["position"], user_id=user)`
    - converts `vel`, `acc`, and `blend_radius` to floats
    - carries `motion_type`
19. Backend creates:

```python
SequenceStrategy(sequence_segments, tool_transform=tool_transform)
```

20. Backend calls:

```python
node.execute(SequenceStrategy(...))
```

21. `RobotController.execute(...)` first checks drive enable again:

```python
is_drive_operation_enabled_for_motion()
```

22. If not enabled, it returns `-13`.
23. `RobotController.execute(...)` then checks hardware readiness again:

```python
is_hardware_ready_for_motion()
```

24. If not ready, it returns `-12`.
25. `RobotController.execute(...)` checks motion-stack readiness:

```python
is_motion_stack_ready()
```

26. If motion stack is not ready, it returns `-12`.
27. If ready, `RobotController.execute(...)` delegates to:

```python
MotionCoordinator.execute(strategy, queue_if_busy=True)
```

28. `MotionCoordinator.execute(...)` checks if another motion is active.
29. If busy and the strategy is queueable:
    - queues `_execute_strategy_task(strategy)` into `motion_queue`
    - stores `last_submitted_task_id`
    - returns queue position as a positive integer.
30. If busy and not queueable:
    - returns `-1`.
31. If not busy:
    - allocates a task ID
    - marks it as the immediate task
    - calls `strategy.execute(node)`.
32. `SequenceStrategy.execute(robot_controller)` imports and calls:

```python
send_motion_sequence(
    planner_context,
    self.segments,
    tool_transform=self.tool_transform,
)
```

33. `planner_context` is `robot_controller.planner_context` when present.
34. `send_motion_sequence(...)` checks `segments` again.
35. If empty, returns `-1`.
36. It waits for the MoveIt motion sequence service:

```python
rc.wait_for_motion_sequence_service(timeout_sec=1.0)
```

37. Internally this calls:

```python
sequence_client.wait_for_service(timeout_sec=1.0)
```

38. If service is unavailable:
    - logs `GetMotionSequence service not available`
    - returns `-2`
    - REST maps this to HTTP `503`.
39. It checks motion-stack readiness again:

```python
rc.is_motion_stack_ready()
```

40. If not ready:
    - logs `rc.get_motion_stack_fault_reason()`
    - returns `config.MOTION_ERROR_HARDWARE_NOT_READY` (`-12`)
    - REST maps this to HTTP `503`.
41. It chooses the tool transform:
    - uses passed `tool_transform`, otherwise `rc.T_tool`.
42. It extracts waypoint poses from segment positions.
43. It calls:

```python
_to_pose_list(rc, waypoints, T_tool, check_last_only=False)
```

44. `_to_pose_list` converts `[x, y, z, rx, ry, rz]` waypoints into MoveIt
    poses using the active tool transform and validates conversion/safety.
45. If `_to_pose_list` returns an error code, `send_motion_sequence` returns
    that code directly.
46. It constructs:

```python
GetMotionSequence.Request()
```

47. If `rc.current_joint_state` exists:
    - deep-copies it
    - stamps it with current ROS time
    - converts it to a `RobotState` for the first segment start state.
48. For each segment and pose:
    - creates `MotionSequenceItem()`
    - calls `_build_motion_plan_request(rc, pose, segment, start_state=...)`
    - converts `blend_radius` from mm to meters
    - appends the item to `req.request.items`.
49. `_build_motion_plan_request(...)` creates a `MotionPlanRequest`:
    - `group_name = config.PLANNING_GROUP`
    - `pipeline_id = config.SEQUENCE_PLANNING_PIPELINE`
    - `planner_id = "PTP"` when `motion_type == "ptp"`, else `"LIN"`
    - `num_planning_attempts = 1`
    - `allowed_planning_time = config.SEQUENCE_ALLOWED_PLANNING_TIME_S`
    - velocity/acceleration scale from segment percent values
    - pose goal constraints from `_pose_goal_constraints(pose)`.
50. `_pose_goal_constraints(...)` builds:
    - position constraint around target pose
    - orientation constraint around target orientation
    - tolerances from sequence config.
51. `send_motion_sequence(...)` calls:

```python
generation = _begin_execution(rc)
```

52. `_begin_execution` marks execution active and increments plan generation.
53. It calls:

```python
future = rc.request_motion_sequence(req)
```

54. `PlannerContext.request_motion_sequence(...)` calls:

```python
sequence_client.call_async(req)
```

55. It attaches async callback:

```python
future.add_done_callback(lambda f: _sequence_response(rc, f, generation, started_at))
```

56. `send_motion_sequence(...)` returns `0` immediately after submitting the
    async MoveIt request.
57. Because immediate result is `0`, `MoveItRobotBackend.execute_sequence(...)`
    logs successful submission.
58. If request was `blocking: false`, backend returns `0` to REST immediately.
59. REST returns HTTP `202`, `success: true`, `accepted: true`,
    `final: false`, `state: "ACCEPTED_ASYNC"`, `queued: false`.
60. If request was `blocking: true`, backend waits while `node.is_executing`
    until `config.BLOCKING_MOVE_TIMEOUT_S`.
61. When execution finishes, backend returns `node.last_move_result`.
62. If blocking wait times out, backend returns `-1`.
63. Later, when MoveIt sequence planning completes, `_sequence_response(...)`
    runs from the async future callback.
64. `_sequence_response(...)` first checks:

```python
_is_stale(rc, generation)
```

65. If stale/preempted, it discards the response.
66. It reads:
    - `future.result()`
    - `response.response.error_code.val`
    - `response.response.planned_trajectories`
67. If MoveIt error code is not success (`1`):
    - logs sequence planning failure
    - calls `_set_result(rc, -6)`.
68. `_set_result` records final result, clears execution state, and completes
    current motion task.
69. If MoveIt succeeds, it calls:

```python
_combine_joint_trajectories(planned_trajectories)
```

70. `_combine_joint_trajectories(...)`:
    - extracts each segment joint trajectory
    - verifies joint names match
    - skips duplicate first point after the first trajectory
    - offsets `time_from_start` so segments are continuous
    - returns one combined `JointTrajectory`.
71. If combining fails or produces no points:
    - logs failure
    - calls `_set_result(rc, -6)`.
72. If combining succeeds, callback calls:

```python
_send_trajectory_to_controller(rc, joint_trajectory)
```

73. `_send_trajectory_to_controller(...)` sends the combined trajectory to the
    `manipulator_controller` action server.
74. Controller result callback eventually updates `last_move_result` and clears
    execution state.

Validation failure:
HTTP `400`, `success: false`.

Examples:

```json
{"success": false, "error": "Missing non-empty 'segments'"}
```

```json
{"success": false, "error": "Invalid segment 0: expected object"}
```

```json
{"success": false, "error": "Invalid segment 0: position must have 6 values"}
```

```json
{"success": false, "error": "Invalid segment 0: motion_type must be linear or ptp"}
```

```json
{"success": false, "error": "Invalid segment 0: vel/acc/blend_radius must be numeric"}
```

Execution exception:
HTTP `500`, `result: -1`, `success: false`.

Immediate accepted response for non-blocking sequence:
HTTP `202`.

```json
{
  "result": 0,
  "success": true,
  "accepted": true,
  "final": false,
  "state": "ACCEPTED_ASYNC",
  "queued": false,
  "task_id": 42,
  "status_url": "/status",
  "message": "sequence accepted; planning/execution completes asynchronously, poll /status for final result"
}
```

Queued response when another queueable motion is active:
HTTP `202`.

```json
{
  "result": 1,
  "success": true,
  "accepted": true,
  "final": false,
  "state": "QUEUED",
  "queued": true,
  "queue_position": 1,
  "task_id": 43,
  "status_url": "/status",
  "message": "sequence queued; poll /status for current_task_id, last_completed_task_id, and last_completed_result"
}
```

Important non-blocking caveat:
For `blocking: false`, HTTP `202` means the sequence request was accepted and
the async MoveIt planning request was submitted. It is not a final motion
success. Final planning/controller success is resolved later through
`_sequence_response(...)`, controller result callbacks, `/status`, and motion
result state.

Blocking success:
If `blocking: true`, backend waits for execution to finish and returns
`node.last_move_result`. REST returns:

HTTP `200`.

```json
{
  "result": 0,
  "success": true,
  "accepted": true,
  "final": true,
  "state": "COMPLETED",
  "queued": false,
  "task_id": 42
}
```

Blocking timeout:
If `blocking: true` and `node.is_executing` remains true past
`config.BLOCKING_MOVE_TIMEOUT_S`, backend returns `-1`; REST maps this through
the global motion result mapping.

Synchronous rejection:
If the request reaches the backend but is rejected before async planning starts,
REST returns the global motion error body plus sequence context:

```json
{
  "result": -13,
  "success": false,
  "accepted": false,
  "final": true,
  "state": "REJECTED",
  "queued": false,
  "task_id": 42,
  "error": "Drive operation is not enabled; call POST /drive/enable before motion"
}
```

Frontend rule:
For this endpoint:

- HTTP `202` means accepted but not final.
- `final: false` means poll `/status`.
- `last_completed_task_id == task_id` means the submitted task has finished.
- `last_completed_result == 0` means final motion success.
- Any negative `last_completed_result` must be mapped through the motion result
  table.
- Do not show the sequence as complete from `success: true` unless `final: true`
  or `/status` reports the task completed successfully.

Motion failure:
Uses the global motion result mapping. Common sequence-specific failures:

| Failure | Result | HTTP |
| --- | --- | --- |
| Drives not enabled | `-13` | `409` |
| Hardware not ready | `-12` | `503` |
| Motion sequence service unavailable | `-2` | `503` |
| Motion stack not ready | `-12` | `503` |
| Pose conversion/safety failure | returned `_to_pose_list` code | mapped globally |
| MoveIt sequence planning failed | async `_set_result(-6)` | visible after async completion |
| Planned trajectories cannot be combined | async `_set_result(-6)` | visible after async completion |
| Controller execution failed | typically `-14` | `409` when surfaced synchronously/status |

### POST /execute/ordered_motion_chain

Purpose:
Execute an ordered chain of motion segments with status tracking.

Flow:
1. Parses JSON through `parse_execute_ordered_motion_chain_request(request.json)`.
2. Parser requires non-empty `segments`.
3. Optional `trajectory_optimizer` must be `TOTG` or `RUCKIG`.
4. Each segment must be an object.
5. Segment `type`/`kind` must be `linear`, `path`, or `unwind_joint6`.
6. Linear segments require a 6-value `position`.
7. Path segments require `path`.
8. Unwind segments accept `vel`, `acc`, and `queue_if_busy`.
9. Calls `robot.execute_ordered_motion_chain(...)`.
10. Backend plans/executes ordered segments and updates ordered-chain status.
11. REST maps integer result.

Validation failure:
HTTP `400`, `success: false`.

Execution exception:
HTTP `500`, `result: -1`, `success: false`.

Queued/accepted/motion failure:
Same result mapping as `/move/linear`.

### GET /execute/ordered_motion_chain/status

Purpose:
Return current ordered motion-chain state.

Flow:
1. Looks for `robot.get_ordered_motion_chain_status`.
2. If missing, returns supported false.
3. If present, calls it and verifies the return is a dictionary.
4. If returned dict has `error`, endpoint returns failure.

Unsupported:
HTTP `200`.

```json
{"success": true, "supported": false, "active": false}
```

Success:
HTTP `200`.

```json
{"success": true, "active": true, "phase": "..."}
```

Invalid status:
HTTP `500`.

```json
{"success": false, "error": "invalid ordered motion chain status"}
```

### POST /unwind/joint6

Purpose:
Submit a standalone Joint 6 unwind operation.

Flow:
1. Reads JSON directly.
2. `blocking` defaults to `true`.
3. `queue_if_busy` defaults to `true`.
4. Optional `vel` and `acc` must be numeric if present.
5. Calls `robot.unwind_joint6(...)`.
6. Backend rejects if no node, recent ordered-unwind failure suppression is active,
   drives are not enabled, or hardware is not ready.
7. Backend calls `_unwind_joint6_with_rotational_path(...)`.
8. REST maps integer result.

Validation failure:
HTTP `400`.

```json
{
  "result": -1,
  "success": false,
  "error": "vel and acc must be numeric when provided"
}
```

Execution exception:
HTTP `500`, `result: -1`, `success: false`.

Queued/accepted/motion failure:
Same result mapping as `/move/linear`.

### POST /jog

Purpose:
Jog along one robot axis.

Flow:
1. Parses JSON through `parse_jog_request(request.json)`.
2. Requires `axis`, `direction`, `step`, `vel`, and `acc`.
3. `axis` accepts names `X`, `Y`, `Z`, `RX`, `RY`, `RZ` or enum values `1..6`.
4. `direction` accepts `PLUS`, `MINUS` or enum values `1`, `-1`.
5. `step`, `vel`, and `acc` must be numeric.
6. Parser flips Z step sign for the runtime convention.
7. Calls `robot.start_jog(axis, direction, step, vel, acc)`.
8. Backend rejects if drives are not enabled, hardware is not ready, monitor/current
   position is unavailable, or motion is already active/queued.
9. Backend computes target pose and submits a motion.
10. REST maps result.

Validation failure:
HTTP `400`, `result: -1`, `success: false`.

Success:
HTTP `200`.

```json
{"result": 0, "success": true}
```

Motion failure:
Uses global motion result mapping.

Unhandled exception:
HTTP `500`, `result: -1`, `success: false`.

### POST /stop

Purpose:
Stop active robot motion and clear queued work.

Flow:
1. Calls `robot.stop_motion()`.
2. Backend sets `_ordered_motion_chain_stop_requested = True`.
3. Backend calls `node.stop_motion()`.
4. If returned value is not a dict, REST converts it to a structured error.
5. REST returns normalized stop fields.

Success or no active motion:
HTTP `200`.

```json
{
  "stop_state": "STOPPED",
  "stopped": true,
  "result": 0,
  "success": true,
  "queue_cleared": 0
}
```

No active motion may return `success: true` or `success: false` depending on
controller result state; frontend should inspect `stop_state`, `stopped`, and
`result`.

Unexpected backend result:
HTTP `200` with `success: false` and an `error` field.

## State Endpoints

### WS /ws/state

Purpose:
Read-only live telemetry stream for GUI clients without ROS 2 dependencies.
The WebSocket server runs in the same `zeroerr_runtime.py` process as Flask,
but on a separate port by default.

Default URL:

```text
ws://localhost:5001/ws/state
```

Configuration:

```yaml
REST_WS_STATE_ENABLED: true
REST_WS_STATE_HOST: 0.0.0.0
REST_WS_STATE_PORT: 5001
REST_WS_STATE_RATE_HZ: 20.0
```

Flow:
1. Runtime starts Flask HTTP server on `REST_PORT`.
2. Runtime starts a WebSocket server thread when `REST_WS_STATE_ENABLED` is true.
3. WebSocket server accepts only path `/ws/state`.
4. Unsupported WebSocket paths are closed with policy violation code `1008`.
5. When a client connects, server sends a `hello` frame:

```json
{
  "type": "hello",
  "endpoint": "/ws/state",
  "rate_hz": 20.0,
  "timestamp": 1785919000.123
}
```

6. Server then sends state frames at `REST_WS_STATE_RATE_HZ`.
7. If `robot` or `node` is not ready yet, frames are still sent with
   `runtime_ready: false`.
8. Once runtime is ready, each frame calls:
   - `robot.get_current_position()`
   - `robot.get_current_flange_position()`
   - `robot.get_current_velocity()`
   - `robot.get_current_acceleration()`
9. Missing fields are reported in `unavailable_fields`.
10. Missing values are `null`; the stream does not fake unavailable velocity or
    acceleration as zero.

Startup/not ready frame:

```json
{
  "type": "state",
  "sequence": 1,
  "timestamp": 1785919000.123,
  "success": false,
  "partial": true,
  "runtime_ready": false,
  "unavailable_fields": ["runtime"],
  "position": null,
  "flange_position": null,
  "velocity": null,
  "acceleration": null
}
```

Complete frame:

```json
{
  "type": "state",
  "sequence": 42,
  "timestamp": 1785919002.123,
  "success": true,
  "partial": false,
  "runtime_ready": true,
  "unavailable_fields": [],
  "position": [300.0, 0.0, 300.0, 180.0, 0.0, 0.0],
  "flange_position": [300.0, 0.0, 300.0, 180.0, 0.0, 0.0],
  "velocity": [0.0, 0.0, 0.0],
  "acceleration": [0.0, 0.0, 0.0]
}
```

Partial frame:

```json
{
  "type": "state",
  "sequence": 43,
  "timestamp": 1785919002.173,
  "success": false,
  "partial": true,
  "runtime_ready": true,
  "unavailable_fields": ["acceleration"],
  "position": [300.0, 0.0, 300.0, 180.0, 0.0, 0.0],
  "flange_position": [300.0, 0.0, 300.0, 180.0, 0.0, 0.0],
  "velocity": [0.0, 0.0, 0.0],
  "acceleration": null
}
```

Client rule:
Use this stream as the primary live telemetry source. If the WebSocket
disconnects, reconnect with backoff and optionally fall back to
`GET /state/kinematics` or `GET /state/snapshot`.

### GET /status

Purpose:
Return robot execution status plus runtime readiness fields.

Flow:
1. Calls `robot.node.status_publisher.get_status_dict()`.
2. If available, calls `robot.get_ordered_motion_chain_status()`.
3. Adds `success: true`.
4. Adds `runtime_state_snapshot()`.
5. `runtime_state_snapshot()` calls:
   - `node.get_drive_operation_status()`
   - `node.get_motion_interlock_status()`
   - `node.is_hardware_ready_for_motion()`
   - `node.get_hardware_fault_reason()` if not ready
   - `node.is_motion_stack_ready()`
   - `node.get_motion_stack_fault_reason()` if not ready

Success:
HTTP `200`, status fields plus `runtime_ready`, `hardware_ready`, `drive`, and
`motion_interlock`.

Failure:
No local try/except in this handler. Unexpected exceptions produce Flask `500`.

### GET /state/snapshot

Purpose:
Return common UI state in one request.

Flow:
1. Calls `robot.get_current_position()`.
2. Calls `robot.get_current_flange_position()`.
3. Calls `robot.get_current_velocity()`.
4. Adds missing fields to `unavailable_fields`.
5. Calls `robot.node.status_publisher.get_status_dict()`.
6. Reads `node.active_tool_name`.
7. Calls `robot.get_safety_walls_status()`.
8. Adds `runtime_state_snapshot()`.

Complete success:
HTTP `200`.

```json
{
  "success": true,
  "partial": false,
  "unavailable_fields": [],
  "position": [0, 0, 0, 0, 0, 0],
  "flange_position": [0, 0, 0, 0, 0, 0],
  "velocity": [0, 0, 0],
  "runtime_ready": true
}
```

Partial state:
HTTP `200`, but `success: false`, `partial: true`, and `unavailable_fields`
names the missing data.

Failure:
No local try/except. Unexpected exceptions produce Flask `500`.

### GET /state/kinematics

Purpose:
Return TCP position, velocity, and acceleration.

Flow:
1. Calls `robot.get_current_position()`.
2. Calls `robot.get_current_velocity()`.
3. Calls `robot.get_current_acceleration()`.
4. If position is unavailable, returns failure.
5. If velocity or acceleration is unavailable, returns partial state.

Position unavailable:
HTTP `503`.

```json
{"success": false, "error": "current position unavailable"}
```

Complete:
HTTP `200`, `success: true`, `partial: false`.

Partial:
HTTP `206`, `success: false`, `partial: true`, `unavailable_fields` lists
`velocity` and/or `acceleration`.

### GET /position/current

Purpose:
Return current TCP position.

Flow:
1. Calls `robot.get_current_position()`.
2. If `None`, returns unavailable.
3. Otherwise returns position.

Unavailable:
HTTP `503`.

```json
{"success": false, "error": "current position unavailable"}
```

Success:
HTTP `200`.

```json
{"success": true, "position": [0, 0, 0, 0, 0, 0]}
```

### GET /position/flange

Purpose:
Return current flange position.

Flow:
1. Calls `robot.get_current_flange_position()`.
2. If `None`, returns unavailable.
3. Otherwise returns position.

Unavailable:
HTTP `503`.

```json
{"success": false, "error": "current flange position unavailable"}
```

Success:
HTTP `200`, `success: true`, `position`.

### GET /velocity/current

Purpose:
Return current TCP velocity.

Flow:
1. Calls `robot.get_current_velocity()`.
2. If `None`, returns unavailable.
3. Otherwise returns velocity.

Unavailable:
HTTP `503`.

```json
{"success": false, "error": "current velocity unavailable"}
```

Success:
HTTP `200`, `success: true`, `velocity`.

## Planning Endpoint

### POST /reachability/pose

Purpose:
Validate whether a target pose is reachable from a start pose.

Flow:
1. Reads JSON directly.
2. Reads `target_position` or fallback `position`.
3. Reads optional `start_position`.
4. If target is missing or not 6 values, returns validation failure.
5. If start is missing, calls `robot.get_current_position()`.
6. If start is missing/unavailable or not 6 values, returns validation failure.
7. Calls `validate_pose_from_start(node, robot, ...)`.
8. Helper performs pose conversion and MoveIt IK/state/path checks.
9. Result is converted to JSON-safe values.
10. HTTP status:
    - reachable: `200`
    - `reason == "cartesian_path_partial"`: `409`
    - other unreachable result: `400`
11. `success` is set equal to `reachable`.

Validation failure:
HTTP `400`.

```json
{"success": false, "reachable": false, "error": "Invalid target_position format"}
```

Reachable:
HTTP `200`.

```json
{"success": true, "reachable": true, "reason": "..."}
```

Unreachable/partial:
HTTP `400` or `409`.

```json
{"success": false, "reachable": false, "reason": "cartesian_path_partial"}
```

Unhandled exception:
HTTP `500`.

```json
{
  "success": false,
  "reachable": false,
  "reason": "rest_handler_exception",
  "error": "<exception text>"
}
```

## Tool Endpoints

### GET /tool/registry

Purpose:
Return configured tool registry.

Flow:
1. Calls `config.get_tool_registry_snapshot()`.
2. Adds `success: true`.

Success:
HTTP `200`.

Failure:
No local try/except. Unexpected exceptions produce Flask `500`.

### GET /tool/active

Purpose:
Return active tool name.

Flow:
1. Reads `node.active_tool_name`.
2. Defaults to `TOOL_0`.

Success:
HTTP `200`.

```json
{"success": true, "tool_name": "TOOL_1"}
```

### POST /tool/active

Purpose:
Set active tool by `tool_id`, `name`, or `tool_name`.

Flow:
1. Reads JSON.
2. If `tool_id` exists, calls `config.resolve_tool_name(tool_id)`.
3. Otherwise reads `name` or `tool_name`.
4. If no name can be resolved, raises `ValueError`.
5. Calls `node.set_tool(tool_name)`.
6. Returns current `node.active_tool_name`.

Validation failure:
HTTP `400`, `success: false`.

Exception:
HTTP `500`, `success: false`.

Success:
HTTP `200`, `success: true`, `tool_name`.

### POST /tool/registry/<tool_id>

Purpose:
Update one tool registry entry.

Flow:
1. Reads path parameter `tool_id`.
2. Reads JSON fields `name`, `transform`, `persist`.
3. Calls `config.update_tool_registry(...)`.
4. If the active tool matches the updated ID, calls `node.set_tool(...)`.
5. Returns updated registry snapshot.

Validation failure:
HTTP `400`, `success: false`.

Exception:
HTTP `500`, `success: false`.

Success:
HTTP `200`, `success: true`, plus registry snapshot fields.

## Safety Wall Endpoints

### GET /safety/walls/enabled

Purpose:
Return only the enabled state plus status details.

Flow:
1. Calls `robot.get_safety_walls_status()`.
2. Converts result with `_as_dict(...)`.
3. If result contains `error`, returns failure.
4. Otherwise returns `enabled`.

Success:
HTTP `200`, `success: true`, `enabled`.

Failure:
HTTP `503`, `success: false`, `error`.

### GET /safety/walls/status

Purpose:
Return full safety wall status.

Flow:
1. Calls `robot.get_safety_walls_status()`.
2. Converts result with `_as_dict(...)`.
3. If result contains `error`, returns HTTP `503`.
4. Otherwise returns HTTP `200`.

Success/failure:
Same success semantics as `/safety/walls/enabled`.

### POST /safety/walls/enable

Purpose:
Enable safety walls and verify returned status says enabled.

Flow:
1. Calls `robot.enable_safety_walls()`.
2. Backend calls `node.enable_safety_walls()`.
3. Node calls `safety_manager.enable_safety()`.
4. Node returns `safety_manager.get_status()`.
5. REST succeeds only if returned `enabled` is true and no `error` exists.

Success:
HTTP `200`, `success: true`, `enabled: true`.

Failure:
HTTP `500`, `success: false`, returned status/error.

### POST /safety/walls/disable

Purpose:
Disable safety walls and verify returned status says disabled.

Flow:
1. Calls `robot.disable_safety_walls()`.
2. Backend calls `node.disable_safety_walls()`.
3. Node calls `safety_manager.disable_safety()`.
4. Node returns `safety_manager.get_status()`.
5. REST succeeds only if returned `enabled` is false and no `error` exists.

Success:
HTTP `200`, `success: true`, `enabled: false`.

Failure:
HTTP `500`, `success: false`, returned status/error.

## Frames and IO Endpoints

### POST /workobject/set

Purpose:
Set active work object origin.

Flow:
1. Reads JSON.
2. Requires `origin` with 6 values.
3. Creates `WorkObject(origin=origin)`.
4. Calls `robot.set_workobject(workobject, user_id=...)`.
5. Returns origin and user ID.

Validation failure:
HTTP `400`.

```json
{"success": false, "error": "Invalid origin format"}
```

Exception:
HTTP `500`, `success: false`, `error`.

Success:
HTTP `200`.

```json
{"success": true, "origin": [0, 0, 0, 0, 0, 0], "user_id": 0}
```

### POST /io/digital_output

Purpose:
Publish a digital output command.

Flow:
1. Reads JSON.
2. Requires `port` and `value`.
3. Converts both to integers.
4. `port` must be `>= 0`.
5. `value` must be `0` or `1`.
6. Calls `robot.setDigitalOutput(port, value)`.
7. Backend creates/reuses publisher on `/set_do`.
8. Backend publishes `Int32MultiArray([port, value])`.
9. Backend returns `0` on publish success, `-1` on failure.

Missing field:
HTTP `400`, `result: -1`, `success: false`.

Invalid type/value:
HTTP `400`, `result: -1`, `success: false`.

Publish success:
HTTP `200`.

```json
{"result": 0, "success": true, "port": 0, "value": 1}
```

Publish failure:
HTTP `500`.

```json
{"result": -1, "success": false, "port": 0, "value": 1}
```

Exception:
HTTP `500`, `result: -1`, `success: false`, `error`.

## Drive Endpoints

### GET /drive/status

Purpose:
Return drive operation-enable state and hardware readiness.

Flow:
1. Calls `node.get_drive_operation_status()`.
2. Calls `node.is_hardware_ready_for_motion()`.
3. If hardware is not ready, calls `node.get_hardware_fault_reason()`.
4. Returns combined status.

Success:
HTTP `200`.

```json
{
  "success": true,
  "requested_enabled": true,
  "actual_enabled": true,
  "motion_allowed_by_drive_enable": true,
  "state": "OPERATION_ENABLED",
  "statusword": [4787, 4787, 4787, 4787, 4787, 4787],
  "status_state": ["operation_enabled", "operation_enabled"],
  "hardware_ready": true,
  "hardware_fault": null
}
```

Exception:
HTTP `500`, `success: false`, `error`.

### POST /drive/enable

Purpose:
Request drive operation enable and verify that the drives actually report
`operation_enabled`.

Flow:
1. Calls `node.set_drive_operation_enabled(True)`.
2. Runtime adapter rejects immediately if `node.is_hardware_ready_for_motion()`
   is false.
3. If hardware is ready, adapter activates drive set controllers.
4. Adapter sends hold-position trajectory before enable.
5. Adapter publishes enable pulse on configured drive-enable command topic.
6. Adapter publishes zeros to release the pulse.
7. Adapter deactivates set controllers.
8. Adapter stores requested enable state.
9. REST calls `drive_command_response(result, desired_enabled=True)`.
10. If command was rejected, REST maps state:
    - `HARDWARE_NOT_READY`: HTTP `503`
    - `UNSUPPORTED`: HTTP `501`
    - anything else: HTTP `500`
11. If command was accepted, REST polls `node.get_drive_operation_status()`
    until `actual_enabled` and `requested_enabled` are both true or timeout.
12. If verified, returns HTTP `200`, `success: true`.
13. If accepted but not verified, returns HTTP `202`, `success: false`.

Verified success:
HTTP `200`.

```json
{
  "success": true,
  "command_accepted": true,
  "desired_enabled": true,
  "requested_enabled": true,
  "actual_enabled": true,
  "motion_allowed_by_drive_enable": true,
  "state": "OPERATION_ENABLED",
  "request": {
    "success": true,
    "requested_enabled": true,
    "state": "ENABLE_REQUESTED"
  }
}
```

Accepted but not verified:
HTTP `202`, `success: false`, `command_accepted: true`, `error`.

Rejected:
HTTP `503`, `501`, or `500`, `success: false`, `command_accepted: false`.

Exception:
HTTP `500`, `success: false`, `error`.

Frontend rule:
Only treat drives as enabled when HTTP is `200`, `success` is true,
`actual_enabled` is true, and `motion_allowed_by_drive_enable` is true.

### POST /drive/disable

Purpose:
Request drive operation disable and verify drives no longer report enabled.

Flow:
1. Calls `node.set_drive_operation_enabled(False)`.
2. Adapter activates drive set controllers.
3. Adapter publishes disable pulse.
4. Adapter publishes zeros to release the pulse.
5. Adapter deactivates set controllers.
6. Adapter stores requested disabled state.
7. REST calls `drive_command_response(result, desired_enabled=False)`.
8. REST polls `node.get_drive_operation_status()` until both are false:
   - `actual_enabled`
   - `requested_enabled`
9. If verified, HTTP `200`, `success: true`.
10. If accepted but not verified, HTTP `202`, `success: false`.

Verified success:
HTTP `200`, `success: true`, `actual_enabled: false`,
`requested_enabled: false`.

Accepted but not verified:
HTTP `202`, `success: false`, `command_accepted: true`, `error`.

Rejected/exception:
Same semantics as `/drive/enable`.

## Motion Interlock Endpoints

### GET /motion/interlock/status

Purpose:
Return motion interlock state.

Flow:
1. Calls `node.get_motion_interlock_status()`.
2. Adds `success: true`.

Success:
HTTP `200`.

```json
{"success": true, "active": false, "reason": ""}
```

Failure:
No local try/except. Unexpected exceptions produce Flask `500`.

### POST /motion/interlock/reset

Purpose:
Reset the motion interlock and return post-reset hardware state.

Flow:
1. Calls `node.reset_motion_interlock()`.
2. Node clears `_motion_interlock_active` and `_motion_interlock_reason`.
3. Node returns whether an interlock was actually reset.
4. REST calls `node.is_hardware_ready_for_motion()`.
5. If hardware is not ready, REST calls `node.get_hardware_fault_reason()`.
6. REST calls `node.get_motion_interlock_status()` for final interlock state.

Success:
HTTP `200`.

```json
{
  "success": true,
  "reset": true,
  "previous_reason": "EtherCAT not fully OP",
  "hardware_ready": true,
  "hardware_fault": null,
  "motion_interlock": {"active": false, "reason": ""}
}
```

Failure:
No local try/except. Unexpected exceptions produce Flask `500`.
