# Error Codes & HTTP Status Reference

## Internal Motion Error Codes

These codes are returned by `RobotController`, `MoveItRobotBackend`, and all motion methods.

| Code | Meaning | REST HTTP Status |
|------|---------|-----------------|
| `> 0` | Queued at position N in motion queue | 202 Accepted |
| `0` | Success / executing immediately | 200 OK |
| `-1` | Busy / invalid input / generic error | 500 Internal Server Error |
| `-2` | MoveIt service unavailable | 503 Service Unavailable |
| `-3` | Safety violation (out of workspace) | 400 Bad Request |
| `-4` | No current position available | 500 Internal Server Error |
| `-5` | Motion queue full (max 10 tasks) | 503 Service Unavailable |
| `-6` | Path planning failed (MoveIt returned no trajectory) | 500 Internal Server Error |
| `-7` | Time parameterization failed (TOTG/Ruckig) | 500 Internal Server Error |
| `-8` | Jacobian fallback failed | 500 Internal Server Error |
| `-9` | Near-singularity detected | 500 Internal Server Error |
| `-10` | Collision detected in Jacobian check | 500 Internal Server Error |
| `-11` | Cartesian path planning failed (target unreachable, collision, or joint-limit constraint) | 400 Bad Request |

## Trajectory Optimizer Selection

Trajectory time parameterization is now selected through an optimizer strategy,
not a planner-level `if/else` branch.

Current options in:
- [/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts/config.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts/config.py)

Setting:

```python
TRAJECTORY_OPTIMIZER = "TOTG"   # or "RUCKIG"
```

Behavior:
- `TOTG`
  - uses `/apply_ipp`
  - current default and known-good path
- `RUCKIG`
  - uses `/apply_ruckig`
  - available behind the same planning flow but not the default

Implementation seam:
- optimizer strategy classes live in:
  - [/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts/motion/execution/trajectory_optimizer.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts/motion/execution/trajectory_optimizer.py)
- `RobotController` builds the configured optimizer once at startup
- planner code delegates to `robot_controller.trajectory_optimizer.optimize(...)`

Result code `-7` still means:
- the selected optimizer failed
- this includes either TOTG or Ruckig failure

## Where Each Code Originates

| Code | Source |
|------|--------|
| `-2` | `planner_utils.py` / `trajectory.py` / `single_target.py` — `/compute_cartesian_path` or IK service call timeout/unavailable |
| `-3` | `safety_wall_manager.py` — waypoint outside workspace mesh boundary |
| `-4` | `robot_monitor.py` — no cached position yet from `/cartesian_position` |
| `-5` | `motion_queue.py` — queue at capacity (10 tasks) |
| `-6` | `trajectory_planner.py` (`_cartesian_path_response`) — MoveIt returns empty or < 1% complete path |
| `-7` | `trajectory_optimization.py` — `/apply_ipp` or `/apply_ruckig` service fails |
| `-8` | `single_target.py` (`_execute_jacobian_move`) — Jacobian-based fallback path also fails |
| `-9` | `jacobian_move.py` — near-zero Δq for unresolved target (near-singularity) |
| `-10` | `jacobian_move.py` (`_jacobian_check_and_execute`) — real collision contact detected during Jacobian validity check |
| `-11` | `trajectory_planner.py` / `trajectory.py` — fraction below threshold: target unreachable, collision, or joint-limit constraint |

## Jacobian Micro-Move Behavior

For sub-5mm single-target moves, the bridge may bypass MoveIt and use the Jacobian fallback path.

Near-zero joint-space solutions are now split into two cases:

- **No-op success**
  - the requested target pose is already satisfied within a very tight tolerance
  - the bridge returns success (`0`) and skips sending motion to the controller
  - this is intended for cases such as two consecutive named positions resolving to the same pose

- **True unresolved micro move / singularity**
  - the requested target pose is still meaningfully different
  - but the Jacobian solution collapses to near-zero joint motion
  - the bridge returns `-9`

This distinction prevents identical consecutive tool-change poses from failing while still preserving real micro-move requests such as `0.15 mm` Cartesian moves.

---

## Queueing Policy

- Queueable:
  - `/move/linear`
  - `/execute/path`
- Not queueable:
  - `/jog`
- `/move/linear` and `/execute/path` share one server-side motion queue.
  Their ordering is preserved across motion types, so a queued single-target move
  will execute before a later queued path, and vice versa.
- Queued tasks are stored as high-level motion requests and are planned when they
  actually start executing. This avoids stale start-state planning when different
  motion types are mixed in the queue.

## REST API Endpoints

Both `rest/server.py` (embedded) and `fairino_bridge_server.py` (standalone) expose the same interface.

### Motion Endpoints — Response Shapes

Queueable motion endpoints (`/move/linear`, `/execute/path`) may return queued responses.
`/jog` never queues; if any motion is executing or pending, jog is rejected.

Successful queued responses use the same shape:

**Success — immediate (HTTP 200)**
```json
{"result": 0, "success": true, "queued": false}
```

**Success — queued (HTTP 202)**
```json
{"result": 3, "success": true, "queued": true, "queue_position": 3}
```

**Error — safety violation (HTTP 400)**
```json
{"result": -3, "success": false, "error": "Safety violation"}
```

**Error — service/queue unavailable (HTTP 503)**
```json
{"result": -2, "success": false, "error": "MoveIt service unavailable"}
{"result": -5, "success": false, "error": "Motion queue is full"}
```

**Error — planning/execution failure (HTTP 500)**
```json
{"result": -6, "success": false, "error": "Path execution failed with code -6"}
```

### `/stop` — Response Shape

`/stop` now returns an explicit `stop_state` instead of overloading `success` alone.
All cases return HTTP 200; callers should inspect `stop_state`.

```json
{"stop_state": "STOPPED", "stopped": true,  "result": 0,  "success": true}   // motion was cancelled or queue was cleared
{"stop_state": "NO_ACTIVE_MOTION", "stopped": false, "result": -1, "success": true}   // nothing was running
{"stop_state": "STOP_REQUESTED_BUT_UNCONFIRMED", "stopped": false, "result": 1, "success": false, "error": "robot executing but no cancellable goal handle was available"}
{"stop_state": "ERROR", "stopped": false, "result": -2, "success": false, "error": "..."}
```

### `/position/current` (GET) — Response Shape

Returns the current TCP pose `[x, y, z, rx, ry, rz]` in mm / degrees, transformed into
the active workobject frame if one is set.

```json
{"position": [x, y, z, rx, ry, rz]}     // HTTP 200
{"error": "Failed to get position"}      // HTTP 500 — no data from /cartesian_position yet
```

### `/velocity/current` (GET) — Response Shape

Returns the current Cartesian velocity `[vx, vy, vz]` in mm/s published by the C++ state publisher at 50 Hz.

```json
{"velocity": [vx, vy, vz]}              // HTTP 200
{"error": "Failed to get velocity"}     // HTTP 500 — no data from /cartesian_velocity yet
```

> **Note:** `get_current_acceleration()` exists in `MoveItRobotBackend` but is not yet exposed
> as a REST endpoint in either server. Add `/acceleration/current` if needed.

### `/jog` — Input Validation And Busy Semantics

`rest/server.py` validates `axis` and `direction` against `RobotAxis` / `Direction` enums
and returns HTTP 400 with a descriptive error before calling the robot:

```json
{"result": -1, "success": false, "error": "Invalid 'axis': 99"}   // HTTP 400
{"result": -1, "success": false, "error": "Missing 'direction'"}  // HTTP 400
```

If any queued or active motion exists, `/jog` returns a busy error instead of queueing.

`fairino_bridge_server.py` does not validate enums — it passes raw values directly.

### `/reachability/pose` (POST) — Response Shape

Checks whether a target pose is kinematically reachable and collision-free from a start pose.
Uses IK + `/check_state_validity` without actually moving the robot.

```json
// HTTP 200 — reachable
{
  "success": true, "reachable": true, "reason": "ok", "fraction": 1.0,
  "target_joint_state": {"name": [...], "position": [...]},
  "start_position": [...], "target_position": [...]
}

// HTTP 409 — partial path
{"success": true, "reachable": false, "reason": "cartesian_path_partial", "fraction": 0.6}

// HTTP 400 — unreachable (IK failed, in collision, or outside workspace)
{"success": true, "reachable": false, "reason": "target_pose_ik_failed", "fraction": 0.0, "result": -11}
```

Accepts optional `start_joint_state` payload `{"name": [...], "position": [...]}` to seed IK from a
specific joint configuration instead of the current robot state.

---

### `/workobject/set` — Response Shape

No failure mode from the underlying `set_workobject()` call; always returns HTTP 200.

```json
{"success": true}
```

### `/health` (GET)

```json
{"status": "ok", "ros2_active": true}
```

### `/status` (GET) — `rest/server.py` only

Returns the live robot status dict from `robot_status_publisher.py`.

```json
{"state": "IDLE", "queue_size": 0, ...}
```
