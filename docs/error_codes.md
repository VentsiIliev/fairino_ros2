# Error Codes & HTTP Status Reference

## Internal Motion Error Codes

These codes are returned by `RobotController`, `FairinoRos2Robot`, and all motion methods.

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

## Where Each Code Originates

| Code | Source |
|------|--------|
| `-2` | `trajectory_planner.py` — `/compute_cartesian_path` service call timeout/unavailable |
| `-3` | `safety_wall_manager.py` — waypoint outside workspace mesh boundary |
| `-4` | `robot_monitor.py` — no cached position yet from `/cartesian_position` |
| `-5` | `motion_queue.py` — queue at capacity (10 tasks) |
| `-6` | `trajectory_planner.py` — MoveIt returns empty or < 1% complete path |
| `-7` | `trajectory_optimization.py` — `/apply_ipp` service fails |
| `-8` | `trajectory_planner.py` — Jacobian-based fallback path also fails |
| `-9` | `trajectory_planner.py` — manipulability below singularity threshold |
| `-10` | `trajectory_planner.py` — collision object detected during Jacobian check |

---

## REST API Endpoints

Both `rest_server.py` (embedded) and `fairino_bridge_server.py` (standalone) expose the same interface.

### Motion Endpoints — Response Shapes

All motion endpoints (`/move/cartesian`, `/move/linear`, `/execute/path`, `/jog`) return the same structured responses:

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

`stop_motion()` returns `0` if motion was actively cancelled, `-1` if nothing was running.
Both cases return HTTP 200 — `-1` is informational, not an error.

```json
{"stopped": true,  "result": 0,  "success": true}   // motion was cancelled
{"stopped": false, "result": -1, "success": false}   // nothing was running
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

> **Note:** `get_current_acceleration()` exists in `FairinoRos2Robot` but is not yet exposed
> as a REST endpoint in either server. Add `/acceleration/current` if needed.

### `/jog` — Input Validation

`rest_server.py` validates `axis` and `direction` against `RobotAxis` / `Direction` enums
and returns HTTP 400 with a descriptive error before calling the robot:

```json
{"result": -1, "success": false, "error": "Invalid 'axis': 99"}   // HTTP 400
{"result": -1, "success": false, "error": "Missing 'direction'"}  // HTTP 400
```

`fairino_bridge_server.py` does not validate enums — it passes raw values directly.

### `/workobject/set` — Response Shape

No failure mode from the underlying `set_workobject()` call; always returns HTTP 200.

```json
{"success": true}
```

### `/health` (GET)

```json
{"status": "ok", "ros2_active": true}
```

### `/status` (GET) — `rest_server.py` only

Returns the live robot status dict from `robot_status_publisher.py`.

```json
{"state": "IDLE", "queue_size": 0, ...}
```
