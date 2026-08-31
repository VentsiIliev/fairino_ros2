# REST API

Base server:
- host: `config.REST_HOST`
- port: `config.REST_PORT`

## Endpoints

### `GET /health`
Basic liveness check.

Response:
- `status`
- `ros2_active`

### `POST /move/linear`
Submit a single Cartesian target.

Payload:
```json
{
  "position": [x, y, z, rx, ry, rz],
  "tool": 0,
  "user": 0,
  "vel": 30,
  "acc": 10,
  "blocking": true
}
```

Behavior:
- request parsing and default filling live in `rest/api_support.py`
- applies workobject transform
- plans Cartesian motion
- may execute immediately or queue

Success response fields:
- `result`
- `success`
- `queued`
- `queue_position` when queued
- `task_id`

### `POST /move/fast_lin`
Submit the same payload as `/move/linear`, but force the Pilz industrial `LIN`
planner through MoveIt's motion-sequence service. This diagnostic endpoint does
not call `compute_cartesian_path`; `/move/linear` remains unchanged so planning
latency can be compared directly. Pilz collision checking remains enabled.

### `POST /execute/path`
Execute a Cartesian path defined by XYZ or XYZ+orientation waypoints.

Payload fields:
- `path`
- optional `rx`, `ry`, `rz`
- `vel`
- `acc`
- `blocking`

Request parsing also lives in `rest/api_support.py`, including nested-path flattening and velocity/acceleration normalization.

### `GET /safety/walls/enabled`
Returns:
```json
{ "enabled": true }
```

### `GET /safety/walls/status`
Returns safety wall state and workspace bounds.

### `POST /safety/walls/enable`
Enable and republish safety walls.

### `POST /safety/walls/disable`
Disable and remove safety walls from the planning scene / RViz.

### `GET /position/current`
Returns current TCP pose.

### `POST /reachability/pose`
Planning-only pose validation from an explicit start pose.

Payload:
```json
{
  "start_position": [x, y, z, rx, ry, rz],
  "target_position": [x, y, z, rx, ry, rz],
  "tool": 0,
  "user": 0
}
```

Current validation semantics:
1. convert both poses into base frame using workobject/user transform
2. solve IK for the start pose
3. solve IK for the target pose, seeded from the start joint state
4. run `check_state_validity` on the target joint state

Response fields:
- `reachable`
- `reason`
- `fraction`
- `num_points`
- `result`
- `start_position`
- `target_position`

Important limitation:
- this endpoint validates endpoint feasibility and endpoint collision only
- it does **not** prove the whole Cartesian path is collision-free

### `GET /velocity/current`
Returns current cartesian velocity.

### `POST /stop`
Stop the active motion and clear queued motions.

Response states include:
- `STOPPED`
- `NO_ACTIVE_MOTION`
- `STOP_REQUESTED_BUT_UNCONFIRMED`
- `ERROR`

### `POST /workobject/set`
Set a workobject/user frame.

### `GET /status`
Return bridge motion/status summary.

### `POST /jog`
Jog one axis in a direction by a configured step.

Request parsing and normalization live in `rest/api_support.py`, not in `rest/server.py`.
Current normalization keeps the bridge's historical Z-axis sign inversion behavior.

## Motion Result Codes

Common result codes returned by the bridge:
- `0`
  - success / started immediately
- `>0`
  - accepted and queued
- `-2`
  - MoveIt service unavailable or service error
- `-3`
  - target outside workspace safety limits
- `-5`
  - motion queue full
- `-11`
  - planning or reachability failure

See `MOTION_ERROR_DESCRIPTIONS` in `rest/api_support.py` for the current full mapping.
