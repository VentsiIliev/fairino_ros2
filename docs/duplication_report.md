# Code Duplication Report — `motion/`

**Scope:** `scripts/motion/planning/` and `scripts/motion/execution/`
**Updated:** 2026-03-13

Legend: ✅ Fixed &nbsp;|&nbsp; 🔴 Open

---

## D1 — `_begin_execution` body inlined ✅ Fully fixed

All five call sites now use `_begin_execution(robot_controller)`.

| File | Function | Line | Status |
|------|----------|------|--------|
| `trajectory.py` | `_execute_path` | 97 | ✅ |
| `trajectory.py` | `_plan_then_approach` | 121 | ✅ |
| `trajectory.py` | `_execute_pending_trajectory` | 244 | ✅ |
| `single_target.py` | `_execute_jacobian_move` | 157 | ✅ |
| `single_target.py` | `_dispatch_moveit` | 167 | ✅ |

---

## D2 — `cart_path_client.wait_for_service` guard ✅ Fixed

Extracted to `_require_cart_path_service(rc, tag)` in `planner_utils.py`.

Both call sites replaced:
- `single_target.py`: `if not _require_cart_path_service(robot_controller, 'Single Point'): return -2`
- `trajectory.py`: `if not _require_cart_path_service(robot_controller, 'EXECUTE_PATH'): return -2`

---

## D3 — TCP → EE pose-building loop body ✅ Fixed

`_to_pose_list` moved to `planner_utils.py` (with `check_last_only` parameter).
`trajectory.py` now pre-builds 6D waypoints and calls `_to_pose_list(…, check_last_only=False)`.
`single_target.py` local definition removed; imports shared function.

```python
# planner_utils.py
def _to_pose_list(robot_controller, waypoints_mm, T_tool, check_last_only=True):
    ...
```

---

## D4 — `GetCartesianPath.Request` built inline ✅ Fully fixed

Both files now delegate to `_build_cartesian_request(...)`.

| File | Line | Status |
|------|------|--------|
| `single_target.py` | 214 | ✅ |
| `trajectory.py` | 95, 118, 151 | ✅ |

---

## D5 — 3D Euclidean distance — redundant recomputation 🔴

**`single_target.py:190`** — from raw mm deltas in `_execute_single_point`
```python
distance_mm = (dx * dx + dy * dy + dz * dz) ** 0.5
```

**`single_target.py:205`** — from `Pose` objects (metres) after `_to_pose_list`, same start/target pair
```python
delta_m = np.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2)
```

Lines 190 and 205 both compute the distance between the same two points (`start_wp` and `target_wp`),
only in different units and from different representations. The second is a redundant recalculation.

**`trajectory.py:68–69`** — `np.linalg.norm` inside per-segment loop
```python
total_dist_mm += np.linalg.norm(np.array(wp[:3]) - np.array(prev[:3]))
```

**`trajectory.py:84`** — `np.linalg.norm`, approach-distance check
```python
approach_dist = np.linalg.norm(np.array(waypoints_mm[0][:3]) - np.array(current_cart[:3]))
```

Note: `_generate_adaptive_waypoints` at `single_target.py:102` uses yet another form:
```python
distance_mm = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
```

**Fix:** Reuse `distance_mm / 1000.0` from line 190 instead of recomputing at line 205; standardise on `np.linalg.norm` everywhere.

---

## D6 — `ApplyIPP.Request` build block ✅ Fixed

`_build_apply_ipp_request(trajectory, vel_scaling, acc_scaling)` extracted in `trajectory_optimization.py`.
Both `apply_ruckig_service` and `apply_ipp_totg` call it.

---

## D7 — `handle_ruckig_response` vs `handle_ipp_response` ✅ Fixed

`_handle_apply_ipp_response(robot_controller, fut, trajectory, callback, tag)` extracted at module level in
`trajectory_optimization.py`. Both functions now delegate with a one-liner:

```python
# apply_ruckig_service:
future.add_done_callback(
    lambda f: _handle_apply_ipp_response(robot_controller, f, trajectory, callback, '[Ruckig]'))

# apply_ipp_totg:
future.add_done_callback(
    lambda f: _handle_apply_ipp_response(robot_controller, f, trajectory, callback, '[TOTG]'))
```

~55 lines removed.

---

## D8 — Inline `_set_result` in `_execute_jacobian_move` ✅ Fixed

`single_target.py` now imports `_set_result` from `planner_utils` and uses it:

```python
    if not ok:
        _set_result(robot_controller, -8)
        return -8
```

---

## D9 — Two clients for the same `/check_state_validity` service ✅ Fixed

`planner_diagnostics.py` now uses `_state_validity_client` (same attribute as `jacobian_move.py`).
Both modules share a single persistent ROS2 client handle on `robot_controller`.

---

## D10 — `except Exception → _set_result(rc, -2)` sibling blocks 🔴

Two closures inside `_plan_then_approach` have identical 4-line exception handlers.

**`trajectory.py:129–132`** (inside `on_ik_done`)
```python
        except Exception as e:
            robot_controller.get_logger().error(f'[EXECUTE_PATH] IK service error: {e}')
            _set_result(robot_controller, -2)
            return
```

**`trajectory.py:165–168`** (inside `on_plan_done`)
```python
        except Exception as e:
            robot_controller.get_logger().error(f'[EXECUTE_PATH] Plan service error: {e}')
            _set_result(robot_controller, -2)
            return
```

Only the log message tag differs (`IK service error` vs `Plan service error`).

---

## Summary

| ID | Files | Type | Status | Notes |
|----|-------|------|--------|-------|
| D1 | `trajectory.py`, `single_target.py` | Exact | ✅ | `_begin_execution` in `planner_utils` |
| D2 | `single_target.py`, `trajectory.py` | Exact | ✅ | `_require_cart_path_service` |
| D3 | `single_target.py`, `trajectory.py` | Near-dup | ✅ | `_to_pose_list` → `planner_utils` |
| D4 | `single_target.py`, `trajectory.py` | Divergence | ✅ | `_build_cartesian_request` |
| D5 | `single_target.py` | Redundant recompute | 🔴 | minor / low risk — no fix planned |
| D6 | `trajectory_optimization.py` | Exact | ✅ | `_build_apply_ipp_request` |
| D7 | `trajectory_optimization.py` | Near-dup | ✅ | `_handle_apply_ipp_response` |
| D8 | `single_target.py` | Near-dup | ✅ | `_set_result` from `planner_utils` |
| D9 | `jacobian_move.py`, `planner_diagnostics.py` | Architectural | ✅ | unified `_state_validity_client` |
| D10 | `trajectory.py` | Exact (closures) | 🔴 | minor sibling closures — no fix planned |

