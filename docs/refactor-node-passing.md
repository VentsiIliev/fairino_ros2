# Refactor: Remove Node-Passing Anti-Pattern from Motion Subsystem

**Status:** Core extraction complete — `PlannerContext` / `MotionCoordinator` / `TrajectoryExecutor` / `RobotStateStore` / `PlannerSupportService` / `TrajectoryOptimizer` are all in place.
Phase 3 (full `TrajectoryPlanner` class consolidation) is deprioritised — see notes below.
**Scope:** `scripts/motion/` + `scripts/robot_controller.py`
**Risk:** High (touches all motion paths) — requires full regression test after each phase

---

## Problem Statement

`RobotController` (the ROS2 `Node`) is passed as the first argument to every free function in the motion subsystem. These functions access ~15 different attributes directly off the node, making them effectively disguised methods of `RobotController` scattered across files.

### Affected files

| File | Offending functions |
|------|---------------------|
| `motion/planning/single_target.py` | `send_cartesian_goal`, `_execute_single_point`, `_dispatch_moveit`, `_execute_jacobian_move` |
| `motion/planning/trajectory.py` | `send_path_cartesian`, `_execute_path`, `_plan_then_approach`, `_execute_pending_trajectory` |
| `motion/planning/trajectory_planner.py` | `_cartesian_path_response`, `_apply_time_param`, `_build_cartesian_request` |
| `motion/planning/planner_utils.py` | `_set_result`, `_is_stale`, `_begin_execution`, `_require_cart_path_service`, `_to_pose_list` |
| `motion/planning/jacobian_move.py` | `_jacobian_fallback_move` (and internals) |
| `motion/planning/planner_diagnostics.py` | `_diagnose_fk_mismatch`, `_diagnose_start_collision` |
| `motion/execution/trajectory_executor.py` | `_send_trajectory_to_controller`, `_controller_goal_response`, `_controller_goal_result`, `_process_next_queued_task` |
| `motion/execution/trajectory_optimization.py` | `apply_ipp_totg`, `apply_ruckig_service` |

### Attributes currently accessed cross-file off the node

```
# ROS2 clients / actions
rc.cart_path_client
rc.controller_client
rc.ipp_client

# Execution state
rc.is_executing
rc.plan_generation
rc.last_move_result
rc.active_controller_goal
rc.active_execute_send_future

# Robot state
rc.prev_cartesian
rc.current_joint_state
rc.T_tool

# Concurrency
rc.lock
rc.execution_lock

# Submodules
rc.safety_manager
rc.motion_queue

# ROS2 node API
rc.get_logger()
rc.get_clock()

# Ephemeral planning state (worst offenders — stored on node between calls)
rc._pending_path_trajectory
rc._pending_path_vel_scaling
rc._pending_path_acc_scaling
rc._last_requested_delta_mm
rc._last_full_waypoints
```

### Concrete harms

1. **Ephemeral state on a long-lived object** — `_pending_path_*` and `_last_*` are per-planning-cycle values stored permanently on the node. A preempted move can corrupt the next one's state.

2. **Lock anti-pattern** — `trajectory_executor.py:137` does `if execution_lock.locked(): execution_lock.release()`. This is only necessary because the lock, its owner, and the release site live in different files with no shared context.

3. **Untestable** — None of these functions can be unit-tested without instantiating a full ROS2 node.

4. **Refactor fragility** — Renaming or restructuring any attribute on `RobotController` silently breaks N call sites across M files.

---

## Current State

The bridge is no longer in the original fully flat "god node passed everywhere" shape.
The following extractions are **done and deployed**:

- `MotionCoordinator` ✅
  - owns execution state, queue arbitration, active goal handles, and stop semantics
- `TrajectoryExecutor` ✅
  - owns controller-goal submission and queue progression after execution
- `RobotStateStore` ✅
  - owns current joint/cartesian monitor state
- `PlannerContext` ✅
  - owns planner-facing orchestration/state access; stores ephemeral planning state
    (`_pending_path_*`, `_last_*`) — these no longer live on `RobotController`
- `PlannerSupportService` ✅
  - owns cached FK and state-validity service clients
- `TrajectoryOptimizer` strategy seam ✅
  - planner no longer hardcodes TOTG vs Ruckig branching

`RobotController` is now a **composition root**: it constructs all of the above and exposes a
stable public API; it does not own ephemeral planning state.

Planning modules still receive a `PlannerContext` (not the full `RobotController`) as their
first argument. The remaining coupling is the `PlannerContext` dependency itself — not the
raw ROS node.

## Current Wiring

`RobotController` is now mainly a composition root for the motion stack:

```text
RobotController
  -> MotionCoordinator
  -> TrajectoryExecutor
  -> RobotStateStore
  -> PlannerSupportService
  -> PlannerContext
  -> TrajectoryOptimizer
```

Planning modules still receive a single first argument, but that object is now
intended to be a planner-facing context rather than the full ROS node.

## Proposed Direction From Here

The highest-value remaining cleanup is:

1. keep planner modules depending only on `PlannerContext` methods
2. avoid adding new cached state or service clients to `RobotController`
3. keep optimizer selection in the execution layer, not the planner layer

The large class-based rewrite proposed below is no longer the immediate next step.
It remains a possible future end-state, but most of the high-risk node-coupling
has already been removed through narrower extractions.

### Original long-term design sketch

Three changes in order of dependency:

### Change 1 — Extract `PlanningContext` dataclass

A lightweight, per-move object that carries ephemeral planning state down the call chain and is discarded on completion. Replaces the `rc._pending_*` and `rc._last_*` attributes.

```python
# motion/planning/planning_context.py
from dataclasses import dataclass, field
from typing import Optional, List

@dataclass
class PlanningContext:
    """
    Holds state for a single motion command lifecycle.
    Created at the start of each move, passed through the async chain,
    discarded when the move completes or is preempted.
    """
    generation: int                          # staleness token (matches RobotController.plan_generation)
    vel_scaling: float
    acc_scaling: float
    delta_mm: float = 0.0                    # Cartesian distance of the move (mm)
    full_waypoints: Optional[List] = None    # Original EE poses for Jacobian fallback
    pending_trajectory: object = None        # Pre-planned trajectory for _plan_then_approach
```

**Migration:** Remove `rc._pending_path_trajectory`, `rc._pending_path_vel_scaling`, `rc._pending_path_acc_scaling`, `rc._last_requested_delta_mm`, `rc._last_full_waypoints` from `RobotController`. All call sites that set/read these switch to `ctx.*`.

---

### Change 2 — Introduce `TrajectoryExecutor` class

Encapsulates the controller action client, execution lock, and result tracking. Replaces the free functions in `trajectory_executor.py`.

```python
# motion/execution/trajectory_executor.py
class TrajectoryExecutor:
    def __init__(self, node, controller_client, execution_lock, lock, motion_queue):
        self._node = node
        self._client = controller_client
        self._execution_lock = execution_lock
        self._lock = lock
        self._queue = motion_queue

    def send(self, joint_trajectory):
        """Send trajectory to hardware controller. Previously _send_trajectory_to_controller."""
        ...

    def _on_goal_response(self, future): ...
    def _on_goal_result(self, future): ...
    def _process_next_queued_task(self): ...
```

`RobotController.__init__` constructs it:
```python
self.executor = TrajectoryExecutor(
    node=self,
    controller_client=self.controller_client,
    execution_lock=self.execution_lock,
    lock=self.lock,
    motion_queue=self.motion_queue,
)
```

**Lock management** moves entirely inside `TrajectoryExecutor` — no more cross-file `.locked()` checks.

---

### Change 3 — Introduce `TrajectoryPlanner` class

Encapsulates the MoveIt service client, safety manager, and all planning logic. Replaces the free functions spread across `trajectory_planner.py`, `single_target.py`, `trajectory.py`, `planner_utils.py`, `jacobian_move.py`.

```python
# motion/planning/trajectory_planner.py
class TrajectoryPlanner:
    def __init__(self, node, cart_path_client, ipp_client, safety_manager, executor):
        self._node = node
        self._cart_path_client = cart_path_client
        self._ipp_client = ipp_client
        self._safety = safety_manager
        self._executor = executor   # TrajectoryExecutor, not the node

    def send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz,
                            vel_scale, acc_scale, tool_transform=None): ...

    def send_path_cartesian(self, waypoints_mm, rx, ry, rz,
                            vel_scaling, acc_scaling): ...

    # All _cartesian_path_response, _plan_then_approach, etc. become private methods
    def _on_cart_path_response(self, ctx, future): ...
    def _apply_time_param(self, ctx, trajectory): ...
    def _jacobian_fallback(self, ctx, poses): ...
```

`RobotController.__init__` constructs it after `TrajectoryExecutor`:
```python
self.planner = TrajectoryPlanner(
    node=self,
    cart_path_client=self.cart_path_client,
    ipp_client=self.ipp_client,
    safety_manager=self.safety_manager,
    executor=self.executor,
)
```

Then `RobotController.send_cartesian_goal` becomes a one-liner delegation:
```python
def send_cartesian_goal(self, x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale,
                        tool_transform=None):
    return self.planner.send_cartesian_goal(
        x_mm, y_mm, z_mm, rx, ry, rz, vel_scale, acc_scale, tool_transform)
```

---

## Phased Execution Plan

Each phase is independently deployable and testable.

### Phase 1 — Extract `PlanningContext` (low risk)

**Goal:** Remove ephemeral `_pending_*` / `_last_*` attributes from `RobotController`.

**Steps:**
1. Create `motion/planning/planning_context.py` with the `PlanningContext` dataclass.
2. In `single_target.py:_execute_single_point`, create a `PlanningContext` instance and pass it into `_dispatch_moveit` and `_execute_jacobian_move` instead of setting `rc._last_*`.
3. In `trajectory.py:on_plan_done`, create a `PlanningContext` and pass it into the deferred `_execute_pending_trajectory` call instead of setting `rc._pending_*`.
4. Update all downstream consumers (`_cartesian_path_response`, `_execute_pending_trajectory`, `_jacobian_fallback_move`) to read from `ctx.*` instead of `rc._*`.
5. Delete `_pending_path_trajectory`, `_pending_path_vel_scaling`, `_pending_path_acc_scaling`, `_last_requested_delta_mm`, `_last_full_waypoints` from `RobotController.__init__`.

**Validation:** Full move cycle (single point, multi-waypoint, approach-then-execute) must complete without errors. Run existing integration tests.

---

### Phase 2 — `TrajectoryExecutor` class (medium risk)

**Goal:** Encapsulate controller action client + execution lock.

**Steps:**
1. Create `TrajectoryExecutor` class in `trajectory_executor.py` with `__init__(node, controller_client, execution_lock, lock, motion_queue)`.
2. Move free functions `_send_trajectory_to_controller`, `_controller_goal_response`, `_controller_goal_result`, `_process_next_queued_task` into the class as methods, replacing `robot_controller.*` with `self.*`.
3. Replace `robot_controller.execution_lock.locked()` release pattern with `try/finally` inside the class.
4. In `RobotController.__init__`, construct `self.executor = TrajectoryExecutor(...)`.
5. In `trajectory_planner.py`, replace the `_send_trajectory_to_controller(rc, ...)` call with `rc.executor.send(...)`.
6. Delete the now-empty module-level free function wrappers.

**Validation:** Trajectory execution, cancellation (`stop_motion`), and queue draining must all work correctly.

---

### Phase 3 — `TrajectoryPlanner` class (high risk, do last)

**Goal:** Encapsulate all MoveIt planning logic; eliminate `rc` from planning call chain.

**Steps:**
1. Create `TrajectoryPlanner` class in `trajectory_planner.py` with `__init__(node, cart_path_client, ipp_client, safety_manager, executor)`.
2. Move all free functions from `single_target.py`, `trajectory.py`, `trajectory_planner.py`, `planner_utils.py`, `jacobian_move.py` into the class as private methods, replacing `robot_controller.*` with `self.*` (for clients/safety/executor) and `ctx.*` (for planning state, from Phase 1).
3. Expose only two public methods: `send_cartesian_goal(...)` and `send_path_cartesian(...)`.
4. In `RobotController.__init__`, construct `self.planner = TrajectoryPlanner(...)`.
5. Update `RobotController.send_cartesian_goal` to delegate to `self.planner`.
6. Remove now-unused imports from `robot_controller.py`.

**Validation:** Full end-to-end test: REST API → bridge → planner → executor → hardware.

---

## What Does NOT Change

- Public API of `RobotController` (callers like `MoveItRobotBackend`, `rest_server.py` unchanged)
- `config.py` (no constants move)
- `SafetyWallManager`, `RobotMonitor`, `MotionQueue` (untouched)
- C++ nodes / hardware interface (untouched)
- ROS2 topic/service/action names

---

## File Layout — Current (Phase 1 + 2 complete)

```
scripts/
  motion/
    planning/
      planner_context.py       ← PlannerContext: planner-facing facade + ephemeral state ✅
      planner_support_service.py ← PlannerSupportService: FK + validity clients ✅
      trajectory_planner.py    ← free functions (_cartesian_path_response, _apply_time_param…)
      planner_utils.py         ← shared helpers (_set_result, _begin_execution…)
      single_target.py         ← free functions (send_cartesian_goal, _execute_single_point…)
      trajectory.py            ← free functions (send_path_cartesian, _execute_path…)
      jacobian_move.py         ← free functions (_jacobian_fallback_move…)
      planner_diagnostics.py   ← diagnostic helpers
    execution/
      motion_coordinator.py    ← MotionCoordinator: execution state, stop semantics ✅
      trajectory_executor.py   ← TrajectoryExecutor class + backward-compat shims ✅
      trajectory_optimizer.py  ← ITrajectoryOptimizer strategy + TOTG/Ruckig impls ✅
      trajectory_optimization.py ← apply_ipp_totg / apply_ruckig_service (unchanged)
      motion_queue.py          ← unchanged
  status/
    robot_state_store.py       ← RobotStateStore: joint/Cartesian state cache ✅
  robot_controller.py          ← composition root; public API unchanged
```

---

## Open Questions for Review

1. **`node` still passed to classes** — `TrajectoryPlanner` and `TrajectoryExecutor` still receive `node` for `get_logger()` and `get_clock()`. Is that acceptable, or should we inject a `Logger` and `Clock` directly? (Stricter isolation vs. more constructor params.)

2. **`RobotMonitor` read access** — `joint_state_callback` in `RobotController` reads `self.monitor.get_latest_data()` directly. Should `RobotMonitor` be injected into `TrajectoryPlanner` too, or is that read path staying in `RobotController`?

3. **Phase 3 atomicity** — Phase 3 touches 6 files simultaneously. Should it be split further (e.g., single_target first, then trajectory, then jacobian)?

4. **`strategies.py`** — `SingleTargetStrategy` / `PathStrategy` currently call the free functions. After Phase 3 they'll need to call `rc.planner.*`. Worth reviewing whether the Strategy pattern still adds value at that point.
