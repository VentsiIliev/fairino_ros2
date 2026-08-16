"""Generic synchronized multi-entity trajectory executor.

This is the single-robot-generic, multi-entity helper behind the twin
choreography coordination layer. It submits N already-prepared controller goals
against ONE common future timestamp so every robot starts on the same
controller clock, and it enforces an acceptance barrier: if any goal is
explicitly rejected before the common start stamp, the accepted partners are
cancelled and the call raises instead of letting a half-synced pair run.

Pipeline (all multi-entity steps run concurrently):

    1. re-prepare every PreparedTrajectory into its final controller goal
       (start policy verification, mutation, tolerances, goal construction)
    2. run every pre-send execution gate per entity — drive readiness,
       execution-lock acquisition, controller action-server readiness
    3. only then choose the single common future start stamp
    4. submit every goal concurrently with that same stamp
    5. acceptance barrier (authoritative, race-free acceptance state)
    6. wait for completion

The common stamp is deliberately chosen only after preparation AND gates, so
the configured start delay only has to cover dispatch + controller acceptance
— never per-entity serialized work.

No twin-specific logic lives here; ``TwinLocalRuntime`` wires the per-robot
executors and prepared trajectories into it. Planners and executors remain
single-robot generic.
"""

import threading
import time

from copy import deepcopy

try:
    import config
except Exception:
    config = None


def _default_config_value(name, default):
    try:
        value = getattr(config, name, None)
        if value is None:
            return default
        return value
    except Exception:
        return default


class SynchronizedTrajectoryExecutor:
    """Coordinate N prepared trajectories against a shared start timestamp."""

    def __init__(self, clock):
        """``clock`` is the shared rclpy Clock (all robots run in one process)."""
        self._clock = clock

    def common_start_time(self, start_time=None, offset_s=None):
        """Return the shared start stamp.

        If ``start_time`` is given it is used verbatim. Otherwise a single
        future stamp is computed once as ``now + offset_s`` (default from
        ``SYNCHRONIZED_EXECUTION_START_DELAY_S``). Computing it once here (and
        passing the same value to every entity) guarantees a common timeline.
        """
        if start_time is not None:
            return start_time
        offset_s = float(
            offset_s
            if offset_s is not None
            else _default_config_value('SYNCHRONIZED_EXECUTION_START_DELAY_S', 0.5)
        )
        from rclpy.duration import Duration
        return (self._clock.now() + Duration(seconds=offset_s)).to_msg()

    def execute(
        self,
        items,
        *,
        start_time=None,
        offset_s=None,
        completion_timeout_s=None,
    ):
        """Execute a set of prepared trajectories against one common stamp.

        Ordering matters: every prepared trajectory is FIRST converted into its
        final controller goal (start_policy verification, start-ramp insertion,
        trajectory mutation, tolerances, goal construction) and preparation
        fails the whole call before anything is dispatched. The single common
        start stamp is computed only AFTER all controller goals exist, so the
        configured start delay only has to cover dispatch + controller
        acceptance — never goal preparation. Every goal is then dispatched with
        that same stamp applied, and an acceptance barrier requires all goals
        to be accepted strictly before ``stamp - margin`` or the accepted
        partners are cancelled and the call raises.

        Args:
            items: iterable of ``(trajectory_executor, prepared, name)`` tuples.
                Each ``prepared`` is a :class:`PreparedTrajectory`. The helper
                calls ``prepare_controller_goal``, then ``ready_for_dispatch``,
                then ``send_prepared_goal`` on each executor directly
                (coordinated operation, not queued).
            start_time: optional shared ``builtin_interfaces/Time``; if None a
                common future stamp is computed from ``offset_s`` once all goals
                are prepared.
            offset_s: seconds into the future for the common stamp (only used
                when ``start_time`` is None).
            completion_timeout_s: how long to wait for every goal to complete.

        Returns:
            dict: ``{name: result_code}`` plus a ``_dispatch_separation_s``
            metric (max-min wall-clock dispatch instants across entities).

        Raises:
            RuntimeError: if preparation or any goal dispatch fails, or if the
            acceptance barrier is not fully reached before the common start
            stamp (accepted peers are cancelled first).
        """
        completion_timeout_s = float(
            completion_timeout_s
            if completion_timeout_s is not None
            else _default_config_value('BLOCKING_MOVE_TIMEOUT_S', 60.0)
        )

        entries = list(items)
        if not entries:
            return {}

        names = [name for _, _, name in entries]

        # 1) Re-prepare every PreparedTrajectory into its final controller goal
        # FIRST. Everything expensive that does not depend on the common start
        # stamp happens here: start_policy verification, start-ramp insertion /
        # trajectory mutation, tolerances, FollowJointTrajectory goal
        # construction. Robots are prepared concurrently (separate executors /
        # nodes, no shared mutable state). If ANY robot fails, abort before
        # dispatching ANY goal.
        prepared_by_name = {}
        prepare_errors = {}
        prep_lock = threading.Lock()

        def _prepare_one(executor, prepared, name):
            try:
                metadata = prepared.metadata or {}
                goal = executor.prepare_controller_goal(
                    deepcopy(prepared.trajectory),
                    preserve_explicit_wrap=bool(metadata.get('preserve_explicit_wrap', False)),
                    unwind_check=metadata.get('unwind_check'),
                    suppress_drive_disable_cancel=bool(
                        metadata.get('suppress_drive_disable_cancel', False)
                    ),
                    start_policy=metadata.get('start_policy', 'live_anchor'),
                )
            except Exception as exc:
                with prep_lock:
                    prepare_errors[name] = exc
                return
            if goal is None:
                with prep_lock:
                    prepare_errors[name] = RuntimeError(
                        f'prepare_controller_goal returned None '
                        f'(result={executor._motion.last_move_result})'
                    )
                return
            with prep_lock:
                prepared_by_name[name] = goal

        threads = []
        for executor, prepared, name in entries:
            if getattr(prepared, 'noop', False):
                prepared_by_name[name] = None
                continue
            threads.append(
                threading.Thread(
                    target=_prepare_one,
                    args=(executor, prepared, name),
                    daemon=False,
                )
            )
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if prepare_errors:
            raise RuntimeError(f'[SYNC_EXEC] prepare failed: {prepare_errors}')
        self._log(entries, '[TWIN_SYNC] all controller goals re-prepared')

        # 2) Run every pre-send execution gate CONCURRENTLY — drive readiness,
        # execution-lock acquisition, controller action-server readiness, and
        # active-trajectory state setup. None of these depend on the common
        # stamp, so completing them here (rather than inside the dispatch
        # window) removes the artificial per-entity serialization. If ANY
        # entity fails to become ready, the others' gates are released and the
        # call aborts before a stamp is chosen or anything is dispatched.
        executor_by_name = {name: executor for executor, _, name in entries}
        ready_names = set()
        ready_errors = {}
        ready_lock = threading.Lock()

        def _ready_one(executor, prepared, name):
            try:
                ok = executor.ready_for_dispatch(prepared)
            except Exception as exc:
                with ready_lock:
                    ready_errors[name] = exc
                return
            if not ok:
                with ready_lock:
                    ready_errors[name] = RuntimeError(
                        f'ready_for_dispatch failed (result={executor._motion.last_move_result})'
                    )
                return
            with ready_lock:
                ready_names.add(name)

        threads = []
        for executor, _, name in entries:
            goal = prepared_by_name.get(name)
            if goal is None:
                continue
            threads.append(
                threading.Thread(
                    target=_ready_one,
                    args=(executor, goal, name),
                    daemon=False,
                )
            )
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if ready_errors:
            self._release_gated_but_unsent(executor_by_name, ready_names, set())
            raise RuntimeError(f'[SYNC_EXEC] dispatch-ready failed: {ready_errors}')
        self._log(entries, '[TWIN_SYNC] all entities ready for dispatch')

        # 3) ONLY after every controller goal AND every pre-send gate is ready,
        # compute the single common future start stamp. The remaining
        # timing-critical window covers only stamp assignment + action-goal
        # submission, so the configured start delay stays meaningful regardless
        # of preparation or gate cost.
        offset_used_s = float(
            offset_s
            if offset_s is not None
            else _default_config_value('SYNCHRONIZED_EXECUTION_START_DELAY_S', 0.5)
        )
        start = self.common_start_time(start_time=start_time, offset_s=offset_s)
        start_epoch = float(start.sec) + float(start.nanosec) / 1e9
        self._log(
            entries,
            f'[TWIN_SYNC] common start stamp: sec={start.sec} nanosec={start.nanosec} '
            f'robots={", ".join(names)} offset_s={offset_used_s:.3f}',
        )

        # 4) Dispatch every goal CONCURRENTLY, each with the SAME common stamp
        # applied at send time. The only per-entity work left is header-stamp
        # assignment and send_goal_async, so dispatch separation collapses to
        # thread-scheduling noise.
        sent = {}
        send_errors = {}
        dispatch_instants = {}
        send_lock = threading.Lock()

        def _send_one(executor, prepared, name):
            dispatched_at = time.perf_counter()
            try:
                executor.send_prepared_goal(prepared, start_time=start)
            except Exception as exc:
                with send_lock:
                    send_errors[name] = exc
                return
            with send_lock:
                dispatch_instants[name] = dispatched_at
                sent[name] = executor

        threads = []
        for executor, _, name in entries:
            goal = prepared_by_name.get(name)
            if goal is None:
                continue
            threads.append(
                threading.Thread(
                    target=_send_one,
                    args=(executor, goal, name),
                    daemon=False,
                )
            )
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if send_errors:
            self._release_gated_but_unsent(executor_by_name, ready_names, sent)
            self._cancel_all(sent)
            raise RuntimeError(f'[SYNC_EXEC] dispatch failed: {send_errors}')
        self._log(
            entries,
            '[TWIN_SYNC] all goals dispatched against common stamp',
        )

        # 5) Acceptance barrier: every goal must be accepted strictly before
        #    common_start_stamp - margin. A rejected partner cancels accepted
        #    peers so no half-synchronized execution can run. Acceptance state
        #    is re-read authoritatively after the deadline (plus a short grace)
        #    so a goal whose response arrived around the deadline is not
        #    misreported as a deadline expiration.
        accept_deadline = self._acceptance_deadline_before_start(start)
        acceptance_instants = {}
        barrier_ok, rejected = self._wait_for_acceptance(
            sent,
            prepared_by_name,
            deadline=accept_deadline,
            acceptance_instants=acceptance_instants,
        )
        for name in sorted(sent):
            dispatched_at = dispatch_instants.get(name)
            accepted_at = acceptance_instants.get(name)
            if dispatched_at is not None and accepted_at is not None:
                accepted_epoch = time.time() + (accepted_at - time.perf_counter())
                margin_s = start_epoch - accepted_epoch
                self._log(
                    entries,
                    f'[TWIN_SYNC] {name} dispatch→acceptance latency: '
                    f'{(accepted_at - dispatched_at) * 1000.0:.1f} ms, '
                    f'acceptance margin before start: {margin_s * 1000.0:.0f} ms',
                )
        if not barrier_ok:
            self._log(entries, f'[TWIN_SYNC] acceptance barrier failed: {rejected}')
            self._cancel_after_barrier_failure(sent, rejected)
            raise RuntimeError(f'[SYNC_EXEC] acceptance barrier failed: {rejected}')
        self._log(entries, '[TWIN_SYNC] acceptance barrier reached — synchronized execution active')

        results = self._wait_for_completion(
            sent, prepared_by_name, completion_timeout_s=completion_timeout_s
        )

        dispatch_values = list(dispatch_instants.values())
        separation = 0.0
        if len(dispatch_values) > 1:
            separation = max(dispatch_values) - min(dispatch_values)
        accept_values = list(acceptance_instants.values())
        accept_separation = 0.0
        if len(accept_values) > 1:
            accept_separation = max(accept_values) - min(accept_values)
        results['_dispatch_separation_s'] = separation
        results['_acceptance_separation_s'] = accept_separation
        results['_stamp_offset_used_s'] = offset_used_s
        self._log(
            entries,
            f'[TWIN_SYNC] synchronized execution complete '
            f'(dispatch separation {separation * 1000.0:.3f} ms, '
            f'acceptance separation {accept_separation * 1000.0:.3f} ms)',
        )
        return results

    def _acceptance_deadline_before_start(self, start):
        """Wall-clock deadline by which every goal must be accepted.

        Derived from the common start stamp: acceptance must complete strictly
        BEFORE the stamp, leaving a safety margin
        (``SYNCHRONIZED_EXECUTION_ACCEPT_MARGIN_S``). The remaining time before
        the stamp is measured on the same ROS clock used for the stamp, then
        translated to wall-clock time, so a goal that is accepted in time for a
        synchronized start can never begin moving before its peer's failure is
        detected.
        """
        margin_s = float(
            _default_config_value('SYNCHRONIZED_EXECUTION_ACCEPT_MARGIN_S', 0.15)
        )
        start_epoch = float(start.sec) + float(start.nanosec) / 1e9
        now_epoch = self._clock.now().nanoseconds / 1e9
        remaining_s = start_epoch - now_epoch
        return time.time() + max(0.0, remaining_s - margin_s)

    @staticmethod
    def _log(entries, message):
        """Log a synchronized-execution event once through the first entry's node."""
        if not entries:
            return
        try:
            entries[0][0]._node.get_logger().info(message)
        except Exception:
            pass


    def _wait_for_acceptance(self, sent, prepared_by_name, *, deadline,
                             acceptance_instants=None):
        """Wait until every sent goal is accepted (or explicitly rejected).

        ``deadline`` is the wall-clock time by which acceptance must be
        complete; it is derived from the common start stamp minus a safety
        margin so the barrier always finishes before any robot starts moving.
        Returns ``(barrier_ok, rejected)``. A goal counts as rejected when the
        executor never recorded goal acceptance (``_goal_acceptance_seen``) and
        is now idle. A goal whose acceptance was observed is never treated as
        rejected here, even if it completed before the acceptance poll.

        Acceptance state is made authoritative and race-free in three layers:

        1. Polling loop up to ``deadline`` for the marker.
        2. An authoritative final sweep that re-reads acceptance/rejection
           state for anything still pending, so a goal whose response arrived
           around the deadline is never misreported.
        3. A short bounded grace re-check (``SYNCHRONIZED_EXECUTION_ACCEPT_GRACE_S``)
           because the goal response is delivered by each entity's node spin
           thread, which can record acceptance a few ms after the poll loop
           observed the deadline.

        Goals that still show neither acceptance nor rejection after the grace
        window are reported as a deadline expiration; goals whose explicit
        rejection was observed are reported with the rejection result code.
        ``acceptance_instants`` (optional dict) is filled with
        ``{name: perf_counter}`` at the moment each acceptance is observed.
        """
        accepted = set()
        rejected = {}
        pending = set(prepared_by_name.keys()) - {n for n, g in prepared_by_name.items() if g is None}

        def _observe():
            for name in list(pending):
                executor = sent.get(name)
                if executor is None:
                    pending.discard(name)
                    continue
                motion = executor._motion
                if getattr(executor, "_goal_acceptance_seen", False) or \
                        motion.active_controller_goal is not None:
                    self._log([(executor, None, name)], f"[TWIN_SYNC] {name} accepted")
                    accepted.add(name)
                    if acceptance_instants is not None:
                        acceptance_instants[name] = time.perf_counter()
                    pending.discard(name)
                    continue
                if getattr(executor, "_goal_rejection_seen", False):
                    rejected[name] = int(motion.last_move_result)
                    pending.discard(name)
                    continue
                if not motion.is_executing:
                    rejected[name] = int(motion.last_move_result)
                    pending.discard(name)

        while pending and time.time() < deadline:
            _observe()
            if pending:
                time.sleep(0.01)

        if pending:
            _observe()

        grace_s = float(
            _default_config_value('SYNCHRONIZED_EXECUTION_ACCEPT_GRACE_S', 0.10)
        )
        grace_deadline = time.time() + grace_s
        while pending and time.time() < grace_deadline:
            _observe()
            if pending:
                time.sleep(0.005)

        if pending:
            for name in pending:
                rejected[name] = "acceptance deadline (before common start stamp)"
        if rejected:
            return False, rejected
        return True, {}

    def _wait_for_completion(self, sent, prepared_by_name, *, completion_timeout_s):
        """Block until every sent goal completes; returns ``{name: result}``."""
        results = {}
        for name, executor in sent.items():
            results[name] = self._wait_for_one(executor, completion_timeout_s)
        for name, goal in prepared_by_name.items():
            if goal is None:
                results[name] = 0
        return results

    @staticmethod
    def _wait_for_one(executor, completion_timeout_s):
        deadline = time.time() + completion_timeout_s
        motion = executor._motion
        while time.time() < deadline:
            if not motion.is_executing:
                return int(motion.last_move_result)
            time.sleep(0.02)
        return int(getattr(config, 'MOTION_ERROR_CONTROLLER_EXECUTION_FAILED', -14))

    @staticmethod
    def _cancel_all(sent):
        """Cancel every submitted goal that is still pending/accepted."""
        for name, executor in sent.items():
            try:
                executor._motion.stop_motion()
            except Exception as exc:
                try:
                    executor._node.get_logger().error(
                        f'[SYNC_EXEC] Failed to cancel {name}: {exc}'
                    )
                except Exception:
                    pass

    def _release_gated_but_unsent(self, executor_by_name, ready_names, sent):
        """Release execution gates for entities that became ready but never sent."""
        for name in ready_names:
            if name in sent:
                continue
            executor = executor_by_name.get(name)
            if executor is None:
                continue
            try:
                executor.clear_execution_gates()
            except Exception as exc:
                try:
                    executor._node.get_logger().error(
                        f'[SYNC_EXEC] Failed to release gates for {name}: {exc}'
                    )
                except Exception:
                    pass

    def _cancel_after_barrier_failure(self, sent, rejected):
        """Cancel the entities that must not continue after a barrier failure.

        An explicit controller rejection always cancels every entity: the
        rejected one is idle and accepted partners must not run alone
        (genuine partial-acceptance safety). A deadline-only failure (no
        explicit rejection) cancels only entities whose acceptance was never
        observed — an entity whose goal was accepted (even if the marker was
        recorded around/after the deadline) is already moving and is left to
        complete rather than being hit with a disruptive replacement stop.
        """
        explicit_rejection = any(
            not isinstance(reason, str) for reason in rejected.values()
        )
        for name, executor in sent.items():
            accepted = bool(getattr(executor, "_goal_acceptance_seen", False)) or \
                executor._motion.active_controller_goal is not None
            if not explicit_rejection and accepted:
                continue
            try:
                executor._motion.stop_motion()
            except Exception as exc:
                try:
                    executor._node.get_logger().error(
                        f'[SYNC_EXEC] Failed to cancel {name}: {exc}'
                    )
                except Exception:
                    pass


def _synchronized_executor(node):
    """Build a SynchronizedTrajectoryExecutor on the given node's clock."""
    return SynchronizedTrajectoryExecutor(node.get_clock())
