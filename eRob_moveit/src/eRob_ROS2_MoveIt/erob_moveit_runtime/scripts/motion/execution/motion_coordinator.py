#!/usr/bin/env python3
"""Execution ownership and queue arbitration for bridge motion commands."""

from threading import Lock


class MotionCoordinator:
    """Owns execution state, active goal handles, queue arbitration, and stop semantics."""

    def __init__(self, node, motion_queue):
        self._node = node
        self._motion_queue = motion_queue
        self.lock = Lock()
        self.execution_lock = Lock()
        self.active_execute_send_future = None
        self.active_controller_goal = None
        self.is_executing = False
        self.plan_generation = 0
        self.last_move_result = 0
        # Human-readable reason for the most recent motion failure. This is
        # deliberately separate from the numeric controller result so callers
        # can report safety cancellations without guessing from the code.
        self.last_motion_error = None
        self.last_submitted_task_id = None

    def _set_last_submitted_task_id(self, task_id):
        """Keep coordinator and owning node task IDs in sync.

        MoveItRobotBackend waits for queued blocking moves through
        ``node.last_submitted_task_id``.  MotionCoordinator previously kept the
        task ID only on itself, causing a positive queue position (for example
        ``1``) to escape back through the REST API as a failure even though the
        queued trajectory subsequently executed successfully.
        """
        self.last_submitted_task_id = task_id
        try:
            self._node.last_submitted_task_id = task_id
        except Exception:
            pass

    def execute(self, strategy, queue_if_busy=True):
        queueable = bool(getattr(strategy, 'queueable', True))
        with self.lock:
            is_busy = bool(self.is_executing)

        if queue_if_busy and queueable and is_busy:
            result = self._motion_queue.submit(
                task_function=self._execute_strategy_task,
                task_args=[strategy],
            )
            if isinstance(result, tuple):
                task_id, position = result
                self._set_last_submitted_task_id(task_id)
                self._node.get_logger().info(
                    f'[Queue] Queued {strategy.__class__.__name__} at position {position} (task #{task_id})')
                return position
            self._node.get_logger().error(
                f'[Queue] Failed to queue {strategy.__class__.__name__}: {result}')
            return result

        if is_busy:
            self._node.get_logger().warning(
                f'[Queue] Busy — rejecting non-queueable {strategy.__class__.__name__}')
            return -1

        task_id = self._motion_queue.allocate_task_id()
        self._set_last_submitted_task_id(task_id)
        self._motion_queue.start_immediate_task(task_id)
        result = strategy.execute(self._node)
        if result != 0:
            self._motion_queue.mark_current_complete(result)
        return result

    def stop_motion(self):
        self._node.get_logger().info('[STOP] Stopping motion called')
        stopped = False
        queue_cleared = 0
        controller_cancel_error = None

        future = self.active_execute_send_future
        if future is not None:
            self._node.get_logger().info('[STOP] Cancelling pending trajectory submission...')
            try:
                future.cancel()
                self.active_execute_send_future = None
                stopped = True
                self._node.get_logger().info('[STOP] ✓ Pending submission cancelled')
            except Exception as e:
                controller_cancel_error = str(e)
                self._node.get_logger().error(f'[STOP] Failed to cancel send future: {e}')

        if self.active_controller_goal is not None:
            send_path_stop = getattr(
                getattr(self._node, 'trajectory_executor', None),
                'send_path_stop_trajectory',
                None,
            )
            if callable(send_path_stop):
                try:
                    if send_path_stop():
                        with self.lock:
                            self.plan_generation += 1
                            self.last_move_result = -1
                        queue_cleared = self._motion_queue.clear()
                        if queue_cleared > 0:
                            self._node.get_logger().warning(f'[STOP] Cleared {queue_cleared} queued motions')
                        self._node.get_logger().warning('[STOP] Path stop trajectory submitted; queue cleared')
                        return {
                            'state': 'STOPPING',
                            'result': 0,
                            'success': True,
                            'stopped': True,
                            'queue_cleared': queue_cleared,
                        }
                except Exception as e:
                    controller_cancel_error = str(e)
                    self._node.get_logger().error(f'[STOP] Failed to submit path stop trajectory: {e}')

            self._node.get_logger().info('[STOP] Cancelling active controller trajectory...')
            try:
                self.active_controller_goal.cancel_goal_async()
                self.active_controller_goal = None
                stopped = True
                self._node.get_logger().warning('[STOP] ✓ Robot motion cancelled!')
            except Exception as e:
                controller_cancel_error = str(e)
                self._node.get_logger().error(f'[STOP] Failed to cancel controller goal: {e}')

        with self.lock:
            was_executing = bool(self.is_executing)
            if stopped or was_executing:
                self.plan_generation += 1
            self.is_executing = False
            self.last_move_result = -1

        if stopped:
            self._motion_queue.mark_current_complete(-1)

        if self.execution_lock.locked():
            self.execution_lock.release()

        queue_cleared = self._motion_queue.clear()
        if queue_cleared > 0:
            self._node.get_logger().warning(f'[STOP] Cleared {queue_cleared} queued motions')

        if stopped or queue_cleared > 0:
            self._node.get_logger().warning('[STOP] 🛑 Robot motion stopped and queue cleared')
            return {
                'state': 'STOPPED',
                'result': 0,
                'success': True,
                'stopped': True,
                'queue_cleared': queue_cleared,
            }

        if was_executing:
            self._node.get_logger().warning(
                '[STOP] Stop requested while robot was executing, but no cancel handle was available; '
                'invalidated in-flight plan generation')
            return {
                'state': 'STOP_REQUESTED_BUT_UNCONFIRMED',
                'result': 1,
                'success': False,
                'stopped': False,
                'queue_cleared': queue_cleared,
                'error': controller_cancel_error or 'robot executing but no cancellable goal handle was available',
            }

        if controller_cancel_error is not None:
            self._node.get_logger().error(
                f'[STOP] Stop failed: {controller_cancel_error}'
            )
            return {
                'state': 'ERROR',
                'result': -2,
                'success': False,
                'stopped': False,
                'queue_cleared': queue_cleared,
                'error': controller_cancel_error,
            }

        self._node.get_logger().info('[STOP] No active motion to cancel')
        return {
            'state': 'NO_ACTIVE_MOTION',
            'result': -1,
            'success': True,
            'stopped': False,
            'queue_cleared': queue_cleared,
        }

    def controlled_stop(self, expected_task_id, *, stop_duration_s=None):
        """Stop only the expected active task and preserve future work."""
        status = self._motion_queue.get_status()
        current_task_id = status.get('current_task_id')
        if expected_task_id is None or str(current_task_id) != str(expected_task_id):
            return {
                'state': 'TASK_MISMATCH', 'result': -3, 'success': False,
                'stopped': False, 'expected_task_id': expected_task_id,
                'current_task_id': current_task_id,
                'error': 'active task does not match expected_task_id',
            }
        if self.active_controller_goal is None:
            return {
                'state': 'NO_CONTROLLER_GOAL', 'result': -1, 'success': False,
                'stopped': False, 'expected_task_id': expected_task_id,
                'current_task_id': current_task_id,
                'error': 'active task has no cancellable controller goal',
            }
        sender = getattr(
            getattr(self._node, 'trajectory_executor', None),
            'send_path_stop_trajectory',
            None,
        )
        if not callable(sender) or not sender(
            preserve_future_work=True,
            stop_duration_s=stop_duration_s,
        ):
            return {
                'state': 'STOP_TRAJECTORY_FAILED', 'result': -2, 'success': False,
                'stopped': False, 'expected_task_id': expected_task_id,
                'current_task_id': current_task_id,
                'error': 'controlled path stop trajectory could not be submitted',
            }
        self._node.get_logger().warning(
            f'[CONTROLLED_STOP] Stopping task #{current_task_id}; preserving future work'
        )
        return {
            'state': 'STOPPING', 'result': 0, 'success': True,
            'stopped': True, 'expected_task_id': expected_task_id,
            'current_task_id': current_task_id, 'future_work_preserved': True,
        }

    def is_motion_active(self):
        return any([
            self.active_controller_goal is not None,
            self.active_execute_send_future is not None,
        ])

    def has_pending_motion(self):
        return self._motion_queue.get_status()['queue_size'] > 0

    def _execute_strategy_task(self, strategy):
        return strategy.execute(self._node)
