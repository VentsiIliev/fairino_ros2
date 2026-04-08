from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from copy import deepcopy
import config


class TrajectoryExecutor:
    """Owns controller goal submission and queue draining for timed joint trajectories."""

    def __init__(self, node, coordinator, motion_queue, controller_client):
        self._node = node
        self._motion = coordinator
        self._queue = motion_queue
        self._controller_client = controller_client
        self._last_sent_trajectory = None
        action_name = getattr(config, 'ACTION_FOLLOW_TRAJECTORY', '') or ''
        self._controller_name = action_name.rsplit('/', 1)[0].strip('/') or 'joint_trajectory_controller'

    def _overwrite_first_point_with_live_state(self, joint_trajectory):
        """Anchor the first controller point to the latest measured joint state."""
        if not joint_trajectory.points:
            return

        current_joint_state = getattr(self._node, 'current_joint_state', None)
        if current_joint_state is None:
            return

        state_names = list(getattr(current_joint_state, 'name', []) or [])
        state_positions = list(getattr(current_joint_state, 'position', []) or [])
        if not state_names or len(state_names) != len(state_positions):
            return

        position_by_name = {
            name: position
            for name, position in zip(state_names, state_positions)
        }
        ordered_positions = []
        for joint_name in joint_trajectory.joint_names:
            if joint_name not in position_by_name:
                return
            ordered_positions.append(position_by_name[joint_name])

        first_point = joint_trajectory.points[0]
        first_point.positions = ordered_positions

        first_point.velocities = [0.0] * len(ordered_positions)
        first_point.accelerations = [0.0] * len(ordered_positions)

    @staticmethod
    def _duration_to_sec(duration_msg):
        return duration_msg.sec + duration_msg.nanosec / 1e9

    @staticmethod
    def _sec_to_duration(seconds):
        whole = int(seconds)
        nanos = int(round((seconds - whole) * 1e9))
        if nanos >= 1_000_000_000:
            whole += 1
            nanos -= 1_000_000_000
        return Duration(sec=whole, nanosec=nanos)

    def _log_final_trajectory_segment(self, joint_trajectory, count=5):
        if not joint_trajectory or not joint_trajectory.points:
            return
        tail = joint_trajectory.points[-count:]
        self._node.get_logger().info(
            f'[Controller] Final {len(tail)} commanded points:'
        )
        start_index = len(joint_trajectory.points) - len(tail)
        for offset, point in enumerate(tail):
            index = start_index + offset
            timestamp = self._duration_to_sec(point.time_from_start)
            self._node.get_logger().info(
                f'[Controller]   Point[{index}] t={timestamp:.3f}s pos='
                f'{[round(p, 6) for p in point.positions]}'
            )

    def _get_latest_joint_state_in_trajectory_order(self, joint_names):
        current_joint_state = getattr(self._node, 'current_joint_state', None)
        if current_joint_state is None:
            return None

        state_names = list(getattr(current_joint_state, 'name', []) or [])
        state_positions = list(getattr(current_joint_state, 'position', []) or [])
        if not state_names or len(state_names) != len(state_positions):
            return None

        position_by_name = {
            name: position
            for name, position in zip(state_names, state_positions)
        }
        ordered_positions = []
        for joint_name in joint_names:
            if joint_name not in position_by_name:
                return None
            ordered_positions.append(position_by_name[joint_name])
        return ordered_positions

    def _log_final_tracking_error(self):
        joint_trajectory = self._last_sent_trajectory
        if joint_trajectory is None or not joint_trajectory.points:
            return

        expected = list(joint_trajectory.points[-1].positions)
        actual = self._get_latest_joint_state_in_trajectory_order(joint_trajectory.joint_names)
        if actual is None or len(actual) != len(expected):
            self._node.get_logger().warning(
                '[Controller] Final joint-state diagnostics unavailable'
            )
            return

        errors = [actual_i - expected_i for actual_i, expected_i in zip(actual, expected)]
        self._node.get_logger().error(
            f'[Controller] Actual final joint state: {[round(v, 6) for v in actual]}'
        )
        self._node.get_logger().error(
            f'[Controller] Expected final joint state: {[round(v, 6) for v in expected]}'
        )
        self._node.get_logger().error(
            f'[Controller] Final joint error (actual - expected): {[round(v, 6) for v in errors]}'
        )

    def _soften_trajectory_start(self, joint_trajectory):
        """Insert a short ramp-in sequence after the live start state."""
        if len(joint_trajectory.points) < 2:
            return

        hold_s = float(getattr(config, 'EXECUTOR_START_HOLD_S', 0.0))
        ramp_points = int(getattr(config, 'EXECUTOR_START_RAMP_POINTS', 0))
        if hold_s <= 0.0:
            return

        first_point = joint_trajectory.points[0]
        second_point = joint_trajectory.points[1]
        second_time = self._duration_to_sec(second_point.time_from_start)

        # Always create a small ramp-in window, but don't over-inflate extremely short moves.
        effective_hold_s = min(hold_s, max(0.0, second_time * 0.8))
        if effective_hold_s <= 1e-6:
            return

        insert_count = max(1, ramp_points)
        ramp_points_to_insert = []
        for idx in range(insert_count):
            ratio = (idx + 1) / insert_count
            inserted_point = deepcopy(first_point)
            inserted_point.time_from_start = self._sec_to_duration(effective_hold_s * ratio)

            if idx < insert_count - 1:
                position_ratio = 0.5 * ratio
                inserted_point.positions = [
                    start + (target - start) * position_ratio
                    for start, target in zip(first_point.positions, second_point.positions)
                ]
                inserted_point.velocities = [0.0] * len(first_point.positions)
                inserted_point.accelerations = [0.0] * len(first_point.positions)
            else:
                inserted_point.positions = list(first_point.positions)
                inserted_point.velocities = [0.0] * len(first_point.positions)
                inserted_point.accelerations = [0.0] * len(first_point.positions)

            ramp_points_to_insert.append(inserted_point)

        for idx in range(1, len(joint_trajectory.points)):
            shifted = self._duration_to_sec(joint_trajectory.points[idx].time_from_start) + effective_hold_s
            joint_trajectory.points[idx].time_from_start = self._sec_to_duration(shifted)

        for offset, inserted_point in enumerate(ramp_points_to_insert, start=1):
            joint_trajectory.points.insert(offset, inserted_point)
        self._node.get_logger().info(
            f'[Controller] Inserted start ramp ({insert_count} points over {effective_hold_s:.3f}s) '
            'to soften motion onset'
        )

    def send_trajectory_to_controller(self, joint_trajectory):
        """Send trajectory directly to the low-level controller for smooth execution."""
        if not self._motion.execution_lock.acquire(blocking=False):
            self._node.get_logger().warning('[Controller] Trajectory already executing, ignoring')
            return

        if not self._controller_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().error(
                f'[Controller] {self._controller_name} not available'
            )
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()
            return

        if len(joint_trajectory.points) == 0:
            self._node.get_logger().error('[Controller] ✗ Empty trajectory - aborting')
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()
            return

        has_valid_timestamps = any(
            pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0
            for pt in joint_trajectory.points
        )
        if not has_valid_timestamps:
            self._node.get_logger().error('[Controller] ✗ Trajectory has NO timestamps - aborting')
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()
            return

        self._overwrite_first_point_with_live_state(joint_trajectory)
        self._soften_trajectory_start(joint_trajectory)
        log_drive_state = getattr(self._node, 'log_drive_state_before_first_motion', None)
        if callable(log_drive_state):
            log_drive_state()

        with self._motion.lock:
            self._motion.is_executing = True

        goal_tolerance = []
        for name in joint_trajectory.joint_names:
            tol = JointTolerance()
            tol.name = name
            tol.position = config.EXECUTOR_GOAL_POS_TOL_RAD
            tol.velocity = 0.0
            tol.acceleration = 0.0
            goal_tolerance.append(tol)

        controller_goal = FollowJointTrajectory.Goal()
        controller_goal.trajectory = joint_trajectory
        controller_goal.path_tolerance = []
        controller_goal.goal_tolerance = goal_tolerance
        self._last_sent_trajectory = deepcopy(joint_trajectory)

        if len(joint_trajectory.points) > 0:
            last_point = joint_trajectory.points[-1]
            traj_duration_sec = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            time_tolerance_sec = max(config.EXECUTOR_TIME_MIN_S, traj_duration_sec * config.EXECUTOR_TIME_MULTIPLIER)

            self._node.get_logger().info(
                f'[Controller] Trajectory duration: {traj_duration_sec:.2f}s, timeout: {time_tolerance_sec:.1f}s')
            self._node.get_logger().info(
                f'[Controller] First point positions: {[round(p, 6) for p in joint_trajectory.points[0].positions]}')
            self._node.get_logger().info(
                f'[Controller] Last point positions: {[round(p, 6) for p in last_point.positions]}')
            self._node.get_logger().info(
                f'[Controller] First point time: {joint_trajectory.points[0].time_from_start.sec + joint_trajectory.points[0].time_from_start.nanosec / 1e9:.3f}s')
            self._node.get_logger().info(f'[Controller] Last point time: {traj_duration_sec:.3f}s')
            self._log_final_trajectory_segment(joint_trajectory)
        else:
            time_tolerance_sec = config.EXECUTOR_TIME_MIN_S

        controller_goal.goal_time_tolerance.sec = int(time_tolerance_sec)
        controller_goal.goal_time_tolerance.nanosec = int((time_tolerance_sec % 1.0) * 1e9)

        self._node.get_logger().info(
            f'[Controller] Sending {len(joint_trajectory.points)} points directly to controller '
            f'(spline interpolation, no path tolerance, goal_time_tolerance={time_tolerance_sec:.1f}s)')

        future = self._controller_client.send_goal_async(controller_goal)
        self._motion.active_execute_send_future = future
        future.add_done_callback(self._on_controller_goal_response)

    def process_next_queued_task(self):
        task = self._queue.get_next_task()
        if task is not None:
            self._node.get_logger().info(f'[Queue] Executing queued task #{task["id"]}')
            try:
                result = task['function'](*task['args'], **task['kwargs'])
                if result != 0:
                    self._queue.mark_current_complete(result)
                    self.process_next_queued_task()
            except Exception as e:
                self._node.get_logger().error(f'[Queue] Failed to execute queued task: {e}')
                with self._motion.lock:
                    self._motion.is_executing = False
                self._queue.mark_current_complete(-1)
                self.process_next_queued_task()
        else:
            self._node.get_logger().info('[Queue] No more queued tasks')

    def _on_controller_goal_response(self, future):
        self._motion.active_execute_send_future = None
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._node.get_logger().error(
                    f'[Controller] Trajectory execution rejected by {self._controller_name}'
                )
                self._motion.active_controller_goal = None
                with self._motion.lock:
                    self._motion.is_executing = False
                self._motion.execution_lock.release()
                return

            self._node.get_logger().info(
                f'[Controller] Trajectory accepted by {self._controller_name}'
            )
            self._motion.active_controller_goal = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_controller_goal_result)
        except Exception as e:
            self._node.get_logger().error(f'[Controller] Goal response error: {e}')
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()

    def _on_controller_goal_result(self, future):
        cancelled_or_stale = False
        with self._motion.lock:
            if self._motion.active_controller_goal is None:
                cancelled_or_stale = True

        if cancelled_or_stale:
            self._node.get_logger().warning('[Controller] Ignoring result for cancelled/stale trajectory goal')
            return

        try:
            result = future.result().result
            if result.error_code == 0:
                self._node.get_logger().info('[Controller] ✓ Trajectory execution succeeded!')
                self._motion.last_move_result = 0
            else:
                self._node.get_logger().error(
                    f'[Controller] Trajectory execution failed with error: {result.error_code}')
                self._log_final_tracking_error()
                self._motion.last_move_result = result.error_code
        except Exception as e:
            self._node.get_logger().error(f'[Controller] Result error: {e}')
            self._motion.last_move_result = -1
        finally:
            self._motion.active_controller_goal = None
            lock_released = False
            if self._motion.execution_lock.locked():
                self._motion.execution_lock.release()
                lock_released = True
            if lock_released:
                with self._motion.lock:
                    self._motion.is_executing = False
                self._queue.mark_current_complete(self._motion.last_move_result)
                self.process_next_queued_task()


def _executor(robot_controller):
    return robot_controller.trajectory_executor


def _send_trajectory_to_controller(robot_controller, joint_trajectory):
    return _executor(robot_controller).send_trajectory_to_controller(joint_trajectory)


def _process_next_queued_task(robot_controller):
    return _executor(robot_controller).process_next_queued_task()
