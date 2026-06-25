from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from copy import deepcopy
import math
import time
import config
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryExecutor:
    """Owns controller goal submission and queue draining for timed joint trajectories."""

    def __init__(self, node, coordinator, motion_queue, controller_client):
        self._node = node
        self._motion = coordinator
        self._queue = motion_queue
        self._controller_client = controller_client
        self._last_sent_trajectory = None
        self._active_unwind_check = None
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
            inserted_point = deepcopy(first_point)
            ratio = (idx + 1) / insert_count
            inserted_point.time_from_start = self._sec_to_duration(effective_hold_s * ratio)
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

    @staticmethod
    def _canonical_angle(value):
        adjusted = float(value)
        two_pi = 2.0 * math.pi
        while adjusted > math.pi:
            adjusted -= two_pi
        while adjusted <= -math.pi:
            adjusted += two_pi
        return adjusted

    @staticmethod
    def _estimate_point_to_point_duration(distance_rad, vel_rad_s, acc_rad_s2):
        distance = abs(float(distance_rad))
        velocity = max(float(vel_rad_s), 1e-3)
        acceleration = max(float(acc_rad_s2), 1e-3)
        accel_distance = (velocity * velocity) / acceleration
        if distance <= accel_distance:
            return 2.0 * math.sqrt(distance / acceleration)
        cruise_distance = distance - accel_distance
        return 2.0 * (velocity / acceleration) + (cruise_distance / velocity)

    @staticmethod
    def _clamp_percentage(value, default_percent=100.0):
        if value is None:
            return float(default_percent)
        return max(0.0, min(100.0, float(value)))

    def _build_post_success_unwind_trajectory(self, require_enabled=True, vel=None, acc=None):
        if require_enabled and not bool(getattr(config, 'EXECUTOR_POST_UNWIND_ENABLED', False)):
            return None

        joint_names = list(getattr(config, 'JOINT_NAMES', []) or [])
        unwind_joint_name = str(
            getattr(config, 'EXECUTOR_POST_UNWIND_JOINT_NAME', 'Joint_6')
        ).strip()
        if unwind_joint_name not in joint_names:
            return None

        current_positions = self._get_latest_joint_state_in_trajectory_order(joint_names)
        if current_positions is None:
            return None

        joint_index = joint_names.index(unwind_joint_name)
        current_value = float(current_positions[joint_index])
        target_value = self._canonical_angle(current_value)
        delta = target_value - current_value
        min_delta = float(getattr(config, 'EXECUTOR_POST_UNWIND_MIN_DELTA_RAD', 0.5))
        if abs(delta) < min_delta:
            return None

        target_range = float(getattr(config, 'EXECUTOR_POST_UNWIND_TARGET_RANGE_RAD', math.pi))
        if abs(target_value) > target_range + 1e-9:
            return None

        vel_percent = self._clamp_percentage(vel)
        acc_percent = self._clamp_percentage(acc)
        base_speed = float(getattr(config, 'EXECUTOR_POST_UNWIND_SPEED_RAD_S', 0.8))
        base_acceleration = float(getattr(config, 'EXECUTOR_POST_UNWIND_ACCEL_RAD_S2', 1.0))
        speed = max(base_speed * (vel_percent / 100.0), 1e-3)
        acceleration = max(base_acceleration * (acc_percent / 100.0), 1e-3)
        min_duration = float(getattr(config, 'EXECUTOR_POST_UNWIND_MIN_DURATION_S', 2.0))
        duration_sec = max(
            min_duration,
            self._estimate_point_to_point_duration(abs(delta), speed, acceleration),
        )

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = self._node.get_clock().now().to_msg()

        max_segment = max(
            float(getattr(config, 'EXECUTOR_POST_UNWIND_MAX_SEGMENT_RAD', 1.2)),
            1e-3,
        )
        segment_count = max(1, int(math.ceil(abs(delta) / max_segment)))
        unwind_velocity = delta / max(duration_sec, 1e-3)
        points = []
        for step_index in range(segment_count + 1):
            fraction = step_index / segment_count
            positions = list(current_positions)
            positions[joint_index] = current_value + delta * fraction

            point = JointTrajectoryPoint()
            point.positions = positions
            velocities = [0.0] * len(positions)
            if 0 < step_index < segment_count:
                velocities[joint_index] = unwind_velocity
            point.velocities = velocities
            point.accelerations = [0.0] * len(positions)
            point.time_from_start = self._sec_to_duration(duration_sec * fraction)
            points.append(point)

        traj.points = points
        return {
            'joint_name': unwind_joint_name,
            'joint_index': joint_index,
            'current_value': current_value,
            'target_value': target_value,
            'delta': delta,
            'duration_sec': duration_sec,
            'segment_count': segment_count,
            'vel_percent': vel_percent,
            'acc_percent': acc_percent,
            'vel': speed,
            'acc': acceleration,
            'trajectory': traj,
        }

    def _should_skip_post_success_unwind(self):
        last_sent = self._last_sent_trajectory
        if last_sent is None or len(getattr(last_sent, 'points', []) or []) < 2:
            return False

        points = list(last_sent.points)
        duration_sec = self._duration_to_sec(points[-1].time_from_start)
        max_duration = float(
            getattr(config, 'EXECUTOR_POST_UNWIND_SKIP_NOOP_DURATION_S', 1.0)
        )
        if duration_sec > max_duration:
            return False

        first_positions = list(points[0].positions)
        last_positions = list(points[-1].positions)
        if len(first_positions) != len(last_positions):
            return False

        max_joint_delta = max(
            abs(float(end) - float(start))
            for start, end in zip(first_positions, last_positions)
        ) if first_positions else 0.0
        max_allowed_delta = float(
            getattr(config, 'EXECUTOR_POST_UNWIND_SKIP_NOOP_MAX_JOINT_DELTA_RAD', 0.02)
        )
        if max_joint_delta > max_allowed_delta:
            return False

        self._node.get_logger().info(
            '[Controller] Skipping post-motion unwind after effectively no-op move '
            f'(duration={duration_sec:.3f}s, max_joint_delta={max_joint_delta:.4f} rad)'
        )
        return True

    @staticmethod
    def _nearest_equivalent_angle(reference, value):
        adjusted = float(value)
        ref = float(reference)
        two_pi = 2.0 * math.pi
        while adjusted - ref > math.pi:
            adjusted -= two_pi
        while adjusted - ref < -math.pi:
            adjusted += two_pi
        return adjusted

    def _unwrap_joint6_continuity(self, joint_trajectory):
        if not joint_trajectory.points:
            return

        joint_index = None
        for index, joint_name in enumerate(joint_trajectory.joint_names):
            name = str(joint_name or '').strip().lower()
            if name in {'joint_6', 'j6', 'axis_6'} or name.endswith('_6'):
                joint_index = index
                break
        if joint_index is None:
            return

        previous = float(joint_trajectory.points[0].positions[joint_index])
        max_adjustment = 0.0
        for point in joint_trajectory.points[1:]:
            positions = list(point.positions)
            original = float(positions[joint_index])
            adjusted = self._nearest_equivalent_angle(previous, original)
            positions[joint_index] = adjusted
            point.positions = positions
            max_adjustment = max(max_adjustment, abs(adjusted - original))
            previous = adjusted

        if max_adjustment > 1e-6:
            self._node.get_logger().info(
                '[Controller] Unwrapped Joint_6 continuity for execution '
                f'(max wrap adjustment {max_adjustment:.4f} rad)'
            )

    def _anchor_joint6_to_live_branch(self, joint_trajectory):
        if not joint_trajectory.points:
            return

        joint_index = None
        for index, joint_name in enumerate(joint_trajectory.joint_names):
            name = str(joint_name or '').strip().lower()
            if name in {'joint_6', 'j6', 'axis_6'} or name.endswith('_6'):
                joint_index = index
                break
        if joint_index is None:
            return

        live_reference = float(joint_trajectory.points[0].positions[joint_index])
        if len(joint_trajectory.points) < 2:
            return

        original_values = [
            float(point.positions[joint_index]) for point in joint_trajectory.points
        ]
        original_end = original_values[-1]
        original_span = max(original_values) - min(original_values)

        max_adjustment = 0.0
        for point in joint_trajectory.points[1:]:
            positions = list(point.positions)
            original = float(positions[joint_index])
            adjusted = self._nearest_equivalent_angle(live_reference, original)
            positions[joint_index] = adjusted
            point.positions = positions
            max_adjustment = max(max_adjustment, abs(adjusted - original))

        anchored_end = float(joint_trajectory.points[-1].positions[joint_index])
        anchored_span = max(
            float(point.positions[joint_index]) for point in joint_trajectory.points
        ) - min(
            float(point.positions[joint_index]) for point in joint_trajectory.points
        )

        if max_adjustment > 1e-6:
            self._node.get_logger().info(
                '[Controller] Re-anchored Joint_6 to live branch for execution '
                f'(end {original_end:.4f} -> {anchored_end:.4f} rad, '
                f'span {original_span:.4f} -> {anchored_span:.4f} rad, '
                f'max adjustment {max_adjustment:.4f} rad)'
            )

    def _execute_post_success_unwind(self, trajectory, unwind_check=None):
        self.send_trajectory_to_controller(
            trajectory,
            preserve_explicit_wrap=True,
            unwind_check=unwind_check,
        )
        return 0

    def _execute_explicit_unwind_from_latest_state(self, vel=None, acc=None):
        queued_unwind = self._build_post_success_unwind_trajectory(
            require_enabled=False,
            vel=vel,
            acc=acc,
        )
        if queued_unwind is None:
            self._node.get_logger().info(
                '[Controller] Queued explicit Joint_6 unwind skipped — no unwind needed'
            )
            self._motion.last_move_result = 0
            self._queue.mark_current_complete(0)
            self.process_next_queued_task()
            return 0

        self._node.get_logger().info(
            '[Controller] Executing queued explicit Joint_6 unwind: '
            f'{queued_unwind["current_value"]:.3f} -> '
            f'{queued_unwind["target_value"]:.3f} rad '
            f'over {queued_unwind["duration_sec"]:.2f}s '
            f'(vel={queued_unwind["vel_percent"]:.1f}%, '
            f'acc={queued_unwind["acc_percent"]:.1f}%, '
            f'{queued_unwind["vel"]:.3f} rad/s, '
            f'{queued_unwind["acc"]:.3f} rad/s^2, '
            f'{queued_unwind["segment_count"]} segments)'
        )
        return self._execute_post_success_unwind(
            queued_unwind['trajectory'],
            unwind_check=queued_unwind,
        )

    def _verify_explicit_unwind_complete(self, unwind_check):
        joint_names = list(unwind_check.get('joint_names') or [])
        joint_name = str(unwind_check.get('joint_name') or '')
        target_value = float(unwind_check.get('target_value'))
        tolerance = float(
            getattr(config, 'EXECUTOR_POST_UNWIND_VERIFY_TOL_RAD', 0.12)
        )
        timeout_s = max(
            0.0,
            float(getattr(config, 'EXECUTOR_POST_UNWIND_VERIFY_TIMEOUT_S', 0.5)),
        )
        deadline = time.time() + timeout_s
        actual_positions = None

        while True:
            actual_positions = self._get_latest_joint_state_in_trajectory_order(joint_names)
            if actual_positions is not None:
                break
            if time.time() >= deadline:
                break
            time.sleep(0.02)

        if actual_positions is None:
            self._node.get_logger().error(
                f'[Controller] Explicit {joint_name} unwind verification failed: '
                'latest joint state unavailable'
            )
            return False

        joint_index = int(unwind_check.get('joint_index'))
        actual_value = float(actual_positions[joint_index])
        error = actual_value - target_value
        if abs(error) <= tolerance:
            self._node.get_logger().info(
                f'[Controller] Explicit {joint_name} unwind verified: '
                f'actual={actual_value:.4f} target={target_value:.4f} '
                f'error={error:.4f} rad tol={tolerance:.4f}'
            )
            return True

        self._node.get_logger().error(
            f'[Controller] Explicit {joint_name} unwind verification failed: '
            f'actual={actual_value:.4f} target={target_value:.4f} '
            f'error={error:.4f} rad tol={tolerance:.4f}'
        )
        return False

    def request_explicit_unwind(self, queue_if_busy=True, vel=None, acc=None):
        with self._motion.lock:
            is_busy = bool(self._motion.is_executing)

        if queue_if_busy and is_busy:
            vel_percent = self._clamp_percentage(vel)
            acc_percent = self._clamp_percentage(acc)
            queue_result = self._queue.submit(
                task_function=self._execute_explicit_unwind_from_latest_state,
                task_args=[vel, acc],
            )
            if isinstance(queue_result, tuple):
                task_id, position = queue_result
                self._motion.last_submitted_task_id = task_id
                self._node.get_logger().info(
                    '[Controller] Queued explicit Joint_6 unwind '
                    f'at position {position} (task #{task_id}); '
                    'target will be computed from latest Joint_6 state at execution time '
                    f'(vel={vel_percent:.1f}%, acc={acc_percent:.1f}%)'
                )
                return position
            self._node.get_logger().error(
                f'[Controller] Failed to queue explicit Joint_6 unwind: {queue_result}'
            )
            return queue_result

        if is_busy:
            self._node.get_logger().warning(
                '[Controller] Busy — rejecting explicit Joint_6 unwind'
            )
            return -1

        queued_unwind = self._build_post_success_unwind_trajectory(
            require_enabled=False,
            vel=vel,
            acc=acc,
        )
        if queued_unwind is None:
            self._node.get_logger().info(
                '[Controller] Explicit Joint_6 unwind skipped — no unwind needed'
            )
            self._motion.last_move_result = 0
            return 0

        task_id = self._queue.allocate_task_id()
        self._motion.last_submitted_task_id = task_id
        self._queue.start_immediate_task(task_id)
        self._node.get_logger().info(
            '[Controller] Executing explicit Joint_6 unwind immediately '
            f'(task #{task_id}): {queued_unwind["current_value"]:.3f} -> '
            f'{queued_unwind["target_value"]:.3f} rad '
            f'over {queued_unwind["duration_sec"]:.2f}s '
            f'(vel={queued_unwind["vel_percent"]:.1f}%, '
            f'acc={queued_unwind["acc_percent"]:.1f}%, '
            f'{queued_unwind["vel"]:.3f} rad/s, '
            f'{queued_unwind["acc"]:.3f} rad/s^2, '
            f'{queued_unwind["segment_count"]} segments)'
        )
        result = self._execute_post_success_unwind(
            queued_unwind['trajectory'],
            unwind_check=queued_unwind,
        )
        if result != 0:
            self._queue.mark_current_complete(result)
        return result

    def send_trajectory_to_controller(
        self,
        joint_trajectory,
        preserve_explicit_wrap=False,
        unwind_check=None,
    ):
        """Send trajectory directly to the low-level controller for smooth execution."""
        if not self._motion.execution_lock.acquire(blocking=False):
            self._node.get_logger().warning('[Controller] Trajectory already executing, ignoring')
            self._motion.last_move_result = -1
            return

        if not self._controller_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().error(
                f'[Controller] {self._controller_name} not available'
            )
            self._motion.last_move_result = -1
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()
            return

        if len(joint_trajectory.points) == 0:
            self._node.get_logger().error('[Controller] ✗ Empty trajectory - aborting')
            self._motion.last_move_result = -1
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
            self._motion.last_move_result = -1
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()
            return

        self._overwrite_first_point_with_live_state(joint_trajectory)
        if not preserve_explicit_wrap:
            self._unwrap_joint6_continuity(joint_trajectory)
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
        self._active_unwind_check = None
        if unwind_check is not None:
            copied_check = dict(unwind_check)
            copied_check['joint_names'] = list(joint_trajectory.joint_names)
            copied_check.pop('trajectory', None)
            self._active_unwind_check = copied_check

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
                self._active_unwind_check = None
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
            self._active_unwind_check = None
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
            queued_unwind = None
            active_unwind_check = self._active_unwind_check
            if result.error_code == 0:
                self._node.get_logger().info('[Controller] ✓ Trajectory execution succeeded!')
                if active_unwind_check is not None:
                    if self._verify_explicit_unwind_complete(active_unwind_check):
                        self._motion.last_move_result = 0
                    else:
                        self._motion.last_move_result = -6
                else:
                    self._motion.last_move_result = 0
                queue_size = self._queue.get_status().get('queue_size', 0)
                if (
                    active_unwind_check is None
                    and queue_size == 0
                    and not self._should_skip_post_success_unwind()
                ):
                    queued_unwind = self._build_post_success_unwind_trajectory()
                    if queued_unwind is not None:
                        queue_result = self._queue.submit(
                            task_function=self._execute_post_success_unwind,
                            task_args=[queued_unwind['trajectory'], queued_unwind],
                        )
                        if isinstance(queue_result, tuple):
                            task_id, _position = queue_result
                            self._node.get_logger().info(
                                '[Controller] Queued explicit post-motion unwind for '
                                f"{queued_unwind['joint_name']} (task #{task_id}): "
                                f"{queued_unwind['current_value']:.3f} -> "
                                f"{queued_unwind['target_value']:.3f} rad "
                                f'over {queued_unwind["duration_sec"]:.2f}s'
                            )
                        else:
                            self._node.get_logger().warning(
                                '[Controller] Failed to queue post-motion unwind '
                                f'for {queued_unwind["joint_name"]}: {queue_result}'
                            )
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
            self._active_unwind_check = None
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
