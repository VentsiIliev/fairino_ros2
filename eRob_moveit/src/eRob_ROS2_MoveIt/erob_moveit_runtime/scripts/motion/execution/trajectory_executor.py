from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from copy import deepcopy
import math
import time
import config
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryExecutor:
    """Owns controller goal submission and queue draining for timed joint trajectories."""

    def __init__(
            self,
            node,
            coordinator,
            motion_queue,
            controller_client,
            action_follow_trajectory: str | None = None,
    ):
        self._node = node
        self._motion = coordinator
        self._queue = motion_queue
        self._controller_client = controller_client
        self._last_sent_trajectory = None
        self._active_goal_started_monotonic = None
        self._goal_sequence = 0
        self._active_goal_sequence = None
        self._active_goal_is_stop = False
        self._active_unwind_check = None
        self._unwind_diag_timers = []
        self._active_drive_monitor_timer = None
        self._active_trajectory_cancel_reason = None
        self._active_drive_cancel_suppressed = False
        self._active_drive_disabled_since = None
        self._active_drive_disabled_samples = 0
        self._active_drive_disabled_reason = None
        self._unwind_cancel_reason = None
        action_name = (
                str(action_follow_trajectory or '').strip()
                or str(getattr(config, 'ACTION_FOLLOW_TRAJECTORY', '') or '').strip()
        )
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

    def _next_goal_sequence(self):
        self._goal_sequence += 1
        return self._goal_sequence

    def _sample_joint_trajectory(self, joint_trajectory, sample_time_s):
        points = list(getattr(joint_trajectory, 'points', []) or [])
        if not points:
            return None

        sample_time_s = max(0.0, float(sample_time_s))
        times = [self._duration_to_sec(point.time_from_start) for point in points]
        joint_count = len(joint_trajectory.joint_names)

        if sample_time_s <= times[0]:
            point = points[0]
            positions = [float(value) for value in point.positions[:joint_count]]
            velocities = list(getattr(point, 'velocities', []) or [])
            if len(velocities) >= joint_count:
                velocities = [float(value) for value in velocities[:joint_count]]
            else:
                velocities = [0.0] * joint_count
            return positions, velocities

        if sample_time_s >= times[-1]:
            point = points[-1]
            positions = [float(value) for value in point.positions[:joint_count]]
            velocities = [0.0] * joint_count
            return positions, velocities

        for index in range(1, len(points)):
            prev_t = times[index - 1]
            next_t = times[index]
            if sample_time_s > next_t:
                continue

            prev_point = points[index - 1]
            next_point = points[index]
            dt = max(next_t - prev_t, 1e-9)
            ratio = max(0.0, min(1.0, (sample_time_s - prev_t) / dt))
            prev_positions = [float(value) for value in prev_point.positions[:joint_count]]
            next_positions = [float(value) for value in next_point.positions[:joint_count]]
            positions = [
                prev_value + (next_value - prev_value) * ratio
                for prev_value, next_value in zip(prev_positions, next_positions)
            ]

            next_velocities = list(getattr(next_point, 'velocities', []) or [])
            prev_velocities = list(getattr(prev_point, 'velocities', []) or [])
            if len(prev_velocities) >= joint_count and len(next_velocities) >= joint_count:
                velocities = [
                    float(prev_value) + (float(next_value) - float(prev_value)) * ratio
                    for prev_value, next_value in zip(
                        prev_velocities[:joint_count],
                        next_velocities[:joint_count],
                    )
                ]
            else:
                velocities = [
                    (next_value - prev_value) / dt
                    for prev_value, next_value in zip(prev_positions, next_positions)
                ]
            return positions, velocities

        return None

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

    def _log_planned_trajectory_metrics(self, joint_trajectory):
        if not bool(getattr(config, 'TRAJ_METRICS_ENABLED', True)):
            return

        points = list(getattr(joint_trajectory, 'points', []) or [])
        joint_names = list(getattr(joint_trajectory, 'joint_names', []) or [])
        if len(points) < 2 or not joint_names:
            return

        times = [self._duration_to_sec(point.time_from_start) for point in points]
        duration = times[-1] - times[0]
        if duration <= 1e-9:
            return

        peak_joint_rates = [0.0] * len(joint_names)
        for prev_point, point, prev_t, point_t in zip(points, points[1:], times, times[1:]):
            dt = point_t - prev_t
            if dt <= 1e-9:
                continue
            for index, (prev_pos, pos) in enumerate(zip(prev_point.positions, point.positions)):
                rate = abs(float(pos) - float(prev_pos)) / dt
                if rate > peak_joint_rates[index]:
                    peak_joint_rates[index] = rate

        for point in points:
            velocities = list(getattr(point, 'velocities', []) or [])
            for index, velocity in enumerate(velocities[:len(peak_joint_rates)]):
                peak_joint_rates[index] = max(peak_joint_rates[index], abs(float(velocity)))

        joint_rate_text = ', '.join(
            f'{name}={rate:.3f}'
            for name, rate in zip(joint_names, peak_joint_rates)
        )
        self._node.get_logger().info(
            f'[TRAJ_METRICS] joint_peak_rad_s: {joint_rate_text}'
        )

        tcp_samples = self._sample_tcp_positions_with_moveit_fk(joint_names, points, times)
        if not tcp_samples:
            return

        path_length_m = 0.0
        peak_speed_m_s = 0.0
        segment_velocities = []
        segment_times = []
        for (prev_t, prev_pos), (point_t, pos) in zip(tcp_samples, tcp_samples[1:]):
            dt = point_t - prev_t
            if dt <= 1e-9:
                continue
            dx = pos[0] - prev_pos[0]
            dy = pos[1] - prev_pos[1]
            dz = pos[2] - prev_pos[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            path_length_m += distance
            speed = distance / dt
            peak_speed_m_s = max(peak_speed_m_s, speed)
            segment_velocities.append([dx / dt, dy / dt, dz / dt])
            segment_times.append((prev_t + point_t) * 0.5)

        peak_accel_m_s2 = 0.0
        for prev_vel, vel, prev_t, t in zip(
            segment_velocities,
            segment_velocities[1:],
            segment_times,
            segment_times[1:],
        ):
            dt = t - prev_t
            if dt <= 1e-9:
                continue
            dvx = vel[0] - prev_vel[0]
            dvy = vel[1] - prev_vel[1]
            dvz = vel[2] - prev_vel[2]
            accel = math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz) / dt
            peak_accel_m_s2 = max(peak_accel_m_s2, accel)

        avg_speed_m_s = path_length_m / duration if duration > 1e-9 else 0.0
        self._node.get_logger().info(
            '[TRAJ_METRICS] tcp_linear '
            f'path={path_length_m:.4f}m duration={duration:.3f}s '
            f'avg_vel={avg_speed_m_s:.4f}m/s '
            f'peak_vel={peak_speed_m_s:.4f}m/s '
            f'peak_acc={peak_accel_m_s2:.4f}m/s^2'
        )

    def _sample_tcp_positions_with_moveit_fk(self, joint_names, points, times):
        get_fk_client = getattr(self._node, 'get_fk_client', None)
        if not callable(get_fk_client):
            self._node.get_logger().warning(
                '[TRAJ_METRICS] TCP metrics unavailable: FK client helper missing'
            )
            return []

        fk_client = get_fk_client()
        if fk_client is None or not fk_client.wait_for_service(timeout_sec=0.05):
            self._node.get_logger().warning(
                '[TRAJ_METRICS] TCP metrics unavailable: /compute_fk service unavailable'
            )
            return []

        try:
            from moveit_msgs.srv import GetPositionFK
        except Exception as exc:
            self._node.get_logger().warning(
                f'[TRAJ_METRICS] TCP metrics unavailable: cannot import GetPositionFK ({exc})'
            )
            return []

        sample_limit = max(
            2,
            int(getattr(config, 'TRAJ_METRICS_FK_SAMPLE_LIMIT', 80)),
        )
        if len(points) <= sample_limit:
            sample_indices = list(range(len(points)))
        else:
            sample_indices = sorted({
                round(index * (len(points) - 1) / (sample_limit - 1))
                for index in range(sample_limit)
            })

        samples = []
        timeout_s = max(
            0.05,
            float(getattr(config, 'TRAJ_METRICS_FK_TIMEOUT_S', 0.25)),
        )
        for index in sample_indices:
            point = points[index]
            request = GetPositionFK.Request()
            request.header.frame_id = getattr(config, 'BASE_LINK', 'base_link')
            request.fk_link_names = [getattr(config, 'EE_LINK', 'ee_link')]
            request.robot_state.joint_state = JointState(
                name=list(joint_names),
                position=[float(value) for value in point.positions],
            )
            request.robot_state.is_diff = False

            future = fk_client.call_async(request)
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                if future.done():
                    break
                time.sleep(0.002)

            if not future.done():
                self._node.get_logger().warning(
                    '[TRAJ_METRICS] TCP metrics unavailable: /compute_fk timed out'
                )
                return []

            response = future.result()
            error_code = int(getattr(getattr(response, 'error_code', None), 'val', 0))
            poses = list(getattr(response, 'pose_stamped', []) or [])
            if error_code != 1 or not poses:
                self._node.get_logger().warning(
                    f'[TRAJ_METRICS] TCP metrics unavailable: /compute_fk failed '
                    f'(error_code={error_code})'
                )
                return []

            position = poses[0].pose.position
            samples.append((
                times[index],
                [float(position.x), float(position.y), float(position.z)],
            ))

        if len(samples) < 2:
            self._node.get_logger().warning(
                '[TRAJ_METRICS] TCP metrics unavailable: fewer than 2 FK samples'
            )
            return []
        return samples

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

    def _joint_value_from_latest_state(self, joint_name):
        current_joint_state = getattr(self._node, 'current_joint_state', None)
        if current_joint_state is None:
            return None
        names = list(getattr(current_joint_state, 'name', []) or [])
        positions = list(getattr(current_joint_state, 'position', []) or [])
        velocities = list(getattr(current_joint_state, 'velocity', []) or [])
        if joint_name not in names:
            return None
        index = names.index(joint_name)
        position = float(positions[index]) if index < len(positions) else None
        velocity = float(velocities[index]) if index < len(velocities) else None
        return position, velocity

    def _log_unwind_diagnostics(self, label, joint_trajectory=None, unwind_check=None):
        if not bool(getattr(config, 'EXECUTOR_UNWIND_DIAGNOSTICS_ENABLED', True)):
            return

        check = unwind_check or self._active_unwind_check
        if check is None:
            return

        joint_name = str(check.get('joint_name') or '')
        latest = self._joint_value_from_latest_state(joint_name)
        if latest is None:
            actual_text = 'actual=unavailable vel=unavailable'
        else:
            actual, velocity = latest
            actual_text = (
                f'actual={actual:.6f} '
                f'vel={velocity:.6f}' if velocity is not None else f'actual={actual:.6f} vel=unavailable'
            )

        context = (
            f'[UNWIND_DIAG] {label}: {joint_name} '
            f'current={float(check.get("current_value", 0.0)):.6f} '
            f'target={float(check.get("target_value", 0.0)):.6f} '
            f'delta={float(check.get("delta", 0.0)):.6f} '
            f'{actual_text}'
        )

        if joint_trajectory is not None and getattr(joint_trajectory, 'points', None):
            joint_names = list(getattr(joint_trajectory, 'joint_names', []) or [])
            if joint_name in joint_names:
                index = joint_names.index(joint_name)
                points = list(joint_trajectory.points)
                first = float(points[0].positions[index])
                second = float(points[1].positions[index]) if len(points) > 1 else first
                last = float(points[-1].positions[index])
                context += (
                    f' commanded_first={first:.6f}'
                    f' commanded_second={second:.6f}'
                    f' commanded_last={last:.6f}'
                    f' points={len(points)}'
                )

        self._node.get_logger().info(context)
        format_drive_state = getattr(self._node, '_format_drive_state_snapshot', None)
        if callable(format_drive_state):
            snapshot = format_drive_state(f'unwind_{label}')
            if snapshot:
                self._node.get_logger().info(snapshot)

    def _get_unwind_drive_state(self, unwind_check):
        get_drive_state = getattr(self._node, 'get_unwind_drive_state', None)
        if callable(get_drive_state):
            return get_drive_state(unwind_check)
        return None

    def _get_all_drive_states(self):
        get_all_drive_states = getattr(self._node, 'get_all_drive_states', None)
        if callable(get_all_drive_states):
            return get_all_drive_states()
        return []

    def _cancel_active_trajectory(self, reason):
        if self._active_goal_is_stop:
            self._node.get_logger().warning(
                f'[STOP] Suppressing cancellation of replacement stop trajectory: {reason}'
            )
            return False
        if self._active_trajectory_cancel_reason == reason:
            return True
        self._active_trajectory_cancel_reason = reason
        self._node.get_logger().error(f'[Controller] Active trajectory cancelled: {reason}')
        self._motion.last_move_result = config.MOTION_ERROR_CONTROLLER_EXECUTION_FAILED

        queue_cleared = self._queue.clear()
        if queue_cleared > 0:
            self._node.get_logger().error(
                f'[Controller] Cleared {queue_cleared} queued motion(s) after active trajectory cancellation'
            )

        goal_handle = None
        with self._motion.lock:
            goal_handle = self._motion.active_controller_goal
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().error(
                    f'[Controller] Failed to cancel active trajectory: {exc}'
                )
        return True

    def _cancel_active_trajectory_if_drive_disabled(self, label):
        if not bool(getattr(config, 'EXECUTOR_CANCEL_ON_DRIVE_DISABLE', True)):
            return False

        disabled_reason = None
        for drive_state in self._get_all_drive_states():
            if drive_state['state'] == 'operation_enabled':
                continue
            disabled_reason = (
                f"{drive_state['joint_name']} left operation_enabled during active trajectory "
                f"at {label}: statusword={drive_state['statusword']} "
                f"state={drive_state['state']} bits={drive_state['bits']}"
            )
            break

        if disabled_reason is None:
            self._active_drive_disabled_since = None
            self._active_drive_disabled_samples = 0
            self._active_drive_disabled_reason = None
            return False

        now = time.monotonic()
        if self._active_drive_disabled_reason != disabled_reason:
            self._active_drive_disabled_reason = disabled_reason
            self._active_drive_disabled_since = now
            self._active_drive_disabled_samples = 1
            self._node.get_logger().warning(
                f'[Controller] Active drive monitor observed non-enabled drive; '
                f'waiting for persistence before cancel: {disabled_reason}'
            )
            return False

        self._active_drive_disabled_samples += 1
        grace_s = max(
            0.0,
            float(getattr(config, 'EXECUTOR_ACTIVE_DRIVE_MONITOR_GRACE_S', 0.25)),
        )
        required_samples = max(
            1,
            int(getattr(config, 'EXECUTOR_ACTIVE_DRIVE_MONITOR_BAD_SAMPLES', 3)),
        )
        elapsed_s = now - float(self._active_drive_disabled_since or now)
        if elapsed_s < grace_s or self._active_drive_disabled_samples < required_samples:
            return False

        reason = (
            f'{disabled_reason} '
            f'(persisted {elapsed_s:.3f}s, samples={self._active_drive_disabled_samples})'
        )
        return self._cancel_active_trajectory(reason)

    def _schedule_active_drive_monitor(self):
        if not bool(getattr(config, 'EXECUTOR_ACTIVE_DRIVE_MONITOR_ENABLED', True)):
            return

        self._cancel_active_drive_monitor()
        period_s = max(0.02, float(getattr(config, 'EXECUTOR_ACTIVE_DRIVE_MONITOR_PERIOD_S', 0.05)))

        def _callback():
            with self._motion.lock:
                goal_active = self._motion.active_controller_goal is not None
            if not goal_active:
                self._cancel_active_drive_monitor()
                return
            self._cancel_active_trajectory_if_drive_disabled('drive_monitor')

        self._active_drive_monitor_timer = self._node.create_timer(period_s, _callback)

    def _cancel_active_drive_monitor(self):
        timer = self._active_drive_monitor_timer
        self._active_drive_monitor_timer = None
        if timer is not None:
            try:
                timer.cancel()
                self._node.destroy_timer(timer)
            except Exception:
                pass

    def _ensure_drive_enabled_before_trajectory(self, suppress_drive_disable_cancel=False):
        if suppress_drive_disable_cancel:
            return True
        if not bool(getattr(config, 'EXECUTOR_DRIVE_ENABLE_BEFORE_TRAJECTORY', True)):
            return True

        is_enabled = getattr(self._node, 'is_drive_operation_enabled_for_motion', None)
        if not callable(is_enabled):
            return True

        stable_required_s = max(
            0.0,
            float(getattr(config, 'EXECUTOR_DRIVE_ENABLE_STABLE_BEFORE_TRAJECTORY_S', 0.15)),
        )
        timeout_s = max(
            stable_required_s,
            float(getattr(config, 'EXECUTOR_DRIVE_ENABLE_WAIT_TIMEOUT_S', 2.0)),
        )
        started_at = time.monotonic()
        stable_since = None

        while time.monotonic() - started_at <= timeout_s:
            now = time.monotonic()
            if is_enabled():
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= stable_required_s:
                    return True
            else:
                stable_since = None
            time.sleep(0.01)

        fault_reason = getattr(self._node, 'get_drive_enable_fault_reason', None)
        reason = fault_reason() if callable(fault_reason) else 'drive is not operation_enabled'
        self._node.get_logger().error(
            f'[Controller] Drive not stably operation_enabled before trajectory: {reason}'
        )
        return False

    def _build_path_stop_trajectory(self):
        if not bool(getattr(config, 'EXECUTOR_PATH_STOP_ENABLED', True)):
            return None, 'disabled'

        source = self._last_sent_trajectory
        if source is None or len(getattr(source, 'points', []) or []) < 2:
            return None, 'no active trajectory copy'

        started_at = self._active_goal_started_monotonic
        if started_at is None:
            return None, 'active trajectory start time unavailable'

        joint_names = list(getattr(source, 'joint_names', []) or [])
        if not joint_names:
            return None, 'active trajectory has no joint names'

        measured_positions = self._get_latest_joint_state_in_trajectory_order(joint_names)
        if measured_positions is None:
            return None, 'measured joint state unavailable'

        elapsed_s = max(0.0, time.monotonic() - started_at)
        duration_s = max(
            0.05,
            float(getattr(config, 'EXECUTOR_PATH_STOP_DURATION_S', 0.30)),
        )
        sample_period_s = max(
            0.01,
            float(getattr(config, 'EXECUTOR_PATH_STOP_SAMPLE_PERIOD_S', 0.04)),
        )
        tracking_tol = max(
            0.0,
            float(getattr(config, 'EXECUTOR_PATH_STOP_TRACKING_TOL_RAD', 0.20)),
        )

        start_sample = self._sample_joint_trajectory(source, elapsed_s)
        if start_sample is None:
            return None, 'could not sample active trajectory at stop time'

        path_positions, path_velocities = start_sample
        max_tracking_error = max(
            abs(float(measured) - float(expected))
            for measured, expected in zip(measured_positions, path_positions)
        )
        if max_tracking_error > tracking_tol:
            return None, (
                f'tracking error {max_tracking_error:.4f} rad exceeds '
                f'{tracking_tol:.4f} rad'
            )

        source_end_s = self._duration_to_sec(source.points[-1].time_from_start)
        remaining_s = max(0.0, source_end_s - elapsed_s)
        forward_window_s = min(remaining_s, duration_s * 0.5)
        point_count = max(2, int(math.ceil(duration_s / sample_period_s)) + 1)
        hold_joint_names = {
            str(name).strip()
            for name in getattr(config, 'EXECUTOR_PATH_STOP_HOLD_JOINT_NAMES', [])
            if str(name).strip()
        }
        hold_joint_indices = [
            index
            for index, name in enumerate(joint_names)
            if name in hold_joint_names
        ]
        hold_joint_positions = {
            index: float(path_positions[index])
            for index in hold_joint_indices
        }

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = self._node.get_clock().now().to_msg()

        points = []
        for index in range(point_count):
            u = index / (point_count - 1)
            # Advance along the original trajectory, but reduce path speed to zero.
            path_offset_s = forward_window_s * (1.0 - (1.0 - u) * (1.0 - u))
            speed_scale = 1.0 - u
            sample = self._sample_joint_trajectory(source, elapsed_s + path_offset_s)
            if sample is None:
                return None, 'could not sample stop trajectory point'
            positions, velocities = sample
            for held_index in hold_joint_indices:
                positions[held_index] = hold_joint_positions[held_index]
                velocities[held_index] = 0.0

            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in positions]
            point.velocities = [
                float(value) * speed_scale
                for value in velocities
            ]
            point.accelerations = [0.0] * len(joint_names)
            point.time_from_start = self._sec_to_duration(duration_s * u)
            points.append(point)

        points[-1].velocities = [0.0] * len(joint_names)
        points[-1].accelerations = [0.0] * len(joint_names)
        traj.points = points

        return traj, (
            f'elapsed={elapsed_s:.3f}s forward_window={forward_window_s:.3f}s '
            f'duration={duration_s:.3f}s points={len(points)} '
            f'max_tracking_error={max_tracking_error:.4f}rad '
            f'held_joints={[joint_names[index] for index in hold_joint_indices]} '
            f'held_positions={[round(hold_joint_positions[index], 6) for index in hold_joint_indices]}'
        )

    def send_path_stop_trajectory(self):
        stop_trajectory, detail = self._build_path_stop_trajectory()
        if stop_trajectory is None:
            self._node.get_logger().warning(
                f'[STOP] Path stop trajectory unavailable: {detail}'
            )
            return False

        if not self._controller_client.wait_for_server(timeout_sec=0.25):
            self._node.get_logger().warning(
                f'[STOP] Cannot send path stop: {self._controller_name} not available'
            )
            return False

        goal_sequence = self._next_goal_sequence()
        controller_goal = FollowJointTrajectory.Goal()
        controller_goal.trajectory = stop_trajectory

        goal_build_started_at = time.perf_counter()
        goal_tolerance = []
        path_tolerance = []
        hold_joint_names = {
            str(name).strip()
            for name in getattr(config, 'EXECUTOR_PATH_STOP_HOLD_JOINT_NAMES', [])
            if str(name).strip()
        }
        held_goal_tolerance = float(
            getattr(config, 'EXECUTOR_PATH_STOP_HELD_GOAL_TOL_RAD', 0.35)
        )
        for name in stop_trajectory.joint_names:
            goal_tol = JointTolerance()
            goal_tol.name = name
            goal_tol.position = held_goal_tolerance if name in hold_joint_names else config.EXECUTOR_GOAL_POS_TOL_RAD
            goal_tol.velocity = 0.0
            goal_tol.acceleration = 0.0
            goal_tolerance.append(goal_tol)

            path_tol = JointTolerance()
            path_tol.name = name
            path_tol.position = float(getattr(config, 'EXECUTOR_PATH_POS_TOL_RAD', 0.35))
            path_tol.velocity = 0.0
            path_tol.acceleration = 0.0
            path_tolerance.append(path_tol)

        controller_goal.path_tolerance = path_tolerance
        controller_goal.goal_tolerance = goal_tolerance
        stop_duration_s = self._duration_to_sec(stop_trajectory.points[-1].time_from_start)
        time_tolerance_sec = max(1.0, stop_duration_s + 1.0)
        controller_goal.goal_time_tolerance.sec = int(time_tolerance_sec)
        controller_goal.goal_time_tolerance.nanosec = int((time_tolerance_sec % 1.0) * 1e9)

        self._last_sent_trajectory = deepcopy(stop_trajectory)
        self._active_unwind_check = None
        self._active_trajectory_cancel_reason = 'operator path stop'
        self._active_drive_cancel_suppressed = True
        self._active_goal_sequence = goal_sequence
        self._active_goal_is_stop = True
        self._active_goal_started_monotonic = time.monotonic()
        self._cancel_unwind_diagnostic_timers()
        self._cancel_active_drive_monitor()

        self._node.get_logger().warning(
            f'[STOP] Sending replacement path stop trajectory ({detail})'
        )
        future = self._controller_client.send_goal_async(controller_goal)
        self._motion.active_execute_send_future = future
        future.add_done_callback(
            lambda done_future, sequence=goal_sequence: self._on_controller_goal_response(
                done_future,
                sequence,
            )
        )
        return True

    def _cancel_active_unwind_if_drive_disabled(self, label):
        if not bool(getattr(config, 'EXECUTOR_UNWIND_CANCEL_ON_DRIVE_DISABLE', True)):
            return False
        unwind_check = self._active_unwind_check
        if unwind_check is None:
            return False

        drive_state = self._get_unwind_drive_state(unwind_check)
        if drive_state is None:
            return False
        if drive_state['state'] == 'operation_enabled':
            return False

        reason = (
            f"explicit {drive_state['joint_name']} unwind cancelled after drive left "
            f"operation_enabled at {label}: statusword={drive_state['statusword']} "
            f"state={drive_state['state']} bits={drive_state['bits']}"
        )
        if self._unwind_cancel_reason == reason:
            return True
        self._unwind_cancel_reason = reason
        self._node.get_logger().error(f'[Controller] {reason}')

        goal_handle = None
        with self._motion.lock:
            goal_handle = self._motion.active_controller_goal
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().error(
                    f'[Controller] Failed to cancel explicit unwind after drive disable: {exc}'
                )
        return True

    def _schedule_unwind_diagnostics(self):
        if not bool(getattr(config, 'EXECUTOR_UNWIND_DIAGNOSTICS_ENABLED', True)):
            return
        if self._active_unwind_check is None:
            return

        self._cancel_unwind_diagnostic_timers()
        delays = getattr(config, 'EXECUTOR_UNWIND_DIAGNOSTIC_DELAYS_S', [0.2, 0.5, 1.0])
        for raw_delay in delays:
            try:
                delay = float(raw_delay)
            except (TypeError, ValueError):
                continue
            if delay <= 0.0:
                continue

            timer_ref = {'timer': None}

            def _callback(delay_s=delay, holder=timer_ref):
                timer = holder.get('timer')
                if timer is not None:
                    try:
                        timer.cancel()
                        self._node.destroy_timer(timer)
                    except Exception:
                        pass
                    if timer in self._unwind_diag_timers:
                        self._unwind_diag_timers.remove(timer)
                self._log_unwind_diagnostics(f'accepted_plus_{delay_s:.1f}s')
                self._cancel_active_unwind_if_drive_disabled(f'accepted_plus_{delay_s:.1f}s')

            timer = self._node.create_timer(delay, _callback)
            timer_ref['timer'] = timer
            self._unwind_diag_timers.append(timer)

    def _cancel_unwind_diagnostic_timers(self):
        timers = list(self._unwind_diag_timers)
        self._unwind_diag_timers.clear()
        for timer in timers:
            try:
                timer.cancel()
                self._node.destroy_timer(timer)
            except Exception:
                pass

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
    def _smoothstep5(fraction):
        u = max(0.0, min(1.0, float(fraction)))
        return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))

    @staticmethod
    def _smoothstep5_derivative(fraction):
        u = max(0.0, min(1.0, float(fraction)))
        return 30.0 * u * u * (1.0 - u) * (1.0 - u)

    @staticmethod
    def _clamp_percentage(value, default_percent=100.0):
        if value is None:
            return float(default_percent)
        return max(0.0, min(100.0, float(value)))

    def _set_post_success_unwind_skip_reason(self, reason, **details):
        skip_reason = {'reason': str(reason)}
        skip_reason.update(details)
        self._last_post_success_unwind_skip_reason = skip_reason
        return None

    def _format_post_success_unwind_skip_reason(self):
        skip_reason = dict(
            getattr(self, '_last_post_success_unwind_skip_reason', {}) or {}
        )
        reason = skip_reason.get('reason') or 'unknown'
        if reason == 'no_unwind_needed':
            return (
                'no unwind needed '
                f"({skip_reason.get('joint_name', 'Joint_6')} "
                f"current={float(skip_reason.get('current_value', 0.0)):.4f}rad "
                f"target={float(skip_reason.get('target_value', 0.0)):.4f}rad "
                f"delta={float(skip_reason.get('delta', 0.0)):.4f}rad "
                f"min_delta={float(skip_reason.get('min_delta', 0.0)):.4f}rad)"
            )
        if reason == 'target_out_of_range':
            return (
                'target outside allowed range '
                f"({skip_reason.get('joint_name', 'Joint_6')} "
                f"target={float(skip_reason.get('target_value', 0.0)):.4f}rad "
                f"range={float(skip_reason.get('target_range', 0.0)):.4f}rad)"
            )
        if reason == 'disabled':
            return 'post-success unwind disabled'
        if reason == 'joint_not_configured':
            return f"joint {skip_reason.get('joint_name', 'Joint_6')!r} is not configured"
        if reason == 'latest_joint_state_unavailable':
            return 'latest joint state unavailable'
        return str(reason)

    def _build_post_success_unwind_trajectory(self, require_enabled=True, vel=None, acc=None):
        self._last_post_success_unwind_skip_reason = None
        if require_enabled and not bool(getattr(config, 'EXECUTOR_POST_UNWIND_ENABLED', False)):
            return self._set_post_success_unwind_skip_reason('disabled')

        joint_names = list(getattr(config, 'JOINT_NAMES', []) or [])
        unwind_joint_name = str(
            getattr(config, 'EXECUTOR_POST_UNWIND_JOINT_NAME', 'Joint_6')
        ).strip()
        if unwind_joint_name not in joint_names:
            return self._set_post_success_unwind_skip_reason(
                'joint_not_configured',
                joint_name=unwind_joint_name,
            )

        current_positions = self._get_latest_joint_state_in_trajectory_order(joint_names)
        if current_positions is None:
            return self._set_post_success_unwind_skip_reason(
                'latest_joint_state_unavailable',
                joint_name=unwind_joint_name,
            )

        joint_index = joint_names.index(unwind_joint_name)
        current_value = float(current_positions[joint_index])
        target_value = self._canonical_angle(current_value)
        delta = target_value - current_value
        min_delta = float(getattr(config, 'EXECUTOR_POST_UNWIND_MIN_DELTA_RAD', 0.5))
        if abs(delta) < min_delta:
            return self._set_post_success_unwind_skip_reason(
                'no_unwind_needed',
                joint_name=unwind_joint_name,
                current_value=current_value,
                target_value=target_value,
                delta=delta,
                min_delta=min_delta,
            )

        target_range = float(getattr(config, 'EXECUTOR_POST_UNWIND_TARGET_RANGE_RAD', math.pi))
        if abs(target_value) > target_range + 1e-9:
            return self._set_post_success_unwind_skip_reason(
                'target_out_of_range',
                joint_name=unwind_joint_name,
                target_value=target_value,
                target_range=target_range,
            )

        vel_percent = self._clamp_percentage(vel)
        acc_percent = self._clamp_percentage(acc)
        base_speed = float(getattr(config, 'EXECUTOR_POST_UNWIND_SPEED_RAD_S', 0.8))
        base_acceleration = float(getattr(config, 'EXECUTOR_POST_UNWIND_ACCEL_RAD_S2', 1.0))
        speed = max(base_speed * (vel_percent / 100.0), 1e-3)
        acceleration = max(base_acceleration * (acc_percent / 100.0), 1e-3)
        min_duration = float(getattr(config, 'EXECUTOR_POST_UNWIND_MIN_DURATION_S', 2.0))
        distance = abs(delta)
        # Dense smoothstep sampling is deliberately used for cable unwind.  The
        # equivalent -2pi -> 0 branch change is mechanically real for the cable,
        # so keep it monotonic and easy for the drive to track.
        duration_sec = max(
            min_duration,
            self._estimate_point_to_point_duration(distance, speed, acceleration),
            1.875 * distance / speed,
            math.sqrt(5.8 * distance / acceleration),
        )

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = self._node.get_clock().now().to_msg()

        max_segment = max(
            float(getattr(config, 'EXECUTOR_POST_UNWIND_MAX_SEGMENT_RAD', 1.2)),
            1e-3,
        )
        sample_period = max(
            float(getattr(config, 'EXECUTOR_POST_UNWIND_SAMPLE_PERIOD_S', 0.05)),
            1e-3,
        )
        segment_count = max(
            1,
            int(math.ceil(distance / max_segment)),
            int(math.ceil(duration_sec / sample_period)),
        )
        points = []
        for step_index in range(segment_count + 1):
            fraction = step_index / segment_count
            smooth_fraction = self._smoothstep5(fraction)
            positions = list(current_positions)
            positions[joint_index] = current_value + delta * smooth_fraction

            point = JointTrajectoryPoint()
            point.positions = positions
            velocities = [0.0] * len(positions)
            if 0 < step_index < segment_count:
                velocities[joint_index] = (
                    delta * self._smoothstep5_derivative(fraction) / max(duration_sec, 1e-3)
                )
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
                '[Controller] Queued explicit Joint_6 unwind skipped - '
                f'{self._format_post_success_unwind_skip_reason()}'
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
                '[Controller] Explicit Joint_6 unwind skipped - '
                f'{self._format_post_success_unwind_skip_reason()}'
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
        suppress_drive_disable_cancel=False,
    ):
        """Send trajectory directly to the low-level controller for smooth execution."""
        controller_prepare_started_at = time.perf_counter()
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(self._node, "controller_prepare_start", points=len(getattr(joint_trajectory, "points", []) or []))
        except Exception:
            pass
        drive_check_started_at = time.perf_counter()
        if not self._ensure_drive_enabled_before_trajectory(
            suppress_drive_disable_cancel=suppress_drive_disable_cancel,
        ):
            self._motion.last_move_result = config.MOTION_ERROR_DRIVE_NOT_ENABLED
            with self._motion.lock:
                self._motion.is_executing = False
            return
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(self._node, "drive_check_done", duration_s=time.perf_counter() - drive_check_started_at)
        except Exception:
            pass

        if not self._motion.execution_lock.acquire(blocking=False):
            self._node.get_logger().warning('[Controller] Trajectory already executing, ignoring')
            self._motion.last_move_result = -1
            return

        controller_server_wait_started_at = time.perf_counter()
        if not self._controller_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().error(
                f'[Controller] {self._controller_name} not available'
            )
            self._motion.last_move_result = -1
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()
            return
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(self._node, "controller_server_ready", duration_s=time.perf_counter() - controller_server_wait_started_at)
        except Exception:
            pass

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

        trajectory_mutation_started_at = time.perf_counter()
        self._overwrite_first_point_with_live_state(joint_trajectory)
        if not preserve_explicit_wrap:
            self._unwrap_joint6_continuity(joint_trajectory)
        self._soften_trajectory_start(joint_trajectory)
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(self._node, "trajectory_mutation_done", duration_s=time.perf_counter() - trajectory_mutation_started_at)
        except Exception:
            pass
        log_drive_state = getattr(self._node, 'log_drive_state_before_first_motion', None)
        if callable(log_drive_state):
            log_drive_state()

        with self._motion.lock:
            self._motion.is_executing = True

        goal_build_started_at = time.perf_counter()
        goal_tolerance = []
        path_tolerance = []
        for name in joint_trajectory.joint_names:
            goal_tol = JointTolerance()
            goal_tol.name = name
            goal_tol.position = config.EXECUTOR_GOAL_POS_TOL_RAD
            goal_tol.velocity = 0.0
            goal_tol.acceleration = 0.0
            goal_tolerance.append(goal_tol)

            path_tol = JointTolerance()
            path_tol.name = name
            path_tol.position = float(getattr(config, 'EXECUTOR_PATH_POS_TOL_RAD', 0.35))
            path_tol.velocity = 0.0
            path_tol.acceleration = 0.0
            path_tolerance.append(path_tol)

        controller_goal = FollowJointTrajectory.Goal()
        controller_goal.trajectory = joint_trajectory
        controller_goal.path_tolerance = path_tolerance
        controller_goal.goal_tolerance = goal_tolerance
        self._last_sent_trajectory = deepcopy(joint_trajectory)
        self._active_unwind_check = None
        self._active_trajectory_cancel_reason = None
        self._active_drive_cancel_suppressed = bool(suppress_drive_disable_cancel)
        self._active_drive_disabled_since = None
        self._active_drive_disabled_samples = 0
        self._active_drive_disabled_reason = None
        self._unwind_cancel_reason = None
        if unwind_check is not None:
            copied_check = dict(unwind_check)
            copied_check['joint_names'] = list(joint_trajectory.joint_names)
            copied_check.pop('trajectory', None)
            self._active_unwind_check = copied_check
            self._log_unwind_diagnostics('submit', joint_trajectory, copied_check)

        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(self._node, "controller_goal_built", duration_s=time.perf_counter() - goal_build_started_at)
        except Exception:
            pass

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
            # self._log_planned_trajectory_metrics(joint_trajectory)
            # self._log_final_trajectory_segment(joint_trajectory)
        else:
            time_tolerance_sec = config.EXECUTOR_TIME_MIN_S

        controller_goal.goal_time_tolerance.sec = int(time_tolerance_sec)
        controller_goal.goal_time_tolerance.nanosec = int((time_tolerance_sec % 1.0) * 1e9)

        self._node.get_logger().info(
            f'[Controller] Sending {len(joint_trajectory.points)} points directly to controller '
            f'(spline interpolation, path_tolerance={float(getattr(config, "EXECUTOR_PATH_POS_TOL_RAD", 0.35)):.3f}rad, '
            f'goal_time_tolerance={time_tolerance_sec:.1f}s)')
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(self._node, "controller_prepare_done", duration_s=time.perf_counter() - controller_prepare_started_at)
        except Exception:
            pass

        goal_sequence = self._next_goal_sequence()
        self._active_goal_sequence = goal_sequence
        self._active_goal_is_stop = False
        self._active_goal_started_monotonic = None

        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(
                self._node,
                "goal_send",
                points=len(joint_trajectory.points),
                goal_time_tolerance_s=float(time_tolerance_sec),
            )
        except Exception:
            pass
        controller_goal.trajectory.header.stamp = self._node.get_clock().now().to_msg()
        future = self._controller_client.send_goal_async(controller_goal)
        self._motion.active_execute_send_future = future
        future.add_done_callback(
            lambda done_future, sequence=goal_sequence: self._on_controller_goal_response(
                done_future,
                sequence,
            )
        )

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

    def _on_controller_goal_response(self, future, goal_sequence=None):
        self._motion.active_execute_send_future = None
        if goal_sequence is not None and goal_sequence != self._active_goal_sequence:
            self._node.get_logger().warning('[Controller] Ignoring stale goal response')
            return
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                try:
                    from motion.move_linear_timing import mark as mark_move_linear_timing, clear as clear_move_linear_timing
                    mark_move_linear_timing(self._node, "goal_rejected", controller=self._controller_name)
                    clear_move_linear_timing(self._node)
                except Exception:
                    pass
                self._node.get_logger().error(
                    f'[Controller] Trajectory execution rejected by {self._controller_name}'
                )
                self._motion.active_controller_goal = None
                self._active_goal_sequence = None
                self._active_goal_started_monotonic = None
                self._active_goal_is_stop = False
                self._active_unwind_check = None
                with self._motion.lock:
                    self._motion.is_executing = False
                self._motion.execution_lock.release()
                return

            try:
                from motion.move_linear_timing import mark as mark_move_linear_timing, clear as clear_move_linear_timing
                mark_move_linear_timing(self._node, "goal_accepted", controller=self._controller_name)
                clear_move_linear_timing(self._node)
            except Exception:
                pass
            self._node.get_logger().info(
                f'[Controller] Trajectory accepted by {self._controller_name}'
            )
            self._motion.active_controller_goal = goal_handle
            self._active_goal_started_monotonic = time.monotonic()
            if self._active_drive_cancel_suppressed:
                self._node.get_logger().info(
                    '[Controller] Active drive-disable cancellation suppressed for this trajectory'
                )
            elif self._active_unwind_check is not None:
                self._node.get_logger().info(
                    '[Controller] Using explicit unwind drive monitor for this trajectory'
                )
            else:
                self._schedule_active_drive_monitor()
                self._cancel_active_trajectory_if_drive_disabled('accepted')
            if self._active_unwind_check is not None:
                self._log_unwind_diagnostics('accepted')
                self._schedule_unwind_diagnostics()
                self._cancel_active_unwind_if_drive_disabled('accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done_future, sequence=goal_sequence: self._on_controller_goal_result(
                    done_future,
                    sequence,
                )
            )
        except Exception as e:
            self._node.get_logger().error(f'[Controller] Goal response error: {e}')
            self._active_goal_sequence = None
            self._active_goal_started_monotonic = None
            self._active_goal_is_stop = False
            self._active_unwind_check = None
            with self._motion.lock:
                self._motion.is_executing = False
            self._motion.execution_lock.release()

    def _on_controller_goal_result(self, future, goal_sequence=None):
        cancelled_or_stale = False
        with self._motion.lock:
            if self._motion.active_controller_goal is None:
                cancelled_or_stale = True
            if goal_sequence is not None and goal_sequence != self._active_goal_sequence:
                cancelled_or_stale = True

        if cancelled_or_stale:
            self._node.get_logger().warning('[Controller] Ignoring result for cancelled/stale trajectory goal')
            return

        try:
            result = future.result().result
            queued_unwind = None
            active_unwind_check = self._active_unwind_check
            active_goal_is_stop = bool(self._active_goal_is_stop)
            if active_goal_is_stop and result.error_code == 0:
                self._node.get_logger().warning('[STOP] Path stop trajectory completed')
                self._motion.last_move_result = -1
            elif self._active_trajectory_cancel_reason:
                self._node.get_logger().error(
                    f'[Controller] Trajectory result ignored after active cancellation: '
                    f'{self._active_trajectory_cancel_reason}'
                )
                self._log_final_tracking_error()
                self._motion.last_move_result = config.MOTION_ERROR_CONTROLLER_EXECUTION_FAILED
            elif result.error_code == 0:
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
                    and not bool(getattr(self._node, "_suppress_post_success_unwind", False))
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
                if self._active_trajectory_cancel_reason:
                    self._node.get_logger().error(
                        f'[Controller] {self._active_trajectory_cancel_reason}'
                    )
                if active_unwind_check is not None:
                    if self._unwind_cancel_reason:
                        self._node.get_logger().error(
                            f'[Controller] {self._unwind_cancel_reason}'
                        )
                    self._log_unwind_diagnostics('failed_result')
                self._log_final_tracking_error()
                controller_error = int(getattr(result, 'error_code', -1))
                self._node.get_logger().error(
                    f'[Controller] Mapping controller error {controller_error} to motion result '
                    f'{config.MOTION_ERROR_CONTROLLER_EXECUTION_FAILED}'
                )
                self._motion.last_move_result = config.MOTION_ERROR_CONTROLLER_EXECUTION_FAILED
        except Exception as e:
            self._node.get_logger().error(f'[Controller] Result error: {e}')
            self._motion.last_move_result = -1
        finally:
            cancelled_by_monitor = self._active_trajectory_cancel_reason is not None
            completed_stop_goal = bool(self._active_goal_is_stop)
            self._cancel_unwind_diagnostic_timers()
            self._cancel_active_drive_monitor()
            self._motion.active_controller_goal = None
            self._active_goal_sequence = None
            self._active_goal_started_monotonic = None
            self._active_goal_is_stop = False
            self._active_unwind_check = None
            cancel_reason = self._active_trajectory_cancel_reason
            self._active_trajectory_cancel_reason = None
            self._active_drive_cancel_suppressed = False
            self._active_drive_disabled_since = None
            self._active_drive_disabled_samples = 0
            self._active_drive_disabled_reason = None
            self._unwind_cancel_reason = None
            lock_released = False
            if self._motion.execution_lock.locked():
                self._motion.execution_lock.release()
                lock_released = True
            if lock_released:
                with self._motion.lock:
                    self._motion.is_executing = False
                self._queue.mark_current_complete(self._motion.last_move_result)
                if completed_stop_goal:
                    queue_cleared = self._queue.clear()
                    self._node.get_logger().warning(
                        '[STOP] Motion queue drain suppressed after path stop completion'
                        + (f'; cleared={queue_cleared}' if queue_cleared > 0 else '')
                    )
                elif cancelled_by_monitor:
                    queue_cleared = self._queue.clear()
                    self._node.get_logger().error(
                        '[Controller] Motion queue drain suppressed after active trajectory cancellation'
                        + (f': {cancel_reason}' if cancel_reason else '')
                        + (f'; cleared={queue_cleared}' if queue_cleared > 0 else '')
                    )
                else:
                    self.process_next_queued_task()


def _executor(robot_controller):
    return robot_controller.trajectory_executor


def _send_trajectory_to_controller(robot_controller, joint_trajectory, **kwargs):
    return _executor(robot_controller).send_trajectory_to_controller(joint_trajectory, **kwargs)


def _process_next_queued_task(robot_controller):
    return _executor(robot_controller).process_next_queued_task()
