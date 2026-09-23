import logging
import json
import sys
import time
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from motion.servo.conditional_servo import ConditionalServoSupervisor


class Value:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class FakeRobot:
    def __init__(self):
        self.started = 0
        self.stopped = 0
        self.pose = [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]

    def start_servo_jog(self, **kwargs):
        self.started += 1
        return 0

    def stop_servo_jog(self, **kwargs):
        self.stopped += 1
        return 0

    def get_current_position(self, user_id=0):
        return list(self.pose)


class FakeFastLinRobot(FakeRobot):
    def __init__(self):
        super().__init__()
        self.node = type("Node", (), {"last_submitted_task_id": 42})()
        self.fast_lin_requests = []
        self.controlled_stops = []

    def move_fast_lin(self, position, **kwargs):
        self.fast_lin_requests.append((list(position), dict(kwargs)))
        return 0

    def controlled_stop(self, task_id, *, stop_duration_s=None):
        self.controlled_stops.append((task_id, stop_duration_s))
        return {
            "success": True,
            "stopped": True,
            "future_work_preserved": True,
            "state": "STOPPING",
        }


def request(supervisor, *, sensor_stale_timeout_s=0.5):
    return supervisor.start(
        servo={
            "axis": Value("Z", 3),
            "direction": Value("MINUS", -1),
            "vel": None,
            "acc": None,
            "frame": "user",
            "tool": 1,
            "user": 1,
            "linear_mm_s": 10.0,
            "angular_deg_s": None,
            "disable_collision_checking": True,
        },
        condition={
            "source": "servo_condition",
            "required_state": True,
            "require_fresh_transition": True,
        },
        boundary={
            "axis": "z",
            "operator": "less_or_equal",
            "value_mm": 50.0,
            "tool": 1,
            "user": 1,
        },
        timeout_s=1.0,
        sensor_stale_timeout_s=sensor_stale_timeout_s,
    )


def test_fast_lin_sensor_event_is_stopped_locally_by_ros_supervisor():
    robot = FakeFastLinRobot()
    supervisor = ConditionalServoSupervisor(
        lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100
    )
    supervisor.set_sensor_connected(True)
    started = supervisor.start(
        execution_mode="fast_lin",
        fast_lin={
            "position": [0.0, 0.0, 25.0, 0.0, 0.0, 0.0],
            "tool": 1,
            "user": 1,
            "vel": 10.0,
            "acc": 30.0,
            "trajectory_optimizer": "TOTG",
            "controlled_stop_duration_s": 0.05,
        },
        servo={
            "axis": Value("Z", 3),
            "direction": Value("MINUS", -1),
            "tool": 1,
            "user": 1,
        },
        condition={
            "source": "servo_condition",
            "required_state": True,
            "require_fresh_transition": True,
        },
        boundary={
            "axis": "z", "operator": "less_or_equal", "value_mm": 25.0,
            "tool": 1, "user": 1,
        },
        timeout_s=1.0,
    )
    assert started["state"] == "moving"
    assert started["execution_mode"] == "fast_lin"
    assert started["task_id"] == 42
    assert robot.fast_lin_requests
    assert supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "inactive",
        "stream_id": "stream-fast", "sequence": 1,
    })
    assert supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "active",
        "stream_id": "stream-fast", "sequence": 2,
    })
    assert robot.controlled_stops == [(42, 0.05)]
    assert supervisor.snapshot()["state"] == "awaiting_stationary"


def test_fresh_transition_stops_then_waits_for_stationary_samples():
    robot = FakeRobot()
    supervisor = ConditionalServoSupervisor(lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100)
    supervisor.set_sensor_connected(True)
    started = request(supervisor)
    assert started["state"] == "moving"
    assert started["servo"]["axis"] == "Z"
    assert started["servo"]["direction"] == "MINUS"
    json.dumps(started)

    assert supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "inactive",
        "stream_id": "stream-a", "sequence": 1,
    })
    assert robot.stopped == 0
    detected_ns = time.monotonic_ns()
    assert supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "active",
        "stream_id": "stream-a", "sequence": 2,
        "detected_monotonic_ns": detected_ns,
    })
    assert robot.stopped == 1
    assert supervisor.snapshot()["state"] == "awaiting_stationary"

    for _ in range(3):
        supervisor.observe_joint_state([0.0] * 6, [0.0] * 6)
    completed = supervisor.snapshot()
    assert completed["state"] == "condition_met"
    assert completed["active"] is False
    assert completed["sensor_detected_monotonic_ns"] == detected_ns
    assert completed["sensor_transport_latency_ms"] >= 0.0


def test_fast_lin_stop_action_success_completes_without_stationary_samples():
    robot = FakeFastLinRobot()
    supervisor = ConditionalServoSupervisor(
        lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100
    )
    supervisor.set_sensor_connected(True)
    supervisor.start(
        execution_mode="fast_lin",
        fast_lin={
            "position": [0.0, 0.0, 25.0, 0.0, 0.0, 0.0],
            "tool": 1,
            "user": 1,
            "vel": 10.0,
            "acc": 30.0,
            "controlled_stop_duration_s": 0.05,
        },
        servo={
            "axis": Value("Z", 3),
            "direction": Value("MINUS", -1),
            "tool": 1,
            "user": 1,
        },
        condition={
            "source": "servo_condition",
            "required_state": True,
            "require_fresh_transition": True,
        },
        boundary={
            "axis": "z", "operator": "less_or_equal", "value_mm": 25.0,
            "tool": 1, "user": 1,
        },
        timeout_s=1.0,
    )
    supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "inactive",
        "stream_id": "stream-fast", "sequence": 1,
    })
    supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "active",
        "stream_id": "stream-fast", "sequence": 2,
    })

    assert supervisor.snapshot()["state"] == "awaiting_stationary"
    assert supervisor.notify_controlled_stop_goal_succeeded()
    completed = supervisor.snapshot()
    assert completed["state"] == "condition_met"
    assert completed["stationary_samples"] == 0


def test_sensor_event_without_state_is_rejected():
    robot = FakeRobot()
    supervisor = ConditionalServoSupervisor(lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100)
    supervisor.set_sensor_connected(True)
    request(supervisor)

    assert not supervisor.accept_sensor_event({
        "sensor": "servo_condition", "stream_id": "stream-a", "sequence": 1,
    })
    assert robot.stopped == 0


def test_sensor_stale_window_starts_after_slow_servo_start_returns():
    class SlowStartRobot(FakeRobot):
        def start_servo_jog(self, **kwargs):
            time.sleep(0.15)
            return super().start_servo_jog(**kwargs)

    robot = SlowStartRobot()
    supervisor = ConditionalServoSupervisor(
        lambda: robot,
        logger=logging.getLogger("test"),
        monitor_rate_hz=100,
    )
    supervisor.set_sensor_connected(True)

    started = request(supervisor, sensor_stale_timeout_s=0.1)
    assert started["state"] == "moving"
    assert supervisor.accept_sensor_event({
        "sensor": "servo_condition",
        "state": "inactive",
        "stream_id": "stream-a",
        "sequence": 1,
    })
    assert robot.stopped == 0


def test_cached_active_does_not_trigger_before_fresh_inactive():
    robot = FakeRobot()
    supervisor = ConditionalServoSupervisor(lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100)
    supervisor.set_sensor_connected(True)
    request(supervisor)

    supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "active",
        "stream_id": "stream-a", "sequence": 1,
    })
    assert robot.stopped == 0
    supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "inactive",
        "stream_id": "stream-a", "sequence": 2,
    })
    supervisor.accept_sensor_event({
        "sensor": "servo_condition", "state": "active",
        "stream_id": "stream-a", "sequence": 3,
    })
    assert robot.stopped == 1


def test_boundary_is_stopped_locally():
    robot = FakeRobot()
    supervisor = ConditionalServoSupervisor(lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100)
    supervisor.set_sensor_connected(True)
    request(supervisor)
    robot.pose[2] = 49.0
    deadline = time.monotonic() + 0.3
    while robot.stopped == 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert robot.stopped == 1
    assert supervisor.snapshot()["state"] == "awaiting_stationary"
