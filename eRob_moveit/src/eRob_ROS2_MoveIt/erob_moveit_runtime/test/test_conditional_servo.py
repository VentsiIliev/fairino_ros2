import logging
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


def request(supervisor):
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
        sensor_stale_timeout_s=0.5,
    )


def test_fresh_transition_stops_then_waits_for_stationary_samples():
    robot = FakeRobot()
    supervisor = ConditionalServoSupervisor(lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100)
    supervisor.set_sensor_connected(True)
    started = request(supervisor)
    assert started["state"] == "moving"

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


def test_sensor_event_without_state_is_rejected():
    robot = FakeRobot()
    supervisor = ConditionalServoSupervisor(lambda: robot, logger=logging.getLogger("test"), monitor_rate_hz=100)
    supervisor.set_sensor_connected(True)
    request(supervisor)

    assert not supervisor.accept_sensor_event({
        "sensor": "servo_condition", "stream_id": "stream-a", "sequence": 1,
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
