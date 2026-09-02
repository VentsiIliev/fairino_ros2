import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from motion.execution.motion_coordinator import MotionCoordinator


class FakeLogger:
    def warning(self, _message):
        pass


class FakeTrajectoryExecutor:
    def __init__(self):
        self.calls = []

    def send_path_stop_trajectory(self, *, preserve_future_work=False, stop_duration_s=None):
        self.calls.append((bool(preserve_future_work), stop_duration_s))
        return True


class FakeNode:
    def __init__(self):
        self.trajectory_executor = FakeTrajectoryExecutor()
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


class FakeQueue:
    def __init__(self, current_task_id):
        self.current_task_id = current_task_id

    def get_status(self):
        return {"current_task_id": self.current_task_id, "queue_size": 2}


def test_controlled_stop_rejects_wrong_task_without_stopping():
    node = FakeNode()
    coordinator = MotionCoordinator(node, FakeQueue(current_task_id=12))
    coordinator.active_controller_goal = object()

    result = coordinator.controlled_stop(expected_task_id=11)

    assert result["success"] is False
    assert result["state"] == "TASK_MISMATCH"
    assert node.trajectory_executor.calls == []


def test_controlled_stop_preserves_future_work_for_expected_task():
    node = FakeNode()
    coordinator = MotionCoordinator(node, FakeQueue(current_task_id=12))
    coordinator.active_controller_goal = object()
    original_generation = coordinator.plan_generation

    result = coordinator.controlled_stop(expected_task_id=12)

    assert result["success"] is True
    assert result["future_work_preserved"] is True
    assert node.trajectory_executor.calls == [(True, None)]
    assert coordinator.plan_generation == original_generation


def test_controlled_stop_passes_request_duration_to_trajectory_executor():
    node = FakeNode()
    coordinator = MotionCoordinator(node, FakeQueue(current_task_id=12))
    coordinator.active_controller_goal = object()

    result = coordinator.controlled_stop(expected_task_id=12, stop_duration_s=0.20)

    assert result["success"] is True
    assert node.trajectory_executor.calls == [(True, 0.20)]
