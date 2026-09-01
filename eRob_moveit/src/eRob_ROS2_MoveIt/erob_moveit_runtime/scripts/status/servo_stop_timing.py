"""Low-overhead, observability-only timing for ServoJog stops."""

from __future__ import annotations

import logging
import math
import threading
import time


# Reuse the REST server logger so physical-stop events reach the same log file
# as the HTTP receive boundary.
_logger = logging.getLogger("erob_moveit_rest_server")
_lock = threading.Lock()
_pending: dict[str, dict] = {}

VELOCITY_THRESHOLD_RAD_S = 0.01
REQUIRED_CONSECUTIVE_SAMPLES = 3
TRACE_EXPIRY_NS = 30_000_000_000


def register_stop(trace_id: str, timing: dict) -> None:
    """Register a stop without waiting for its measured outcome."""
    now_ns = time.monotonic_ns()
    record = {
        "trace_id": trace_id,
        "timing": dict(timing),
        "registered_ns": now_ns,
        "consecutive_stopped_samples": 0,
    }
    with _lock:
        _pending[trace_id] = record
    _log_event(trace_id, "stop_measurement_armed", now_ns, record["timing"])


def observe_joint_velocity(velocities) -> None:
    """Consume the existing /joint_velocity callback; never blocks motion."""
    try:
        values = [float(value) for value in velocities]
    except (TypeError, ValueError):
        return
    if not values or not all(math.isfinite(value) for value in values):
        return

    observed_ns = time.monotonic_ns()
    max_abs_velocity = max(abs(value) for value in values)
    completed = []
    expired = []
    with _lock:
        for trace_id, record in list(_pending.items()):
            if observed_ns - record["registered_ns"] > TRACE_EXPIRY_NS:
                expired.append((trace_id, record))
                del _pending[trace_id]
                continue
            if max_abs_velocity <= VELOCITY_THRESHOLD_RAD_S:
                record["consecutive_stopped_samples"] += 1
            else:
                record["consecutive_stopped_samples"] = 0
            if record["consecutive_stopped_samples"] >= REQUIRED_CONSECUTIVE_SAMPLES:
                completed.append((trace_id, record))
                del _pending[trace_id]

    for trace_id, record in completed:
        timing = dict(record["timing"])
        timing["physical_stop_ns"] = observed_ns
        timing["max_abs_joint_velocity_rad_s"] = max_abs_velocity
        timing["stop_velocity_threshold_rad_s"] = VELOCITY_THRESHOLD_RAD_S
        timing["stop_consecutive_samples"] = REQUIRED_CONSECUTIVE_SAMPLES
        _log_event(trace_id, "physical_stop", observed_ns, timing)
    for trace_id, record in expired:
        _log_event(trace_id, "physical_stop_timeout", observed_ns, record["timing"])


def _log_event(trace_id: str, event: str, event_ns: int, timing: dict) -> None:
    receive_ns = timing.get("ros_http_route_enter_ns")
    since_receive_ms = None
    if isinstance(receive_ns, int):
        since_receive_ms = (event_ns - receive_ns) / 1_000_000.0
    _logger.info(
        "[SERVO_STOP_TIMING] trace_id=%s event=%s monotonic_ns=%d "
        "since_ros_receive_ms=%s details=%s",
        trace_id,
        event,
        event_ns,
        f"{since_receive_ms:.3f}" if since_receive_ms is not None else "na",
        timing,
    )
