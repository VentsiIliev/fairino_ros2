#!/usr/bin/env python3
"""Small timing context for one active /move/linear request."""

from __future__ import annotations

import itertools
from time import perf_counter

from motion.async_logging import info as async_info

_ATTR = "_move_linear_timing"
_COUNTER = itertools.count(1)


def _related_contexts(context):
    seen = set()
    stack = [context]
    while stack:
        item = stack.pop()
        if item is None or id(item) in seen:
            continue
        seen.add(id(item))
        yield item
        for attr in ("planner_context", "_node", "node"):
            try:
                related = getattr(item, attr, None)
            except Exception:
                related = None
            if related is not None:
                stack.append(related)


def _logger(context):
    for item in _related_contexts(context):
        getter = getattr(item, "get_logger", None)
        if callable(getter):
            try:
                return getter()
            except Exception:
                pass
    return None


def _active(context):
    for item in _related_contexts(context):
        timing = getattr(item, _ATTR, None)
        if isinstance(timing, dict):
            return timing
    return None


def _set_all(context, timing):
    for item in _related_contexts(context):
        try:
            setattr(item, _ATTR, timing)
        except Exception:
            pass


def begin(context, *, source="move_linear"):
    timing = {
        "id": next(_COUNTER),
        "source": source,
        "request_received_at": perf_counter(),
        "planning_started_at": None,
        "marks": {},
    }
    _set_all(context, timing)
    mark(context, "request_received", source=source)
    return timing


def ensure(context, *, source="move_linear"):
    timing = _active(context)
    if timing is None:
        return begin(context, source=source)
    return timing


def activate(context, timing):
    """Restore a request's timing context when a queued strategy starts."""
    if isinstance(timing, dict):
        _set_all(context, timing)


def elapsed_since_request(context):
    timing = _active(context)
    if not isinstance(timing, dict):
        return None
    received_at = timing.get("request_received_at")
    return perf_counter() - float(received_at) if received_at is not None else None


def mark(context, stage, **fields):
    timing = _active(context)
    now = perf_counter()
    elapsed_s = None
    request_id = "unknown"
    source = None
    if timing is not None:
        request_id = timing.get("id", "unknown")
        source = timing.get("source")
        received_at = timing.get("request_received_at")
        if received_at is not None:
            elapsed_s = now - float(received_at)
        if stage == "planning_start":
            timing["planning_started_at"] = now
        elif stage == "planning_done":
            planning_started_at = timing.get("planning_started_at")
            if planning_started_at is not None and "plan_elapsed_s" not in fields:
                fields["plan_elapsed_s"] = now - float(planning_started_at)
        marks = timing.setdefault("marks", {})
        previous_at = timing.get("last_mark_at")
        if previous_at is not None and "stage_elapsed_s" not in fields:
            fields["stage_elapsed_s"] = now - float(previous_at)
        marks[stage] = now
        timing["last_mark_at"] = now
    if source == "ordered_motion_chain":
        tag = "[ORDERED_CHAIN_TIMING]"
    elif source == "/move/fast_lin":
        tag = "[FAST_LIN_TIMING]"
    else:
        tag = "[MOVE_LIN_TIMING]"
    parts = [f"{tag} {stage}", f"request_id={request_id}"]
    if elapsed_s is not None:
        parts.append(f"elapsed_s={elapsed_s:.3f}")
    else:
        parts.append("elapsed_s=unknown")
    for key, value in fields.items():
        if isinstance(value, float):
            parts.append(f"{key}={value:.3f}")
        else:
            parts.append(f"{key}={value}")
    log = _logger(context)
    if log is not None:
        async_info(log, " ".join(parts))


def clear(context, *, force=False):
    timing = _active(context)
    if not force and isinstance(timing, dict) and timing.get("source") == "ordered_motion_chain":
        return
    _set_all(context, None)
