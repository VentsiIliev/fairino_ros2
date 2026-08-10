#!/usr/bin/env python3
"""Legacy ordered-chain planned queue wrapper."""

from __future__ import annotations

from queue import Queue
from typing import Any


class OrderedPlannedQueue:
    """Small wrapper around the legacy planned queue tuple protocol."""

    def __init__(self):
        self._queue = Queue()

    def put_planned(self, index: int, planned_segment: dict[str, Any]) -> None:
        self._queue.put((int(index), planned_segment, None))

    def put_done(self) -> None:
        self._queue.put((None, None, None))

    def put_error(self, exc: BaseException) -> None:
        self._queue.put((None, None, exc))

    def get(self, timeout: float):
        return self._queue.get(timeout=timeout)


__all__ = ["OrderedPlannedQueue"]
