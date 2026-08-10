#!/usr/bin/env python3
"""Observation state for the legacy ordered-chain scheduler bridge."""

from __future__ import annotations

from threading import Lock
from typing import Any

from motion.scheduling.status_adapter import ordered_chain_preplanned_snapshot


class OrderedChainObservation:
    """Thread-safe planned-segment observation used by compatibility status."""

    def __init__(self):
        self._lock = Lock()
        self._planned_by_index: dict[int, dict[str, Any]] = {}

    def mark_planned(self, index: int, planned_segment: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._planned_by_index[int(index)] = {
                "label": planned_segment.get("label"),
                "type": planned_segment.get("type"),
            }
            return ordered_chain_preplanned_snapshot(self._planned_by_index, current_index=0)

    def mark_consumed(self, index: int) -> None:
        with self._lock:
            self._planned_by_index.pop(int(index), None)

    def preplanned_snapshot(self, current_index: int = 0) -> dict[str, Any]:
        with self._lock:
            return ordered_chain_preplanned_snapshot(
                self._planned_by_index,
                current_index=int(current_index),
            )


__all__ = ["OrderedChainObservation"]
