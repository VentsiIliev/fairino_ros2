"""Non-blocking INFO logging for latency-sensitive motion paths.

The event timestamp is captured by the producer, before queueing.  ROS still
prints its normal emission timestamp, while ``event_ts`` identifies when the
motion thread actually generated the record.
"""

from __future__ import annotations

from queue import Full, Queue
from threading import Lock, Thread
from time import time_ns


_QUEUE = Queue(maxsize=8192)
_START_LOCK = Lock()
_WORKER = None


def _run() -> None:
    while True:
        logger, event_time_ns, message, args = _QUEUE.get()
        try:
            if args:
                message = message.format(*args)
            logger.info(f'[event_ts={event_time_ns / 1_000_000_000.0:.9f}] {message}')
        except Exception:
            # Logging must never interfere with motion execution.
            pass
        finally:
            _QUEUE.task_done()


def _ensure_worker() -> None:
    global _WORKER
    if _WORKER is not None and _WORKER.is_alive():
        return
    with _START_LOCK:
        if _WORKER is None or not _WORKER.is_alive():
            _WORKER = Thread(
                target=_run,
                name="zeroerr-async-info-logger",
                daemon=True,
            )
            _WORKER.start()


def info(logger, message: str, *, timestamp_ns: int | None = None) -> bool:
    """Queue a preformatted INFO record without waiting for output."""
    return infof(logger, message, timestamp_ns=timestamp_ns)


def infof(logger, message: str, *args, timestamp_ns: int | None = None) -> bool:
    """Queue an INFO format string; interpolation happens on the worker.

    ``timestamp_ns`` can be supplied by a caller that already captured an event
    time. Otherwise it is captured immediately, before the queue operation.
    """
    if logger is None:
        return False
    _ensure_worker()
    captured_ns = time_ns() if timestamp_ns is None else int(timestamp_ns)
    try:
        _QUEUE.put_nowait((logger, captured_ns, str(message), tuple(args)))
        return True
    except Full:
        # Dropping diagnostic INFO is preferable to delaying controller dispatch.
        return False


# Pay thread-start cost during module import, not on the first motion event.
_ensure_worker()
