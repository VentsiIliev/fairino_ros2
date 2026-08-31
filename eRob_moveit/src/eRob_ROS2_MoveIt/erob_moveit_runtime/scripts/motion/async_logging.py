"""Non-blocking INFO logging for latency-sensitive motion paths.

The event timestamp is captured by the producer, before queueing.  ROS still
prints its normal emission timestamp, while ``event_ts`` identifies when the
motion thread actually generated the record.
"""

from __future__ import annotations

from queue import Full, Queue
from threading import Lock, Thread
from time import monotonic, time_ns


_QUEUE = Queue(maxsize=8192)
_START_LOCK = Lock()
_WORKER = None


def _run() -> None:
    while True:
        logger, event_time_ns, message, args, style, kwargs = _QUEUE.get()
        try:
            if args and style == 'format':
                message = message.format(*args)
            elif args and style == 'percent':
                message = message % args
            logger.info(
                f'[event_ts={event_time_ns / 1_000_000_000.0:.9f}] {message}',
                **kwargs,
            )
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
    return _enqueue(logger, message, (), 'plain', {}, timestamp_ns)


def infof(logger, message: str, *args, timestamp_ns: int | None = None) -> bool:
    """Queue an INFO format string; interpolation happens on the worker.

    ``timestamp_ns`` can be supplied by a caller that already captured an event
    time. Otherwise it is captured immediately, before the queue operation.
    """
    return _enqueue(logger, message, args, 'format', {}, timestamp_ns)


def _enqueue(logger, message, args, style, kwargs, timestamp_ns) -> bool:
    if logger is None:
        return False
    _ensure_worker()
    raw_logger = getattr(logger, 'sync_logger', logger)
    captured_ns = time_ns() if timestamp_ns is None else int(timestamp_ns)
    try:
        _QUEUE.put_nowait((
            raw_logger,
            captured_ns,
            str(message),
            tuple(args),
            str(style),
            dict(kwargs),
        ))
        return True
    except Full:
        # Dropping diagnostic INFO is preferable to delaying controller dispatch.
        return False


class AsyncInfoLogger:
    """Transparent logger proxy that makes only INFO records asynchronous."""

    def __init__(self, sync_logger):
        self.sync_logger = sync_logger

    def info(self, message, *args, **kwargs):
        return _enqueue(
            self.sync_logger,
            message,
            args,
            'percent' if args else 'plain',
            kwargs,
            None,
        )

    def __getattr__(self, name):
        return getattr(self.sync_logger, name)


def wrap(logger):
    """Return an INFO-async proxy, preserving all other logger operations."""
    if logger is None or isinstance(logger, AsyncInfoLogger):
        return logger
    return AsyncInfoLogger(logger)


def flush(timeout_s: float = 1.0) -> bool:
    """Wait briefly for queued records; intended for orderly node shutdown."""
    deadline = monotonic() + max(0.0, float(timeout_s))
    with _QUEUE.all_tasks_done:
        while _QUEUE.unfinished_tasks:
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                return False
            _QUEUE.all_tasks_done.wait(remaining)
    return True


# Pay thread-start cost during module import, not on the first motion event.
_ensure_worker()
