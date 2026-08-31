"""Non-blocking INFO logging for latency-sensitive motion paths."""

from __future__ import annotations

from queue import Full, Queue
from threading import Lock, Thread


_QUEUE = Queue(maxsize=2048)
_START_LOCK = Lock()
_WORKER = None


def _run() -> None:
    while True:
        logger, message = _QUEUE.get()
        try:
            logger.info(message)
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


def info(logger, message: str) -> bool:
    """Queue one INFO record without waiting for console/file output."""
    if logger is None:
        return False
    _ensure_worker()
    try:
        _QUEUE.put_nowait((logger, str(message)))
        return True
    except Full:
        # Dropping diagnostic INFO is preferable to delaying controller dispatch.
        return False
