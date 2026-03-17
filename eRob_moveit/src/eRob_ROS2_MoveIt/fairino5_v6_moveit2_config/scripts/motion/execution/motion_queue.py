#!/usr/bin/env python3
"""Motion queue for sequential trajectory execution."""

from collections import deque
from threading import Lock
import time
import config


class MotionQueue:
    def __init__(self, max_size=config.MOTION_QUEUE_MAX_SIZE):
        self.queue = deque(maxlen=max_size)
        self.max_size = max_size
        self.lock = Lock()
        self.current_task = None
        self.task_id_counter = 0
        self.last_completed_task_id = None
        self.last_completed_result = None

    def allocate_task_id(self):
        with self.lock:
            self.task_id_counter += 1
            return self.task_id_counter

    def start_immediate_task(self, task_id):
        with self.lock:
            self.current_task = {
                'id': task_id,
                'function': None,
                'args': [],
                'kwargs': {},
                'submitted_at': time.time(),
            }

    def submit(self, task_function, task_args=None, task_kwargs=None):
        """
        Submit a motion task to the queue.

        Args:
            task_function: Function to execute (e.g., send_path_cartesian)
            task_args: Positional arguments for the function
            task_kwargs: Keyword arguments for the function

        Returns:
            tuple: (task_id, queue_position) if queued successfully
            int: -5 if queue is full
        """
        if task_args is None:
            task_args = []
        if task_kwargs is None:
            task_kwargs = {}

        with self.lock:
            if len(self.queue) >= self.max_size:
                return -5  # Queue full

            self.task_id_counter += 1
            task = {
                'id': self.task_id_counter,
                'function': task_function,
                'args': task_args,
                'kwargs': task_kwargs,
                'submitted_at': time.time()
            }

            self.queue.append(task)
            position = len(self.queue)

            return (self.task_id_counter, position)

    def get_next_task(self):
        """Get the next task from the queue (non-blocking)."""
        with self.lock:
            if len(self.queue) > 0:
                task = self.queue.popleft()
                self.current_task = task
                return task
            return None

    def clear(self):
        """Clear all pending tasks (not current executing task)."""
        with self.lock:
            size = len(self.queue)
            self.queue.clear()
            return size

    def get_status(self):
        """Get queue status."""
        with self.lock:
            return {
                'queue_size': len(self.queue),
                'current_task_id': self.current_task['id'] if self.current_task else None,
                'last_completed_task_id': self.last_completed_task_id,
                'last_completed_result': self.last_completed_result,
                'max_size': self.max_size
            }

    def mark_current_complete(self, result=0):
        """Mark current task as complete and record its result."""
        with self.lock:
            if self.current_task is not None:
                self.last_completed_task_id = self.current_task['id']
                self.last_completed_result = result
            self.current_task = None
