"""Two ROS callback workers without duplicate thread-pool submissions.

rclpy 7.1.3 can yield a Task again while it is already queued. Keep one
thread-pool Future per Task *invocation*, so an awaiting coroutine can still
resume after its previous invocation returns. Spin and shutdown on the main
thread, as in the collection and Quest entry points.
"""

from concurrent.futures import wait
from queue import Empty, SimpleQueue
from threading import Event, Lock

from rclpy.executors import (
    ConditionReachedException,
    Executor,
    ExternalShutdownException,
    MultiThreadedExecutor,
    ShutdownException,
    TimeoutException,
    TimeoutObject,
)


class TeleopExecutor(MultiThreadedExecutor):
    def __init__(self, *, context=None):
        super().__init__(num_threads=2, context=context)
        self._pending = {}
        self._pending_lock = Lock()
        self._completed = SimpleQueue()
        self._activity = Event()
        self._stopping = Event()
        self._submitted = 0
        self._duplicate_suppressed = 0
        self._pending_peak = 0

    def diagnostics(self):
        with self._pending_lock:
            return {
                "workers": 2,
                "submitted": self._submitted,
                "duplicate_suppressed": self._duplicate_suppressed,
                "in_flight": len(self._pending),
                "peak_in_flight": self._pending_peak,
            }

    def _finished(self, task, future):
        # Do not remove the registration here: the spin thread must retrieve
        # exceptions before this Task is allowed to resume or disappear.
        self._completed.put((task, future))
        self._activity.set()
        self.wake()

    def _reap(self):
        error = None
        while True:
            try:
                task, future = self._completed.get_nowait()
            except Empty:
                break
            with self._pending_lock:
                if self._pending.get(task) is future:
                    del self._pending[task]
            try:
                if not future.cancelled():
                    future.result()
                    if task.done():
                        task.result()
            except BaseException as exc:
                if error is None:
                    error = exc
        if error is not None:
            raise error

    def _spin_once_impl(self, timeout_sec=None, wait_condition=lambda: False):
        self._activity.clear()
        self._reap()
        if self._stopping.is_set():
            return
        try:
            handler, _, _ = self.wait_for_ready_callbacks(
                timeout_sec, None, wait_condition
            )
        except (TimeoutException, ShutdownException, ExternalShutdownException,
                ConditionReachedException):
            self._reap()
            return
        self._reap()
        if handler.done():
            handler.result()
            return
        if handler.cancelled():
            return
        with self._pending_lock:
            if self._stopping.is_set():
                return
            duplicate = handler in self._pending
            if duplicate:
                self._duplicate_suppressed += 1
            else:
                # Register under the same lock as shutdown's snapshot. The
                # completion callback is attached outside this non-reentrant
                # lock because add_done_callback may run immediately.
                future = self._executor.submit(handler)
                self._pending[handler] = future
                self._submitted += 1
                self._pending_peak = max(self._pending_peak, len(self._pending))
        if duplicate:
            # Bound the pause so another ready callback (especially B) can be
            # dispatched even while a long homing callback occupies one worker.
            timeout = timeout_sec.timeout if isinstance(timeout_sec, TimeoutObject) else timeout_sec
            delay = 0.0002 if timeout is None or timeout < 0 else min(0.0002, timeout)
            self._activity.wait(delay)
        else:
            future.add_done_callback(lambda done: self._finished(handler, done))

    def shutdown(self, timeout_sec=None):
        """Stop submitting; join workers before destroying ROS wait handles.

        A timeout returns False with ROS handles intact. The caller must first
        unblock its callbacks and call shutdown again before destroying nodes.
        """
        self._stopping.set()
        self._is_shutdown = True
        self._activity.set()
        self.wake()
        with self._pending_lock:
            futures = list(self._pending.values())
        self._executor.shutdown(wait=False, cancel_futures=True)
        timeout = None if timeout_sec is None or timeout_sec < 0 else timeout_sec
        # ThreadPoolExecutor.cancel_futures cancels queued Futures without a
        # worker notifying concurrent.futures.wait; exclude those already done.
        _, unfinished = wait([future for future in futures if not future.done()], timeout=timeout)
        if unfinished:
            return False
        self._executor.shutdown(wait=True)
        try:
            self._reap()
        finally:
            Executor.shutdown(self, timeout_sec=0)
        return True
