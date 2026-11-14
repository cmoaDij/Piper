"""Best-effort, non-blocking diagnostics for live policy runs.

The control path must remain usable when the dashboard is closed or the
diagnostic consumer is slower than the policy.  Publishers therefore use a
non-blocking Unix datagram and silently drop diagnostic events on a full or
absent socket.  Control decisions never depend on this module.
"""
import json
import queue
import socket
import threading
from pathlib import Path


class DiagnosticPublisher:
    """Publish small JSON events without blocking the caller."""

    def __init__(self, path=None):
        self.path = None if path is None else str(Path(path))
        self.socket = None
        self.queue = queue.Queue(maxsize=2048)
        self.dropped = 0
        if self.path:
            self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.socket.setblocking(False)
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def send(self, event):
        if self.socket is None or not self.path:
            return False
        try:
            self.queue.put_nowait(event)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        while self.socket is not None:
            event = self.queue.get()
            if event is None:
                return
            try:
                payload = json.dumps(event, allow_nan=False, separators=(',', ':')).encode('utf-8')
                self.socket.sendto(payload, self.path)
            except (BlockingIOError, FileNotFoundError, ConnectionRefusedError, OSError,
                    TypeError, ValueError):
                self.dropped += 1

    def close(self):
        if self.socket is not None:
            self.queue.put(None)
            self.thread.join(timeout=1.0)
            self.socket.close()
            self.socket = None


class DiagnosticReceiver:
    """Receive controller events on a daemon thread for the local dashboard."""

    def __init__(self, path, callback):
        self.path = str(Path(path))
        self.callback = callback
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.bind(self.path)
        self.socket.settimeout(.2)
        self.closed = False
        import threading
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.closed:
            try:
                payload = self.socket.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self.callback(json.loads(payload.decode('utf-8')))
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
                continue

    def close(self):
        self.closed = True
        self.socket.close()
        self.thread.join(timeout=1.0)
        Path(self.path).unlink(missing_ok=True)


class AsyncJsonlWriter:
    """Bounded diagnostic writer; a slow disk drops diagnostics, never control."""

    def __init__(self, path, maxsize=4096):
        self.file = open(path, 'w', buffering=1, encoding='utf-8')
        self.queue = queue.Queue(maxsize=maxsize)
        self.dropped = 0
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def write(self, event):
        try:
            self.queue.put_nowait(event)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        try:
            while True:
                event = self.queue.get()
                if event is None:
                    break
                self.file.write(json.dumps(event, allow_nan=False) + '\n')
        except (OSError, TypeError, ValueError) as exc:
            self.error = str(exc)

    def close(self):
        self.queue.put(None)
        self.thread.join(timeout=5)
        self.file.close()
