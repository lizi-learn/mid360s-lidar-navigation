"""Bounded private planning evidence; never participates in motion decisions."""
import base64
import gzip
import json
import os
from pathlib import Path
import queue
import threading
import time


def _encode(value):
    if isinstance(value, bytes):
        return {'encoding': 'base64-u8', 'data': base64.b64encode(value).decode('ascii')}
    raise TypeError(type(value).__name__)


class FlightRecorder:
    def __init__(self, path, limit_bytes=16*1024*1024, queue_size=8):
        self.path = Path(path)
        self.limit_bytes = limit_bytes
        self.queue = queue.Queue(maxsize=queue_size)
        self.dropped = 0
        self.errors = 0
        self.written = 0
        self.rotations = 0
        self.closed = False
        self.worker = threading.Thread(target=self._run, daemon=True, name='nav-private-evidence')
        self.worker.start()

    def submit(self, kind, data):
        if self.closed:
            return False
        row = dict(kind=kind, received_monotonic_ns=time.monotonic_ns(),
                   received_wall_ns=time.time_ns(), data=data)
        try:
            self.queue.put_nowait(row)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def status(self):
        return dict(written=self.written, dropped=self.dropped, errors=self.errors,
                    rotations=self.rotations, pending=self.queue.qsize())

    def _run(self):
        while not self.closed or not self.queue.empty():
            try:
                row = self.queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                # JSON/base64/compression/disk I/O stay off the ROS callback.
                encoded = json.dumps(row, default=_encode, allow_nan=False,
                                     separators=(',', ':')).encode()+b'\n'
                self.path.parent.mkdir(parents=True, exist_ok=True)
                if self.path.exists() and self.path.stat().st_size >= self.limit_bytes:
                    os.replace(self.path, str(self.path)+'.1')
                    self.rotations += 1
                fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
                with os.fdopen(fd, 'ab') as raw:
                    with gzip.GzipFile(fileobj=raw, mode='ab') as stream:
                        stream.write(encoded)
                self.written += 1
            except (OSError, ValueError, TypeError):
                self.errors += 1
            finally:
                self.queue.task_done()

    def close(self, timeout=1.):
        self.closed = True
        self.worker.join(timeout)
