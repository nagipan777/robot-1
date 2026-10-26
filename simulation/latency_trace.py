"""Low-overhead JSONL timestamps for the Roomba latency investigation.

Tracing is disabled when ``path`` is empty.  The monotonic clock is used for
latency calculations; wall time is included only to make separate logs easy
to align for humans.
"""

import json
import os
import threading
import time


class LatencyTrace:
    def __init__(self, path="", component="unknown"):
        self.path = str(path or "")
        self.component = component
        self._fd = None
        self._lock = threading.Lock()
        if self.path:
            self._fd = os.open(
                self.path,
                os.O_APPEND | os.O_CREAT | os.O_WRONLY,
                0o644,
            )

    @property
    def enabled(self):
        return self._fd is not None

    def record(self, stage, **fields):
        if self._fd is None:
            return
        entry = {
            "monotonic_ns": time.monotonic_ns(),
            "wall_time_ns": time.time_ns(),
            "component": self.component,
            "stage": stage,
        }
        entry.update(fields)
        encoded = (json.dumps(entry, sort_keys=True) + "\n").encode("utf-8")
        with self._lock:
            os.write(self._fd, encoded)

    def close(self):
        with self._lock:
            if self._fd is not None:
                os.close(self._fd)
                self._fd = None

