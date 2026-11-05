"""The same business client for the P2 and WATER backend providers."""
import time
import urllib.parse
from .http import JsonHTTP
from .errors import NavigationError

class NavigationClient:
    def __init__(self, base_url="http://127.0.0.1:18940", *, api_key=None, timeout=20.):
        self.http = JsonHTTP(base_url, api_key=api_key, timeout=timeout)

    def capabilities(self): return self.http.request("GET", "/v1/capabilities")
    def status(self): return self.http.request("GET", "/v1/status")
    def maps(self): return self.http.request("GET", "/v1/maps")
    def markers(self): return self.http.request("GET", "/v1/markers")
    def routes(self): return self.http.request("GET", "/v1/routes")

    def move(self, *, marker=None, x=None, y=None, theta=None, idempotency_key=None):
        if marker is not None and any(v is not None for v in (x,y,theta)):
            raise NavigationError("INVALID_ARGUMENT","Specify marker OR pose",status=400)
        target = {"marker": marker} if marker is not None else {"x": x, "y": y, "theta": theta}
        return self._task({"kind": "move", "target": target}, idempotency_key)

    def route(self, route_id, *, idempotency_key=None):
        return self._task({"kind": "route", "route_id": route_id}, idempotency_key)

    def visit(self, marker, *, idempotency_key=None):
        """Navigate, face the configured direction, introduce, dwell/wait."""
        return self._task({"kind":"visit", "marker":marker}, idempotency_key)

    def forward(self, meters, *, idempotency_key=None):
        """Target displaced in the measured body-forward direction; metres, not seconds."""
        return self._task(dict(kind='forward',meters=meters),idempotency_key)

    def rotate(self, radians, *, idempotency_key=None):
        """Signed accumulated angle: positive left; 2*pi is a full revolution."""
        return self._task(dict(kind='rotate',radians=radians),idempotency_key)

    def tour(self, markers, *, idempotency_key=None):
        """One temporary ordered visit mission, preserving saved introduction/wait settings."""
        return self._task(dict(kind='tour',markers=markers),idempotency_key)

    def _task(self, body, key):
        return self.http.request("POST", "/v1/tasks", body,
                                 {"Idempotency-Key": key} if key else None)

    def task(self, task_id):
        return self.http.request("GET", "/v1/tasks/" + urllib.parse.quote(task_id, safe=""))

    def _control(self, task_id, operation):
        return self.http.request("POST", "/v1/tasks/" + urllib.parse.quote(task_id, safe="") + "/" + operation, {})

    def cancel(self, task_id): return self._control(task_id, "cancel")
    def pause(self, task_id): return self._control(task_id, "pause")
    def resume(self, task_id): return self._control(task_id, "resume")
    def continue_visit(self, task_id): return self._control(task_id, "continue")
    def stop(self): return self.http.request("POST", "/v1/stop", {})

    def save_marker(self, marker):
        return self.http.request("POST", "/v1/markers", marker)

    def delete_marker(self, name):
        return self.http.request("DELETE", "/v1/markers/" + urllib.parse.quote(name, safe=""))

    def wait(self, task_id, *, timeout=120., interval=.5):
        deadline = time.monotonic() + timeout
        while True:
            task = self.task(task_id)
            if task["state"] in ("succeeded", "failed", "cancelled", "interrupted", "unknown"):
                return task
            if time.monotonic() >= deadline:
                raise NavigationError("WAIT_TIMEOUT", "Waiting timed out; the task has NOT been cancelled",
                                      status=408, details={"task_id": task_id})
            time.sleep(min(interval, max(0., deadline-time.monotonic())))
