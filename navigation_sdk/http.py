"""Bounded HTTP JSON transport. Mutations are never silently retried."""
import json
import urllib.error
import urllib.parse
import urllib.request
from .errors import NavigationError

class JsonHTTP:
    def __init__(self, base_url, *, api_key=None, timeout=20., maximum_bytes=2000000):
        url = urllib.parse.urlsplit(base_url)
        if url.scheme not in ("http", "https") or not url.hostname or url.username or url.password:
            raise ValueError("base_url must be an HTTP(S) URL without credentials")
        self.base = base_url.rstrip("/")
        self.api_key, self.timeout, self.maximum_bytes = api_key, timeout, maximum_bytes

    def request(self, method, path, data=None, headers=None):
        if not path.startswith("/") or path.startswith("//"):
            raise ValueError("An absolute local API path is required")
        h = {"Accept": "application/json", **(headers or {})}
        if self.api_key: h["Authorization"] = "Bearer " + self.api_key
        raw = None
        if data is not None:
            raw = json.dumps(data, allow_nan=False, ensure_ascii=False).encode()
            h["Content-Type"] = "application/json"
        req = urllib.request.Request(self.base + path, data=raw, headers=h, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as response:
                body = response.read(self.maximum_bytes + 1)
        except urllib.error.HTTPError as exc:
            body = exc.read(self.maximum_bytes + 1)
            try: error = json.loads(body).get("error", {})
            except (ValueError, UnicodeError): error = {}
            if not isinstance(error, dict): error = {"message": str(error)}
            raise NavigationError(error.get("code", "UPSTREAM_REJECTED"),
                                  error.get("message", "HTTP request rejected"),
                                  status=exc.code, details=error.get("details")) from None
        except (OSError, TimeoutError, urllib.error.URLError):
            raise NavigationError("UPSTREAM_UNCERTAIN" if method != "GET" else "UPSTREAM_UNAVAILABLE",
                                  "Request outcome unknown; do not replay a mutation automatically"
                                  if method != "GET" else "Backend cannot be reached", status=503) from None
        if len(body) > self.maximum_bytes:
            raise NavigationError("RESPONSE_TOO_LARGE", "Backend response exceeds configured bound", status=502)
        try: return json.loads(body)
        except (ValueError, UnicodeError):
            raise NavigationError("INVALID_RESPONSE", "Backend did not return JSON", status=502) from None
