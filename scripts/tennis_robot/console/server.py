"""HTTP layer — ConsoleServer (DI host) + ControlPanelHandler (thin controller).

The handler does only HTTP work: parse the request, validate inputs, route to a
ConsoleApp use-case, and format the response. It owns no state and no business
logic — the app is reached through ``self.server.app`` (injected once into the
ConsoleServer), so there are no class-level singletons.
"""

from __future__ import annotations

import gzip
import json
import math
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from .app import CommandRejected, ConsoleApp
from .config import STATIC_CONTENT_TYPES


_JSON_GZIP_MIN_BYTES = 1024


def _accepts_gzip(header: str) -> bool:
    """Return whether an HTTP Accept-Encoding value permits gzip."""

    for item in header.lower().split(","):
        fields = [field.strip() for field in item.split(";")]
        if not fields or fields[0] not in {"gzip", "*"}:
            continue
        quality = 1.0
        for parameter in fields[1:]:
            if parameter.startswith("q="):
                try:
                    quality = float(parameter[2:])
                except ValueError:
                    quality = 0.0
        if quality > 0.0:
            return True
    return False


def encode_json_response(data: dict, accept_encoding: str) -> tuple[bytes, str | None]:
    """Encode compact JSON and optionally gzip it for transport.

    Compression changes only the HTTP representation; the decoded JSON
    contract consumed by the UI is byte-for-byte equivalent as data.
    """

    payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
    if len(payload) >= _JSON_GZIP_MIN_BYTES and _accepts_gzip(accept_encoding):
        return gzip.compress(payload, compresslevel=1), "gzip"
    return payload, None


class ConsoleServer(ThreadingHTTPServer):
    """ThreadingHTTPServer that carries the ConsoleApp for the handler."""

    # Allow quick restarts without waiting out TIME_WAIT on the listen socket.
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], app: ConsoleApp) -> None:
        self.app = app
        super().__init__(address, ControlPanelHandler)


class ControlPanelHandler(BaseHTTPRequestHandler):
    # GET endpoints that simply serialize an app read to JSON.
    GET_JSON_ROUTES = {
        "/api/status": "command_status",
        "/api/robot-status": "robot_status",
        "/api/sensors": "sensors",
        "/api/history": "history",
        "/api/diagnostics": "build_diagnostics",
        "/api/path": "path_points",
        "/api/webcam/frame": "webcam_frame",
        "/api/vendors": "vendors",
        "/api/surveys": "surveys",
        "/api/survey-archive": "survey_archive",
        "/api/collector": "collector_status",
        "/api/throwing": "throwing_status",
    }
    # Quiet access-log paths (polled frequently by the UI).
    _QUIET_PATHS = {
        "/api/status", "/api/robot-status", "/api/sensors", "/api/diagnostics",
        "/api/webcam/frame", "/api/vendors", "/api/surveys", "/api/survey-archive",
        "/favicon.ico",
    }
    _NAV_STATUS_BY_KIND = {
        "out_of_bounds": HTTPStatus.BAD_REQUEST,
        "preflight_timeout": HTTPStatus.SERVICE_UNAVAILABLE,
        "not_ready": HTTPStatus.SERVICE_UNAVAILABLE,
        "timeout": HTTPStatus.GATEWAY_TIMEOUT,
    }

    @property
    def app(self) -> ConsoleApp:
        return self.server.app  # type: ignore[attr-defined]

    # ------------------------------------------------------------------
    # routing
    # ------------------------------------------------------------------
    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path in {"/", "/index.html"}:
            self._send_html(self._load_html())
            return
        if path.startswith("/static/"):
            self._send_static(path[len("/static/"):])
            return
        if path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
            return
        if path == "/api/diagnostics":
            view = parse_qs(parsed.query).get("view", ["dashboard"])[0]
            self._send_json(self.app.build_diagnostics(view=view))
            return
        name = self.GET_JSON_ROUTES.get(path)
        if name is not None:
            self._send_json(getattr(self.app, name)())
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/vendors":
            self._post_vendors()
            return
        if path == "/api/path/clear":
            self.app.clear_path()
            self._send_json({"ok": True})
            return
        if path == "/api/nav-test":
            self._post_nav_test()
            return
        if path == "/api/nav-test/cancel":
            self._send_json(self.app.nav_cancel())
            return
        if path == "/api/collector":
            data = self._read_json_body()
            if not isinstance(data, dict):
                return
            self._send_json(self.app.collector_control(str(data.get("action", ""))))
            return
        if path == "/api/throwing":
            data = self._read_json_body()
            if not isinstance(data, dict):
                return
            self._send_json(self.app.throwing_command(str(data.get("action", "")), data))
            return
        if path in {"/command", "/api/command"}:
            self._post_command(path)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    # ------------------------------------------------------------------
    # POST handlers (parse -> app -> format)
    # ------------------------------------------------------------------
    def _post_vendors(self) -> None:
        data = self._read_json_body()
        if data is None:
            return
        if not isinstance(data, dict):
            self.send_error(HTTPStatus.BAD_REQUEST, "Expected JSON object")
            return
        self.app.save_vendors(data)
        self._send_json({"ok": True})

    def _post_command(self, path: str) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        mode = parse_qs(body).get("mode", ["idle"])[0]
        try:
            command = self.app.set_command(mode)
        except CommandRejected as exc:
            # Another subsystem owns the machine. The browser form path still
            # redirects; only the JSON API surfaces the conflict status.
            if path == "/api/command":
                self._send_json({"ok": False, "message": str(exc)},
                                status=HTTPStatus.CONFLICT)
                return
            command = None
        if path == "/api/command":
            self._send_json(command.to_mapping())
            return
        self.send_response(HTTPStatus.SEE_OTHER)
        self.send_header("Location", "/")
        self.end_headers()

    def _post_nav_test(self) -> None:
        data = self._read_json_body()
        if data is None:
            return
        if not isinstance(data, dict):
            self.send_error(HTTPStatus.BAD_REQUEST, "Expected JSON object")
            return
        try:
            x_m = float(data["x_m"])
            y_m = float(data["y_m"])
            yaw_rad = float(data.get("yaw_rad", 0.0))
        except (KeyError, TypeError, ValueError):
            self.send_error(HTTPStatus.BAD_REQUEST, "Expected numeric x_m, y_m, yaw_rad")
            return
        if not all(math.isfinite(v) for v in (x_m, y_m, yaw_rad)):
            self.send_error(HTTPStatus.BAD_REQUEST, "Pose values must be finite")
            return

        outcome = self.app.nav_test(x_m, y_m, yaw_rad)
        if outcome.kind == "sent":
            status = HTTPStatus.OK if outcome.payload.get("returncode") == 0 else HTTPStatus.BAD_GATEWAY
        else:
            status = self._NAV_STATUS_BY_KIND.get(outcome.kind, HTTPStatus.BAD_REQUEST)
        self._send_json(outcome.payload, status=status)

    # ------------------------------------------------------------------
    # request/response helpers
    # ------------------------------------------------------------------
    def _read_json_body(self):
        """Return the parsed JSON body, or None after sending a 400."""
        length = int(self.headers.get("Content-Length", "0"))
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, ValueError):
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid JSON")
            return None

    def _load_html(self) -> str:
        try:
            return self.app.config.html_path.read_text(encoding="utf-8")
        except OSError:
            return "<!doctype html><html><body><h1>Tennis Robot Console</h1><p>control_panel.html missing.</p></body></html>"

    def _send_html(self, html: str) -> None:
        payload = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _send_static(self, rel_path: str) -> None:
        static_dir = self.app.config.static_dir
        candidate = (static_dir / rel_path).resolve()
        try:
            candidate.relative_to(static_dir.resolve())
        except ValueError:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = STATIC_CONTENT_TYPES.get(candidate.suffix.lower(), "application/octet-stream")
        try:
            payload = candidate.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_json(self, data: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        payload, content_encoding = encode_json_response(
            data, self.headers.get("Accept-Encoding", "")
        )
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Vary", "Accept-Encoding")
        if content_encoding:
            self.send_header("Content-Encoding", content_encoding)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format: str, *args: object) -> None:
        if urlparse(self.path).path in self._QUIET_PATHS:
            return
        print(f"{self.address_string()} - {format % args}")
