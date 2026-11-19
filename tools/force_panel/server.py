"""Loopback-only HTTP API and static UI for the experiment manager."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import signal
import threading
from urllib.parse import urlsplit
from .jobs import JobManager

JOB_ID = re.compile(r"^\d{8}-\d{6}-[a-f0-9]{8}$")
STATIC = Path(__file__).parent / "static"


def make_handler(manager, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, status, data, content_type="application/json", download=None):
            if isinstance(data, (dict, list)):
                data = json.dumps(data, allow_nan=False).encode()
            elif isinstance(data, str):
                data = data.encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self'; connect-src 'self'; object-src 'none'; frame-ancestors 'none'",
            )
            if download:
                self.send_header(
                    "Content-Disposition", f'attachment; filename="{download}"'
                )
            self.end_headers()
            self.wfile.write(data)

        def host_ok(self):
            port = self.server.server_address[1]
            return self.headers.get("Host") in (
                f"127.0.0.1:{port}",
                f"localhost:{port}",
            )

        def do_GET(self):
            if not self.host_ok():
                return self.respond(403, {"error": "Local host required"})
            path = urlsplit(self.path).path
            if path in ("/", "/index.html"):
                html = (
                    (STATIC / "index.html")
                    .read_text()
                    .replace("__PANEL_TOKEN__", token)
                )
                return self.respond(200, html, "text/html; charset=utf-8")
            if path in ("/app.css", "/app.js"):
                name = path[1:]
                kind = "text/css" if name.endswith(".css") else "text/javascript"
                return self.respond(
                    200, (STATIC / name).read_bytes(), kind + "; charset=utf-8"
                )
            if path == "/api/state":
                return self.respond(200, manager.snapshot())
            match = re.fullmatch(r"/api/jobs/([^/]+)/(plot|summary)", path)
            if match and JOB_ID.fullmatch(match[1]):
                job_id, kind = match.groups()
                try:
                    folder = manager.output(job_id)
                except KeyError:
                    return self.respond(404, {"error": "Unknown experiment"})
                file = folder / ("response.png" if kind == "plot" else "summary.json")
                if not file.is_file():
                    return self.respond(404, {"error": "Artifact is not ready"})
                return self.respond(
                    200,
                    file.read_bytes(),
                    "image/png" if kind == "plot" else "application/json",
                    f"{job_id}-summary.json" if kind == "summary" else None,
                )
            return self.respond(404, {"error": "Not found"})

        def do_POST(self):
            port = self.server.server_address[1]
            origin = self.headers.get("Origin")
            if (
                not self.host_ok()
                or self.headers.get("X-Force-Panel-Token") != token
                or origin
                not in (None, f"http://127.0.0.1:{port}", f"http://localhost:{port}")
            ):
                return self.respond(403, {"error": "Local panel request required"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 8192:
                    raise ValueError("Invalid request size")
                if (
                    self.headers.get("Content-Type", "").split(";")[0]
                    != "application/json"
                ):
                    raise ValueError("Expected JSON")
                payload = json.loads(self.rfile.read(size))
                path = urlsplit(self.path).path
                if path == "/api/run":
                    return self.respond(202, {"id": manager.start(payload)})
                if path == "/api/reaudit":
                    if not isinstance(payload, dict):
                        raise ValueError("Expected a saved experiment ID")
                    return self.respond(202, {"id": manager.reaudit(payload.get("id"))})
                if path == "/api/stop":
                    if not isinstance(payload, dict):
                        raise ValueError("Expected an experiment id")
                    manager.cancel(payload.get("id"))
                    return self.respond(202, {"stopping": True})
                if path == "/api/shutdown":

                    def close():
                        manager.shutdown()
                        self.server.shutdown()

                    threading.Thread(target=close, daemon=True).start()
                    return self.respond(202, {"closing": True})
                return self.respond(404, {"error": "Not found"})
            except (ValueError, TypeError) as error:
                return self.respond(400, {"error": str(error)})
            except RuntimeError as error:
                return self.respond(409, {"error": str(error)})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if args.port != 0 and not 1024 <= args.port <= 65535:
        parser.error("Use port 1024–65535 or 0 for automatic selection")
    root = Path(__file__).resolve().parents[2]
    manager = JobManager(root)
    token = secrets.token_hex(24)
    try:
        server = ThreadingHTTPServer(
            ("127.0.0.1", args.port), make_handler(manager, token)
        )
    except OSError as error:
        parser.exit(1, f"Cannot open the local panel: {error}. Try --port 8766.\n")
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print(f"Force panel ready: http://127.0.0.1:{server.server_address[1]}", flush=True)
    print(
        "The panel stays local. Close it with its Close panel button or Ctrl+C.",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        manager.shutdown()
        server.server_close()
