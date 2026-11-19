#!/usr/bin/env python3
"""Install the loopback experiment panel as a login-started user service."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import json
import os
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
UNIT = "biped-force-panel.service"


def quote(value):
    return (
        '"'
        + str(value).replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%")
        + '"'
    )


def main():
    # Do not interrupt a user's active measurement to install autostart.
    try:
        with urllib.request.urlopen(
            "http://127.0.0.1:8765/api/state", timeout=3
        ) as response:
            state = json.load(response)
        if state.get("active_id"):
            raise RuntimeError(
                "Wait for the active experiment to finish before installing autostart."
            )
    except urllib.error.HTTPError:
        raise
    except urllib.error.URLError:
        pass
    target = Path.home() / ".config/systemd/user" / UNIT
    text = f"""# Managed by scripts/services/install_force_panel_service.py
[Unit]
Description=Biped lab local experiment panel

[Service]
Type=simple
WorkingDirectory={str(ROOT).replace('%','%%')}
ExecStart={quote(ROOT/'scripts/start_force_panel.sh')} --port 8765
Restart=on-failure
RestartSec=3
KillMode=mixed
TimeoutStopSec=45

[Install]
WantedBy=default.target
"""
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and target.read_text() != text:
        backup = target.with_suffix(".service.before-autostart")
        if backup.exists():
            raise RuntimeError(
                f"Backup already exists; inspect {backup} before replacing the service."
            )
        backup.write_bytes(target.read_bytes())
    with tempfile.NamedTemporaryFile(mode="w", suffix=".service") as candidate:
        candidate.write(text)
        candidate.flush()
        subprocess.run(
            ["systemd-analyze", "--user", "verify", candidate.name], check=True
        )
    if (
        subprocess.run(["systemctl", "--user", "is-active", "--quiet", UNIT]).returncode
        == 0
    ):
        subprocess.run(["systemctl", "--user", "stop", UNIT], check=True)
    # A collected transient unit must unload before the persistent one is read.
    subprocess.run(
        ["systemctl", "--user", "reset-failed", UNIT],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    target.write_text(text)
    subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
    display_vars = [
        key for key in ["DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY"] if key in os.environ
    ]
    if display_vars:
        subprocess.run(
            ["systemctl", "--user", "import-environment", *display_vars], check=True
        )
    subprocess.run(["systemctl", "--user", "enable", UNIT], check=True)
    subprocess.run(["systemctl", "--user", "start", UNIT], check=True)
    for attempt in range(30):
        try:
            with urllib.request.urlopen(
                "http://127.0.0.1:8765/api/state", timeout=1
            ) as response:
                if response.status == 200:
                    break
        except urllib.error.URLError:
            if attempt == 29:
                raise
            time.sleep(0.2)
    print("Installed login autostart:", target)
    print("Open http://127.0.0.1:8765/ - no experiment starts automatically.")


if __name__ == "__main__":
    main()
