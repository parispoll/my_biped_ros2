"""Preserve controller failures even when terminal stderr scrolls away."""

import json
from pathlib import Path


def save_fault(path, node, traceback_text):
    """Write diagnostics only; do not change gains, limits or motion behavior."""
    if not path:
        return
    session = getattr(node, "interactive", None)
    report = dict(
        traceback=traceback_text,
        raw_contact_rows=getattr(node, "zero_contact_rows", {}),
        joint_stamp=getattr(node, "last_stamp", None),
        imu_stamp=getattr(node, "imu_stamp", None),
        tilt_rad=getattr(node, "tilt", None),
        session=session.status(node.ready) if session is not None else None,
    )
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n")
