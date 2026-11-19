"""Identify the geometry, actuator data and planner used by an offline preflight."""

import hashlib
from pathlib import Path


def planning_fingerprint(package):
    package = Path(package)
    digest = hashlib.sha256()
    for folder in ("motion", "model", "urdf", "config"):
        directory = (
            package / folder
            if folder in ("urdf", "config")
            else package / "my_robot_description" / folder
        )
        for path in sorted(directory.rglob("*")):
            if path.is_file() and path.suffix in (".py", ".yaml", ".xacro"):
                digest.update(
                    str(path.relative_to(package)).encode() + b"\0" + path.read_bytes()
                )
    return digest.hexdigest()
