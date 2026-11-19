"""Process-local workaround for ROS diagnostics Jazzy ABI issue #663.

Use the unmodified official 4.2.6 library only when controller_manager requires
its constructor and the system library no longer exports it. No system changes.
"""

import os
from pathlib import Path
import subprocess


def symbols(path):
    result = subprocess.run(
        ["nm", "-D", str(path)], capture_output=True, text=True, check=True
    )
    return {line.split()[-1] for line in result.stdout.splitlines() if line.split()}


def gazebo_compatibility_env():
    ros_lib = Path("/opt/ros/jazzy/lib")
    manager = ros_lib / "libcontroller_manager.so"
    diagnostic = ros_lib / "libdiagnostic_updater.so"
    if not manager.exists() or not diagnostic.exists():
        return {}
    required = {
        s
        for s in symbols(manager)
        if "diagnostic_updater7UpdaterC1" in s and s.endswith("Ed")
    }
    if not required or required <= symbols(diagnostic):
        return {}
    for parent in Path(__file__).resolve().parents:
        library = (
            parent / ".runtime/diagnostics-4.2.6/install/lib/libdiagnostic_updater.so"
        )
        if library.is_file() and required <= symbols(library):
            print(
                "[biped:diagnostic-compat] Using process-local official 4.2.6 library: "
                + str(library),
                flush=True,
            )
            return {
                "LD_LIBRARY_PATH": str(library.parent)
                + os.pathsep
                + os.environ.get("LD_LIBRARY_PATH", "")
            }
    raise RuntimeError(
        "ROS diagnostic_updater ABI mismatch. Run scripts/services/build_diagnostic_compat.sh, or align system ros2_control/diagnostics packages."
    )
