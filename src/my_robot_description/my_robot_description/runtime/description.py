"""Load robot XML and prepare the controller configuration for Gazebo."""

from pathlib import Path
import tempfile
import math
import xacro
from my_robot_description.model.stance import model_from_urdf


def prepare_description(
    robot_file,
    controllers_file,
    initial_knee=0.0,
    joint_friction=None,
    damping_model="implicit",
):
    """Return expanded URDF and temporary configuration owner.

    Keep the owner alive until shutdown: its directory contains the YAML consumed
    by Gazebo. The temporary path avoids the whitespace parsing failure observed
    with our workspace's name. Clean it up if expansion fails, too.
    """
    for path in (robot_file, controllers_file):
        if not path.is_file():
            raise FileNotFoundError(
                f"[description] Required file does not exist: {path}"
            )
    temporary = tempfile.TemporaryDirectory(prefix="biped_")
    try:
        config = Path(temporary.name) / "controllers.yaml"
        config.write_bytes(controllers_file.read_bytes())
        if damping_model not in ("implicit", "explicit"):
            raise ValueError("Unknown damping model")
        mappings = {"controllers_yaml": str(config), "damping_model": damping_model}
        if joint_friction is not None:
            if not math.isfinite(joint_friction) or not 0 <= joint_friction <= 0.2:
                raise ValueError("Diagnostic joint friction must be between 0 and 0.2")
            mappings["joint_friction"] = str(joint_friction)
        xml = xacro.process_file(str(robot_file), mappings=mappings).toxml()
        if initial_knee != 0.0:
            # Start crouch tests inside the knee range instead of on its hard stop.
            hip, knee, ankle = model_from_urdf(xml).pose(initial_knee)
            mappings.update(
                initial_hip=str(hip), initial_knee=str(knee), initial_ankle=str(ankle)
            )
            xml = xacro.process_file(str(robot_file), mappings=mappings).toxml()
    except Exception:
        temporary.cleanup()
        raise
    return xml, temporary
