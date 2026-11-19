"""Named output-shaft actuator budgets shared with the Xacro model.

Select before importing robot modules. The legacy profile is the default so
recorded baseline experiments remain reproducible. No thermal/electrical model.
"""

import os
from pathlib import Path
import yaml


def profile(name=None):
    name = name or os.environ.get("BIPED_ACTUATOR_PROFILE", "legacy")
    path = Path(__file__).resolve().parents[2] / "config/roll_actuators.yaml"
    if not path.is_file():
        from ament_index_python.packages import get_package_share_directory

        path = (
            Path(get_package_share_directory("my_robot_description"))
            / "config/roll_actuators.yaml"
        )
    profiles = yaml.safe_load(path.read_text())["profiles"]
    if name not in profiles:
        raise ValueError("Unknown actuator profile: " + name)
    return dict(name=name, **profiles[name])


ACTIVE = profile()
MOTOR_CAPS = ACTIVE["caps_nm"] * 2
MOTOR_SPEEDS = ACTIVE["speed_rad_s"] * 2
