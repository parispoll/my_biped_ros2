#!/usr/bin/env python3
"""Print predicted stance geometry without running Gazebo or moving joints."""

import argparse
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import xacro
from my_robot_description.model.stance import model_from_urdf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--knee", type=float, default=0.4, help="Knee bend in radians")
    args = parser.parse_args()
    path = (
        Path(get_package_share_directory("my_robot_description"))
        / "urdf/my_biped.urdf.xacro"
    )
    model = model_from_urdf(xacro.process_file(str(path)).toxml())
    pose = model.pose(args.knee)
    print(
        json.dumps(
            {
                "mass_kg": model.mass,
                "hip_knee_ankle_radians": pose,
                "predicted_geometry": model.geometry(pose),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
