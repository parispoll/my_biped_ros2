#!/usr/bin/env python3
"""Calculate static holding torques; does not launch or command the robot."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json

import xacro
from my_robot_description.model.stance import model_from_urdf
from my_robot_description.model.statics import standing_loads


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--knee",
        type=float,
        default=0.4,
        help="Symmetric knee bend in radians (default: 0.4)",
    )
    args = parser.parse_args()
    path = (
        Path(__file__).resolve().parents[2]
        / "src/my_robot_description/urdf/my_biped.urdf.xacro"
    )
    model = model_from_urdf(xacro.process_file(str(path)).toxml())
    print(json.dumps(standing_loads(model, args.knee), indent=2))


if __name__ == "__main__":
    main()
