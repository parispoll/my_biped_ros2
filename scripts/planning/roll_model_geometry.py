"""Compatibility import for offline roll-model tools; math lives in the package."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "src/my_robot_description")
)
from my_robot_description.model.roll_geometry import *
