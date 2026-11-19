"""Regression checks for test roll model. See the folder README for its role."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import xacro

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scripts.planning.roll_model_geometry import (
    validate_roll_model,
    link_transforms,
    model_com,
)
from my_robot_description.model.stance import model_from_urdf

ROOT = Path(__file__).resolve().parents[1]


class RollModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
        cls.old = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/my_biped.urdf.xacro")
        ).toxml()

    def test_tree_axes_and_physical_inertias(self):
        report = validate_roll_model(self.xml)
        self.assertEqual(report["joints"], 10)
        self.assertEqual(report["links"], 11)
        self.assertAlmostEqual(report["total_mass_kg"], 14.42)
        self.assertAlmostEqual(report["neutral_com_from_base_m"][1], 0)

    def test_zero_roll_retains_original_leg_geometry(self):
        model = model_from_urdf(self.old)
        for knee in [0, 0.4, 0.6]:
            pose = model.pose(knee)
            old_q = {
                f"{side}_{j}": q
                for side in ["left", "right"]
                for j, q in zip(["hip", "knee", "ankle"], pose)
            }
            new_q = {
                f"{side}_{j}_pitch": q
                for side in ["left", "right"]
                for j, q in zip(["hip", "knee", "ankle"], pose)
            }
            old = link_transforms(self.old, old_q)
            new = link_transforms(self.xml, new_q)
            for name in [
                "base_link",
                *[
                    f"{s}_{p}"
                    for s in ["left", "right"]
                    for p in ["thigh", "shin", "foot"]
                ],
            ]:
                for a, b in zip(sum(old[name], []), sum(new[name], [])):
                    self.assertAlmostEqual(a, b, places=12)

    def test_hip_roll_creates_lateral_motion_and_ankle_roll_keeps_sole_flat(self):
        for side in ["left", "right"]:
            neutral = link_transforms(self.xml)[f"{side}_foot"]
            for angle in [-0.1, 0.1]:
                foot = link_transforms(
                    self.xml, {f"{side}_hip_roll": angle, f"{side}_ankle_roll": -angle}
                )[f"{side}_foot"]
                self.assertGreater((foot[1][3] - neutral[1][3]) * angle, 0)
                for i in range(3):
                    for j in range(3):
                        self.assertAlmostEqual(foot[i][j], float(i == j), places=12)
                self.assertAlmostEqual(foot[0][3], neutral[0][3], places=12)

    def test_mirrored_roll_poses_have_mirrored_feet_and_centered_com(self):
        q = {
            "left_hip_roll": 0.1,
            "left_ankle_roll": -0.1,
            "right_hip_roll": -0.1,
            "right_ankle_roll": 0.1,
        }
        t = link_transforms(self.xml, q)
        self.assertAlmostEqual(t["left_foot"][1][3], -t["right_foot"][1][3], places=12)
        self.assertAlmostEqual(t["left_foot"][2][3], t["right_foot"][2][3], places=12)
        self.assertAlmostEqual(model_com(self.xml, q)[1][1], 0, places=12)

    def test_rejects_bad_axis_and_mass(self):
        for selector, attribute, value in [
            ("joint[@name='left_hip_roll']/axis", "xyz", "0 1 0"),
            ("link[@name='left_hip_roll_carrier']/inertial/mass", "value", "0"),
        ]:
            root = ET.fromstring(self.xml)
            root.find(selector).set(attribute, value)
            with self.assertRaises(ValueError):
                validate_roll_model(ET.tostring(root))

    def test_rejects_unknown_and_out_of_limit_targets(self):
        for q in [
            {"left_hip_roll": 0.36},
            {"missing": 0},
            {"left_hip_roll": float("nan")},
        ]:
            with self.assertRaises(ValueError):
                link_transforms(self.xml, q)
