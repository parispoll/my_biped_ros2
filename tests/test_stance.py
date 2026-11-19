"""Physical invariants for the calculated symmetric stance."""

from pathlib import Path
import unittest
import xacro
from my_robot_description.model.stance import model_from_urdf

MODEL = (
    Path(__file__).resolve().parents[1]
    / "src/my_robot_description/urdf/my_biped.urdf.xacro"
)


class StanceTests(unittest.TestCase):
    def setUp(self):
        self.xml = xacro.process_file(str(MODEL)).toxml()
        self.model = model_from_urdf(self.xml)

    def test_straight_geometry_matches_link_lengths(self):
        self.assertEqual(self.model.pose(0), (0, 0, 0))
        self.assertAlmostEqual(
            self.model.geometry((0, 0, 0))["expected_pelvis_height"], 0.565
        )

    def test_crouch_keeps_flat_feet_and_mass_centered(self):
        for knee in (0.1, 0.2, 0.4, 0.6):
            pose = self.model.pose(knee)
            geometry = self.model.geometry(pose)
            self.assertAlmostEqual(geometry["foot_pitch"], 0)
            self.assertAlmostEqual(geometry["com_offset_from_foot_center"], 0)
            self.assertLess(geometry["expected_pelvis_height"], 0.565)
            self.assertGreater(geometry["fore_aft_static_margin"], 0)

    def test_rejects_impossible_or_invalid_angles(self):
        for knee in (-0.1, 3, float("nan")):
            with self.assertRaises(ValueError):
                self.model.pose(knee)

    def test_rejects_changed_joint_axis(self):
        with self.assertRaisesRegex(ValueError, "axes"):
            model_from_urdf(self.xml.replace('xyz="0 1 0"', 'xyz="1 0 0"', 1))

    def test_rejects_asymmetric_mass(self):
        from xml.etree import ElementTree as ET

        root = ET.fromstring(self.xml)
        root.find("link[@name='left_thigh']/inertial/mass").set("value", "2")
        with self.assertRaisesRegex(ValueError, "identical"):
            model_from_urdf(ET.tostring(root))
