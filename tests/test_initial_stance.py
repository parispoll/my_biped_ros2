"""Check that crouch startup has knee clearance without changing physical limits."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from my_robot_description.runtime.description import prepare_description
from my_robot_description.model.stance import model_from_urdf

PACKAGE = Path(__file__).resolve().parents[1] / "src/my_robot_description"


class InitialStanceTests(unittest.TestCase):
    def expanded(self, knee):
        xml, owner = prepare_description(
            PACKAGE / "urdf/my_biped.urdf.xacro",
            PACKAGE / "config/controllers.yaml",
            knee,
        )
        self.addCleanup(owner.cleanup)
        return ET.fromstring(xml), model_from_urdf(xml)

    def test_crouch_initialization_is_balanced_and_away_from_stop(self):
        root, model = self.expanded(0.02)
        pose = model.pose(0.02)
        for side in ("left", "right"):
            for name, expected in zip(("hip", "knee", "ankle"), pose):
                value = root.find(
                    f"ros2_control/joint[@name='{side}_{name}']/state_interface[@name='position']/param[@name='initial_value']"
                )
                self.assertAlmostEqual(float(value.text), expected)
        self.assertAlmostEqual(sum(pose), 0)
        self.assertAlmostEqual(model.geometry(pose)["com_offset_from_foot_center"], 0)
        self.assertEqual(model.limits[1], (0, 2.4))

    def test_zero_initialization_preserves_straight_baseline(self):
        root, model = self.expanded(0)
        for value in root.findall(
            "ros2_control/joint/state_interface[@name='position']/param"
        ):
            self.assertEqual(float(value.text), 0)

    def test_invalid_initial_angle_is_rejected(self):
        with self.assertRaises(ValueError):
            self.expanded(-0.1)
