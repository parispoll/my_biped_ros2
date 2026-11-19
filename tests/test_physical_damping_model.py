"""Preparation relocates physical damping; feedforward must retain its values."""

import unittest
import xml.etree.ElementTree as ET
from my_robot_description.model.roll_stance import JOINT_NAMES
from my_robot_description.model.physical_damping import damping_coefficients


class DampingModelTests(unittest.TestCase):
    def xml(self, prepared):
        root = ET.Element("robot")
        values = [0.2, 2.5, 3.0, 2.5, 0.2] * 2
        for n, b in zip(JOINT_NAMES, values):
            e = ET.SubElement(root, "joint", name=n)
            ET.SubElement(e, "dynamics", damping=str(0 if prepared else b))
        if prepared:
            plugin = ET.SubElement(
                ET.SubElement(root, "gazebo"), "plugin", name="biped::ViscousDamping"
            )
            for n, b in zip(JOINT_NAMES, values):
                ET.SubElement(
                    ET.SubElement(plugin, "joint", name=n), "damping"
                ).text = str(b)
        return root

    def test_original_and_prepared_agree(self):
        self.assertEqual(
            damping_coefficients(ET.tostring(self.xml(False))),
            damping_coefficients(ET.tostring(self.xml(True))),
        )

    def test_double_counted_damping_is_rejected(self):
        root = self.xml(True)
        root.find("joint/dynamics").set("damping", "1")
        with self.assertRaises(ValueError):
            damping_coefficients(ET.tostring(root))
