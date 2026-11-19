"""Validate sensor collision bindings after the actual URDF-to-SDF conversion."""

from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
import xacro

MODEL = (
    Path(__file__).resolve().parents[1]
    / "src/my_robot_description/urdf/my_biped.urdf.xacro"
)


class ContactModelTests(unittest.TestCase):
    def test_both_sensor_references_resolve_after_conversion(self):
        xml = xacro.process_file(str(MODEL)).toxml()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot.urdf"
            path.write_text(xml)
            converted = subprocess.run(
                ["gz", "sdf", "-p", str(path)],
                capture_output=True,
                text=True,
                check=True,
            )
        root = ET.fromstring(converted.stdout)
        for side in ("left", "right"):
            link = root.find(f"model/link[@name='{side}_foot']")
            sensor = link.find(f"sensor[@name='{side}_foot_contact']")
            self.assertEqual(sensor.get("type"), "contact")
            collision = sensor.find("contact/collision").text
            self.assertIsNotNone(link.find(f"collision[@name='{collision}']"))
            self.assertEqual(
                sensor.find("contact/topic").text, f"/biped/{side}_foot/contacts"
            )
