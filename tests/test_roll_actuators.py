"""Profile consistency, composite mass/inertia and independent profile processes."""

import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from my_robot_description.model.roll_actuators import profile

ROOT = Path(__file__).resolve().parents[1]


class RollActuatorTests(unittest.TestCase):
    def test_published_continuous_budgets_and_mass(self):
        p = profile("robotis_p54")
        self.assertEqual(p["caps_nm"], [44.7, 25.3, 44.7, 25.3, 25.3])
        self.assertAlmostEqual(2 * sum(p["added_mass_kg"]), 7.86)
        self.assertTrue(
            all(
                v * 60 / (2 * 3.141592653589793) < rpm
                for v, rpm in zip(p["speed_rad_s"], p["published_continuous_rpm"])
            )
        )
        with self.assertRaises(ValueError):
            profile("unknown")

    def inspect(self, name):
        code = """
import json,xacro,xml.etree.ElementTree as ET
from my_robot_description.model.roll_stance import standing_plan,JOINT_NAMES,MOTOR_CAPS
from my_robot_description.model.roll_geometry import validate_roll_model
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml();root=ET.fromstring(xml)
p=standing_plan(xml);v=validate_roll_model(xml)
y=xacro.process_file('src/my_robot_description/urdf/roll_effort.urdf.xacro').toxml();controlled=ET.fromstring(y)
limits={}
for n,c in zip(JOINT_NAMES,MOTOR_CAPS):
 e=controlled.find("ros2_control/joint[@name='%s']/command_interface"%n)
 limits[n]={a.get('name'):float(a.text) for a in e.findall('param')}
 assert limits[n]==dict(min=-c,max=c)
print(json.dumps(dict(plan=p,validation=v,links={l.get('name'):dict(mass=float(l.find('inertial/mass').get('value')),z=float(l.find('inertial/origin').get('xyz').split()[2]),inertia={k:float(l.find('inertial/inertia').get(k)) for k in ['ixx','iyy','izz']}) for l in root.findall('link')},limits=limits)))
"""
        r = subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE=name),
            capture_output=True,
            text=True,
            check=True,
        )
        return json.loads(r.stdout)

    def test_both_models_include_motor_mass_and_correct_parallel_axis_inertia(self):
        old = self.inspect("legacy")
        new = self.inspect("robotis_p54")
        self.assertAlmostEqual(old["plan"]["mass_kg"], 14.42)
        self.assertAlmostEqual(new["plan"]["mass_kg"], 22.28)
        self.assertEqual(new["validation"]["links"], 11)
        self.assertEqual(new["validation"]["joints"], 10)
        oldlink = old["links"]["left_thigh"]
        newlink = new["links"]["left_thigh"]
        m = oldlink["mass"]
        motor = 0.740
        z = oldlink["z"]
        cz = m * z / (m + motor)
        self.assertAlmostEqual(newlink["mass"], m + motor)
        self.assertAlmostEqual(newlink["z"], cz)
        expected = (
            oldlink["inertia"]["ixx"]
            + m * (z - cz) ** 2
            + motor * cz**2
            + motor * (0.108**2 + 0.054**2) / 12
        )
        self.assertAlmostEqual(newlink["inertia"]["ixx"], expected, places=12)
        self.assertGreater(newlink["inertia"]["iyy"], oldlink["inertia"]["iyy"])
        self.assertEqual(old["plan"]["motor_caps_nm"], [6, 6, 10, 6, 6] * 2)
        self.assertEqual(
            new["plan"]["motor_caps_nm"], [44.7, 25.3, 44.7, 25.3, 25.3] * 2
        )

    def test_p54_roll_velocity_damping_is_explicit_and_legacy_unchanged(self):
        code = "from my_robot_description.control.roll_posture import KD; import json; print(json.dumps(KD))"
        for name, expected in [("legacy", 0.1), ("robotis_p54", 2.0)]:
            result = subprocess.run(
                [sys.executable, "-c", code],
                cwd=ROOT,
                env=dict(os.environ, BIPED_ACTUATOR_PROFILE=name),
                capture_output=True,
                text=True,
                check=True,
            )
            kd = json.loads(result.stdout)
            self.assertEqual(kd[0], expected)
            self.assertEqual(kd[4], expected)
            self.assertEqual(kd[5], expected)
            self.assertEqual(kd[9], expected)
            self.assertEqual([kd[i] for i in [1, 2, 3]], [0.1, 0.15, 0.05])
