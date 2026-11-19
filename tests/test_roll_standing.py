"""Regression checks for test roll standing. See the folder README for its role."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.model.roll_stance import (
    standing_plan,
    JOINT_NAMES,
    MOTOR_CAPS,
)
from my_robot_description.control.roll_posture import RollPosture, INTEGRAL_CAPS
from my_robot_description.runtime.roll_description import prepare_roll_description
from my_robot_description.model.roll_geometry import link_transforms

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / "src/my_robot_description"


class RollStandingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = xacro.process_file(str(SHARE / "urdf/biped_roll.urdf.xacro")).toxml()
        cls.plan = standing_plan(cls.xml)

    def test_centered_flat_foot_stance_with_all_eleven_masses(self):
        for knee in [0.2, 0.4, 0.6]:
            p = standing_plan(self.xml, knee)
            self.assertAlmostEqual(p["mass_kg"], 14.42)
            self.assertAlmostEqual(p["com_x_from_foot_m"], 0, places=12)
            self.assertEqual(p["target_rad"][:5], p["target_rad"][5:])
            t = link_transforms(self.xml, dict(zip(JOINT_NAMES, p["target_rad"])))
            for side in ["left", "right"]:
                self.assertAlmostEqual(t[f"{side}_foot"][2][2], 1, places=12)
            self.assertEqual(p["feedforward_nm"][:5], p["feedforward_nm"][5:])
            self.assertAlmostEqual(p["feedforward_nm"][0], 0, places=12)
            self.assertAlmostEqual(p["feedforward_nm"][4], 0, places=12)
            self.assertAlmostEqual(p["feedforward_nm"][3], 0, places=12)
            self.assertLess(p["feedforward_nm"][2], -1)

    def test_control_profile_initial_values_limits_and_explicit_damping(self):
        xml, temp, p = prepare_roll_description(SHARE, Path("/tmp/fake_prefix"))
        try:
            root = ET.fromstring(xml)
            controls = root.findall("ros2_control/joint")
            self.assertEqual([j.get("name") for j in controls], JOINT_NAMES)
            for j, target, cap in zip(controls, p["target_rad"], MOTOR_CAPS):
                initial = float(j.find("state_interface[@name='position']/param").text)
                self.assertEqual(initial, target)
                self.assertEqual(
                    float(j.find("command_interface/param[@name='max']").text), cap
                )
            plugin = root.find("gazebo/plugin[@name='biped::ViscousDamping']")
            self.assertEqual(plugin.find("expected_joints").text, "10")
            self.assertEqual(len(plugin.findall("joint")), 10)
            self.assertEqual(
                [float(j.find("damping").text) for j in plugin.findall("joint")],
                [0.2, 2.5, 3, 2.5, 0.2] * 2,
            )
            self.assertTrue(
                all(
                    j.find("dynamics").get("damping") == "0.0"
                    for j in root.findall("joint")
                )
            )
        finally:
            temp.cleanup()

    def test_feedforward_without_error(self):
        c = RollPosture(self.plan)
        r = c.update(self.plan["target_rad"], [0.0] * 10)
        self.assertEqual(r["command"], self.plan["feedforward_nm"])

    def test_saturation_and_antiwindup(self):
        c = RollPosture(self.plan)
        position = [q - 1 for q in c.target]
        for _ in range(100):
            r = c.update(position, [0.0] * 10, 0.004)
        self.assertEqual(r["command"], MOTOR_CAPS)
        self.assertEqual(c.integral, [0.0] * 10)
        for _ in range(100):
            r = c.update([q - 0.001 for q in c.target], [0.0] * 10, 0.004)
        self.assertTrue(all(0 < i <= lim for i, lim in zip(c.integral, INTEGRAL_CAPS)))

    def test_rejects_invalid_vectors_and_time(self):
        c = RollPosture(self.plan)
        for q, v, dt in [
            ([0.0] * 6, [0.0] * 6, 0),
            ([float("nan")] * 10, [0.0] * 10, 0),
            ([0.0] * 10, [0.0] * 10, 0.1),
        ]:
            with self.assertRaises(ValueError):
                c.update(q, v, dt)
        with self.assertRaises(ValueError):
            standing_plan(self.xml, 0)


class RollAuditTests(unittest.TestCase):
    def test_bad_damping_and_out_of_budget_motor_are_rejected(self):
        from my_robot_description.validation.roll_audit import audit_damping

        rows = [
            dict(
                joint=n,
                coefficient=b,
                velocity=0.0,
                torque=0.0,
                power=0.0,
                max_power_in_block=0.0,
                motor_nm=0.0,
                total_nm=0.0,
                max_abs_motor_in_block_nm=0.0,
                steps=10,
            )
            for n, b in zip(JOINT_NAMES, [0.2, 2.5, 3, 2.5, 0.2] * 2)
        ]
        self.assertTrue(audit_damping(rows)["passed"])
        rows[0]["coefficient"] = 0.3
        with self.assertRaises(ValueError):
            audit_damping(rows)
        rows[0]["coefficient"] = 0.2
        rows[0].update(motor_nm=7.0, total_nm=7.0, max_abs_motor_in_block_nm=7.0)
        self.assertFalse(audit_damping(rows)["passed"])

    def test_posture_audit_detects_wrong_command_and_missing_coverage(self):
        from my_robot_description.validation.roll_audit import audit_posture

        xml = xacro.process_file(str(SHARE / "urdf/biped_roll.urdf.xacro")).toxml()
        p = standing_plan(xml)
        rows = []
        for t in [0.0, 0.05, 0.1]:
            terms = RollPosture(p).update(p["target_rad"], [0.0] * 10)
            rows.append(
                dict(
                    sim_time=t,
                    active=True,
                    ready=True,
                    position=p["target_rad"],
                    velocity=[0.0] * 10,
                    **terms,
                )
            )
        self.assertTrue(audit_posture(rows, p, 0, 0.1)["passed"])
        rows[1]["command"][0] = 0.1
        self.assertFalse(audit_posture(rows, p, 0, 0.1)["passed"])
        rows[1]["command"][0] = 0.0
        self.assertFalse(audit_posture(rows, p, 0, 1.0)["passed"])


class RollTimingTests(unittest.TestCase):
    def test_latest_imu_uses_clock_even_if_newer_than_joint_message(self):
        from my_robot_description.control.roll_timing import imu_source_age

        # Joint callback timestamp 10.01; IMU 10.04; clock 10.05.
        # The IMU is fresh despite being 0.03 s newer than the joint message.
        self.assertAlmostEqual(imu_source_age(10.05, 10.04), 0.01)

    def test_rejects_genuinely_old_or_future_imu(self):
        from my_robot_description.control.roll_timing import imu_source_age

        for now, stamp in [(10.0, 9.94), (10.0, 10.03), (10.0, float("nan"))]:
            with self.assertRaises(ValueError):
                imu_source_age(now, stamp)

    def test_startup_wait_does_not_accept_stale_data_as_fresh(self):
        from my_robot_description.control.roll_timing import imu_source_age

        self.assertIsNone(imu_source_age(1.051, 1.0, wait_for_startup=True))
        with self.assertRaises(ValueError):
            imu_source_age(1.051, 1.0)
        with self.assertRaises(ValueError):
            imu_source_age(float("nan"), 1.0, wait_for_startup=True)
