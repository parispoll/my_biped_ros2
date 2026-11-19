"""Test body rotation, asymmetric IK and gravity/contact consistency."""

from pathlib import Path
import unittest
import numpy as np
import xacro
from my_robot_description.studies.support_feasibility import SupportSearch
from my_robot_description.model.roll_geometry import multiply, origin, vector
from my_robot_description.model.roll_stance import JOINT_NAMES

ROOT = Path(__file__).resolve().parents[1]


class LeanSupportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s = SupportSearch(
            xacro.process_file(
                str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
            ).toxml()
        )

    def test_zero_roll_agrees_with_upright_model(self):
        for shift, lower in [(0.0, 0.0), (0.07, 0.02)]:
            a = self.s.pose(shift, lower)
            b = self.s.lean_pose(shift, lower, 0.0)
            np.testing.assert_allclose(a["matrix"], b["matrix"], atol=1e-12)
            np.testing.assert_allclose(a["gravity"], b["gravity"], atol=1e-12)
            for n in JOINT_NAMES:
                self.assertAlmostEqual(a["q"][n], b["q"][n], places=12)

    def test_lean_keeps_both_world_soles_fixed_but_legs_differ(self):
        p = self.s.lean_pose(0.07, 0.02, -0.1)
        self.assertLess(p["geometry_error"], 1e-12)
        self.assertGreater(
            abs(p["q"]["left_knee_pitch"] - p["q"]["right_knee_pitch"]), 0.1
        )
        for side in ("left", "right"):
            tf = p["tf"][side + "_foot"]
            np.testing.assert_allclose(np.array(tf)[:3, :3], np.eye(3), atol=1e-12)

    def test_world_gravity_and_pressure_torque_independently(self):
        s = self.s
        p = s.lean_pose(0.07, 0.02, -0.1)
        r = s.solve(p, 0.25)
        for n in JOINT_NAMES:
            j = s.joints[n]
            frame = multiply(
                p["tf"][j.find("parent").get("link")], origin(j.find("origin"))
            )
            axis = np.array(
                [
                    sum(
                        frame[i][k] * vector(j.find("axis"), "xyz")[k] for k in range(3)
                    )
                    for i in range(3)
                ]
            )
            at = np.array(frame)[:3, 3]
            moment = np.zeros(3)
            for child in s.distal[n]:
                link = s.links[child]
                tf = multiply(p["tf"][child], origin(link.find("inertial/origin")))
                moment += np.cross(
                    np.array(tf)[:3, 3] - at,
                    [0, 0, -9.81 * float(link.find("inertial/mass").get("value"))],
                )
            side = n.split("_")[0]
            off = r["pressure_offsets"][side]
            foot = np.array(p["tf"][side + "_foot"])[:3, 3] + [
                off["x_offset_m"],
                off["y_offset_m"],
                0,
            ]
            moment += np.cross(foot - at, [0, 0, r["forces_n"][side]])
            self.assertAlmostEqual(-axis @ moment, r["torques_nm"][n], places=10)
        for i, key in [(0, "x_offset_m"), (1, "y_offset_m")]:
            moment = sum(
                r["forces_n"][side]
                * (p["tf"][side + "_foot"][i][3] + r["pressure_offsets"][side][key])
                for side in ("left", "right")
            )
            self.assertAlmostEqual(moment, p["mass"] * 9.81 * p["com"][i], places=10)

    def test_mirror_swaps_support_and_reverses_lean(self):
        s = self.s
        for d, h, roll, f in [(0.07, 0.02, -0.1, 0.25), (0.05, 0.06, -0.35, 0.0)]:
            a = s.solve(s.lean_pose(d, h, roll), f)
            b = s.solve(s.lean_pose(-d, h, -roll), 1 - f)
            self.assertIsNotNone(a)
            self.assertIsNotNone(b)
            self.assertAlmostEqual(
                a["peak_utilization"], b["peak_utilization"], places=9
            )
            self.assertAlmostEqual(a["com_world_m"][1], -b["com_world_m"][1], places=10)

    def test_invalid_lean_and_joint_limit_rejected(self):
        for args in [(0, 0, float("nan")), (0, 0, 0.36), (0.12, 0, 0), (0, 0, 0.35)]:
            with self.assertRaises(ValueError):
                self.s.lean_pose(*args)

    def test_pressure_path_starts_centered_and_respects_step_bound(self):
        s = self.s
        ref = {side: dict(x_offset_m=0.0, y_offset_m=0.0) for side in ("left", "right")}
        r = s.solve(
            s.lean_pose(0, 0, 0), 0.5, pressure_reference=ref, pressure_step=0.0
        )
        self.assertIsNotNone(r)
        for side in ref:
            for key in ref[side]:
                self.assertAlmostEqual(r["pressure_offsets"][side][key], 0.0, places=12)
        for u in np.linspace(0.01, 1, 100):
            r = s.solve(
                s.lean_pose(0.0675 * u, 0.015 * u, -0.15 * u),
                0.5 - 0.25 * u,
                pressure_reference=ref,
                pressure_step=0.002,
            )
            self.assertIsNotNone(r)
            for side in ref:
                for key in ref[side]:
                    self.assertLessEqual(
                        abs(r["pressure_offsets"][side][key] - ref[side][key]),
                        0.002 + 1e-9,
                    )
            self.assertLess(r["peak_utilization"], 0.8)
            ref = r["pressure_offsets"]

    def test_pressure_step_options_require_finite_paired_input(self):
        p = self.s.pose(0, 0)
        with self.assertRaises(ValueError):
            self.s.solve(p, 0.5, pressure_step=0.002)
        ref = {side: dict(x_offset_m=0.0, y_offset_m=0.0) for side in ("left", "right")}
        with self.assertRaises(ValueError):
            self.s.solve(p, 0.5, pressure_reference=ref, pressure_step=-0.01)
