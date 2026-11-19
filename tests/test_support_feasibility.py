"""Independent checks of static pressure optimization and its physical limits."""

from pathlib import Path
import unittest
import numpy as np
import xacro
from my_robot_description.studies.support_feasibility import SupportSearch
from my_robot_description.model.roll_stance import (
    JOINT_NAMES,
    MOTOR_CAPS,
    double_support_loads,
)

ROOT = Path(__file__).resolve().parents[1]


class SupportFeasibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.search = SupportSearch(
            xacro.process_file(
                str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
            ).toxml()
        )

    def test_center_pressure_torques_match_existing_static_model(self):
        s = self.search
        for shift, lower in [(0.0, 0.0), (0.03582038838959125, 0.0), (0.05, 0.03)]:
            p = s.pose(shift, lower)
            w = p["mass"] * 9.81
            forces = {"left": 0.65 * w, "right": 0.35 * w}
            old = double_support_loads(s.xml, p["q"], forces)
            derived = p["gravity"] + p["matrix"] @ np.array(
                [forces["left"], forces["right"], 0, 0, 0, 0]
            )
            np.testing.assert_allclose(
                derived, [old[n] for n in JOINT_NAMES], atol=1e-12
            )

    def test_pressure_solution_independent_equilibrium_and_moment_sum(self):
        s = self.search
        p = s.pose(0.05, 0.02)
        r = s.solve(p, 0.1)
        self.assertIsNotNone(r)
        w = p["mass"] * 9.81
        for sname in ["left", "right"]:
            off = r["pressure_offsets"][sname]
            hx, hy = s.half[sname]
            self.assertLessEqual(abs(off["x_offset_m"]), hx + 1e-9)
            self.assertLessEqual(abs(off["y_offset_m"]), hy + 1e-9)
        forces = r["forces_n"]
        self.assertAlmostEqual(sum(forces.values()), w, places=9)
        for axis in (0, 1):
            key = ("x_offset_m", "y_offset_m")[axis]
            moment = sum(
                forces[side]
                * (p["tf"][side + "_foot"][axis][3] + r["pressure_offsets"][side][key])
                for side in forces
            )
            self.assertAlmostEqual(moment, w * p["com"][axis], places=9)
        # Compute every motor torque again using shifted contact points and r×F,
        # independent of the optimizer's pressure coefficient matrix.
        import xml.etree.ElementTree as ET
        from my_robot_description.model.roll_geometry import multiply, origin, vector

        for name in JOINT_NAMES:
            j = s.joints[name]
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
            side = name.split("_")[0]
            foot = p["tf"][side + "_foot"]
            offset = r["pressure_offsets"][side]
            arm = np.array([foot[i][3] - frame[i][3] for i in range(3)]) + np.array(
                [offset["x_offset_m"], offset["y_offset_m"], 0]
            )
            expected = p["gravity"][JOINT_NAMES.index(name)] - axis @ np.cross(
                arm, [0, 0, forces[side]]
            )
            self.assertAlmostEqual(r["torques_nm"][name], expected, places=10)
        self.assertAlmostEqual(
            r["peak_utilization"],
            max(abs(r["torques_nm"][n]) / c for n, c in zip(JOINT_NAMES, MOTOR_CAPS)),
            places=10,
        )

    def test_zero_load_never_divides_by_zero_and_has_no_pressure_moment(self):
        r = self.search.solve(self.search.pose(0.05, 0.06), 0.0)
        self.assertIsNotNone(r)
        self.assertIsNone(r["pressure_offsets"]["right"])
        self.assertAlmostEqual(r["forces_n"]["right"], 0.0, places=10)
        self.assertAlmostEqual(r["first_moments_n_m"][1], 0.0, places=10)
        self.assertAlmostEqual(r["first_moments_n_m"][3], 0.0, places=10)
        self.assertGreater(r["peak_utilization"], 2.0)

    def test_mirrored_support_has_same_optimal_torque_fraction(self):
        s = self.search
        for shift, lower, fraction in [(0.05, 0.0, 0.35), (0.05, 0.06, 0.0)]:
            a = s.solve(s.pose(shift, lower), fraction)
            b = s.solve(s.pose(-shift, lower), 1 - fraction)
            self.assertAlmostEqual(
                a["peak_utilization"], b["peak_utilization"], places=9
            )

    def test_reject_bad_inputs_and_unreachable_geometry(self):
        for margin in (-0.01, 0.06, float("nan")):
            with self.assertRaises(ValueError):
                SupportSearch(self.search.xml, margin)
        for shift, lower in [
            (float("nan"), 0.0),
            (0.05, -0.01),
            (0.05, 0.07),
            (0.12, 0.0),
        ]:
            with self.assertRaises(ValueError):
                self.search.pose(shift, lower)
        with self.assertRaises(ValueError):
            self.search.solve(self.search.pose(0.0, 0.0), -1.0)
