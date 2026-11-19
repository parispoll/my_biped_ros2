"""Check that offline limit diagnosis agrees with the command planner where valid."""

import sys
from pathlib import Path
import unittest
import xacro

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from scripts.analysis.analyze_unloading_limits import point, crossing
from my_robot_description.motion.lateral_shift import LateralPlanner
from my_robot_description.model.roll_stance import JOINT_NAMES


class UnloadingLimitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner = LateralPlanner(
            xacro.process_file(
                str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
            ).toxml()
        )

    def test_diagnostic_matches_live_planner_inside_budget(self):
        for d in (0.0, 0.03582038838959125, -0.03582038838959125):
            observed = point(self.planner, d)
            planned = self.planner.fixed_foot_pose(d)
            for name, value in zip(JOINT_NAMES, planned["feedforward_nm"]):
                self.assertAlmostEqual(observed["torques_nm"][name], value, places=12)
            self.assertAlmostEqual(
                sum(observed["forces_n"].values()), 14.42 * 9.81, places=10
            )
            self.assertLess(observed["fixed_foot_error_m"], 1e-12)

    def test_boundaries_bracket_reserve_and_absolute_cap(self):
        for threshold in (0.8, 1.0):
            r = crossing(self.planner, threshold)
            d = r["displacement_m"]
            self.assertAlmostEqual(r["peak_utilization"], threshold, places=10)
            self.assertLess(
                point(self.planner, d - 1e-5)["peak_utilization"], threshold
            )
            self.assertGreater(
                point(self.planner, d + 1e-5)["peak_utilization"], threshold
            )
            self.assertGreater(r["lighter_fraction"], 0.2)
            mirror = point(self.planner, -d)
            self.assertAlmostEqual(mirror["peak_utilization"], threshold, places=10)

    def test_geometry_blocks_zero_load_even_ignoring_motor_caps(self):
        r = point(self.planner, 0.1033)
        self.assertGreater(r["lighter_fraction"], 0.06)
        self.assertGreater(r["peak_utilization"], 1.0)
        with self.assertRaisesRegex(ValueError, "unreachable|singularity"):
            point(self.planner, 0.1034)
        with self.assertRaises(ValueError):
            self.planner.fixed_foot_pose(0.07)
