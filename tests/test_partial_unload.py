"""Regression checks for test partial unload. See the folder README for its role."""

from pathlib import Path
import unittest
import xacro
from my_robot_description.motion.partial_unload import (
    PartialUnloadProgram,
    load_guard,
    audit_load_guards,
)
from my_robot_description.motion.lateral_shift import LateralPlanner, ShiftProgram
from my_robot_description.model.roll_stance import MOTOR_CAPS

ROOT = Path(__file__).resolve().parents[1]


class PartialUnloadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
        cls.program = PartialUnloadProgram(cls.xml, 0.35, "left")

    def test_derived_load_goal_geometry_and_torque_reserve(self):
        p = self.program
        s = p.setup
        self.assertAlmostEqual(
            s["endpoint"]["predicted_force_n"]["right"] / (14.42 * 9.81),
            0.35,
            places=10,
        )
        self.assertAlmostEqual(s["displacement_m"], 0.03582038839, places=10)
        for point in p.table:
            self.assertLess(point["max_fixed_foot_geometry_error"], 1e-12)
            self.assertTrue(
                all(
                    abs(t) <= 0.8 * c
                    for t, c in zip(point["feedforward_nm"], MOTOR_CAPS)
                )
            )

    def test_mirror_and_smaller_goal(self):
        r = PartialUnloadProgram(self.xml, 0.35, "right")
        small = PartialUnloadProgram(self.xml, 0.4, "left")
        self.assertAlmostEqual(r.displacement, -self.program.displacement, places=12)
        self.assertEqual(r.setup["unloaded_foot"], "left")
        self.assertLess(small.displacement, self.program.displacement)
        self.assertAlmostEqual(
            small.setup["endpoint"]["predicted_force_n"]["right"] / (14.42 * 9.81),
            0.4,
            places=10,
        )

    def test_schedule_and_old_experiment_limit_remain_explicit(self):
        self.assertEqual(self.program.setup["total_time_s"], 37)
        for t, phase in [
            (0, "baseline"),
            (3, "move_out"),
            (11, "hold"),
            (19, "move_back"),
            (27, "recovery"),
            (37, "done"),
        ]:
            self.assertEqual(self.program.sample(t)["phase"], phase)
        self.assertEqual(self.program.sample(37)["pelvis_y_m"], 0.0)
        with self.assertRaises(ValueError):
            ShiftProgram(self.xml, 0.035)
        with self.assertRaises(ValueError):
            LateralPlanner(self.xml).pose(0.035)
        for target in [0.2, 0.0]:
            with self.assertRaises(ValueError):
                PartialUnloadProgram(self.xml, target)


class LoadGuardTests(unittest.TestCase):
    def values(self, left, right, t=1.0):
        return {
            s: dict(sim_time=t, force_world_n=[0.0, 0.0, f])
            for s, f in [("left", left), ("right", right)]
        }

    def test_fresh_loads_and_support_floor(self):
        w = 14.42 * 9.81
        self.assertTrue(
            load_guard(self.values(0.65 * w, 0.35 * w), 1.01, 14.42)["eligible"]
        )
        self.assertFalse(
            load_guard(self.values(0.8 * w, 0.2 * w), 1.01, 14.42)["eligible"]
        )
        self.assertFalse(
            load_guard(self.values(0.65 * w, 0.35 * w), 1.06, 14.42)["eligible"]
        )
        self.assertFalse(load_guard({}, 1.01, 14.42)["eligible"])
        self.assertFalse(load_guard(self.values(w, w), 1.01, 14.42)["eligible"])

    def test_audit_catches_faked_guard_and_lost_torque_headroom(self):
        import copy

        w = 14.42 * 9.81
        g = load_guard(self.values(0.65 * w, 0.35 * w), 1.01, 14.42)
        rows = [dict(sim_time=1.0, shift=dict(), load_guard=g, command=[0.0] * 10)]
        self.assertTrue(audit_load_guards(rows, 1, 2, 14.42)["passed"])
        bad = copy.deepcopy(rows)
        bad[0]["load_guard"]["shares"]["left"] = 0.5
        with self.assertRaises(ValueError):
            audit_load_guards(bad, 1, 2, 14.42)
        bad = copy.deepcopy(rows)
        bad[0]["command"][0] = 5.5
        self.assertFalse(audit_load_guards(bad, 1, 2, 14.42)["passed"])
