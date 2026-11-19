"""Geometric preparation for stepping; no claim of dynamic walking."""

from pathlib import Path
import math
import unittest
import xacro
from my_robot_description.model.stance import model_from_urdf
from my_robot_description.model.sagittal_step import (
    ankle_position,
    leg_angles,
    swing_target,
    preview_step,
)


class SagittalStepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = (
            Path(__file__).resolve().parents[1]
            / "src/my_robot_description/urdf/my_biped.urdf.xacro"
        )
        cls.model = model_from_urdf(xacro.process_file(str(path)).toxml())

    def test_existing_stance_is_reproduced(self):
        for knee in [0.2, 0.4, 0.6, 1.0]:
            stance = self.model.pose(knee)
            g = self.model.geometry(stance)
            target = (
                g["foot_x_from_pelvis"],
                -g["expected_pelvis_height"] + self.model.foot_thickness,
                0.0,
            )
            for q, expected in zip(leg_angles(self.model, *target), stance):
                self.assertAlmostEqual(q, expected, places=12)

    def test_nonflat_orientation_and_bent_pose(self):
        for pose in [(-0.3, 0.7, -0.2), (0.1, 0.5, -0.4), (-0.8, 1.4, -0.5)]:
            for q, expected in zip(
                leg_angles(self.model, *ankle_position(self.model, pose)), pose
            ):
                self.assertAlmostEqual(q, expected, places=12)

    def test_preview_clears_ground_and_respects_limits(self):
        samples = preview_step(self.model)
        x0, z0, pitch = samples[0]["ankle_target_xz_pitch"]
        self.assertAlmostEqual(samples[-1]["ankle_target_xz_pitch"][0] - x0, 0.03)
        self.assertAlmostEqual(samples[-1]["ankle_target_xz_pitch"][1], z0)
        self.assertAlmostEqual(samples[50]["ankle_target_xz_pitch"][1] - z0, 0.02)
        for r in samples:
            target = r["ankle_target_xz_pitch"]
            self.assertGreaterEqual(target[1], z0)
            for actual, expected in zip(
                ankle_position(self.model, r["swing_hip_knee_ankle_rad"]), target
            ):
                self.assertAlmostEqual(actual, expected, places=12)
            for q, (low, high) in zip(r["swing_hip_knee_ankle_rad"], self.model.limits):
                self.assertTrue(low <= q <= high)
            self.assertEqual(r["stance_hip_knee_ankle_rad"], self.model.pose(0.4))

    def test_endpoints_are_smooth_and_lift_nonnegative(self):
        h = 1e-4
        for end, neighbor in [(0, h), (1, 1 - h)]:
            a = swing_target(0, -0.5, 0.03, 0.02, end)
            b = swing_target(0, -0.5, 0.03, 0.02, neighbor)
            self.assertLess(max(abs(x - y) / h for x, y in zip(a, b)), 1e-7)
            c = swing_target(0, -0.5, 0.03, 0.02, 2 * h if end == 0 else 1 - 2 * h)
            self.assertLess(
                max(abs(x - 2 * y + z) / (h * h) for x, y, z in zip(a, b, c)), 0.001
            )

    def test_rejects_unreachable_singular_and_limit_violating_targets(self):
        for target in [
            (0, -0.6, 0),
            (0, 0, 0),
            (0, -0.52, 0),
            (0, -0.4, 3),
            (math.nan, -0.4, 0),
        ]:
            with self.assertRaises(ValueError):
                leg_angles(self.model, *target)
        for kwargs in [
            dict(duration=0),
            dict(clearance=0),
            dict(samples=2),
            dict(advance=1),
        ]:
            with self.assertRaises(ValueError):
                preview_step(self.model, **kwargs)
