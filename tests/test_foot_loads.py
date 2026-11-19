"""Regression checks for test foot loads. See the folder README for its role."""

import copy
import math
import unittest
from my_robot_description.sensing.foot_loads import (
    decode_foot_load,
    audit_foot_loads,
    paired_loads,
)


def sample(side, t, normal):
    xyz = lambda x, y, z: dict(x=x, y=y, z=z)
    y = 0.085 if side == "left" else -0.085
    wrench = lambda f: dict(
        header=dict(frame_id=""),
        body_1_wrench=dict(force=xyz(0, 0, f)),
        body_2_wrench=dict(force=xyz(0, 0, -f)),
    )
    sec = int(t)
    nsec = round((t - sec) * 1e9)
    return dict(
        side=side,
        sim_time=t,
        message=dict(
            header=dict(stamp=dict(sec=sec, nanosec=nsec), frame_id=""),
            contacts=[
                dict(
                    collision1=dict(
                        name=f"my_biped::{side}_foot::{side}_foot_collision"
                    ),
                    collision2=dict(name="ground_plane::ground::collision"),
                    positions=[
                        xyz(x, y + dy, 0)
                        for x, dy in [
                            (-0.1, -0.06),
                            (0.1, -0.06),
                            (0.1, 0.06),
                            (-0.1, 0.06),
                        ]
                    ],
                    normals=[xyz(0, 0, 1)] * 4,
                    depths=[0.0] * 4,
                    wrenches=[wrench(normal / 4) for _ in range(4)],
                )
            ],
        ),
    )


class FootLoadDecoderTests(unittest.TestCase):
    def test_sums_contact_points_once_without_a_mass_estimate(self):
        row = sample("left", 1.0, 100.0)
        r = decode_foot_load(row)
        self.assertEqual(r["point_count"], 4)
        self.assertEqual(r["force_world_n"], [0.0, 0.0, 100.0])
        self.assertAlmostEqual(r["cop_world_xy_m"][0], 0)
        self.assertAlmostEqual(r["cop_world_xy_m"][1], 0.085)

    def test_missing_force_unknown_frame_or_reversed_pair_is_not_zeroed(self):
        row = sample("left", 1.0, 70.0)
        for mode in ["missing", "frame", "reverse", "duplicate", "nonfinite", "empty"]:
            bad = copy.deepcopy(row)
            c = bad["message"]["contacts"][0]
            if mode == "missing":
                c["wrenches"] = []
            if mode == "frame":
                bad["message"]["header"]["frame_id"] = "foot"
            if mode == "reverse":
                c["collision1"], c["collision2"] = c["collision2"], c["collision1"]
            if mode == "duplicate":
                bad["message"]["contacts"].append(copy.deepcopy(c))
            if mode == "nonfinite":
                c["wrenches"][0]["body_1_wrench"]["force"]["z"] = math.nan
            if mode == "empty":
                bad["message"]["contacts"] = []
            with self.assertRaises(ValueError, msg=mode):
                decode_foot_load(bad)

    def test_action_reaction_and_upward_sign_are_checked(self):
        row = sample("left", 1.0, 70.0)
        row["message"]["contacts"][0]["wrenches"][0]["body_2_wrench"]["force"]["z"] = 0
        with self.assertRaises(ValueError):
            decode_foot_load(row)
        with self.assertRaises(ValueError):
            decode_foot_load(sample("left", 1.0, -70.0))


class FootLoadAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw = []
        cls.com = []
        weight = 14.42 * 9.81
        for i in range(1251):
            t = 10 + i * 0.02
            shift = 7 <= t - 10 < 13
            d = 0.007119097 if shift else 0.0
            left = weight * (0.5 + d / 0.17)
            right = weight - left
            cls.raw.extend([sample("left", t, left), sample("right", t, right)])
            cls.com.append(dict(sim_time=t, com=[0.0, d, 0.415]))

    def test_validates_weight_and_load_transfer_independently_of_prediction(self):
        r = audit_foot_loads(self.raw, self.com, 10, 35, 14.42, 0.01)
        self.assertTrue(r["passed"])
        self.assertTrue(r["measurement_validated"])
        self.assertAlmostEqual(
            r["phase_means"]["baseline"]["left_n"], 70.7301, places=8
        )
        self.assertGreater(r["held_load_changes_n"]["left"], 5)

    def test_doubled_forces_fail_even_when_action_reaction_still_holds(self):
        raw = copy.deepcopy(self.raw)
        for r in raw:
            for c in r["message"]["contacts"]:
                for w in c["wrenches"]:
                    for key in ["body_1_wrench", "body_2_wrench"]:
                        w[key]["force"]["z"] *= 2
        report = audit_foot_loads(raw, self.com, 10, 35, 14.42, 0.01)
        self.assertFalse(report["measurement_validated"])

    def test_stale_other_foot_cannot_be_paired_with_future_data(self):
        raw = [
            r
            for r in self.raw
            if r["side"] == "left" or r["sim_time"] < 20 or r["sim_time"] > 21
        ]
        with self.assertRaises(ValueError):
            paired_loads(raw, 10, 35)

    def test_frozen_loads_with_fresh_timestamps_fail_independent_moment_check(self):
        raw = copy.deepcopy(self.raw)
        point = 14.42 * 9.81 / 8
        for r in raw:
            for c in r["message"]["contacts"]:
                for w in c["wrenches"]:
                    w["body_1_wrench"]["force"]["z"] = point
                    w["body_2_wrench"]["force"]["z"] = -point
        report = audit_foot_loads(raw, self.com, 10, 35, 14.42, 0.01)
        self.assertFalse(report["measurement_validated"])
        self.assertGreater(report["max_cop_com_distance_m"], 0.003)
