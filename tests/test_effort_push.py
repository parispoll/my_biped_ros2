"""Check recovery decisions, reference runs and effort-only experiment bounds."""

import math
import unittest
from my_robot_description.experiments.effort_push import (
    EffortPushConfig,
    summarize_effort,
)
from my_robot_description.experiments.disturbance import PushConfig


class EffortPushTests(unittest.TestCase):
    def fixture(self, force=1.0):
        config = EffortPushConfig(force_x=force)
        events = [
            {"event": "recording_started", "sim_time": 0},
            {"event": "push_published", "sim_time": 2},
            {"event": "clear_published", "sim_time": 2.2},
        ]
        rows = []
        for step in range(245):
            t = step * 0.05
            phase = "baseline" if t < 2 else ("push" if t < 2.2 else "recovery")
            pulse = 0.01 * math.exp(-(t - 2)) if t >= 2 and force else 0.0
            rows.append(
                dict(
                    sim_time=t,
                    phase=phase,
                    pitch=pulse,
                    pitch_rate=-pulse,
                    acceleration_x=pulse * 10,
                    height=0.554,
                    body_x=0.0,
                    joint_error=[0.0] * 6,
                    actual_joints=[0.4 + pulse] * 6,
                    joint_command_nm=[-3.0 + pulse] * 6,
                    contacts={
                        s: {"ground_contact": True, "age": 0.001}
                        for s in ("left", "right")
                    },
                )
            )
        return rows, events, config

    def test_recovered_response_passes(self):
        rows, events, config = self.fixture()
        report = summarize_effort(rows, events, config)
        self.assertTrue(report["passed"])
        self.assertTrue(report["response_detected"])
        self.assertAlmostEqual(report["peak_pitch_change_rad"], 0.01)
        self.assertIsNotNone(report["last_outside_band_after_clear_s"])

    def test_zero_reference_passes_without_detected_response(self):
        rows, events, config = self.fixture(0)
        report = summarize_effort(rows, events, config)
        self.assertTrue(report["passed"])
        self.assertFalse(report["response_detected"])

    def test_nonzero_push_without_measured_response_does_not_pass(self):
        rows, events, _ = self.fixture(0)
        self.assertFalse(summarize_effort(rows, events, EffortPushConfig())["passed"])

    def test_persistent_joint_offset_prevents_return(self):
        rows, events, config = self.fixture()
        for row in rows:
            if row["phase"] != "baseline":
                row["actual_joints"] = [0.403] * 6
        report = summarize_effort(rows, events, config)
        self.assertFalse(report["passed"])
        self.assertFalse(report["final_second_within_return_band"])

    def test_unknown_contact_is_not_confirmed_support(self):
        rows, events, config = self.fixture()
        rows[-1]["contacts"]["left"]["ground_contact"] = None
        report = summarize_effort(rows, events, config)
        self.assertFalse(report["continuous_contact_evidence"])
        self.assertFalse(report["passed"])

    def test_sparse_final_samples_cannot_prove_continuous_return(self):
        rows, events, config = self.fixture()
        rows = [r for r in rows if not 11.5 < r["sim_time"] < 11.8]
        self.assertFalse(
            summarize_effort(rows, events, config)["final_second_within_return_band"]
        )

    def test_new_bounds_do_not_change_position_push_settings(self):
        PushConfig(force_x=100).validate()
        for force in (-100, 0, 100):
            EffortPushConfig(force_x=force, duration=0.5).validate()
        for values in (
            dict(force_x=100.1),
            dict(duration=0.51),
            dict(force_x=float("nan")),
        ):
            with self.assertRaises(ValueError):
                EffortPushConfig(**values).validate()
