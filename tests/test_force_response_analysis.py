"""Regression checks for test force response analysis. See the folder README for its role."""

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scripts.analysis.analyze_force_response import analyze


class ForceDiagnosisTests(unittest.TestCase):
    def test_partial_trace_counts_only_recorded_response_and_distinguishes_caps(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            recording = root / "push/recording"
            recording.mkdir(parents=True)
            sample = lambda t, phase: dict(
                sim_time=t,
                phase=phase,
                pitch=0.01,
                pitch_rate=0.1,
                height=0.55,
                contacts={
                    side: dict(ground_contact=True) for side in ("left", "right")
                },
            )
            (recording / "samples.jsonl").write_text(
                "\n".join(
                    json.dumps(r) for r in (sample(0, "baseline"), sample(2, "push"))
                )
                + '\n{"unfinished":'
            )
            (recording / "events.json").write_text(
                json.dumps([dict(event="push_published", sim_time=2)])
            )
            control = lambda t: dict(
                sim_time=t,
                balance_gate=True,
                balance_pitch_rad=0.1,
                balance_rate_rad_s=0,
                balance_nm=[0, 0, 0.5, 0, 0, 0.5],
                requested=[0, 0, 1, 0, 0, 1],
                command=[0, 0, 1, 0, 0, 1],
            )
            (root / "push/controller.jsonl").write_text(
                "\n".join(json.dumps(control(t)) for t in (1, 2, 3))
            )
            result = analyze(root)
            self.assertEqual(result["controller_response_samples"], 1)
            self.assertEqual(result["balance_clipped_samples"], 1)
            self.assertEqual(result["motor_saturated_samples"], 0)
            self.assertIsNone(result["clear_sim_time"])

    def test_missing_recordings_produce_no_physical_measurement(self):
        with tempfile.TemporaryDirectory() as d:
            result = analyze(Path(d))
            self.assertIsNone(result["peak_pitch_change_deg"])
            self.assertIsNone(result["balance_clipped_fraction"])
