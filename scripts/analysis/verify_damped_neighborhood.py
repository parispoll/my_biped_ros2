#!/usr/bin/env python3
"""Verify saved neighborhood evidence, including control math in partial trials."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
from scripts.analysis.analyze_force_response import rows
from scripts.studies.run_duration_comparison import source_snapshot
from my_robot_description.sensing.balance import BalanceConfig
from my_robot_description.control.balance_control import active_correction


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    folder = parser.parse_args().folder
    study = json.loads((folder / "study.json").read_text())
    assert study["completed"] and len(study["cases"]) == 8
    assert source_snapshot() == study["source_hashes"]
    caps = [6, 10, 6] * 2
    verified = []
    for case in study["cases"]:
        p = Path(case["path"])
        summary = json.loads((p / "summary.json").read_text())
        assert summary["reference"]["passed"]
        launch = (p / "push/launch.log").read_text()
        assert (
            "Queueing persistent wrench:" in launch
            and "Clearing persistent wrench" in launch
        )
        events = case["events"]
        start = next(e["sim_time"] for e in events if e["event"] == "push_published")
        clear = next(e["sim_time"] for e in events if e["event"] == "clear_published")
        assert clear > start
        if case["outcome"] == "posture_guard_stopped_partial_trial":
            assert (
                not summary["completed"] and "Pelvis tilt exceeded 0.15 rad" in launch
            )
        else:
            assert summary["completed"]
        controller = rows(p / "push/controller.jsonl")
        assert controller
        setup = json.loads((p / "push/controller.setup.json").read_text())
        assert setup["motor_limits_nm"] == caps
        residual = 0.0
        for r in controller:
            assert (
                r["balance_settings"]["kd"] == 8
                and r["balance_settings"]["kp"] == 20
                and r["balance_settings"]["ankle_limit"] == 1.5
            )
            calc = active_correction(
                r["balance_pitch_rad"],
                r["balance_rate_rad_s"],
                r["balance_imu_age"],
                r["balance_contacts"],
                r["balance_elapsed"],
                BalanceConfig(**r["balance_settings"]),
                r["balance_ramp_seconds"],
            )
            assert (
                max(abs(a - b) for a, b in zip(calc["balance_nm"], r["balance_nm"]))
                < 1e-12
            )
            for i, cap in enumerate(caps):
                expected = sum(
                    r[k][i]
                    for k in [
                        "feedforward",
                        "proportional",
                        "damping",
                        "integral",
                        "balance_nm",
                    ]
                )
                assert all(
                    math.isfinite(v)
                    for v in [expected, r["requested"][i], r["command"][i]]
                )
                residual = max(
                    residual,
                    abs(expected - r["requested"][i]),
                    abs(max(-cap, min(cap, expected)) - r["command"][i]),
                )
        assert residual < 1e-9
        response = [r for r in controller if r["sim_time"] >= start]
        com = [r for r in rows(p / "push/com.jsonl") if r["sim_time"] >= start]
        assert com and all(
            abs(r["mass_kg"] - 14.1) < 1e-9 and all(math.isfinite(v) for v in r["com"])
            for r in com
        )
        verified.append(
            dict(
                name=case["name"],
                outcome=case["outcome"],
                reference_passed=True,
                actual_gain_and_motor_caps_verified=True,
                max_motor_composition_error_nm=residual,
                command_window_s=clear - start,
                force_queued_and_cleared_in_gazebo=True,
                response_motor_saturated_samples=sum(
                    any(abs(a - b) > 1e-9 for a, b in zip(r["requested"], r["command"]))
                    for r in response
                ),
                response_integrator_frozen_feedback_gap_samples=sum(
                    r.get("integral_frozen_gap", False) for r in response
                ),
                response_balance_gate_off_samples=sum(
                    not r["balance_gate"] for r in response
                ),
                response_com_samples=len(com),
                note="Partial-trial checks do not certify full recovery or full COM coverage.",
            )
        )
    result = dict(
        passed=True,
        source_unchanged=True,
        cases=verified,
        guard_stop_count=sum(
            c["outcome"] == "posture_guard_stopped_partial_trial" for c in verified
        ),
        note="Assertions verify saved evidence. Guard outcomes remain partial; no falling threshold established.",
    )
    (folder / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
