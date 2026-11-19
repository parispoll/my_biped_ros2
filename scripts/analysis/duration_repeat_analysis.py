"""Verify duration-study evidence and describe two observed repeats per direction."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import json
import math
from statistics import mean
from scripts.studies.run_duration_comparison import compare


def verify(folder, sources, direction):
    folder = Path(folder)
    study = json.loads((folder / "comparison.json").read_text())
    if not study.get("completed") or study.get("source_hashes") != sources:
        raise ValueError("Incomplete duration comparison or changed source snapshot")
    cases = study["cases"]
    if sorted(c["duration_s"] for c in cases) != [0.2, 0.3]:
        raise ValueError("Expected one .20 s and one .30 s case")
    force = 2.0 if direction == "forward" else -2.0
    reports = []
    for case in cases:
        if case.get("runner_returncode") != 0:
            raise ValueError("Failed experiment process")
        outcome = json.loads((Path(case["path"]) / "summary.json").read_text())
        if not outcome.get("completed") or not outcome.get("passed"):
            raise ValueError("Incomplete or failed recovery case")
        for phase in ("reference", "push"):
            trial = outcome[phase]
            if trial["requested"]["force_x"] != (
                0.0 if phase == "reference" else force
            ):
                raise ValueError("Wrong force or direction")
            if trial["requested"]["duration"] != case["duration_s"]:
                raise ValueError("Wrong pulse duration")
            if (
                trial["damping_model"] != "explicit"
                or trial["controller_mode"] != "pid"
                or not trial["balance_enabled"]
                or trial["physics_step_requested_s"] != 0.001
                or trial["real_time_factor_requested"] != 0.25
            ):
                raise ValueError("Changed controller/physics protocol")
            if (
                trial["control_gap_frozen_samples"]
                or not trial["balance_observer"]["passed"]
                or not trial["viscous_damping_audit"]["passed"]
            ):
                raise ValueError("Invalid measurement audit")
            handling = trial["force_handling"]
            if not all(
                handling[k]
                for k in (
                    "queued_in_gazebo",
                    "cleared_in_gazebo",
                    "clear_observed_while_recording",
                )
            ):
                raise ValueError("Force handling not confirmed")
        reports.append((case["duration_s"], outcome))
    return dict(
        path=str(folder.resolve()), direction=direction, responses=compare(reports)
    )


def describe(entries):
    result = {}
    for direction in ("forward", "backward"):
        group = [e for e in entries if e["direction"] == direction]
        if len(group) != 2 or {e["repeat"] for e in group} != {1, 2}:
            raise ValueError("Require two distinct repeat IDs per direction")
        if len({e["path"] for e in group}) != 2:
            raise ValueError("Cannot count the same comparison twice")
        effects = []
        values = {0.2: [], 0.3: []}
        for entry in sorted(group, key=lambda e: e["repeat"]):
            rows = entry["responses"]
            if len(rows) != 2 or {r["duration_s"] for r in rows} != {0.2, 0.3}:
                raise ValueError("Missing or duplicate duration")
            by_duration = {r["duration_s"]: r for r in rows}
            for duration, row in by_duration.items():
                peak = row["peak_pitch_deg"]
                if not math.isfinite(peak) or peak <= 0:
                    raise ValueError("Positive finite peak pitch required")
                if row["force_x_n"] != (2.0 if direction == "forward" else -2.0):
                    raise ValueError("Mismatched force direction")
                if not row["recovery_passed"]:
                    raise ValueError(
                        "Recovery failure cannot be counted as a successful repeat"
                    )
                values[duration].append(peak)
            effects.append(
                (
                    by_duration[0.3]["peak_pitch_deg"]
                    / by_duration[0.2]["peak_pitch_deg"]
                    - 1
                )
                * 100
            )
        result[direction] = dict(
            repeats=2,
            peak_pitch_deg={
                str(d): dict(values=v, mean=mean(v), min=min(v), max=max(v))
                for d, v in values.items()
            },
            paired_peak_increase_percent=effects,
            paired_peak_increase_range_percent=[min(effects), max(effects)],
            longer_pulse_increased_peak_in_both=all(value > 0 for value in effects),
        )
    return result
