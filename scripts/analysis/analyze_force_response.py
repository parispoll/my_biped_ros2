#!/usr/bin/env python3
"""Diagnose one measured trial, including partial guard-stopped recordings."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math


def rows(path):
    if not path.exists():
        return []
    result = []
    for line in path.read_text().splitlines():
        try:
            result.append(json.loads(line))
        except ValueError:
            pass  # A killed process may leave its last line incomplete.
    return result


def analyze(folder):
    recording = folder / "push" / "recording"
    samples = rows(recording / "samples.jsonl")
    events = (
        json.loads((recording / "events.json").read_text())
        if (recording / "events.json").exists()
        else []
    )
    push = next((e["sim_time"] for e in events if e["event"] == "push_published"), None)
    clear = next(
        (e["sim_time"] for e in events if e["event"] == "clear_published"), None
    )
    controller = rows(folder / "push" / "controller.jsonl")
    end = samples[-1]["sim_time"] if samples else None
    controlled = [
        r
        for r in controller
        if push is not None and end is not None and push <= r["sim_time"] <= end
    ]
    response = [r for r in samples if push is not None and r["sim_time"] >= push]
    baseline = [r for r in samples if r["phase"] == "baseline"]
    pitch0 = sum(r["pitch"] for r in baseline) / len(baseline) if baseline else None
    clipped = []
    for r in controlled:
        settings = r.get("balance_settings", {})
        raw = 0.5 * (
            settings.get("kp", 20)
            * (r.get("balance_pitch_rad", 0) - settings.get("target_pitch", 0))
            + settings.get("kd", 4) * r.get("balance_rate_rad_s", 0)
        )
        if r.get("balance_gate") and abs(raw) > settings.get("ankle_limit", 0.5) + 1e-9:
            clipped.append(r)
    summary = dict(
        push_sim_time=push,
        clear_sim_time=clear,
        recorded_response_samples=len(response),
        controller_response_samples=len(controlled),
        balance_clipped_samples=len(clipped),
        balance_clipped_fraction=len(clipped) / len(controlled) if controlled else None,
        balance_gate_off_samples=sum(
            not r.get("balance_gate", False) for r in controlled
        ),
        motor_saturated_samples=sum(
            any(abs(a - b) > 1e-9 for a, b in zip(r["requested"], r["command"]))
            for r in controlled
        ),
        peak_abs_balance_ankle_nm=max(
            (abs(r["balance_nm"][2]) for r in controlled), default=None
        ),
        peak_abs_motor_ankle_nm=max(
            (abs(r["command"][2]) for r in controlled), default=None
        ),
        peak_pitch_change_deg=(
            max(
                (abs(r["pitch"] - pitch0) * 180 / math.pi for r in response),
                default=None,
            )
            if pitch0 is not None
            else None
        ),
        peak_pitch_rate_rad_s=max(
            (abs(r["pitch_rate"]) for r in response), default=None
        ),
        min_height_m=min((r["height"] for r in response), default=None),
        contact_unknown_samples={
            side: sum(r["contacts"][side]["ground_contact"] is None for r in response)
            for side in ("left", "right")
        },
        contact_non_ground_samples={
            side: sum(r["contacts"][side]["ground_contact"] is False for r in response)
            for side in ("left", "right")
        },
        note="Clipping is a candidate bottleneck, not proof that raising a gain or cap improves recovery. Partial traces do not establish completed recovery.",
    )
    (folder / "diagnosis.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    print(json.dumps(analyze(parser.parse_args().folder), indent=2))
