"""Direction-aware measured response metrics; no ROS or plotting."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import math


def response_metrics(samples, summary, force):
    if not summary.get("completed"):
        raise ValueError("Partial trial has no completed recovery metrics")
    if not math.isfinite(force) or force == 0:
        raise ValueError("Nonzero finite force required")
    sign = 1 if force > 0 else -1
    start = next(
        e["sim_time"] for e in summary["events"] if e["event"] == "push_published"
    )
    clear = next(
        e["sim_time"] for e in summary["events"] if e["event"] == "clear_published"
    )
    response = [s for s in samples if s["sim_time"] >= start]
    recovery = [s for s in response if s["sim_time"] >= clear]
    if not recovery:
        raise ValueError("Missing recovery samples")
    baseline = summary["baseline_pitch_rad"]
    signed = lambda s: sign * (s["pitch"] - baseline) * 180 / math.pi
    return dict(
        primary_pitch_deg=max(signed(s) for s in response),
        opposite_rebound_deg=max(0.0, -min(signed(s) for s in recovery)),
        peak_pitch_rate_rad_s=max(abs(s["pitch_rate"]) for s in response),
        last_outside_band_after_clear_s=summary["last_outside_band_after_clear_s"],
        recovered=summary["passed"],
        command_window_s=clear - start,
    )
