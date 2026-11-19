#!/usr/bin/env python3
"""Audit/plot captured contact forces; never synthesize missing measurements."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
from bisect import bisect_right
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
from my_robot_description.sensing.foot_loads import audit_foot_loads, paired_loads


def read(path):
    return [json.loads(s) for s in path.read_text().splitlines()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    p = args.folder
    s = json.loads((p / "summary.json").read_text())
    if not all(s[k]["passed"] for k in ["shift", "sensors", "physical_damping"]):
        raise ValueError(
            "Motion/sensor/damping checks must pass before validating these loads"
        )
    start, end = s["shift"]["origin_sim_time"], s["shift"]["end_sim_time"]
    com = read(p / "com.jsonl")
    raw = read(p / "contact-wrenches.jsonl")
    windows = s.get("unloading_plan", {}).get("analysis_windows")
    report = audit_foot_loads(
        raw,
        com,
        start,
        end,
        s["plan"]["mass_kg"],
        s["requested_pelvis_shift_m"],
        phase_windows=windows,
    )
    if s.get("load_guard", {}).get("passed"):
        report["used_for_control"] = (
            "bounded pelvis correction and safety guard"
            if s.get("load_feedback", {}).get("enabled")
            else "safety guard only"
        )
    (p / "foot-load-audit.json").write_text(json.dumps(report, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pairs = paired_loads(raw, start, end)
    t = [r["sim_time"] - start for r in pairs]
    weight = report["expected_weight_n"]
    schedule = s.get("unloading_plan", {}).get(
        "schedule",
        [
            ("baseline", 3),
            ("move_out", 4),
            ("hold", 6),
            ("move_back", 4),
            ("recovery", 8),
        ],
    )
    boundaries = []
    elapsed = 0.0
    hold_span = None
    for name, duration in schedule:
        if name == "hold":
            hold_span = (elapsed, elapsed + duration)
        elapsed += duration
        boundaries.append(elapsed)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    for side, color in [("left", "#167d9a"), ("right", "#c77a22")]:
        values = [r[side]["force_world_n"][2] for r in pairs]
        axes[0, 0].plot(t, values, color=color, label=side + " · measured")
        axes[1, 0].plot(
            t,
            [100 * v / r["total_vertical_n"] for v, r in zip(values, pairs)],
            color=color,
            label=side,
        )
    axes[0, 1].plot(
        t,
        [r["total_vertical_n"] for r in pairs],
        color="#167d9a",
        label="Measured left + right",
    )
    axes[0, 1].axhline(weight, color="#444", ls="--", label=f"m × g = {weight:.4f} N")
    axes[0, 1].set_ylim(weight * 0.95, weight * 1.05)
    axes[1, 0].axhline(50, color="#aaa", ls=":")
    com_times = [r["sim_time"] for r in com]
    matched = [com[bisect_right(com_times, r["sim_time"]) - 1]["com"][1] for r in pairs]
    baseline = [i for i, v in enumerate(t) if 1 <= v < 2.5]
    mean = lambda values: sum(values) / len(values)
    cop0 = mean([pairs[i]["cop_world_xy_m"][1] for i in baseline])
    com0 = mean([matched[i] for i in baseline])
    axes[1, 1].plot(
        t,
        [(r["cop_world_xy_m"][1] - cop0) * 1000 for r in pairs],
        color="#167d9a",
        label="CoP from measured point forces",
    )
    axes[1, 1].plot(
        t,
        [(v - com0) * 1000 for v in matched],
        color="#444",
        ls="--",
        label="Independent simulated COM",
    )
    for ax, title, ylabel in zip(
        axes.flat,
        [
            "Vertical ground reaction per foot",
            "Total vertical support versus weight",
            "Measured vertical load shares",
            "Pressure center versus center of mass",
        ],
        ["N", "N", "Percent of total", "Lateral change (mm)"],
    ):
        ax.set(
            title=title,
            ylabel=ylabel,
            xlabel="Simulation seconds after readiness",
            xlim=(0, end - start),
        )
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
        for v in boundaries[:-1]:
            ax.axvline(v, color="#aaa", alpha=0.3, ls=":")
        ax.axvspan(*hold_span, color="#167d9a", alpha=0.06)
    if s.get("unloading_plan"):
        axes[0, 0].axhline(
            0.25 * weight, color="#999", ls=":", label="Live 25% support floor"
        )
        axes[0, 0].legend(fontsize=8)
        caption = "Force-guarded partial unloading · both feet grounded · no motor-cap increase"
    else:
        caption = "Bilateral shift · same controller; read-only force measurement"
    fig.suptitle(
        "Validated simulated foot loads · contact-wrench measurements\n" + caption
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(p / "foot-loads.png", dpi=150)
    plt.close(fig)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
