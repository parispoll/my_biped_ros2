#!/usr/bin/env python3
"""Compare independently captured forces in matched biased-reference trials."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import ast
import hashlib
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
from my_robot_description.sensing.foot_loads import paired_loads


def read(p):
    return [json.loads(x) for x in p.read_text().splitlines()]


def case(folder):
    summary = json.loads((folder / "summary.json").read_text())
    if not summary["passed"] or not summary.get("source_unchanged"):
        raise ValueError("Every original trial audit must pass")
    start, end = summary["shift"]["origin_sim_time"], summary["shift"]["end_sim_time"]
    setup = summary["unloading_plan"]
    fraction = setup["target_unloaded_fraction"]
    target = 1 - fraction if setup["direction"] == "left" else fraction
    pairs = paired_loads(read(folder / "contact-wrenches.jsonl"), start, end)
    lo, hi = setup["analysis_windows"]["hold"]
    held = [p for p in pairs if lo <= p["sim_time"] - start < hi]
    if len(held) < 50:
        raise ValueError("Insufficient independent late-hold forces")
    errors = [
        target - r["left"]["force_world_n"][2] / r["total_vertical_n"] for r in held
    ]
    metrics = dict(
        folder=str(folder),
        enabled=setup["feedback_enabled"],
        direction=setup["direction"],
        target_left_fraction=target,
        independent_force_samples=len(held),
        mean_absolute_error_fraction=sum(abs(e) for e in errors) / len(errors),
        mean_signed_error_fraction=sum(errors) / len(errors),
        peak_correction_m=summary["load_feedback"]["peak_correction_m"],
        peak_motor_utilization=summary["load_guard"]["peak_measured_motor_utilization"],
        final_com_peak_offset_m=summary["shift"]["max_final_com_offset_m"],
    )
    return summary, pairs, read(folder / "joints.jsonl"), metrics


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("off", type=Path)
    a.add_argument("on", type=Path)
    a.add_argument("--mirror", type=Path)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    off = case(args.off)
    on = case(args.on)
    cases = [off, on]
    if off[3]["enabled"] or not on[3]["enabled"]:
        raise ValueError("Expected feedback OFF then ON")
    for key in ["actuator_profile", "real_time_factor"]:
        if off[0][key] != on[0][key]:
            raise ValueError("Comparator model/timing mismatch")
    for key in [
        "direction",
        "reference_bias_world_m",
        "schedule",
        "nominal_displacement_m",
    ]:
        if off[0]["unloading_plan"][key] != on[0]["unloading_plan"][key]:
            raise ValueError("Comparator motion/bias mismatch")
    if (args.off / "joints.gains.json").read_bytes() != (
        args.on / "joints.gains.json"
    ).read_bytes():
        raise ValueError("Comparator inner gains differ")
    hashes = [
        json.loads((p / "source-hashes.json").read_text()) for p in [args.off, args.on]
    ]
    changes = [
        k
        for k in set(hashes[0]) | set(hashes[1])
        if hashes[0].get(k) != hashes[1].get(k)
    ]
    inactive_gain_change = False
    if changes:
        key = "src/my_robot_description/my_robot_description/motion/load_feedback.py"
        if changes != [key]:
            raise ValueError(
                "Comparator model/controller sources differ beyond the disabled outer-loop constants"
            )
        old_source = (args.off / "load_feedback-source.py").read_bytes()
        new_path = args.on / "load_feedback-source.py"
        new_source = (
            new_path.read_bytes() if new_path.exists() else (ROOT / key).read_bytes()
        )
        if (
            hashlib.sha256(old_source).hexdigest() != hashes[0][key]
            or hashlib.sha256(new_source).hexdigest() != hashes[1][key]
        ):
            raise ValueError("Saved source does not match trial hashes")

        def normalized(source):
            tree = ast.parse(source)
            for node in tree.body:
                if isinstance(node, ast.Assign) and any(
                    isinstance(t, ast.Name)
                    and t.id in ("GAIN_M_S_PER_SHARE", "FILTER_TIME_S")
                    for t in node.targets
                ):
                    node.value = ast.Constant(None)
            return ast.dump(tree, include_attributes=False)

        if normalized(old_source) != normalized(new_source):
            raise ValueError(
                "Changes affect more than inactive feedback gain/filter constants"
            )
        replay = json.loads((args.off / "reaudit.json").read_text())
        if (
            not replay["passed"]
            or not replay["load_feedback"]["passed"]
            or replay["load_feedback"]["enabled"]
        ):
            raise ValueError("Disabled comparator must pass corrected-state replay")
        inactive_gain_change = True
    if args.mirror:
        cases.append(case(args.mirror))
        mirror = cases[-1][0]
        if (
            mirror["actuator_profile"] != on[0]["actuator_profile"]
            or mirror["real_time_factor"] != on[0]["real_time_factor"]
        ):
            raise ValueError("Mirror actuator/timing mismatch")
        if json.loads((args.mirror / "source-hashes.json").read_text()) != hashes[1]:
            raise ValueError("Mirror package sources differ")
        if (args.mirror / "joints.gains.json").read_bytes() != (
            args.on / "joints.gains.json"
        ).read_bytes():
            raise ValueError("Mirror inner gains differ")
        if (
            abs(
                mirror["unloading_plan"]["reference_bias_world_m"]
                + on[0]["unloading_plan"]["reference_bias_world_m"]
            )
            > 1e-12
        ):
            raise ValueError("Mirror bias magnitude/sign mismatch")
    initial = off[3]["mean_absolute_error_fraction"]
    final = on[3]["mean_absolute_error_fraction"]
    if initial < 0.005:
        raise ValueError("Comparator error too small for this bias-rejection test")
    reduction = 1 - final / initial
    mirror_ok = not args.mirror or (
        cases[2][3]["enabled"]
        and cases[2][3]["direction"] != on[3]["direction"]
        and cases[2][3]["mean_absolute_error_fraction"] < initial * 0.5
    )
    report = dict(
        passed=reduction >= 0.5 and mirror_ok,
        disabled_comparator_gain_filter_change_verified=inactive_gain_change,
        late_hold_error_reduction=reduction,
        metrics=[c[3] for c in cases],
        comparison="Same model, inner gains, timing and 3 mm reference offset; raw contact-force late-hold errors against nominal load target",
        scope="Bilateral reference-error rejection; not external push recovery or single-support balance",
    )
    (args.output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    colors = ["#888", "#167d9a", "#c77a22"]
    for i, (s, pairs, rows, m) in enumerate(cases):
        start = s["shift"]["origin_sim_time"]
        label = ("Off", "On", "On · mirrored")[i]
        color = colors[i]
        axes[0].plot(
            [p["sim_time"] - start for p in pairs],
            [
                100 * p["left"]["force_world_n"][2] / p["total_vertical_n"]
                for p in pairs
            ],
            label=label,
            color=color,
        )
        selected = [
            r
            for r in rows
            if r.get("load_feedback") is not None
            and start <= r["sim_time"] <= s["shift"]["end_sim_time"]
        ]
        axes[1].plot(
            [r["sim_time"] - start for r in selected],
            [1000 * r["load_feedback"]["correction_m"] for r in selected],
            label=label,
            color=color,
        )
        axes[2].bar(
            i, 100 * m["mean_absolute_error_fraction"], color=color, label=label
        )
    nominal = [r for r in off[2] if r.get("shift") is not None]
    start = off[0]["shift"]["origin_sim_time"]
    axes[0].plot(
        [r["sim_time"] - start for r in nominal],
        [
            100
            * r["shift"]["predicted_force_n"]["left"]
            / sum(r["shift"]["predicted_force_n"].values())
            for r in nominal
        ],
        "--",
        color="black",
        label="Nominal leftward reference",
    )
    if args.mirror:
        mirror_rows = [r for r in cases[2][2] if r.get("shift") is not None]
        mirror_start = cases[2][0]["shift"]["origin_sim_time"]
        axes[0].plot(
            [r["sim_time"] - mirror_start for r in mirror_rows],
            [
                100
                * r["shift"]["predicted_force_n"]["left"]
                / sum(r["shift"]["predicted_force_n"].values())
                for r in mirror_rows
            ],
            "--",
            color="#c77a22",
            label="Nominal mirrored reference",
        )
    axes[0].set(
        title="Independent measured left-foot share",
        xlabel="Simulation seconds after readiness",
        ylabel="% of vertical support",
    )
    axes[0].axvspan(17, 19, color="#167d9a", alpha=0.08)
    axes[1].axhline(4, color="#777", ls=":")
    axes[1].axhline(-4, color="#777", ls=":")
    axes[1].set(
        title="Bounded pelvis correction",
        xlabel="Simulation seconds after readiness",
        ylabel="mm",
    )
    axes[2].set(
        title="Late-hold mean absolute load-share error", ylabel="Percentage points"
    )
    axes[2].set_xticks(
        range(len(cases)), ["Off", "On"] + (["On mirrored"] if args.mirror else [])
    )
    for ax in axes:
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    fig.suptitle(
        f"Measured load-share feedback · leftward error reduction {100*reduction:.1f}%\nSame injected reference offset; motor caps and safety guards unchanged"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(args.output / "comparison.png", dpi=150)
    plt.close(fig)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
