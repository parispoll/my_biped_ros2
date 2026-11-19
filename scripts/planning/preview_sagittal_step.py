#!/usr/bin/env python3
"""Preview one geometric foot swing; never starts Gazebo or publishes commands."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
import xacro
from my_robot_description.model.stance import model_from_urdf
from my_robot_description.model.sagittal_step import preview_step, ankle_position


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--advance", type=float, default=0.03, help="Forward ankle travel in metres"
    )
    parser.add_argument(
        "--clearance", type=float, default=0.02, help="Maximum sole lift in metres"
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    model = model_from_urdf(
        xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/my_biped.urdf.xacro")
        ).toxml()
    )
    samples = preview_step(model, args.advance, args.clearance, args.duration)
    error = max(
        max(
            abs(a - b)
            for a, b in zip(
                ankle_position(model, r["swing_hip_knee_ankle_rad"]),
                r["ankle_target_xz_pitch"],
            )
        )
        for r in samples
    )
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(
        kinematics_passed=error < 1e-10,
        max_fk_target_error=error,
        joint_count=6,
        advance_m=args.advance,
        clearance_m=args.clearance,
        duration_s=args.duration,
        executed_in_simulation=False,
        walking_achieved=False,
        note="Fixed-pelvis one-foot geometric swing only. Requires support/contact/balance integration before execution.",
        samples=samples,
    )
    (folder / "preview.json").write_text(json.dumps(report, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    ax = axes[0]
    stance = samples[0]["stance_hip_knee_ankle_rad"]
    for phase, color in [(0, "#999"), (0.5, "#167d9a"), (1, "#c65442")]:
        r = samples[round(phase * (len(samples) - 1))]
        h, k, a = r["swing_hip_knee_ankle_rad"]
        import math

        knee = (-model.thigh * math.sin(h), -model.thigh * math.cos(h))
        x, z, _ = r["ankle_target_xz_pitch"]
        ax.plot(
            [0, knee[0], x],
            [0, knee[1], z],
            "-o",
            color=color,
            label=f"Phase {phase:.1f}",
        )
        ax.plot(
            [x - model.foot_length / 2, x + model.foot_length / 2],
            [z - model.foot_thickness] * 2,
            color=color,
            lw=3,
        )
    ground = samples[0]["ankle_target_xz_pitch"][1] - model.foot_thickness
    ax.axhline(ground, color="#777", ls=":")
    ax.set(
        aspect="equal",
        xlabel="Forward X relative to hip (m)",
        ylabel="Vertical Z relative to hip (m)",
        title=(
            "Swing leg: 3 cm forward, 2 cm lift"
            if args.advance == 0.03 and args.clearance == 0.02
            else "Swing leg geometry"
        ),
    )
    ax.legend(fontsize=8)
    for i, name in enumerate(["Hip", "Knee", "Ankle"]):
        axes[1].plot(
            [r["time_s"] for r in samples],
            [math.degrees(r["swing_hip_knee_ankle_rad"][i]) for r in samples],
            label=name,
        )
    axes[1].set(
        xlabel="Planned time (s)",
        ylabel="Joint angle (degrees)",
        title="Inverse kinematics targets",
    )
    axes[1].legend()
    axes[1].grid(alpha=0.2)
    fig.suptitle(
        "Offline step preview · fixed pelvis · no balance or torque validation"
    )
    fig.tight_layout()
    fig.savefig(folder / "preview.png", dpi=150)
    plt.close(fig)
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))


if __name__ == "__main__":
    main()
