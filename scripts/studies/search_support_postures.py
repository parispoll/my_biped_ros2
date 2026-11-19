#!/usr/bin/env python3
"""Search static pelvis/pressure candidates offline; sends no robot commands."""

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
import numpy as np
from my_robot_description.studies.support_feasibility import SupportSearch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    search = SupportSearch(xml)
    fractions = [0.35, 0.3, 0.25, 0.2, 0.1, 0.05, 0.01, 0.0]
    records = []
    rejected = 0
    for lower in np.linspace(0, 0.06, 7):
        for shift in np.linspace(0, 0.12, 25):
            try:
                pose = search.pose(float(shift), float(lower))
            except ValueError:
                rejected += 1
                continue
            for fraction in fractions:
                r = search.solve(pose, fraction)
                if r:
                    records.append(r)
    best = {
        str(f): min(
            (r for r in records if r["right_fraction"] == f),
            key=lambda r: r["peak_utilization"],
            default=None,
        )
        for f in fractions
    }
    report = dict(
        scope="Sampled static upright-pelvis family; fixed flat soles, vertical reactions only, no yaw moment; no dynamics, collision certification or single-support measurement validation",
        grid=dict(
            lowering_m=[0, 0.06, 0.01],
            lateral_shift_m=[0, 0.12, 0.005],
            right_fractions=fractions,
        ),
        sole_edge_margin_m=search.margin,
        motor_caps_unchanged=True,
        reserve_utilization=0.8,
        rejected_geometry_points=rejected,
        best=best,
        samples=records,
    )
    (args.output / "search.json").write_text(json.dumps(report, indent=2) + "\n")
    from scripts.plots.plot_support_search import plot

    plot(report, args.output)
    print(
        json.dumps(
            {
                k: (
                    None
                    if r is None
                    else {
                        n: r[n]
                        for n in [
                            "pelvis_shift_m",
                            "pelvis_lowering_m",
                            "peak_utilization",
                            "pressure_offsets",
                            "torques_nm",
                        ]
                    }
                )
                for k, r in best.items()
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
