#!/usr/bin/env python3
"""Offline asymmetric-leg and pelvis-roll support search; no robot commands."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
import numpy as np
import xacro
from my_robot_description.studies.support_feasibility import SupportSearch


def scan(search, shifts, lowers, rolls, fractions):
    rows = []
    rejected = 0
    for lower in lowers:
        for shift in shifts:
            for roll in rolls:
                try:
                    p = search.lean_pose(float(shift), float(lower), float(roll))
                except ValueError:
                    rejected += 1
                    continue
                for fraction in fractions:
                    r = search.solve(p, fraction)
                    if r:
                        rows.append(r)
    return rows, rejected


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    search = SupportSearch(xml)
    fractions = [0.25, 0.1, 0.05, 0.0]
    rows, rejected = scan(
        search,
        np.linspace(0, 0.12, 13),
        np.linspace(0, 0.06, 4),
        np.linspace(-0.35, 0.35, 15),
        fractions,
    )
    best = {
        str(f): min(
            (r for r in rows if r["right_fraction"] == f),
            key=lambda r: r["peak_utilization"],
            default=None,
        )
        for f in fractions
    }
    refined = []
    for fraction in fractions:
        seed = best[str(fraction)]
        if seed is None:
            continue
        local, rejected_local = scan(
            search,
            np.linspace(
                max(0.0, seed["pelvis_shift_m"] - 0.01),
                min(0.12, seed["pelvis_shift_m"] + 0.01),
                9,
            ),
            np.linspace(
                max(0.0, seed["pelvis_lowering_m"] - 0.01),
                min(0.06, seed["pelvis_lowering_m"] + 0.01),
                5,
            ),
            np.linspace(
                max(-0.35, seed["pelvis_roll_rad"] - 0.05),
                min(0.35, seed["pelvis_roll_rad"] + 0.05),
                5,
            ),
            [fraction],
        )
        refined.extend(local)
    rows.extend(refined)
    best = {
        str(f): min(
            (r for r in rows if r["right_fraction"] == f),
            key=lambda r: r["peak_utilization"],
            default=None,
        )
        for f in fractions
    }
    # Static samples of a transfer toward the best 75/25 candidate. These are
    # not a time trajectory or a dynamically validated pressure controller.
    endpoint = min(
        (
            r
            for r in rows
            if r["right_fraction"] == 0.25 and abs(r["pelvis_roll_rad"]) <= 0.1 + 1e-12
        ),
        key=lambda r: r["peak_utilization"],
        default=None,
    )
    path = []
    path_failure = None
    if endpoint:
        reference = {
            side: dict(x_offset_m=0.0, y_offset_m=0.0) for side in ("left", "right")
        }
        for u in np.linspace(0, 1, 101):
            try:
                p = search.lean_pose(
                    endpoint["pelvis_shift_m"] * u,
                    endpoint["pelvis_lowering_m"] * u,
                    endpoint["pelvis_roll_rad"] * u,
                )
                r = search.solve(
                    p,
                    0.5 - 0.25 * u,
                    pressure_reference=reference,
                    pressure_step=0.0 if u == 0 else 0.002,
                )
                if r is None:
                    raise ValueError("No pressure solution")
                path.append(dict(progress=float(u), **r))
                reference = r["pressure_offsets"]
            except ValueError as e:
                path_failure = dict(progress=float(u), reason=str(e))
                break
    report = dict(
        scope="Static upright-pitch pelvis with world-X roll and independent leg IK; fixed flat soles, vertical reactions, 10 mm sole-edge reserve; not full posture/dynamic/collision search",
        coarse_grid=dict(
            shift_m=[0, 0.12, 0.01],
            lowering_m=[0, 0.06, 0.02],
            pelvis_roll_rad=[-0.35, 0.35, 0.05],
        ),
        local_refinement="Each coarse best: shift ±10 mm / 9 points; lowering ±10 mm / 5 points; roll ±0.05 rad / 5 points, clipped to coarse bounds",
        coarse_geometry_rejected=rejected,
        best=best,
        samples=rows,
        partial_transfer=dict(
            target_right_fraction=0.25,
            selected_endpoint=endpoint,
            planning_roll_limit_rad=0.1,
            samples=path,
            failure=path_failure,
            complete=len(path) == 101,
            pressure_step_limit_m=0.002,
            pressure_starts_at_sole_centers=True,
            peak_static_utilization=max(
                (r["peak_utilization"] for r in path), default=None
            ),
            warning="Pointwise static solutions only; pressure continuity, dynamics and collision clearance not certified",
        ),
    )
    (args.output / "search.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {
                f: (
                    None
                    if r is None
                    else {
                        k: r[k]
                        for k in [
                            "pelvis_shift_m",
                            "pelvis_lowering_m",
                            "pelvis_roll_rad",
                            "peak_utilization",
                            "torques_nm",
                        ]
                    }
                )
                for f, r in best.items()
            },
            indent=2,
        ),
        flush=True,
    )
    print(
        "Partial transfer: "
        + str(
            {
                k: v
                for k, v in report["partial_transfer"].items()
                if k not in ("samples", "selected_endpoint")
            }
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
