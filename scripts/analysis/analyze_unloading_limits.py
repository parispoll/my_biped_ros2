#!/usr/bin/env python3
"""Offline scan of the current fixed-height, fixed-sole unloading path.

Deliberately computes beyond command budgets to identify which constraint binds.
Never sends commands or changes live experiment limits.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
import xacro
from my_robot_description.motion.lateral_shift import LateralPlanner
from my_robot_description.model.sagittal_step import leg_angles
from my_robot_description.model.roll_geometry import model_com, link_transforms
from my_robot_description.model.roll_stance import (
    JOINT_NAMES,
    MOTOR_CAPS,
    double_support_loads,
)


def point(p, d):
    roll = math.atan2(-d, -p.ankle_z)
    pitch = leg_angles(p.model, p.foot_x, -math.hypot(d, p.ankle_z), 0.0)
    q = dict(zip(JOINT_NAMES, [roll, *pitch, -roll] * 2))
    if any(not low <= q[n] <= high for n, (low, high) in p.roll_limits.items()):
        raise ValueError("Roll joint limit")
    mass, com = model_com(p.xml, q)
    y = com[1] + d
    shares = {
        "left": 0.5 + y / (2 * p.half_spacing),
        "right": 0.5 - y / (2 * p.half_spacing),
    }
    if min(shares.values()) < 0:
        raise ValueError("Negative sole-center reaction required")
    torques = double_support_loads(
        p.xml, q, {s: mass * 9.81 * f for s, f in shares.items()}
    )
    ratios = {n: abs(torques[n]) / c for n, c in zip(JOINT_NAMES, MOTOR_CAPS)}
    active = max(ratios, key=ratios.get)
    feet = link_transforms(p.xml, q)
    error = max(
        abs(
            feet[s + "_foot"][i][3] + (d if i == 1 else 0.0) - p.feet[s + "_foot"][i][3]
        )
        for s in shares
        for i in range(3)
    )
    return dict(
        displacement_m=d,
        com_y_m=y,
        lighter_fraction=min(shares.values()),
        forces_n={s: mass * 9.81 * f for s, f in shares.items()},
        torques_nm=torques,
        peak_utilization=ratios[active],
        limiting_motor=active,
        fixed_foot_error_m=error,
    )


def crossing(p, threshold):
    lo, hi = 0.0, 0.001
    while point(p, hi)["peak_utilization"] < threshold:
        lo, hi = hi, hi + 0.001
    for _ in range(35):
        mid = (lo + hi) / 2
        if point(p, mid)["peak_utilization"] < threshold:
            lo = mid
        else:
            hi = mid
    return point(p, (lo + hi) / 2)


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    p = LateralPlanner(xml)
    rows = []
    failure = None
    for i in range(1501):
        d = i * 0.0001
        try:
            rows.append(point(p, d))
        except ValueError as e:
            failure = dict(displacement_m=d, reason=str(e))
            break
    limits = {
        name: crossing(p, v)
        for name, v in [("reserved_headroom", 0.8), ("absolute_motor_cap", 1.0)]
    }
    # Independent checks: known validated target, mirror symmetry and link geometry.
    known = p.fixed_foot_pose(0.03582038838959125)
    check = point(p, 0.03582038838959125)
    assert abs(check["lighter_fraction"] - 0.35) < 1e-10
    assert (
        max(
            abs(check["torques_nm"][n] - v)
            for n, v in zip(JOINT_NAMES, known["feedforward_nm"])
        )
        < 1e-10
    )
    for r in rows:
        mirror = point(p, -r["displacement_m"])
        assert abs(mirror["lighter_fraction"] - r["lighter_fraction"]) < 1e-10
        assert abs(mirror["peak_utilization"] - r["peak_utilization"]) < 1e-10
        assert r["fixed_foot_error_m"] < 1e-10
    report = dict(
        scope="Static sole-center reactions, upright fixed-height pelvis and fixed soles; not a search over all possible poses or foot pressure distributions",
        mass_kg=p.base["mass_kg"],
        limits=limits,
        last_geometrically_reachable=rows[-1],
        first_geometry_failure=failure,
        checks=dict(known_target=True, mirrored_scan=True, fixed_foot_geometry=True),
        samples=rows,
    )
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "limits.json").write_text(json.dumps(report, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    x = [1000 * r["displacement_m"] for r in rows]
    ax[0].plot(x, [100 * r["lighter_fraction"] for r in rows])
    ax[0].axhline(0, color="gray", ls=":")
    ax[0].set(
        ylabel="Lighter-foot share (%)",
        title="Current fixed-height path: geometry and torque limits",
    )
    for n, c in zip(JOINT_NAMES[:5], MOTOR_CAPS[:5]):
        ax[1].plot(x, [100 * abs(r["torques_nm"][n]) / c for r in rows], label=n)
    ax[1].axhline(80, color="gray", ls="--", label="20% reserve")
    ax[1].axhline(100, color="black", ls=":", label="Motor cap")
    ax[1].set(xlabel="Pelvis shift (mm)", ylabel="Loaded-leg static torque / cap (%)")
    ax[1].legend(fontsize=8)
    for axis in ax:
        for name, r in limits.items():
            axis.axvline(1000 * r["displacement_m"], ls="--", alpha=0.6, label=name)
        axis.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(args.output / "limits.png", dpi=150)
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))


if __name__ == "__main__":
    main()
