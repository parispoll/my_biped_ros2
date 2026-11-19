"""Separate height/pressure checks for bilateral lowering; no unloading claim."""

from bisect import bisect_right
import math
from my_robot_description.sensing.foot_loads import paired_loads


def audit_lowering(
    sensors, com, raw, start, end, program, force_reference=None, pairer=None
):
    windows = program.setup["analysis_windows"]
    poses = [r["pose"] for r in sensors if start <= r["sim_time"] <= end]

    def phase(name):
        return [
            p
            for p in poses
            if windows[name][0] <= p["sim_time"] - start < windows[name][1]
        ]

    def avg(rows, i):
        if len(rows) < 20:
            raise ValueError("Missing lowering pose phase")
        return sum(p["position"][i] for p in rows) / len(rows)

    baseline = phase("baseline")
    held = phase("hold")
    tail = phase("recovery")
    height0 = avg(baseline, 2)
    x0 = avg(baseline, 0)
    drop = height0 - avg(held, 2)
    forward = avg(held, 0) - x0
    height_return = max(abs(p["position"][2] - height0) for p in tail)
    x_return = max(abs(p["position"][0] - x0) for p in tail)
    height_tracking = max(
        abs(
            p["position"][2]
            - (
                height0
                + program.sample(max(0.0, p["sim_time"] - start))["pelvis_height_m"]
                - program.table[0]["pelvis_height_m"]
            )
        )
        for p in poses
    )
    x_tracking = max(
        abs(
            p["position"][0]
            - (x0 + program.sample(max(0.0, p["sim_time"] - start))["pelvis_x_m"])
        )
        for p in poses
    )
    pairs = (pairer or paired_loads)(raw, start, end)
    times = [r["sim_time"] for r in pairs]
    gaps = [b - a for a, b in zip(times, times[1:])]
    coverage = (
        0 < min(gaps)
        and max(gaps) <= 0.05
        and times[0] - start <= 0.05
        and end - times[-1] <= 0.05
    )
    weight = program.planner.base["mass_kg"] * 9.81
    force_error = max(abs(r["total_vertical_n"] - weight) for r in pairs)
    target_share = lambda r: (
        force_reference.sample(max(0.0, r["sim_time"] - start))["predicted_force_n"][
            "left"
        ]
        / weight
        if force_reference
        else 0.5
    )
    share_error = max(
        abs(r["left"]["force_world_n"][2] / r["total_vertical_n"] - target_share(r))
        for r in pairs
    )

    def share_mean(name):
        lo, hi = windows[name]
        selected = [r for r in pairs if lo <= r["sim_time"] - start < hi]
        if len(selected) < 20:
            raise ValueError("Missing measured force phase")
        return sum(
            r["left"]["force_world_n"][2] / r["total_vertical_n"] for r in selected
        ) / len(selected)

    share_change = share_mean("hold") - share_mean("baseline")
    transfer_ok = (
        not force_reference
        or share_change * program.displacement > 0
        and abs(share_change) * weight > 1.0
    )
    crows = [r for r in com if start - 0.1 <= r["sim_time"] <= end]
    ctimes = [r["sim_time"] for r in crows]
    cop_errors = []
    for r in pairs:
        i = bisect_right(ctimes, r["sim_time"]) - 1
        if i < 0 or r["sim_time"] - ctimes[i] > 0.1:
            raise ValueError("Missing/stale COM for lowering pressure check")
        cop_errors.append(math.dist(r["cop_world_xy_m"], crows[i]["com"][:2]))
    passed = (
        coverage
        and force_error <= 0.05 * weight
        and share_error <= 0.02
        and transfer_ok
        and max(cop_errors) <= 0.003
        and abs(drop - program.lowering) <= 0.002
        and height_return <= 0.001
        and x_return <= 0.001
        and height_tracking <= 0.002
        and x_tracking <= 0.002
    )
    return dict(
        passed=passed,
        requested_lowering_m=program.lowering,
        measured_lowering_m=drop,
        measured_pelvis_x_shift_m=forward,
        expected_pelvis_x_shift_m=program.setup["endpoint"]["pelvis_x_m"],
        max_height_tracking_error_relative_baseline_m=height_tracking,
        max_x_tracking_error_m=x_tracking,
        max_final_height_offset_m=height_return,
        max_final_x_offset_m=x_return,
        force_coverage_passed=coverage,
        force_samples=len(pairs),
        max_force_gap_s=max(gaps),
        max_vertical_force_error_n=force_error,
        max_left_share_deviation=share_error,
        max_cop_com_distance_m=max(cop_errors),
        measured_left_share_change=share_change,
        load_transfer_observed=bool(transfer_ok) if force_reference else None,
        note="Measured planned lowering/return and bilateral forces; share deviation is relative to planned split. No single support or foot lift.",
    )
