"""World-frame ground forces from this Harmonic foot-first contact schema.

Only source-stamped foot->ground manifolds are accepted. Missing wrench data,
unsupported ordering/frames, or broken action/reaction are errors, never zero.
No COM or assumed load split is used to construct the measured forces.
"""

from bisect import bisect_right
import math

# A contact solver may leave tiny tangential impulses when normal load reaches
# zero. This is a numerical decoding floor, not the 0.5 N unloaded-foot guard.
ZERO_FORCE_TOLERANCE_N = 1e-6
ZERO_MOMENT_TOLERANCE_NM = 1e-6


def vector(value):
    result = tuple(float(value[k]) for k in ["x", "y", "z"])
    if not all(math.isfinite(v) for v in result):
        raise ValueError("Nonfinite contact vector")
    return result


def decode_foot_load(row, *, allow_zero=False):
    side = row["side"]
    message = row["message"]
    if side not in ["left", "right"]:
        raise ValueError("Unknown foot")
    header = message["header"]
    stamp = header["stamp"]["sec"] + header["stamp"]["nanosec"] / 1e9
    if not math.isfinite(stamp) or stamp <= 0 or abs(stamp - row["sim_time"]) > 1e-9:
        raise ValueError("Invalid contact source timestamp")
    if header["frame_id"] not in ["", "world"]:
        raise ValueError("Unsupported contact frame")
    contacts = message["contacts"]
    if not contacts:
        raise ValueError(
            "No contact: bilateral force validation does not infer a zero load"
        )
    total = [0.0, 0.0, 0.0]
    moment = [0.0, 0.0, 0.0]
    weighted_xy = [0.0, 0.0]
    points = 0
    residual = 0.0
    peak_point_force = 0.0
    pairs = set()
    for c in contacts:
        pair = (c["collision1"]["name"], c["collision2"]["name"])
        if pair[0].split("::")[
            -1
        ] != f"{side}_foot_collision" or "ground_plane" not in pair[1].split("::"):
            raise ValueError(
                "Expected foot-first ground-second collision ordering; do not guess a force sign"
            )
        if pair in pairs:
            raise ValueError("Duplicate foot/ground manifold")
        pairs.add(pair)
        n = len(c["positions"])
        if n < 1 or any(len(c[k]) != n for k in ["normals", "depths", "wrenches"]):
            raise ValueError("Missing/incomplete per-point contact force data")
        for pos, normal, depth, w in zip(
            c["positions"], c["normals"], c["depths"], c["wrenches"]
        ):
            p = vector(pos)
            normal = vector(normal)
            if (
                not math.isfinite(depth)
                or abs(p[2]) > 0.003
                or abs(normal[2] - 1.0) > 1e-6
                or abs(normal[0]) + abs(normal[1]) > 1e-6
            ):
                raise ValueError("Expected contacts on the horizontal world ground")
            if w["header"]["frame_id"] not in ["", "world"]:
                raise ValueError("Unsupported wrench frame")
            first = vector(w["body_1_wrench"]["force"])
            second = vector(w["body_2_wrench"]["force"])
            peak_point_force = max(
                peak_point_force,
                math.sqrt(sum(v * v for v in first)),
                math.sqrt(sum(v * v for v in second)),
            )
            error = math.sqrt(sum((a + b) ** 2 for a, b in zip(first, second)))
            residual = max(residual, error)
            if error > 1e-8:
                raise ValueError("Contact forces are not equal and opposite")
            if first[2] < -1e-8:
                raise ValueError(
                    "Negative upward foot force: unsupported sign convention or invalid contact"
                )
            for i in range(3):
                total[i] += first[i]
            moment[0] += p[1] * first[2] - p[2] * first[1]
            moment[1] += p[2] * first[0] - p[0] * first[2]
            moment[2] += p[0] * first[1] - p[1] * first[0]
            for i in range(2):
                weighted_xy[i] += p[i] * first[2]
            points += 1
    raw_total, raw_moment = total.copy(), moment.copy()
    normalized_zero = False
    if total[2] <= 0:
        # Require EVERY point to be negligible: opposing large forces must not
        # cancel into a falsely accepted zero net load. All schema, frame,
        # sign and action/reaction checks above remain mandatory.
        if (
            allow_zero
            and peak_point_force <= ZERO_FORCE_TOLERANCE_N
            and max(abs(v) for v in moment) <= ZERO_MOMENT_TOLERANCE_NM
        ):
            total, moment = [0.0] * 3, [0.0] * 3
            normalized_zero = True
        else:
            raise ValueError(
                f"No positive load in this bilateral validation: force={total}, moment={moment}, points={points}"
            )
    return dict(
        side=side,
        sim_time=stamp,
        force_world_n=total,
        raw_force_world_n=raw_total,
        raw_moment_world_origin_nm=raw_moment,
        numerical_zero_normalized=normalized_zero,
        moment_world_origin_nm=moment,
        cop_world_xy_m=[v / total[2] for v in weighted_xy] if total[2] > 0 else None,
        point_count=points,
        action_reaction_error_n=residual,
        frame="world",
        source="Gazebo contact wrench; not a mass/COM-derived force estimate",
    )


def paired_loads(raw, start, end):
    feet = {s: [] for s in ["left", "right"]}
    for row in raw:
        if start - 0.05 <= row["sim_time"] <= end:
            feet[row["side"]].append(decode_foot_load(row))
    for side, values in feet.items():
        if len(values) < 2 or any(
            b["sim_time"] <= a["sim_time"] for a, b in zip(values, values[1:])
        ):
            raise ValueError("Missing or non-increasing force timestamps for " + side)
    right = feet["right"]
    times = [r["sim_time"] for r in right]
    pairs = []
    for left in feet["left"]:
        t = left["sim_time"]
        if t < start:
            continue
        i = bisect_right(times, t) - 1
        if i < 0:
            continue
        r = right[i]
        lag = t - r["sim_time"]
        if lag > 0.03:
            raise ValueError(
                "Right force stale when pairing; future samples are not used"
            )
        total = left["force_world_n"][2] + r["force_world_n"][2]
        pairs.append(
            dict(
                sim_time=t,
                left=left,
                right=r,
                total_vertical_n=total,
                right_age_s=lag,
                cop_world_xy_m=[
                    (
                        left["cop_world_xy_m"][k] * left["force_world_n"][2]
                        + r["cop_world_xy_m"][k] * r["force_world_n"][2]
                    )
                    / total
                    for k in range(2)
                ],
            )
        )
    if len(pairs) < 2:
        raise ValueError("Missing paired force samples")
    return pairs


def audit_foot_loads(
    raw, com, start, end, mass, displacement, gravity=9.81, phase_windows=None
):
    if (
        not all(math.isfinite(v) for v in [start, end, mass, displacement, gravity])
        or end <= start
        or min(mass, gravity) <= 0
        or displacement == 0
    ):
        raise ValueError("Invalid load-audit parameters")
    pairs = paired_loads(raw, start, end)
    times = [r["sim_time"] for r in pairs]
    gaps = [b - a for a, b in zip(times, times[1:])]
    coverage = (
        times[0] - start <= 0.05
        and end - times[-1] <= 0.05
        and 0 < min(gaps)
        and max(gaps) <= 0.05
    )
    phase_windows = phase_windows or dict(
        baseline=(1.0, 2.5), hold=(11.0, 13.0), recovery=(22.0, 25.00001)
    )
    phase = {
        name: [p for p in pairs if lo <= p["sim_time"] - start < hi]
        for name, (lo, hi) in phase_windows.items()
    }
    weight = mass * gravity
    stats = {}
    for name, values in phase.items():
        if len(values) < 20:
            raise ValueError("Insufficient force samples in " + name)
        avg = lambda fn: sum(fn(p) for p in values) / len(values)
        stats[name] = dict(
            samples=len(values),
            left_n=avg(lambda p: p["left"]["force_world_n"][2]),
            right_n=avg(lambda p: p["right"]["force_world_n"][2]),
            total_n=avg(lambda p: p["total_vertical_n"]),
        )
        stats[name]["left_fraction"] = stats[name]["left_n"] / stats[name]["total_n"]
    mean_weight_errors = {s: abs(v["total_n"] - weight) for s, v in stats.items()}
    total_peak = max(abs(p["total_vertical_n"] - weight) for p in pairs)
    baseline_symmetry = abs(stats["baseline"]["left_n"] - stats["baseline"]["right_n"])
    changes = {
        s: stats["hold"][s + "_n"] - stats["baseline"][s + "_n"]
        for s in ["left", "right"]
    }
    returns = {
        s: stats["recovery"][s + "_n"] - stats["baseline"][s + "_n"]
        for s in ["left", "right"]
    }
    sign = 1 if displacement > 0 else -1
    transfer = sign * changes["left"] > 1.0 and sign * changes["right"] < -1.0
    # Independent static moment check: force-weighted contact CoP vs world COM.
    com_times = [r["sim_time"] for r in com]
    cop_errors = []
    for p in pairs:
        i = bisect_right(com_times, p["sim_time"]) - 1
        if i < 0 or p["sim_time"] - com_times[i] > 0.1:
            raise ValueError("COM unavailable/stale for independent pressure check")
        center = com[i]["com"]
        cop_errors.append(
            math.hypot(
                p["cop_world_xy_m"][0] - center[0], p["cop_world_xy_m"][1] - center[1]
            )
        )
    force_ok = (
        coverage
        and max(mean_weight_errors.values()) <= 0.01 * weight
        and total_peak <= 0.05 * weight
        and baseline_symmetry <= 0.01 * weight
        and max(cop_errors) <= 0.003
    )
    returned = max(abs(v) for v in returns.values()) <= 1.0
    return dict(
        passed=bool(force_ok and transfer and returned),
        measurement_validated=bool(force_ok),
        load_transfer_observed=bool(transfer),
        returned_near_baseline=returned,
        frame="world",
        used_for_control=False,
        source="Per-point contact wrenches summed once per foot; no estimated forces substituted",
        samples=len(pairs),
        coverage_passed=coverage,
        max_gap_s=max(gaps),
        max_pair_age_s=max(p["right_age_s"] for p in pairs),
        expected_weight_n=weight,
        phase_means=stats,
        mean_total_weight_errors_n=mean_weight_errors,
        max_total_weight_error_n=total_peak,
        max_cop_com_distance_m=max(cop_errors),
        baseline_left_right_difference_n=baseline_symmetry,
        held_load_changes_n=changes,
        recovery_load_changes_n=returns,
        minimum_foot_vertical_load_n={
            s: min(p[s]["force_world_n"][2] for p in pairs) for s in ["left", "right"]
        },
        max_action_reaction_error_n=max(
            p[s]["action_reaction_error_n"] for p in pairs for s in ["left", "right"]
        ),
        note="Validated simulated contact-force feedback for this slow bilateral experiment; not hardware calibration, unloaded-foot validation or a dynamic stability guarantee.",
    )
