"""Descriptive paired-repeat statistics; no confidence or stability claims."""

import math
import statistics

MIN_REDUCTION_PERCENT = 5.0


def spread(values):
    if not values or not all(math.isfinite(x) for x in values):
        raise ValueError("Nonfinite repeat metric")
    return dict(
        mean=statistics.mean(values),
        minimum=min(values),
        maximum=max(values),
        range=max(values) - min(values),
    )


def summarize_repeats(repeats):
    if len(repeats) < 2:
        raise ValueError("At least two complete repeats required")
    ids = [r["repeat"] for r in repeats]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate repeat identity")
    groups = {}
    all_checks = True
    protocol = None
    for direction, force in (("forward", 2.0), ("backward", -2.0)):
        pairs = []
        for repeat in repeats:
            report = repeat["comparisons"][direction]
            if not report["completed"]:
                raise ValueError("Incomplete comparison")
            request = report["requested"]
            if request["force_x"] != force:
                raise ValueError("Wrong direction/force")
            common = {k: v for k, v in request.items() if k != "force_x"}
            if protocol is None:
                protocol = common
            if common != protocol:
                raise ValueError("Repeat protocols differ")
            rows = {row["mode"]: row for row in report["rows"]}
            if set(rows) != {"off", "on"} or len(report["rows"]) != 2:
                raise ValueError("Need exactly one off/on pair")
            off, on = rows["off"], rows["on"]
            if off["peak_pitch_deg"] <= 0 or on["peak_pitch_deg"] < 0:
                raise ValueError("Invalid peak amplitude")
            reduction = 100 * (1 - on["peak_pitch_deg"] / off["peak_pitch_deg"])
            checks = all(
                row["recovery_passed"]
                and row["contacts_passed"]
                and row["saturated_samples"] == 0
                and row["audit"]["passed"]
                for row in (off, on)
            )
            window = report["published_window_relative_difference"]
            if not math.isfinite(window) or window < 0:
                raise ValueError("Invalid command window difference")
            checks = checks and window <= 0.05
            all_checks = all_checks and checks
            pairs.append(
                dict(
                    repeat=repeat["repeat"],
                    off=off,
                    on=on,
                    peak_reduction_percent=reduction,
                    comparison_checks_passed=checks,
                    published_window_relative_difference=window,
                )
            )
        metrics = {}
        for mode in ("off", "on"):
            metrics[mode] = {
                key: spread([pair[mode][key] for pair in pairs])
                for key in (
                    "peak_pitch_deg",
                    "peak_pitch_rate_rad_s",
                    "final_joint_offset_rad",
                )
            }
            metrics[mode]["last_outside_band_s"] = spread(
                [pair[mode]["last_outside_band_after_clear_s"] or 0.0 for pair in pairs]
            )
        groups[direction] = dict(
            n_pairs=len(pairs),
            pairs=pairs,
            metrics=metrics,
            peak_reduction_percent=spread([p["peak_reduction_percent"] for p in pairs]),
            every_pair_reduced_peak_at_least_five_percent=all(
                p["peak_reduction_percent"] >= MIN_REDUCTION_PERCENT for p in pairs
            ),
        )
    return dict(
        groups=groups,
        all_comparison_checks_passed=all_checks,
        peak_reduction_consistent=all_checks
        and all(
            g["every_pair_reduced_peak_at_least_five_percent"] for g in groups.values()
        ),
        minimum_paired_peak_reduction_percent=MIN_REDUCTION_PERCENT,
        protocol_without_direction=protocol,
        limitation="Two repeats provide observed means/ranges, not confidence intervals or a general stability proof. Published command windows are not independently measured force impulses.",
    )
