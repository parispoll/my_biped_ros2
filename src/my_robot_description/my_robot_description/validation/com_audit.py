"""Coverage checks for read-only COM observations in measured trials."""

import math


def audit_com(rows, start, end):
    selected = [r for r in rows if start <= r["sim_time"] <= end]
    if len(selected) < 2:
        return dict(
            passed=False, samples=len(selected), reason="Missing COM observations"
        )
    times = [r["sim_time"] for r in selected]
    gaps = [b - a for a, b in zip(times, times[1:])]
    valid = all(
        math.isfinite(r["mass_kg"])
        and r["mass_kg"] > 0
        and len(r["com"]) == 3
        and all(math.isfinite(x) for x in r["com"])
        for r in selected
    )
    mass_change = max(r["mass_kg"] for r in selected) - min(
        r["mass_kg"] for r in selected
    )
    return dict(
        passed=valid
        and min(gaps) > 0
        and max(gaps) <= 0.1
        and times[0] - start <= 0.1
        and end - times[-1] <= 0.1
        and mass_change < 1e-9,
        samples=len(selected),
        max_gap_s=max(gaps),
        first_sim_time=times[0],
        last_sim_time=times[-1],
        mass_kg=selected[0]["mass_kg"],
        mass_variation_kg=mass_change,
    )
