"""Pure geometry for COM telemetry; no ROS, Gazebo or motor commands."""

import math


def decode_com_state(values):
    """Schema v1: time, kg, COM xyz, eight world sole-corner xyz triples."""
    if len(values) != 29 or not all(math.isfinite(v) for v in values):
        raise ValueError("COM state requires 29 finite values")
    if values[0] < 0 or values[1] <= 0:
        raise ValueError("Invalid COM time or total mass")
    return dict(
        sim_time=values[0],
        mass_kg=values[1],
        com=list(values[2:5]),
        feet={
            side: [list(values[i : i + 3]) for i in range(start, start + 12, 3)]
            for side, start in (("left", 5), ("right", 17))
        },
    )


def convex_hull(points):
    points = sorted(set(tuple(p[:2]) for p in points))
    if len(points) < 3:
        return []
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (
        b[0] - o[0]
    )
    lower = []
    upper = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    return hull if len(hull) >= 3 else []


def geometric_support(state, contacts):
    """Flat soles with fresh collision evidence only; never a ZMP/CoP estimate."""
    points = []
    qualified = []
    for side, corners in state["feet"].items():
        contact = contacts.get(side, {})
        age = contact.get("age")
        flat = max(c[2] for c in corners) - min(c[2] for c in corners) <= 0.006
        near_ground = all(-0.01 <= c[2] <= 0.01 for c in corners)
        if (
            contact.get("ground") is True
            and age is not None
            and math.isfinite(age)
            and -0.01 <= age <= 0.05
            and flat
            and near_ground
        ):
            qualified.append(side)
            points.extend(corners)
    hull = convex_hull(points)
    return dict(
        hull=hull,
        qualified_feet=qualified,
        meaning="Contact-qualified flat-sole geometric hull; not measured contact patch, CoP, ZMP or dynamic stability.",
    )


def contact_at(sim_time, history):
    """Use collision evidence at/before the COM sample, never a newer callback."""
    candidates = [c for c in history if c["time"] <= sim_time + 1e-9]
    if not candidates:
        return dict(ground=False, age=None)
    contact = max(candidates, key=lambda c: c["time"])
    return dict(ground=contact["ground"], age=sim_time - contact["time"])
