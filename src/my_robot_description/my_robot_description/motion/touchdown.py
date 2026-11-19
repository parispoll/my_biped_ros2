"""Phase-specific touchdown qualification, separate from airborne unloading."""

import math


def contact_force_limit(phase, clearance, mass, enabled=False):
    if not math.isfinite(clearance) or not math.isfinite(mass) or mass <= 0:
        raise ValueError("Finite clearance and positive mass required")
    # Near-floor landing already remains subject to the existing 2 percentage
    # point load-share and 5% total-weight guards. Never permit a loaded air swing.
    landing = enabled and phase in ("lower_foot", "recontact") and clearance < 0.0005
    return 0.02 * mass * 9.81 if landing else 0.5
