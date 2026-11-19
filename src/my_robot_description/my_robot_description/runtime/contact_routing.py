"""Select contact feedback without changing source stamps or freshness limits."""


def feedback_topic(side, zero_load, interactive):
    if side not in ("left", "right"):
        raise ValueError("Unknown foot")
    if interactive:
        if not zero_load:
            raise ValueError("Interactive control requires source contact-state probes")
        # Avoid the 50 Hz relay's extra timer and executor scheduling delay.
        return f"/biped/state/{side}_foot/contacts"
    prefix = "/biped/state/balance" if zero_load else "/biped/balance"
    return f"{prefix}/{side}_foot/contacts"
