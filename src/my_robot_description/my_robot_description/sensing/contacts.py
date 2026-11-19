"""Interpret raw collision pairs; this module does not estimate support forces."""


def ground_contact_pairs(message, side):
    """Return this foot's named collision pairs with our world ground plane.

    An empty result applies only to this message. Missing/stale messages require
    a separate freshness check and must not be treated as a reliable false value.
    """
    if side not in ("left", "right"):
        raise ValueError("Foot side must be left or right")
    expected = f"{side}_foot_collision"
    pairs = []
    for contact in message.contacts:
        pair = (contact.collision1.name, contact.collision2.name)
        if not all(pair):
            raise ValueError(
                "Contact collision names are missing; cannot identify ground contact"
            )
        for foot, other in (pair, pair[::-1]):
            if foot.split("::")[-1] == expected and "ground_plane" in other.split("::"):
                pairs.append(pair)
                break
    return pairs
