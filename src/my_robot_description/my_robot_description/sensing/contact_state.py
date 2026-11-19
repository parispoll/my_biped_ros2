"""Explicit source-stamped contact states; silence is never zero support."""

import math
from my_robot_description.sensing.foot_loads import decode_foot_load


def contact_state(row, now, side, max_age=0.05, unloaded_threshold_n=1.0):
    if (
        side not in ("left", "right")
        or not math.isfinite(now)
        or not math.isfinite(max_age)
        or max_age <= 0
        or not math.isfinite(unloaded_threshold_n)
        or unloaded_threshold_n < 0
    ):
        raise ValueError("Invalid contact-state configuration")

    def result(state, reason, **extra):
        return dict(
            side=side,
            state=state,
            reason=reason,
            measurement_valid=state
            in ("loaded_contact", "touching_unloaded", "no_contact"),
            **extra
        )

    if row is None:
        return result(
            "unknown", "No message received", force_world_n=None, cop_world_xy_m=None
        )
    try:
        message = row["message"]
        h = message["header"]
        stamp = h["stamp"]["sec"] + h["stamp"]["nanosec"] / 1e9
        if (
            row["side"] != side
            or not math.isfinite(stamp)
            or stamp <= 0
            or abs(stamp - row["sim_time"]) > 1e-9
            or h["frame_id"] not in ("", "world")
        ):
            raise ValueError("Invalid source stamp, frame or side")
        age = now - stamp
        if age < -0.02:
            return result(
                "invalid",
                "Future source timestamp",
                age_s=age,
                force_world_n=None,
                cop_world_xy_m=None,
            )
        if age > max_age:
            return result(
                "stale",
                "Source timestamp expired",
                age_s=age,
                force_world_n=None,
                cop_world_xy_m=None,
            )
        if not isinstance(message["contacts"], list):
            raise ValueError("Invalid contacts array")
        if not message["contacts"]:
            return result(
                "no_contact",
                "Fresh explicitly empty contact message",
                sim_time=stamp,
                age_s=age,
                force_world_n=[0.0, 0.0, 0.0],
                cop_world_xy_m=None,
            )
        load = decode_foot_load(row, allow_zero=True)
        state = (
            "touching_unloaded"
            if load["force_world_n"][2] <= unloaded_threshold_n
            else "loaded_contact"
        )
        return result(
            state,
            "Validated foot/ground manifold",
            sim_time=stamp,
            age_s=age,
            force_world_n=load["force_world_n"],
            cop_world_xy_m=load["cop_world_xy_m"],
            raw_force_world_n=load["raw_force_world_n"],
            numerical_zero_normalized=load["numerical_zero_normalized"],
        )
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        return result("invalid", str(error), force_world_n=None, cop_world_xy_m=None)
