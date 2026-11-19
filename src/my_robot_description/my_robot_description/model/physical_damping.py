"""Read the same physical damping from geometry or prepared Gazebo models."""

import math
import xml.etree.ElementTree as ET
from .roll_stance import JOINT_NAMES


def damping_coefficients(xml):
    root = ET.fromstring(xml)
    plugins = root.findall("gazebo/plugin[@name='biped::ViscousDamping']")
    if plugins:
        if len(plugins) != 1:
            raise ValueError("One explicit physical damping plugin required")
        entries = plugins[0].findall("joint")
        values = {e.get("name"): float(e.findtext("damping")) for e in entries}
        if len(entries) != 10 or set(values) != set(JOINT_NAMES):
            raise ValueError("Physical damping plugin must name all ten joints")
        if any(
            float(root.find(f"joint[@name='{n}']/dynamics").get("damping", "0")) != 0
            for n in JOINT_NAMES
        ):
            raise ValueError("Prepared model duplicates native and explicit damping")
        result = [values[n] for n in JOINT_NAMES]
    else:
        result = [
            float(root.find(f"joint[@name='{n}']/dynamics").get("damping", "0"))
            for n in JOINT_NAMES
        ]
    if any(not math.isfinite(v) or v < 0 for v in result):
        raise ValueError("Finite nonnegative physical damping required")
    return result
