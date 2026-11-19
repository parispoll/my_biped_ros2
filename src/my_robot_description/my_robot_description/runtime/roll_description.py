"""Prepare the roll standing description and explicit physical damping."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.model.roll_geometry import validate_roll_model
from my_robot_description.model.roll_stance import (
    standing_plan,
    JOINT_NAMES,
    MOTOR_CAPS,
)


def prepare_roll_description(share, prefix):
    share = Path(share)
    prefix = Path(prefix)
    geometry = xacro.process_file(str(share / "urdf/biped_roll.urdf.xacro")).toxml()
    validate_roll_model(geometry)
    plan = standing_plan(geometry)
    temporary = tempfile.TemporaryDirectory(prefix="biped_roll_")
    try:
        config = Path(temporary.name) / "controllers.yaml"
        config.write_bytes((share / "config/roll_standing.yaml").read_bytes())
        xml = xacro.process_file(
            str(share / "urdf/roll_effort.urdf.xacro"),
            mappings=dict(
                controllers_yaml=str(config),
                initial_hip=str(plan["target_rad"][1]),
                initial_knee=str(plan["target_rad"][2]),
                initial_ankle=str(plan["target_rad"][3]),
            ),
        ).toxml()
        root = ET.fromstring(xml)
        # Recompute the actual controlled model rather than assuming it matches.
        actual = standing_plan(xml)
        if actual != plan:
            raise ValueError("Controlled model differs from planned geometry")
        for name, cap in zip(JOINT_NAMES, MOTOR_CAPS):
            joint = root.find(f"ros2_control/joint[@name='{name}']")
            params = {
                p.get("name"): float(p.text)
                for p in joint.findall("command_interface/param")
            }
            if params != {"min": -cap, "max": cap}:
                raise ValueError("Incorrect ten-joint command limits")
        plugin = ET.SubElement(
            ET.SubElement(root, "gazebo"),
            "plugin",
            name="biped::ViscousDamping",
            filename=str(prefix / "lib/libbiped_viscous_damping.so"),
        )
        ET.SubElement(plugin, "expected_joints").text = "10"
        for name in JOINT_NAMES:
            dynamics = root.find(f"joint[@name='{name}']/dynamics")
            entry = ET.SubElement(plugin, "joint", name=name)
            ET.SubElement(entry, "damping").text = dynamics.get("damping")
            dynamics.set("damping", "0.0")
            dynamics.set("friction", "0.0")
        return ET.tostring(root, encoding="unicode"), temporary, plan
    except BaseException:
        temporary.cleanup()
        raise
