"""Import our authoritative Xacro geometry, then add MuJoCo-only world elements."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import xacro
from my_robot_description.model.roll_stance import JOINT_NAMES, standing_plan
from my_robot_description.model.roll_geometry import link_transforms
from my_robot_description.model.roll_actuators import MOTOR_CAPS

ROOT = Path(__file__).resolve().parents[2]


def build_model(output):
    """Keep masses/inertias/axes/limits/damping; attach a truly free pelvis."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    xml = xacro.process_file(str(ROOT / 'src/my_robot_description/urdf/biped_roll.urdf.xacro')).toxml()
    plan = standing_plan(xml)
    urdf = ET.fromstring(xml)
    extension = ET.SubElement(urdf, 'mujoco')
    ET.SubElement(extension, 'compiler', fusestatic='false', discardvisual='true')
    imported = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding='unicode'))
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'import.xml'
        mujoco.mj_saveLastXML(str(path), imported)
        root = ET.parse(path).getroot()
    ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.81', integrator='implicitfast')
    world = root.find('worldbody')
    pelvis = world.find("body[@name='base_link']")
    if pelvis is None:
        raise ValueError('Importer did not preserve free pelvis body')
    ET.SubElement(pelvis, 'freejoint', name='floating_base')
    # Gazebo baseline has self-collision disabled. In particular the shin and
    # foot overlap at their co-located ankle carrier; do not invent new contacts.
    for geom in pelvis.iter('geom'):
        geom.set('contype', '1')
        geom.set('conaffinity', '2')
    # The controlled Gazebo model removes Coulomb joint friction explicitly.
    for joint in pelvis.iter('joint'):
        joint.set('frictionloss', '0')
    ET.SubElement(world, 'geom', name='floor', contype='2', conaffinity='1', type='plane', size='5 5 .1', rgba='.25 .28 .3 1', friction='1 .005 .0001')
    ET.SubElement(world, 'light', pos='0 -2 3', dir='0 0 -1')
    actuators = ET.SubElement(root, 'actuator')
    for name, cap in zip(JOINT_NAMES, MOTOR_CAPS):
        ET.SubElement(actuators, 'motor', name=name+'_motor', joint=name, gear='1', ctrllimited='true', ctrlrange=f'{-cap} {cap}')
    path = output / 'biped.xml'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    data.qpos[:3] = [0, 0, plan['expected_pelvis_height_m']]
    data.qpos[3:7] = [1, 0, 0, 0]
    for name, q in zip(JOINT_NAMES, plan['target_rad']):
        data.qpos[model.joint(name).qposadr[0]] = q
    mujoco.mj_forward(model, data)
    # Catch conversion errors before running any controller, using independent URDF FK.
    transforms = link_transforms(xml, dict(zip(JOINT_NAMES, plan['target_rad'])))
    errors = {}
    for name, tf in transforms.items():
        expected = np.array(tf)[:3, 3] + data.qpos[:3]
        errors[name] = float(np.linalg.norm(data.body(name).xpos - expected))
    if max(errors.values()) > 1e-8 or abs(sum(model.body_mass)-plan['mass_kg']) > 1e-8:
        raise ValueError('URDF/MuJoCo mass or kinematic parity failed')
    return model, data, xml, plan, {'mass_kg':float(sum(model.body_mass)), 'max_fk_error_m':max(errors.values()), 'actuated_joints':model.nu, 'free_base':model.nv == 16}
