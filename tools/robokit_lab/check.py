"""Run model agreement and a 75 mm first-step reproduction, entirely offline."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/my_robot_description'))
from my_robot_description.model.roll_geometry import link_transforms, origin, vector
from tools.robokit_lab.adapter import RobotAdapter


def reference(xml, positions, base):
    """Existing independent double-precision URDF FK, including inertial offsets."""
    tf = {n: base @ np.array(t) for n, t in link_transforms(xml, positions).items()}
    weighted = np.zeros(3)
    mass = 0.
    for link in ET.fromstring(xml).findall('link'):
        inertial = link.find('inertial')
        if inertial is None:  # massless sensor/contact reference frames
            continue
        m = float(inertial.find('mass').get('value'))
        weighted += m * (tf[link.get('name')] @ np.array(origin(inertial.find('origin'))))[:3, 3]
        mass += m
    return tf, weighted / mass, mass


def reference_jacobian(xml, positions, base, link, names):
    """Independent geometric derivative of the link origin in world coordinates."""
    tf, _, _ = reference(xml, positions, base)
    joints = ET.fromstring(xml).findall('joint')
    by_child = {j.find('child').get('link'): j for j in joints}
    jac = np.zeros((3, len(names)))
    cursor = link
    while cursor in by_child:
        j = by_child[cursor]
        parent = j.find('parent').get('link')
        if j.get('name') in names:
            joint_pose = tf[parent] @ np.array(origin(j.find('origin')))
            axis = joint_pose[:3, :3] @ np.array(vector(j.find('axis'), 'xyz', '1 0 0'))
            jac[:, names.index(j.get('name'))] = np.cross(axis, tf[link][:3, 3] - joint_pose[:3, 3])
        cursor = parent
    return jac


def agreement(adapter, positions, base, feet):
    actual = adapter.evaluate(positions, base)
    expected, com, mass = reference(adapter.xml, positions, base)
    errors = dict(
        transform_max_abs=float(max(np.max(np.abs(actual['transforms'][n] - t)) for n, t in expected.items())),
        foot_position_max_m=float(max(np.max(np.abs(actual['transforms'][n][:3, 3] - expected[n][:3, 3])) for n in feet)),
        com_max_m=float(np.max(np.abs(actual['com_world_m'] - com))),
        mass_error_kg=abs(adapter.mass_kg - mass),
    )
    # Compare RoboKit spatial twists with an independent geometric point Jacobian.
    state = actual['state']
    adapter.robot.compute_motion_subspace(state)
    je = []
    for foot in feet:
        spatial = state.get_link_jacobian(adapter.links.index(foot), 'spatial').numpy()[0]
        # Spatial twists are [linear, angular]; v(point) = v + omega x point.
        point = actual['transforms'][foot][:3, 3]
        nj = len(adapter.backend_names)
        spatial = spatial[:, -nj:]  # discard floating-base columns
        point_j = spatial[:3] + np.cross(spatial[3:].T, point).T
        expected_j = reference_jacobian(adapter.xml, positions, base, foot, list(adapter.backend_names))
        je.append(np.max(np.abs(point_j - expected_j)))
    errors['jacobian_max_m_per_rad'] = float(max(je))
    return errors


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ['BIPED_ACTUATOR_PROFILE'] = 'robotis_p54'
    import xacro
    from my_robot_description.model.roll_stance import JOINT_NAMES
    from my_robot_description.motion.first_step import FirstStepProgram
    xml = xacro.process_file(str(ROOT / 'src/my_robot_description/urdf/biped_roll.urdf.xacro')).toxml()
    (out / 'biped.urdf').write_text(xml)
    biped = RobotAdapter(out / 'biped.urdf', JOINT_NAMES)
    aria_positions = json.loads((ROOT / 'practice/aria_handoffs/config/nominal_positions.json').read_text())
    (out / 'aria.urdf').write_text((ROOT / 'practice/aria_handoffs/mechanical.urdf').read_text())
    aria = RobotAdapter(out / 'aria.urdf', aria_positions)
    source_paths = sorted((ROOT / 'src/my_robot_description/my_robot_description').rglob('*.py'))
    source_paths += sorted((ROOT / 'tools/robokit_lab').glob('*.py'))
    def snapshot():
        return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    sources_before = snapshot()
    program = FirstStepProgram(xml, 'left', advance=0.075)
    times = sorted(set(np.linspace(0, 241, 101).tolist() + [0, 117, 132, 142, 152, 167, 169, 233, 241]))
    rows, checks = [], []
    for t in times:
        row = program.sample(t)
        positions = dict(zip(JOINT_NAMES, row['target_rad']))
        base = np.eye(4)
        base[:3, 3] = [row['pelvis_x_m'], row['pelvis_y_m'], row['pelvis_height_m']]
        result = biped.evaluate(positions, base)
        checks.append(agreement(biped, positions, base, ['left_foot', 'right_foot']))
        rows.append(dict(time_s=t, phase=row['phase'], positions_rad=positions,
                         world_base=base.tolist(),
                         feet_world_m={s: result['transforms'][s+'_foot'][:3, 3].tolist() for s in ('left','right')},
                         com_world_m=result['com_world_m'].tolist()))
    # Rotated + translated base catches quaternion ordering and frame errors.
    base = np.array([[0,-1,0,.2],[1,0,0,-.1],[0,0,1,.7],[0,0,0,1]], dtype=float)
    checks.append(agreement(biped, rows[30]['positions_rad'], base, ['left_foot','right_foot']))
    aria_checks = []
    for offset in (0., .03, -.03):
        q = dict(aria_positions)
        for name in q:
            if name.endswith(('hip_pitch', 'abad')) or name in ('shoulder', 'elbow'):
                q[name] += offset
        aria_checks.append(agreement(aria, q, base, ['FL_contact','FR_contact','RL_contact','RR_contact']))
    peaks = {k: max(c[k] for c in checks) for k in checks[0]}
    aria_peaks = {k: max(c[k] for c in aria_checks) for k in aria_checks[0]}
    limits = dict(transform_max_abs=2e-6, foot_position_max_m=2e-6, com_max_m=2e-6,
                  mass_error_kg=1e-10, jacobian_max_m_per_rad=3e-6)
    swing = program.swing
    advance = rows[-1]['feet_world_m'][swing][0] - rows[0]['feet_world_m'][swing][0]
    support_drift = max(np.linalg.norm(np.array(r['feet_world_m'][program.support]) - rows[0]['feet_world_m'][program.support]) for r in rows)
    passed = all(peaks[k] <= limits[k] and aria_peaks[k] <= limits[k] for k in limits)
    sources_unchanged = snapshot() == sources_before
    passed = passed and sources_unchanged and abs(advance - .075) < 2e-6 and support_drift < 2e-6
    report = dict(passed=bool(passed), scope='Offline kinematic reproduction only; no executed gait or stance acceptance.',
                  robokit_version=importlib.metadata.version('robokit'), device='cpu',
                  biped=dict(samples=len(rows), joint_names=list(biped.names), backend_joint_names=list(biped.backend_names),
                             errors=peaks, swing_advance_m=advance, support_drift_m=float(support_drift)),
                  aria=dict(poses=len(aria_checks), errors=aria_peaks, joint_names=list(aria.names)),
                  tolerances=limits, source_unchanged=sources_unchanged, source_sha256=sources_before,
                  model_sha256={a.path.name: hashlib.sha256(a.xml.encode()).hexdigest() for a in (biped, aria)},
                  timing='Original FirstStepProgram clock (241 s); geometry matches the 75 mm profile, not its faster clock.')
    write_json(out / 'trajectory.json', rows)
    write_json(out / 'summary.json', report)
    from tools.robokit_lab.plot import plot
    plot(out)
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
