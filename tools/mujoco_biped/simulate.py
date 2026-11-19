"""Direct torque loop, optional live viewer, and independently measured state log.

Standing establishes simulator parity. Walking reuses the existing trajectory;
it is a transfer test, not a claim that the Gazebo controller is already validated.
"""
import argparse
import contextlib
import json
import time
import hashlib
from pathlib import Path
import mujoco
import numpy as np
from .validation import PlacementObserver
from .model import build_model
from my_robot_description.control.roll_posture import RollPosture
from my_robot_description.model.roll_stance import JOINT_NAMES
from my_robot_description.model.roll_actuators import MOTOR_CAPS, MOTOR_SPEEDS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['stand', 'walk'], default='stand')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--steps', type=int, default=3)
    parser.add_argument('--time-scale', type=float, default=.5, choices=[1., .5, .4, .25])
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('seconds must be positive and finite')
    args.output.mkdir(parents=True, exist_ok=False)
    model, data, xml, plan, parity = build_model(args.output)
    program = None
    if args.mode == 'walk':
        from my_robot_description.motion.profiled_walk import ProfiledWalkProgram
        from my_robot_description.motion.walking_profile import WalkingProfile
        program = ProfiledWalkProgram(xml, profile=WalkingProfile(step_count=args.steps, step_length_m=.075, time_scale=args.time_scale))
    provenance = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                  for folder in (Path('tools/mujoco_biped'), Path('src/my_robot_description/my_robot_description'), Path('src/my_robot_description/urdf'), Path('src/my_robot_description/config'))
                  for path in folder.rglob('*') if path.suffix in ('.py','.xacro','.yaml')}
    (args.output/'source-hashes.json').write_text(json.dumps(provenance,indent=2))
    (args.output/'request.json').write_text(json.dumps({**vars(args),'output':str(args.output)},indent=2))
    if program:
        (args.output/'plan.json').write_text(json.dumps(program.setup,indent=2))
    observer = PlacementObserver(model,data)
    controller = RollPosture(plan)
    qi = [model.joint(n).qposadr[0] for n in JOINT_NAMES]
    vi = [model.joint(n).dofadr[0] for n in JOINT_NAMES]
    horizon = 3 + program.total if program else args.seconds
    context = contextlib.nullcontext(None)
    if args.viewer:
        import mujoco.viewer as viewer_module
        context = viewer_module.launch_passive(model, data)
    min_height = float('inf')
    max_tilt = peak_speed = peak_utilization = peak_error = 0.
    initial_com = data.subtree_com[model.body('base_link').id].copy()
    reason = None
    started = time.monotonic()
    with context as viewer, (args.output/'state.jsonl').open('w') as log:
        if viewer:
            viewer.cam.distance = 1.8
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -10
            viewer.cam.lookat[:] = [0, 0, .4]
        tick = 0
        while data.time < horizon:
            if viewer and not viewer.is_running():
                reason = 'viewer_closed'
                break
            if tick % 10 == 0:
                elapsed = max(0., data.time-3)
                target = program.sample(elapsed) if program and data.time >= 3 else plan
                controller.target = target['target_rad']
                controller.ff = target['feedforward_nm']
                gains = {} if not program or data.time < 3 else dict(derivative_gains=program.derivative_gains(elapsed), proportional_gains=program.proportional_gains(elapsed))
                terms = controller.update(data.qpos[qi].tolist(), data.qvel[vi].tolist(), .01, **gains)
                data.ctrl[:] = terms['command']
                height = float(data.body('base_link').xpos[2])
                tilt = float(np.arccos(np.clip(data.body('base_link').xmat.reshape(3,3)[2,2],-1,1)))
                min_height = min(min_height, height)
                max_tilt = max(max_tilt, tilt)
                peak_speed = max(peak_speed, float(max(abs(data.qvel[vi]))))
                peak_utilization = max(peak_utilization, max(abs(x)/c for x,c in zip(terms['requested'],MOTOR_CAPS)))
                peak_error = max(peak_error, max(abs(x) for x in terms['error']))
                forces = {'left':0., 'right':0.}
                for i in range(data.ncon):
                    contact = data.contact[i]
                    names = [mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,model.geom_bodyid[g]) for g in (contact.geom1,contact.geom2)]
                    for side in forces:
                        if side+'_foot' in names and 'world' in names:
                            wrench = np.zeros(6)
                            mujoco.mj_contactForce(model,data,i,wrench)
                            forces[side] += float((contact.frame.reshape(3,3).T @ wrench[:3])[2]) * (1 if names[0]=='world' else -1)
                foot_geometry = observer.update(data,forces)
                row = dict(feet=foot_geometry,time=float(data.time),phase=target.get('phase','standing'),height_m=height,tilt_rad=tilt,com_m=data.subtree_com[model.body('base_link').id].tolist(),foot_vertical_force_n=forces,q=data.qpos[qi].tolist(),torque_nm=data.ctrl.tolist())
                log.write(json.dumps(row)+'\n')
                if not np.all(np.isfinite(data.qpos)) or height < .3 or tilt > .5:
                    reason = 'fall_or_nonfinite_state'
                    break
                if peak_speed > min(MOTOR_SPEEDS):
                    reason = 'motor_speed_guard'
                    break
            mujoco.mj_step(model,data)
            tick += 1
            if viewer and tick % 16 == 0:
                viewer.sync()
                time.sleep(max(0, data.time-(time.monotonic()-started)))
    progress = float(data.subtree_com[model.body('base_link').id][0]-initial_com[0])
    source_unchanged = all(Path(p).is_file() and hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest for p,digest in provenance.items())
    events = observer.events
    basic_passed = reason is None and source_unchanged and peak_utilization <= 1 and (not program or (len(events)==args.steps and all(e['advance_m']>.05 for e in events) and all(a['side']!=b['side'] for a,b in zip(events,events[1:]))))
    result = dict(basic_migration_checks_passed=basic_passed,source_unchanged=source_unchanged,measured_placements=events,mode=args.mode,mujoco_version=mujoco.__version__,parity=parity,completed=reason is None,stop_reason=reason,simulated_seconds=float(data.time),wall_seconds=time.monotonic()-started,min_height_m=min_height,max_tilt_rad=max_tilt,peak_joint_speed_rad_s=peak_speed,peak_requested_motor_fraction=peak_utilization,max_joint_error_rad=peak_error,com_progress_m=progress,average_progress_m_s=progress/max(float(data.time),1e-9),validation_scope='Basic MuJoCo migration checks: no fall/speed violation, requested torque within caps, and for walking independently observed alternating foot flights/landings. Not the full Gazebo geometry/contact audits.')
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)

if __name__ == '__main__':
    main()
