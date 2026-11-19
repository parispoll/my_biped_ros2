"""Re-solve the saved 75 mm step's foot targets using offline RoboKit IK."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
from tools.robokit_lab.check import ROOT, reference, write_json
from tools.robokit_lab.adapter import RobotAdapter
from tools.robokit_lab.ik import FootIK


def static_check(xml, q, row):
    """Reuse the first-step common-CoP-offset statics and original 80% cap.

    This check applies to lift onward. Earlier preparation phases have different
    support policies; they receive geometry/collision checks only here.
    """
    from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS, double_support_loads
    from my_robot_description.model.roll_geometry import link_transforms, model_com, multiply, origin, vector
    from my_robot_description.motion.first_step import smooth
    import xml.etree.ElementTree as ET
    t = row['time_s']
    if t < 117:
        return None
    reload = smooth((t-169)/64) if t >= 169 else 0.
    tf = link_transforms(xml, q)
    mass, com = model_com(xml, q)
    forces = {'left':mass*9.81*(1-.5*reload), 'right':mass*9.81*.5*reload}
    offset = [com[k]-sum(forces[s]*tf[s+'_foot'][k][3] for s in forces)/(mass*9.81) for k in (0,1)]
    if abs(offset[0]) > .09 or abs(offset[1]) > .05:
        raise ValueError('Static pressure point lacks original 10 mm sole margin')
    loads = double_support_loads(xml,q,forces)
    for joint in ET.fromstring(xml).findall('joint'):
        name = joint.get('name')
        force = forces[name.split('_')[0]]
        frame = multiply(tf[joint.find('parent').get('link')],origin(joint.find('origin')))
        axis = vector(joint.find('axis'),'xyz')
        delta = [offset[1]*force,-offset[0]*force,0.]
        loads[name] -= sum(sum(frame[i][k]*axis[k] for k in range(3))*delta[i] for i in range(3))
    utilization = max(abs(loads[n])/cap for n,cap in zip(JOINT_NAMES,MOTOR_CAPS))
    if utilization > .8:
        raise ValueError('Original first-step static motor reserve exceeded')
    return dict(peak_static_cap_fraction=utilization, cop_offset_xy_m=offset,
                feedforward_nm=loads, prescribed_vertical_force_n=forces)


def plot_comparison(out, solved, baseline):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=[r['time_s'] for r in solved]
    fig, axes=plt.subplots(3,1,figsize=(10,9),constrained_layout=True)
    for side,color in [('left','#268078'),('right','#db8133')]:
        for data,style,label in [(baseline,'--','baseline'),(solved,'-', 'IK')]:
            feet=np.array([r['feet_world_m'][side] for r in data])
            axes[0].plot(t,1000*(feet[:,0]-feet[0,0]),style,color=color,label=f'{side} {label}')
    axes[0].set_ylabel('Forward travel (mm)')
    axes[1].plot(t,[1e6*r['position_error_m'] for r in solved],color='#5369a3')
    axes[1].set_ylabel('Foot pose position error (µm)')
    for key,label,style in [('baseline_static_check','Baseline','--'),('static_check','IK','-')]:
        points=[r for r in solved if r[key] is not None]
        axes[2].plot([r['time_s'] for r in points],[100*r[key]['peak_static_cap_fraction'] for r in points],style,label=label)
    axes[2].axhline(80,color='#b64b4b',linestyle=':',label='Existing static cap')
    axes[2].set_ylabel('Peak static motor utilization (%)')
    axes[2].set_xlabel('Original first-step clock (s)')
    for ax in axes:
        ax.grid(alpha=.2)
    axes[0].legend(ncol=2)
    axes[2].legend()
    fig.suptitle('75 mm biped step: RoboKit IK versus existing planner\nOffline sampled checks; prescribed pelvis, no physics execution')
    fig.savefig(out/'ik-comparison.png',dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,default=ROOT/'evidence/robokit/validation-002')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    baseline = args.baseline.resolve()
    prior = json.loads((baseline/'summary.json').read_text())
    if not prior.get('passed'):
        raise ValueError('Baseline model agreement must have passed')
    xml_path = baseline/'biped.urdf'
    if hashlib.sha256(xml_path.read_bytes()).hexdigest() != prior['model_sha256']['biped.urdf']:
        raise ValueError('Baseline model snapshot hash mismatch')
    rows = json.loads((baseline/'trajectory.json').read_text())
    os.environ['BIPED_ACTUATOR_PROFILE']='robotis_p54'
    from my_robot_description.motion.live_foot_lift import collision_check
    out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    adapter=RobotAdapter(xml_path,prior['biped']['joint_names'])
    links=['left_foot','right_foot']
    ik=FootIK(adapter,links)
    sources = sorted((ROOT/'tools/robokit_lab').glob('*.py')) + sorted((ROOT/'src/my_robot_description/my_robot_description').rglob('*.py'))
    def snapshot():
        return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    before=snapshot()
    seed={n:q+.02 for n,q in rows[0]['positions_rad'].items()}
    # The first solve starts off-target; later solves warm-start from the prior solution.
    solved=[]
    started=time.perf_counter()
    for row in rows:
        local,_,_=reference(adapter.xml,row['positions_rad'],np.eye(4))
        targets={n:local[n] for n in links}
        result=ik.solve(targets,seed)
        q=result['positions_rad']
        tf,com,_=reference(adapter.xml,q,np.array(row['world_base']))
        collision=collision_check(adapter.xml,q,row['world_base'][2][3])
        statics=static_check(adapter.xml,q,row)
        baseline_statics=static_check(adapter.xml,row['positions_rad'],row)
        delta=max(abs(q[n]-row['positions_rad'][n]) for n in q)
        solved.append(dict(time_s=row['time_s'],phase=row['phase'],**result,
                           world_base=row['world_base'],
                           feet_world_m={s:tf[s+'_foot'][:3,3].tolist() for s in ('left','right')},
                           com_world_m=com.tolist(),joint_difference_max_rad=delta,
                           static_check=statics,baseline_static_check=baseline_statics,collision_check=collision))
        seed=q
    q=np.array([[r['positions_rad'][n] for n in adapter.names] for r in solved])
    baseline_q=np.array([[r['positions_rad'][n] for n in adapter.names] for r in rows])
    times=np.array([r['time_s'] for r in solved])
    sampled_speed=float(np.max(np.abs(np.diff(q,axis=0)/np.diff(times)[:,None])))
    baseline_speed=float(np.max(np.abs(np.diff(baseline_q,axis=0)/np.diff(times)[:,None])))
    report=dict(passed=bool(snapshot()==before and sampled_speed<.05),samples=len(solved),device='cpu',
        scope='Offline IK candidate reproduction with sampled geometry/static checks; no physics execution.',
        base_policy='Prescribed baseline pelvis; both feet solved together in root frame.',
        seed_policy='First pose +0.02 rad at every joint; subsequent poses use only the preceding IK solution.',
        max_position_error_m=max(r['position_error_m'] for r in solved),
        max_rotation_error_rad=max(r['rotation_error_rad'] for r in solved),
        max_joint_difference_rad=max(r['joint_difference_max_rad'] for r in solved),
        sampled_joint_speed_rad_s=sampled_speed,baseline_sampled_joint_speed_rad_s=baseline_speed,
        static_checked_samples=sum(r['static_check'] is not None for r in solved),
        baseline_peak_static_cap_fraction=max(r['baseline_static_check']['peak_static_cap_fraction'] for r in solved if r['baseline_static_check']),
        peak_static_cap_fraction=max(r['static_check']['peak_static_cap_fraction'] for r in solved if r['static_check']),
        duration_wall_s=time.perf_counter()-started,source_unchanged=snapshot()==before,source_sha256=before,
        baseline_sha256={n:hashlib.sha256((baseline/n).read_bytes()).hexdigest() for n in ('summary.json','trajectory.json','biped.urdf')},
        limitations=['Sampled checks only; no swept collision or between-sample speed guarantee.',
                     'Original 241 s first-step clock; no faster gait claim.',
                     'Prescribed support forces and quasi-static model omit acceleration, impact and friction dynamics.',
                     'No gait improvement or Aria stance acceptance is established.'])
    write_json(out/'trajectory.json',solved)
    write_json(out/'summary.json',report)
    (out/'biped.urdf').write_text(adapter.xml)
    plot_comparison(out, solved, rows)
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
