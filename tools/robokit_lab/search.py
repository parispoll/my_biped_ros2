"""Offline bounded pelvis search with fixed world foot targets and unchanged gates."""
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
from tools.robokit_lab.ik_check import static_check


def envelope(t):
    """Zero position/velocity/acceleration offset at lift start and reload end."""
    from my_robot_description.motion.first_step import smooth
    if t <= 117 or t >= 233:
        return 0.
    if t < 132:
        return smooth((t-117)/15)
    if t <= 169:
        return 1.
    return 1.-smooth((t-169)/64)


def prepare_frames(adapter, rows):
    frames=[]
    for row in rows:
        base=np.array(row['world_base'])
        tf,_,_=reference(adapter.xml,row['positions_rad'],base)
        frames.append(dict(time_s=row['time_s'],phase=row['phase'],base=base,
                           targets={n:tf[n] for n in ('left_foot','right_foot')},
                           seed=row['positions_rad']))
    return frames


def densify(frames, subdivisions):
    """Linear base/foot translation interpolation of the saved flat-foot path.

    This defines a new sampled candidate path; it does not reconstruct the exact
    original planner between samples. Flat orientation is verified, never inferred.
    """
    result=[]
    for a,b in zip(frames,frames[1:]):
        for j in range(subdivisions):
            u=j/subdivisions
            base=(1-u)*a['base']+u*b['base']
            targets={n:(1-u)*a['targets'][n]+u*b['targets'][n] for n in a['targets']}
            result.append(dict(time_s=(1-u)*a['time_s']+u*b['time_s'],phase=a['phase'],base=base,
                               targets=targets,seed={n:(1-u)*a['seed'][n]+u*b['seed'][n] for n in a['seed']}))
    return result+[frames[-1]]


def evaluate(adapter, ik, frames, dx, dz):
    from my_robot_description.motion.live_foot_lift import collision_check
    rows=[]
    seed=dict(frames[0]['seed'])
    for f in frames:
        base=f['base'].copy()
        base[:3,3]+=envelope(f['time_s'])*np.array([dx,0.,dz])
        targets={n:np.linalg.inv(base)@target for n,target in f['targets'].items()}
        solved=ik.solve(targets,seed)
        q=solved['positions_rad']
        tf,com,_=reference(adapter.xml,q,base)
        collision=collision_check(adapter.xml,q,base[2,3])
        static=static_check(adapter.xml,q,dict(time_s=f['time_s']))
        rows.append(dict(time_s=f['time_s'],phase=f['phase'],**solved,world_base=base.tolist(),
                         feet_world_m={s:tf[s+'_foot'][:3,3].tolist() for s in ('left','right')},
                         com_world_m=com.tolist(),static_check=static,collision_check=collision))
        seed=q
    q=np.array([[r['positions_rad'][n] for n in adapter.names] for r in rows])
    t=np.array([r['time_s'] for r in rows])
    speed=float(np.max(np.abs(np.diff(q,axis=0)/np.diff(t)[:,None])))
    if speed>.05:
        raise ValueError('Sampled joint speed exceeds original 0.05 rad/s envelope')
    active=[r for r in rows if 117 <= r['time_s'] <= 233]
    tt=np.array([r['time_s'] for r in active])
    util=np.array([r['static_check']['peak_static_cap_fraction'] for r in active])
    score=float(np.trapz(util**2,tt)/(tt[-1]-tt[0]))
    return dict(dx_m=dx,dz_m=dz,passed=True,objective=score,
                peak_static_cap_fraction=float(max(r['static_check']['peak_static_cap_fraction'] for r in rows if r['static_check'])),
                sampled_joint_speed_rad_s=speed,
                max_position_error_m=max(r['position_error_m'] for r in rows),
                max_rotation_error_rad=max(r['rotation_error_rad'] for r in rows),
                samples=len(rows)),rows


def choose(candidates,baseline):
    """Only gate-passing candidates with no material peak-load regression qualify."""
    eligible=[c for c in candidates if c.get('passed') and
              c['peak_static_cap_fraction'] <= baseline['peak_static_cap_fraction']+1e-5]
    return min(eligible,key=lambda c:(c['objective'],abs(c['dx_m'])+abs(c['dz_m'])))


def plot(out, baseline, candidate):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,9),constrained_layout=True)
    for rows,label,style in [(baseline,'Baseline','--'),(candidate,'Candidate','-')]:
        t=[r['time_s'] for r in rows]
        axes[0].plot(t,[r['world_base'][2][3]*1000 for r in rows],style,label=label)
        active=[r for r in rows if r['static_check']]
        axes[1].plot([r['time_s'] for r in active],[r['static_check']['peak_static_cap_fraction']*100 for r in active],style,label=label)
        axes[2].plot(t,[r['feet_world_m']['right'][0]*1000 for r in rows],style,label=label)
    axes[0].set_ylabel('Pelvis height (mm)')
    axes[1].set_ylabel('Peak static motor utilization (%)')
    axes[2].set_ylabel('Right foot world X (mm)')
    axes[2].set_xlabel('Original first-step clock (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend()
    fig.suptitle('Bounded pelvis search with fixed foot targets\nOffline estimates; no Gazebo or hardware acceptance')
    fig.savefig(out/'candidate-comparison.png',dpi=160)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,default=ROOT/'evidence/robokit/validation-002')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    os.environ['BIPED_ACTUATOR_PROFILE']='robotis_p54'
    source=args.baseline.resolve()
    prior=json.loads((source/'summary.json').read_text())
    if not prior.get('passed') or hashlib.sha256((source/'biped.urdf').read_bytes()).hexdigest()!=prior['model_sha256']['biped.urdf']:
        raise ValueError('Passing baseline and unchanged model snapshot required')
    out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    adapter=RobotAdapter(source/'biped.urdf',prior['biped']['joint_names'])
    frames=prepare_frames(adapter,json.loads((source/'trajectory.json').read_text()))
    for f in frames:
        if not np.allclose(f['base'][:3,:3],np.eye(3),atol=1e-8) or any(not np.allclose(m[:3,:3],np.eye(3),atol=1e-8) for m in f['targets'].values()):
            raise ValueError('This search requires the level-base flat-foot baseline')
    paths=sorted((ROOT/'tools/robokit_lab').glob('*.py'))+sorted((ROOT/'src/my_robot_description/my_robot_description').rglob('*.py'))
    def snapshot():
        return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    before=snapshot()
    ik=FootIK(adapter,['left_foot','right_foot'])
    baseline,_=evaluate(adapter,ik,frames,0.,0.)
    candidates=[]
    start=time.perf_counter()
    for dx in (-.01,-.005,0.,.005,.01):
        for dz in (-.01,-.005,0.,.005,.01):
            try:
                c,_=evaluate(adapter,ik,frames,dx,dz)
            except ValueError as exc:
                c=dict(dx_m=dx,dz_m=dz,passed=False,reason=str(exc))
            candidates.append(c)
            write_json(out/'candidates.json',candidates)
    winner=choose(candidates,baseline)
    # Validate the selected path AND baseline at five times the search density.
    dense=densify(frames,5)
    dense_baseline,baseline_rows=evaluate(adapter,ik,dense,0.,0.)
    dense_winner,winning_rows=evaluate(adapter,ik,dense,winner['dx_m'],winner['dz_m'])
    improvement=1.-dense_winner['objective']/dense_baseline['objective']
    no_peak_regression=dense_winner['peak_static_cap_fraction'] <= dense_baseline['peak_static_cap_fraction']+1e-5
    report=dict(passed=bool(snapshot()==before and no_peak_regression),
        candidate_count=len(candidates),passing_candidates=sum(c['passed'] for c in candidates),
        objective='Time-average squared maximum static motor-cap fraction over 117..233 s; not energy.',
        search_baseline=baseline,search_winner=winner,dense_baseline=dense_baseline,dense_winner=dense_winner,
        dense_objective_reduction_fraction=improvement,
        minimum_improvement_fraction=.005,
        disposition='offline_candidate_for_review' if improvement>.005 and no_peak_regression else 'retain_existing_baseline',
        improvement_established_offline=bool(improvement>.005 and no_peak_regression),
        source_unchanged=snapshot()==before,source_sha256=before,duration_wall_s=time.perf_counter()-start,
        baseline_sha256={n:hashlib.sha256((source/n).read_bytes()).hexdigest() for n in ('summary.json','trajectory.json','biped.urdf')},
        scope='Offline candidate only. Existing controllers and Gazebo acceptance unchanged.',
        limitations=['Fivefold interpolation defines a sampled path, not exact replay between baseline samples.',
                     'No continuous collision/speed guarantee, actuator dynamics, impact or friction validation.',
                     'Peak load at unchanged boundary poses may remain unchanged despite lower interior loads.',
                     'No Gazebo trial or Aria stance acceptance is claimed.'])
    write_json(out/'summary.json',report)
    write_json(out/'candidate-trajectory.json',winning_rows)
    write_json(out/'baseline-dense-trajectory.json',baseline_rows)
    (out/'biped.urdf').write_text(adapter.xml)
    plot(out,baseline_rows,winning_rows)
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
