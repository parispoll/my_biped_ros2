"""Longer/faster gait keeps world feet, role changes and strict sensor guards."""

import os, subprocess, sys, unittest
from pathlib import Path
from my_robot_description.motion.walking_profile import WalkingProfile

ROOT = Path(__file__).resolve().parents[1]


class WalkingProfileTests(unittest.TestCase):
    def test_invalid_profiles(self):
        for p in [
            WalkingProfile(True),
            WalkingProfile(2),
            WalkingProfile(21),
            WalkingProfile(10, 0.2),
            WalkingProfile(10, 0.075, 0.1),
        ]:
            with self.assertRaises(ValueError):
                p.validate()
        self.assertEqual(WalkingProfile().joint_speed_limit_rad_s, 0.05)
        self.assertEqual(WalkingProfile(10, 0.075, 0.25).joint_speed_limit_rad_s, 0.25)

    def test_ten_steps_time_mapping_world_geometry_and_contact_guards(self):
        code = """
import copy,xacro
from my_robot_description.motion.profiled_walk import ProfiledWalkProgram
from my_robot_description.motion.walking_profile import WalkingProfile
from my_robot_description.model.roll_geometry import link_transforms,model_com
from my_robot_description.model.roll_stance import JOINT_NAMES
from my_robot_description.motion.foot_lift import sole_corners
p=ProfiledWalkProgram(xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml(),profile=WalkingProfile(10,.075,.25))
assert p.setup['step_count']==10 and p.total<700 and p.base.reused_cycles>=7
assert sum(d for n,d in p.setup['schedule'])==p.total
assert p.setup['sampled_peak_reference_speed_rad_s']<.25
assert [s['swing'] for s in p.steps]==['right','left']*5
for a,b in p.knots:
 assert abs(p.forward_time(a)-b)<1e-8 and abs(p.base_time(b)-a)<1e-8
for step in p.steps:
 assert step['reload_start']-step['recontact_start']>=2
 assert step['end']-step['hold_start']>=4
 t=(step['advance_start']+step['lower_start'])/2
 r=p.sample(t);tf=link_transforms(p.xml,dict(zip(JOINT_NAMES,r['target_rad'])));mass,c=model_com(p.xml,dict(zip(JOINT_NAMES,r['target_rad'])));tr=[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']]
 obs=dict(sim_time=t+1,mass_kg=mass,com=[c[k]+tr[k] for k in range(3)],feet={s:[[v+tr[k] for k,v in enumerate(corner)] for corner in sole_corners(p.xml,tf,s)] for s in ('left','right')})
 for side in ('left','right'):
  assert max(abs(tf[side+'_foot'][k][3]+tr[k]-r[side+'_foot_xyz_m'][k]) for k in range(3))<1e-8
 states={s:dict(side=s,sim_time=t+1,measurement_valid=True,state='loaded_contact' if f>1 else 'no_contact',force_world_n=[0,0,f]) for s,f in r['predicted_force_n'].items()}
 assert p.guard(states,t+1,t,obs)['eligible'],(step['index'],p.guard(states,t+1,t,obs))
 stale=copy.deepcopy(obs);stale['sim_time']-=.2
 assert not p.guard(states,t+1,t,stale)['eligible']
end=p.sample(p.total)
assert end['step_index']==10 and end['phase']=='done'
assert abs(end['left_foot_xyz_m'][0]-p.feet['left'][0]-.75)<1e-9
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
