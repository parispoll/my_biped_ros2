"""Live lift guards must reject false clearance/contact/landing evidence."""

import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class LiveLiftTests(unittest.TestCase):
    def test_planner_and_independent_geometry_contact_guards(self):
        code = """
import copy,xacro,json
from pathlib import Path
from my_robot_description.motion.live_foot_lift import LiveFootLiftProgram
from my_robot_description.motion.foot_lift import sole_corners
from my_robot_description.model.roll_geometry import link_transforms,model_com
from my_robot_description.model.roll_stance import JOINT_NAMES
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
for support in ('left','right'):
 p=LiveFootLiftProgram(xml,support);swing=p.setup['unloaded_foot']
 assert p.setup['total_time_s']==257 and p.sample(257)['phase']=='done'
 assert p.sample(0)['target_rad']==p.sample(257)['target_rad']
 assert len(p.collision_checks)==41 and p.setup['collision_check']['minimum_gap_m']>.01
 assert p.sample(128)['phase']=='lifted_hold' and p.sample(140)['phase']=='recontact'
 assert p.sample(141)['phase']=='move_back'
 for t in [0.,50.,110.,116.,117.,120.,128.,134.,139.,140.5,142.,200.,256.]:
  r=p.sample(t);q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);mass,c=model_com(xml,q)
  translation=[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']]
  obs=dict(sim_time=t+1,mass_kg=mass,com=[c[k]+translation[k] for k in range(3)],feet={side:[[v+translation[k] for k,v in enumerate(corner)] for corner in sole_corners(xml,tf,side)] for side in ('left','right')})
  states={side:dict(side=side,sim_time=t+1,measurement_valid=True,state='loaded_contact' if force>1 else 'no_contact' if p.clearance(t)>0 else 'touching_unloaded',force_world_n=[0,0,force]) for side,force in r['predicted_force_n'].items()}
  assert p.guard(states,t+1,t,obs)['eligible'],(t,p.guard(states,t+1,t,obs))
  if t==128.:
   Path('evidence/preparation/live-foot-lift/plan-'+support+'.json').write_text(json.dumps(p.setup,indent=2))
   stale=copy.deepcopy(obs);stale['sim_time']-=.2
   assert not p.guard(states,t+1,t,stale)['eligible']
   fixed=copy.deepcopy(obs)
   for corner in fixed['feet'][swing]:corner[2]=0
   assert not p.guard(states,t+1,t,fixed)['eligible']
   touching=copy.deepcopy(states);touching[swing]['state']='touching_unloaded'
   assert not p.guard(touching,t+1,t,obs)['eligible']
   no_support=copy.deepcopy(states);no_support[support]['state']='no_contact'
   assert not p.guard(no_support,t+1,t,obs)['eligible']
   assert not p.guard(states,t+1,t,None)['eligible']
  if t==140.5:
   absent=copy.deepcopy(states);absent[swing]['state']='no_contact'
   assert not p.guard(absent,t+1,t,obs)['eligible']
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
