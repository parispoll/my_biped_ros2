"""Reject fake forward/landing evidence and keep the old stance out of return."""

import os, subprocess, sys, unittest, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from force_panel.config import ExperimentRequest
from force_panel.jobs import JobManager


class FirstStepTests(unittest.TestCase):
    def test_routes_and_gates(self):
        payload = dict(
            mode="first_step",
            lift_mm=30,
            lowering_mm=40,
            shift_side="left",
            force_n=0,
            balance_enabled=False,
            gazebo_gui=False,
        )
        self.assertIn(
            "--first-step",
            ExperimentRequest.parse(payload).command(ROOT, "/tmp/step-test"),
        )
        for bad in [
            dict(lift_mm=2),
            dict(lowering_mm=10),
            dict(force_n=1),
            dict(balance_enabled=True),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(payload, **bad))
        with tempfile.TemporaryDirectory() as tmp:
            m = JobManager(tmp)
            with self.assertRaisesRegex(ValueError, "30 mm"):
                m.start(payload)
            m.jobs["lift"] = dict(
                status="completed",
                request=dict(mode="foot_lift", shift_side="left", lift_mm=30),
                result=dict(passed=True, foot_lift=dict(requested_clearance_m=0.03)),
            )
            with self.assertRaisesRegex(ValueError, "preflight"):
                m.start(payload)
            with self.assertRaisesRegex(ValueError, "30 mm"):
                m.start(dict(payload, shift_side="right"))

    def test_geometry_continuity_guards_both_sides(self):
        code = """
import copy,math,xacro
from my_robot_description.motion.first_step import FirstStepProgram
from my_robot_description.motion.foot_lift import sole_corners,gravity_gradient_check
from my_robot_description.model.roll_geometry import link_transforms,model_com
from my_robot_description.model.roll_stance import JOINT_NAMES
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
for side in ('left','right'):
 p=FirstStepProgram(xml,side)
 assert abs(sum(d for _,d in p.setup['schedule'])-241)<1e-9
 assert p.setup['sampled_peak_reference_speed_rad_s']<.05
 for t in [117,132,152,167,169,233,241]:
  a=p.sample(t-1e-7);b=p.sample(t)
  assert max(abs(x-y) for x,y in zip(a['target_rad'],b['target_rad']))<1e-7
  assert max(abs(x-y) for x,y in zip(a['feedforward_nm'],b['feedforward_nm']))<1e-7
 assert p.sample(241)['target_rad']!=p.sample(0)['target_rad']
 for t in [0.,50.,116.,120.,140.,160.,168.5,180.,220.,239.]:
  r=p.sample(t);q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);mass,c=model_com(xml,q);tr=[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']]
  obs=dict(sim_time=t+1,mass_kg=mass,com=[c[k]+tr[k] for k in range(3)],feet={s:[[v+tr[k] for k,v in enumerate(corner)] for corner in sole_corners(xml,tf,s)] for s in ('left','right')})
  states={s:dict(side=s,sim_time=t+1,measurement_valid=True,state='loaded_contact' if force>1 else 'no_contact' if r['swing_clearance_m']>0 else 'touching_unloaded',force_world_n=[0,0,force]) for s,force in r['predicted_force_n'].items()}
  assert p.guard(states,t+1,t,obs)['eligible'],(side,t,p.guard(states,t+1,t,obs))
  if t==140.:
   assert gravity_gradient_check(xml,r['target_rad'],side)['passed']
   fake=copy.deepcopy(obs)
   for c in fake['feet'][p.swing]:c[0]-=r['swing_advance_m']
   assert not p.guard(states,t+1,t,fake)['eligible']
   fake=copy.deepcopy(states);fake[p.swing]['state']='touching_unloaded'
   assert not p.guard(fake,t+1,t,obs)['eligible']
   stale=copy.deepcopy(obs);stale['sim_time']-=.2
   assert not p.guard(states,t+1,t,stale)['eligible']
  if t==168.5:
   fake=copy.deepcopy(states);fake[p.swing]['state']='no_contact'
   assert not p.guard(fake,t+1,t,obs)['eligible']
 final=p.sample(241);tf=link_transforms(xml,dict(zip(JOINT_NAMES,final['target_rad'])))
 assert abs(tf[p.swing+'_foot'][0][3]-tf[p.support+'_foot'][0][3]-.05)<1e-9
 assert abs(p.expected_fraction(241)-.5)<1e-12
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
