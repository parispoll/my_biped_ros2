"""Walking must change support, retain foot positions, and reject bad contacts."""

import os, subprocess, sys, tempfile, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from force_panel.config import ExperimentRequest
from force_panel.jobs import JobManager


class WalkingTests(unittest.TestCase):
    def test_panel_routes_and_prerequisites(self):
        p = dict(
            mode="alternating_walk",
            lift_mm=30,
            lowering_mm=40,
            shift_side="left",
            force_n=0,
            balance_enabled=False,
            gazebo_gui=False,
        )
        c = ExperimentRequest.parse(p).command(ROOT, "/tmp/walking-test")
        self.assertIn("--alternating-walk", c)
        self.assertEqual(c[c.index("--real-time-factor") + 1], "1.0")
        for bad in [
            dict(force_n=1),
            dict(lift_mm=10),
            dict(lowering_mm=20),
            dict(balance_enabled=True),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(p, **bad))
        with tempfile.TemporaryDirectory() as tmp:
            m = JobManager(tmp)
            with self.assertRaisesRegex(ValueError, "first step"):
                m.start(p)
            m.jobs["step"] = dict(
                status="completed",
                request=dict(mode="first_step", shift_side="left"),
                result=dict(passed=True),
            )
            with self.assertRaisesRegex(ValueError, "preflight"):
                m.start(p)
            with self.assertRaisesRegex(ValueError, "first step"):
                m.start(dict(p, shift_side="right"))

    def test_alternating_geometry_and_guard_rejections(self):
        code = """
import copy,json,xacro
from my_robot_description.motion.alternating_walk import AlternatingWalkProgram
from my_robot_description.model.roll_geometry import link_transforms,model_com
from my_robot_description.model.roll_stance import JOINT_NAMES
from my_robot_description.motion.foot_lift import sole_corners,gravity_gradient_check
p=AlternatingWalkProgram(xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml());xml=p.xml
assert p.total==669 and [s['swing'] for s in p.steps]==['right','left','right']
assert p.setup['sampled_peak_reference_speed_rad_s']<.05
for step in p.steps:
 for t in [step['advance_start']+10,step['reload_start']-.5,step['end']-1]:
  r=p.sample(t);q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);mass,c=model_com(xml,q);tr=[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']]
  obs=dict(sim_time=t+1,mass_kg=mass,com=[c[k]+tr[k] for k in range(3)],feet={s:[[v+tr[k] for k,v in enumerate(corner)] for corner in sole_corners(xml,tf,s)] for s in ('left','right')})
  states={s:dict(side=s,sim_time=t+1,measurement_valid=True,state='loaded_contact' if f>1 else 'no_contact' if r[s+'_foot_xyz_m'][2]>.045 else 'touching_unloaded',force_world_n=[0,0,f]) for s,f in r['predicted_force_n'].items()}
  assert p.guard(states,t+1,t,obs)['eligible'],(t,p.guard(states,t+1,t,obs))
  if t==step['advance_start']+10:
   assert gravity_gradient_check(xml,r['target_rad'],step['support'])['passed']
   fake=copy.deepcopy(states);fake[step['support']]['state']='no_contact'
   assert not p.guard(fake,t+1,t,obs)['eligible']
   fake=copy.deepcopy(obs)
   for c in fake['feet'][step['swing']]:c[0]=step['start_feet'][step['swing']][0]
   assert not p.guard(states,t+1,t,fake)['eligible']
  if t==step['reload_start']-.5:
   fake=copy.deepcopy(states);fake[step['swing']]['state']='no_contact'
   assert not p.guard(fake,t+1,t,obs)['eligible']
end=p.sample(p.total)
assert abs(end['right_foot_xyz_m'][0]-p.feet['right'][0]-.15)<1e-9
assert abs(end['left_foot_xyz_m'][0]-p.feet['left'][0]-.10)<1e-9
assert p.proportional_gains(p.total)==p.proportional_gains(0)
from pathlib import Path
Path('evidence/preparation/alternating-walk/test-plan.json').write_text(json.dumps(p.setup,indent=2))
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
