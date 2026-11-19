"""Height-specific gates and 10 mm geometry; retain the 2 mm default."""

import os, subprocess, sys, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from force_panel.config import ExperimentRequest
from force_panel.jobs import JobManager, verified_lift
import tempfile


class ClearancePanelTests(unittest.TestCase):
    def test_height_specific_requests_evidence_and_prerequisites(self):
        request = dict(
            mode="foot_lift",
            lift_mm=10,
            shift_side="left",
            lowering_mm=40,
            force_n=0,
            balance_enabled=False,
            gazebo_gui=False,
        )
        c = ExperimentRequest.parse(request)
        cmd = c.command(ROOT, ROOT / "evidence/example")
        self.assertEqual(cmd[cmd.index("--lift-mm") + 1], "10")
        for height in [True, 2.0, 5, 11]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(request, lift_mm=height))

        def job(mode, mm, actual):
            return dict(
                status="completed",
                request=dict(mode=mode, lift_mm=mm, shift_side="left"),
                result=dict(
                    passed=True,
                    requested_clearance_m=actual,
                    foot_lift=dict(requested_clearance_m=actual),
                ),
            )

        self.assertFalse(
            verified_lift(job("foot_lift", 10, 0.002), "foot_lift", "left", 10)
        )
        self.assertFalse(
            verified_lift(
                job("foot_lift_preview", 2, 0.002), "foot_lift_preview", "left", 10
            )
        )
        self.assertTrue(
            verified_lift(job("foot_lift", 10, 0.01), "foot_lift", "left", 10)
        )
        with tempfile.TemporaryDirectory() as folder:
            manager = JobManager(folder)
            with self.assertRaisesRegex(ValueError, "2 mm"):
                manager.start(request)
            manager.jobs["two"] = job("foot_lift", 2, 0.002)
            manager.jobs["zero"] = dict(
                status="completed",
                request=dict(mode="zero_load", shift_side="left"),
                result=dict(passed=True),
            )
            manager.jobs["preview"] = job("foot_lift_preview", 2, 0.002)
            with self.assertRaisesRegex(ValueError, "matching"):
                manager.start(request)
            self.assertNotEqual(
                next(
                    r
                    for r in manager.roadmap()
                    if r.get("mode") == "foot_lift"
                    and r.get("lift_mm") == 10
                    and r["shift_side"] == "left"
                )["status"],
                "verified",
            )


class LargerLiftGeometryTests(unittest.TestCase):
    def test_both_sides_10mm_and_rejection_of_2mm_measured_geometry(self):
        code = """
import copy,xacro
from my_robot_description.motion.live_foot_lift import LiveFootLiftProgram
from my_robot_description.motion.foot_lift import sole_corners,gravity_gradient_check
from my_robot_description.model.roll_geometry import model_com,link_transforms
from my_robot_description.model.roll_stance import JOINT_NAMES
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
for side,clearance in [('left',.01),('right',.01),('left',.03),('right',.03)]:
 p=LiveFootLiftProgram(xml,side,clearance);hold=p.hold_end-1.;now=hold+1.;r=p.sample(hold);q=dict(zip(JOINT_NAMES,r['target_rad']))
 assert len(p.preview.table)==round(clearance/.00005)+1 and len(p.collision_checks)==len(p.preview.table)
 assert abs(p.clearance(hold)-clearance)<1e-12 and p.sample(p.setup['total_time_s'])['target_rad']==p.sample(0)['target_rad']
 assert p.preview.setup['sampled_peak_reference_speed_rad_s']<.05
 for row in p.preview.table[::100]:assert gravity_gradient_check(xml,row['target_rad'],side)['passed']
 tf=link_transforms(xml,q);m,c=model_com(xml,q);v=[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']]
 obs=dict(sim_time=now,mass_kg=m,com=[c[i]+v[i] for i in range(3)],feet={s:[[a+v[k] for k,a in enumerate(corner)] for corner in sole_corners(xml,tf,s)] for s in ('left','right')})
 states={s:dict(side=s,sim_time=now,measurement_valid=True,state='loaded_contact' if s==side else 'no_contact',force_world_n=[0,0,r['predicted_force_n'][s]]) for s in ('left','right')}
 assert p.guard(states,now,hold,obs)['eligible']
 wrong=copy.deepcopy(obs)
 for corner in wrong['feet'][p.setup['unloaded_foot']]:corner[2]=.002
 assert not p.guard(states,now,hold,wrong)['eligible']
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
