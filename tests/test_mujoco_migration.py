"""MuJoCo-specific invariants; run with the isolated simulator Python."""
import os
os.environ['BIPED_ACTUATOR_PROFILE']='robotis_p54'
import tempfile
import unittest
import numpy as np
from tools.mujoco_biped.model import build_model
from tools.mujoco_biped.validation import PlacementObserver
from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS


class MigrationTests(unittest.TestCase):
    def test_free_robot_preserves_limits_and_native_damping(self):
        with tempfile.TemporaryDirectory() as folder:
            model,data,xml,plan,parity=build_model(folder)
        self.assertTrue(parity['free_base'])
        self.assertAlmostEqual(parity['mass_kg'],22.28)
        self.assertLess(parity['max_fk_error_m'],1e-12)
        np.testing.assert_allclose(model.actuator_ctrlrange,np.array([[-c,c] for c in MOTOR_CAPS]))
        np.testing.assert_allclose([model.dof_damping[model.joint(n).dofadr[0]] for n in JOINT_NAMES],[.2,2.5,3,2.5,.2]*2)
        self.assertTrue(np.all(model.dof_frictionloss==0))
        # No force, equality or kinematic pin may hold the pelvis in space.
        self.assertEqual(model.neq,0)
        self.assertEqual(model.nmocap,0)
        np.testing.assert_allclose(data.qfrc_applied,0)
        robot_geoms=model.geom_bodyid!=0
        self.assertTrue(np.all(model.geom_contype[robot_geoms]==1))
        self.assertTrue(np.all(model.geom_conaffinity[robot_geoms]==2))

    def test_static_contact_never_counts_as_a_step(self):
        with tempfile.TemporaryDirectory() as folder:
            model,data,*_=build_model(folder)
        observer=PlacementObserver(model,data)
        for _ in range(100):
            observer.update(data,{'left':109.,'right':109.})
        self.assertEqual(observer.events,[])

if __name__=='__main__':
    unittest.main()
