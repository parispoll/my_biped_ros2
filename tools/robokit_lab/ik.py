"""Offline multi-link IK with explicit residual acceptance and named output."""
import numpy as np
from scipy.spatial.transform import Rotation


class FootIK:
    """Solve link poses relative to the robot root; the base stays prescribed.

    RoboKit cost terms are soft. A returned candidate is accepted only after
    independent forward-kinematic pose and hard joint-limit checks.
    """
    def __init__(self, adapter, links, iterations=60):
        if type(iterations) is not int or iterations <= 0:
            raise ValueError('Iterations must be a positive integer')
        from robokit.helpers.ik import IKSolver, IKSolverConfig, IKContext
        from robokit.opt.population_solver import PopulationSolverConfig, Stage
        from robokit.opt.lm_optimizer import LMOptimizerConfig
        from robokit.terms import PositionTask, RotationTask, PositionLimit
        if not links or len(set(links)) != len(links) or any(n not in adapter.links for n in links):
            raise ValueError('Provide unique known target links')
        self.adapter, self.links = adapter, tuple(links)
        cfg = IKSolverConfig(solver=PopulationSolverConfig(
            stages=[Stage(iterations, 1, LMOptimizerConfig())], cuda_graph_mode='none'))
        cfg.add(PositionTask(frame_index=IKContext.TARGET_LINKS, T_world_target=IKContext.TARGET, weight=20.))
        cfg.add(RotationTask(frame_index=IKContext.TARGET_LINKS, T_world_target=IKContext.TARGET, weight=10.))
        cfg.add(PositionLimit(weight=50.))
        self.solver = IKSolver(cfg, robot=adapter.robot, target_links=list(links), device='cpu')

    def solve(self, targets, seed, position_tolerance_m=1e-5, rotation_tolerance_rad=1e-4):
        if not np.isfinite([position_tolerance_m, rotation_tolerance_rad]).all() or min(position_tolerance_m, rotation_tolerance_rad) <= 0:
            raise ValueError('Residual tolerances must be finite and positive')
        from tools.robokit_lab.check import reference
        if set(targets) != set(self.links):
            raise ValueError('Supply every configured target link exactly once')
        poses = []
        for link in self.links:
            matrix = np.asarray(targets[link], dtype=float)
            if (matrix.shape != (4,4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], [0,0,0,1], atol=1e-8)
                    or not np.allclose(matrix[:3,:3].T @ matrix[:3,:3], np.eye(3), atol=1e-8)
                    or not np.isclose(np.linalg.det(matrix[:3,:3]), 1., atol=1e-8)):
                raise ValueError('Targets must be rigid root-from-link matrices')
            quat = Rotation.from_matrix(matrix[:3,:3]).as_quat()  # xyzw -> wxyz
            poses.append(np.r_[matrix[:3,3], quat[3], quat[:3]])
        initial = self.adapter.robot.state(self.adapter.pack(seed)[None].astype(np.float32))
        result = self.solver.solve_numpy(np.array(poses), init_state=initial)
        q = dict(zip(self.adapter.backend_names, map(float, result.q.numpy()[0])))
        self.adapter.pack(q)  # reject nonfinite/out-of-limit optimizer results
        tf, _, _ = reference(self.adapter.xml, q, np.eye(4))
        position_error = max(float(np.linalg.norm(tf[n][:3,3] - targets[n][:3,3])) for n in self.links)
        rotation_error = max(float(Rotation.from_matrix(targets[n][:3,:3].T @ tf[n][:3,:3]).magnitude()) for n in self.links)
        if position_error > position_tolerance_m or rotation_error > rotation_tolerance_rad:
            raise ValueError(f'IK residual rejected: {position_error:g} m, {rotation_error:g} rad')
        return dict(positions_rad=q, position_error_m=position_error, rotation_error_rad=rotation_error)
