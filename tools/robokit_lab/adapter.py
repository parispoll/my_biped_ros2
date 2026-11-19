"""Named, SI-unit interface to RoboKit's CPU kinematics.

Matrices map link coordinates into world coordinates. The base is a rigid 4x4
transform, never an additional actuated joint. Callers retain command ownership.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


class RobotAdapter:
    def __init__(self, urdf_path, joint_names):
        from robokit.robo import Robot
        self.path = Path(urdf_path)
        self.xml = self.path.read_text()
        self.root = ET.fromstring(self.xml)
        self.names = tuple(joint_names)
        if len(set(self.names)) != len(self.names):
            raise ValueError("Duplicate configured joint names")
        self.robot = Robot.load(str(self.path))
        self.backend_names = tuple(self.robot.spec.actuated_joint_names)
        if set(self.names) != set(self.backend_names):
            raise ValueError("Configured joints must exactly match RoboKit actuated joints")
        self.links = tuple(self.robot.spec.link_names)
        self.limits = {}
        for j in self.root.findall('joint'):
            if j.get('type') == 'revolute':
                lim = j.find('limit')
                self.limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        self.mass_kg = self.robot.spec.total_mass

    def pack(self, positions):
        if set(positions) != set(self.names):
            raise ValueError("Supply every configured joint exactly once by name")
        q = np.array([positions[n] for n in self.backend_names], dtype=np.float64)
        if q.shape != (len(self.names),) or not np.isfinite(q).all():
            raise ValueError("Joint positions must be finite scalars in radians")
        for name, value in zip(self.backend_names, q):
            if name in self.limits and not self.limits[name][0] <= value <= self.limits[name][1]:
                raise ValueError(f"Joint outside URDF limits: {name}")
        return q

    def evaluate(self, positions, world_base=None):
        from robokit.lie.se3 import se3_from_matrix, se3_to_matrix
        import warp as wp
        base = np.eye(4) if world_base is None else np.asarray(world_base, dtype=float)
        if (base.shape != (4, 4) or not np.isfinite(base).all()
                or not np.allclose(base[3], [0, 0, 0, 1], atol=1e-8)
                or not np.allclose(base[:3, :3].T @ base[:3, :3], np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(base[:3, :3]), 1, atol=1e-8)):
            raise ValueError("Base must be a finite rigid world-from-base matrix")
        q = self.pack(positions)[None].astype(np.float32)
        b = se3_from_matrix(wp.array(base[None].astype(np.float32), dtype=wp.mat44, device='cpu'))
        state = self.robot.state(q, T_world_base=b)
        self.robot.forward_kinematics(state)
        matrices = se3_to_matrix(state.T_world_link).numpy()[0]
        self.robot.compute_center_of_mass(state)
        return dict(transforms=dict(zip(self.links, matrices)),
                    com_world_m=state.com_world.numpy()[0], state=state)
