"""Joint-goal IK for this URDF's yaw + parallel-pitch + sliding-wrist arm.

This avoids six-dimensional pose sampling for an underactuated chain. Returned
joint states still require MoveIt collision checking and trajectory planning.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from .gripper_geometry import rotation, vector


JOINTS = ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_prism')


class ArmGeometry:
    def __init__(self):
        root = Path(get_package_share_directory('roboarm_urdf_description'))
        robot = ET.fromstring(xacro.process_file(str(root / 'urdf/roboArm_urdf.urdf.xacro')).toxml())
        self.chain = []
        self.limits = {}
        for name in JOINTS + ('nuckles_joint',):
            joint = robot.find(f"joint[@name='{name}']")
            origin = joint.find('origin')
            xyz = vector(origin, 'xyz')
            rpy = vector(origin, 'rpy')
            if not np.allclose(rpy, 0):
                raise ValueError('Analytic arm IK expects the current zero-RPY joint origins')
            axis = np.zeros(3) if name == 'nuckles_joint' else vector(joint.find('axis'), 'xyz')
            self.chain.append((name, xyz, axis))
            if name != 'nuckles_joint':
                limit = joint.find('limit')
                self.limits[name] = (float(limit.get('lower')), float(limit.get('upper')))
        origins = {name: xyz for name, xyz, _ in self.chain}
        expected_axes = ([0,0,1], [1,0,0], [1,0,0], [1,0,0], [0,1,0])
        for (_, _, axis), expected in zip(self.chain, expected_axes):
            if not np.allclose(axis, expected):
                raise ValueError('Analytic arm IK does not match the installed joint axes')
        # Explicitly validate assumptions instead of silently solving another robot.
        for name, free in [('joint_1', [2]), ('joint_2', [0,2]), ('joint_3', [2]),
                           ('joint_4', [1]), ('joint_prism', [1]), ('nuckles_joint', [0,1])]:
            if any(abs(origins[name][i]) > 1e-9 for i in range(3) if i not in free):
                raise ValueError(f'Unexpected origin for {name}; update arm IK')
        self.shoulder_z = origins['joint_1'][2] + origins['joint_2'][2]
        self.lateral = sum(xyz[0] for _, xyz, _ in self.chain)
        self.upper = origins['joint_3'][2]
        self.fore = origins['joint_4'][1]
        self.tail = origins['joint_prism'][1] + origins['nuckles_joint'][1]

    def forward(self, values, tcp):
        """Independently compose all URDF transforms for a fingertip point."""
        basis = np.eye(3)
        point = np.zeros(3)
        for name, origin, axis in self.chain:
            point += basis @ origin
            q = values.get(name, 0.)
            if name == 'joint_prism':
                point += basis @ (axis * q)
            elif name != 'nuckles_joint':
                basis = basis @ rotation(axis * q)
        return point + basis @ np.asarray(tcp), basis

    def solve(self, target, tcp, yaw, roll, seed=None, samples=21):
        local = rotation((0., 0., yaw)).T @ np.asarray(target, dtype=float)
        tcp = np.asarray(tcp, dtype=float)
        if abs(local[0] - self.lateral - tcp[0]) > 1e-5:
            return []
        l1, l2 = self.upper, self.fore
        solutions = []
        lower, upper = self.limits['joint_prism']
        for slide in np.linspace(lower, upper, samples):
            length = self.tail + slide + tcp[1]
            y = local[1] - length * math.cos(roll) + tcp[2] * math.sin(roll)
            z = local[2] - self.shoulder_z - length * math.sin(roll) - tcp[2] * math.cos(roll)
            cosine = (y*y + z*z - l1*l1 - l2*l2) / (2*l1*l2)
            if abs(cosine) > 1 + 1e-9:
                continue
            for delta in (-math.acos(np.clip(cosine, -1, 1)), math.acos(np.clip(cosine, -1, 1))):
                alpha = math.atan2(z, y) - math.atan2(l2*math.sin(delta), l1+l2*math.cos(delta))
                q2 = math.atan2(math.sin(alpha-math.pi/2), math.cos(alpha-math.pi/2))
                q3 = delta + math.pi/2
                q4 = roll - q2 - q3
                values = dict(zip(JOINTS, (yaw, q2, q3, q4, float(slide))))
                if not all(self.limits[n][0]-1e-9 <= q <= self.limits[n][1]+1e-9
                           for n, q in values.items()):
                    continue
                actual, orientation = self.forward(values, tcp)
                if not np.allclose(actual, target, atol=1e-7):
                    continue
                if not np.allclose(orientation, rotation((roll, 0., yaw)), atol=1e-7):
                    continue
                solutions.append(values)
        seed = seed or {name: 0. for name in JOINTS}
        solutions.sort(key=lambda q: sum(((q[n]-seed.get(n, 0.)) /
                                         (self.limits[n][1]-self.limits[n][0]))**2 for n in JOINTS))
        return solutions
