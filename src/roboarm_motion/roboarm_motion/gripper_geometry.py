"""Derive finger-pad geometry from the installed URDF and binary STL meshes."""
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def vector(element, attribute, default='0 0 0'):
    return np.array(list(map(float, element.get(attribute, default).split())))


def load_finger_pads(directory=None, pad_depth=0.03):
    """Use the distal 3 cm of each mesh, including mesh scale and URDF origins.

    Pad center is the bounding-box center of that region, not an inferred
    force/contact center. The current fingers extend in local joint-frame +Y.
    """
    directory = Path(directory or get_package_share_directory('roboarm_urdf_description'))
    robot = ET.fromstring(xacro.process_file(
        str(directory / 'urdf' / 'roboArm_urdf.urdf.xacro')).toxml())
    pads = []
    for link_name, joint_name in [('gripper_right', 'joint_5'), ('gripper_left', 'joint_6')]:
        joint = robot.find(f"joint[@name='{joint_name}']")
        origin = joint.find('origin')
        if not np.allclose(vector(joint.find('axis'), 'xyz'), [0, 0, 1]):
            raise ValueError('Finger geometry expects the current local-Z revolute axes')
        collision = robot.find(f"link[@name='{link_name}']/collision")
        mesh = collision.find('geometry/mesh')
        path = directory / mesh.get('filename').split('roboarm_urdf_description/', 1)[1]
        raw = path.read_bytes()
        count = struct.unpack_from('<I', raw, 80)[0]
        if len(raw) != 84 + count * 50:
            raise ValueError(f'Expected binary STL: {path}')
        vertices = np.array([struct.unpack_from('<9f', raw, 96 + i * 50)
                             for i in range(count)]).reshape(-1, 3)
        vertices *= vector(mesh, 'scale', '1 1 1')
        mesh_origin = collision.find('origin')
        vertices = vertices @ rotation(vector(mesh_origin, 'rpy')).T + vector(mesh_origin, 'xyz')
        vertices = np.unique(vertices, axis=0)
        pad = vertices[vertices[:, 1] >= vertices[:, 1].max() - pad_depth]
        pads.append({'link': link_name, 'vertices': pad,
                     'center': (pad.min(0) + pad.max(0)) / 2,
                     'origin': vector(origin, 'xyz'), 'rotation': rotation(vector(origin, 'rpy'))})
    # All arm joints after joint_1 rotate about local X in the current URDF.
    lateral = sum(float(robot.find(f"joint[@name='{name}']/origin").get('xyz').split()[0])
                  for name in ['joint_2', 'joint_3', 'joint_4', 'joint_prism', 'nuckles_joint'])
    return pads, lateral


def fingertip_geometry(pads, angle):
    """Midpoint and inner pad gap in claw_link at the specified joint_5 angle."""
    points, centers = [], []
    for pad, q in zip(pads, (angle, -angle)):
        transform = pad['rotation'] @ rotation((0., 0., q))
        points.append(pad['vertices'] @ transform.T + pad['origin'])
        centers.append(transform @ pad['center'] + pad['origin'])
    midpoint = (centers[0] + centers[1]) / 2
    gap = points[0][:, 0].min() - points[1][:, 0].max()
    return midpoint, float(gap), centers


def closing_angle_for_width(pads, width):
    """Find the first tip-gap crossing; used for the nominal block-size example."""
    previous = 0.
    for angle in np.linspace(0., -math.pi / 2, 401)[1:]:
        if fingertip_geometry(pads, angle)[1] >= width:
            low, high = angle, previous
            for _ in range(40):
                middle = (low + high) / 2
                if fingertip_geometry(pads, middle)[1] >= width:
                    low = middle
                else:
                    high = middle
            return (low + high) / 2
        previous = angle
    raise ValueError('Requested width exceeds the measured tip gap range')


def grasp_yaw_for_target(x, y, lateral_offset):
    """Solve yaw for this arm's yaw + parallel pitch-axis kinematic structure."""
    radius = math.hypot(x, y)
    if radius <= abs(lateral_offset) + 1e-9:
        raise ValueError('Target is too close to the base axis for this grasp')
    yaw = math.atan2(y, x) - math.acos(lateral_offset / radius)
    return math.atan2(math.sin(yaw), math.cos(yaw))


def inset_grasp_point(tcp, depth):
    """Move the desired contact point toward the palm along claw-local -Y."""
    point = np.asarray(tcp, dtype=float).copy()
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('TCP must be a finite XYZ vector')
    if not math.isfinite(depth) or not 0. <= depth <= 0.10:
        raise ValueError('grasp_insertion_depth must be between 0 and 0.10 metres')
    if point[1] <= depth:
        raise ValueError('Insertion depth would move the grasp point behind the claw')
    point[1] -= depth
    return point
