"""Verify the measured finger transform chain and target reference point."""
import math
import numpy as np
import pytest
import xacro
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from roboarm_motion.gripper_geometry import (
    load_finger_pads, fingertip_geometry, closing_angle_for_width,
    grasp_yaw_for_target, rotation)


def test_finger_geometry_and_tip_frames():
    pads, lateral = load_finger_pads()
    assert np.isclose(lateral, -.044)
    midpoint, gap, _ = fingertip_geometry(pads, 0.)
    assert np.isclose(midpoint[1], .715, atol=.001)
    assert gap < 0.  # Near-zero angle is fully closed, not a 28 cm opening.
    directory = Path(get_package_share_directory('roboarm_urdf_description'))
    robot = ET.fromstring(xacro.process_file(str(directory / 'urdf/roboArm_urdf.urdf.xacro')).toxml())
    for pad, side in zip(pads, ('right', 'left')):
        origin = robot.find(f"joint[@name='{side}_fingertip_joint']/origin")
        assert np.allclose(list(map(float, origin.get('xyz').split())), pad['center'], atol=1e-8)


def test_nominal_block_close_and_midpoint():
    pads, _ = load_finger_pads()
    angle = closing_angle_for_width(pads, .28)
    assert np.isclose(angle, -.2406546887, atol=1e-8)
    point, gap, centers = fingertip_geometry(pads, angle)
    assert np.isclose(gap, .28, atol=1e-8)
    assert np.allclose(point, (centers[0] + centers[1]) / 2)
    assert point[1] > .70
    assert abs(point[1] - fingertip_geometry(pads, -.8)[0][1]) > .10


@pytest.mark.parametrize('target', [(2.6, 0.), (2.6, -1.), (2.1, .35)])
def test_yaw_includes_arm_and_tcp_lateral_offset(target):
    pads, lateral = load_finger_pads()
    point, _, _ = fingertip_geometry(pads, closing_angle_for_width(pads, .28))
    offset = lateral + point[0]
    yaw = grasp_yaw_for_target(*target, offset)
    local = rotation((0., 0., yaw)).T @ np.array([*target, 0.])
    assert np.isclose(local[0], offset)
    assert local[1] > 0
    with pytest.raises(ValueError):
        grasp_yaw_for_target(0., 0., offset)


def test_insertion_depth_moves_contact_inward_without_changing_lateral_axes():
    from roboarm_motion.gripper_geometry import inset_grasp_point
    tcp = np.array([-.00923864, .71493633, .04545])
    inset = inset_grasp_point(tcp, .03)
    assert np.allclose(inset, [-.00923864, .68493633, .04545])
    assert np.allclose(tcp, [-.00923864, .71493633, .04545])
    assert np.allclose(inset_grasp_point(tcp, 0.), tcp)
    # Keeping the same world target moves the claw forward by the chosen depth.
    basis = rotation((-.3, 0., -1.95))
    target = np.array([2.532, -.965, .441])
    assert np.allclose((target - basis @ inset) - (target - basis @ tcp),
                       basis @ np.array([0., .03, 0.]))
    for invalid in [-.01, .11, float('nan')]:
        with pytest.raises(ValueError):
            inset_grasp_point(tcp, invalid)
