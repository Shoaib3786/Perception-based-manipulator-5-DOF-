"""Regression for the target that failed constrained-pose sampling."""
import numpy as np
import pytest
from roboarm_motion.arm_ik import ArmGeometry
from roboarm_motion.gripper_geometry import load_finger_pads, fingertip_geometry, grasp_yaw_for_target,rotation


@pytest.mark.parametrize('z', [.441, .641])
def test_logged_target_has_exact_joint_solutions(z):
    geometry = ArmGeometry()
    tcp = fingertip_geometry(load_finger_pads()[0], -.2406546887035444)[0]
    yaw = grasp_yaw_for_target(2.532, -.965, geometry.lateral + tcp[0])
    target = [2.532, -.965, z]
    candidates = geometry.solve(target, tcp, yaw, -.3)
    assert len(candidates) >= 10
    for q in candidates:
        point, basis = geometry.forward(q, tcp)
        assert np.allclose(point, target, atol=1e-7)
        assert np.allclose(basis, rotation((-.3, 0., yaw)), atol=1e-7)
        assert all(geometry.limits[n][0] <= value <= geometry.limits[n][1] for n,value in q.items())


def test_incompatible_yaw_and_unreachable_target_rejected():
    geometry = ArmGeometry()
    tcp = fingertip_geometry(load_finger_pads()[0], -.2406546887035444)[0]
    assert not geometry.solve([2.532, -.965, .641], tcp, 0., -.3)
    yaw = grasp_yaw_for_target(100., 0., geometry.lateral + tcp[0])
    assert not geometry.solve([100., 0., .641], tcp, yaw, -.3)
