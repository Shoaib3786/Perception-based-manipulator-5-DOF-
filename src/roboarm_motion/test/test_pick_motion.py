"""Check motion requests and sequence transitions without moving a robot."""
from types import SimpleNamespace
import copy
import numpy as np
import pytest
from geometry_msgs.msg import PointStamped
from moveit_msgs.action import MoveGroup
from rclpy.parameter import Parameter
from roboarm_motion.approach_node import ApproachNode
from roboarm_motion.pick_node import PickNode, predicted_state, grasp_quaternion, STAGES
from roboarm_motion.arm_ik import ArmGeometry
from roboarm_motion.gripper_geometry import grasp_yaw_for_target


def fake_node():
    node = SimpleNamespace(settings={
        'group': 'arm', 'target_frame': 'base_link', 'tip_link': 'claw_link',
        'position_tolerance': .01, 'velocity_scaling': .6, 'acceleration_scaling': .4})
    node.pick_settings = {
        'pregrasp_height': .2, 'lift_height': .3, 'gripper_open': -.8, 'gripper_closed': -.4,
        'tcp_x': -.00923864, 'tcp_y': .71493633, 'tcp_z': .04545, 'grasp_yaw': 0., 'grasp_roll': -.3,
        'orientation_tolerance': .15, 'contact_velocity_scaling': .1, 'object_radius': .14}
    node.pick_target = PointStamped()
    node.pick_target.header.frame_id = 'base_link'
    node.pick_target.point.x = 2.532
    node.pick_target.point.y = -.965
    node.pick_target.point.z = .441
    node.arm_geometry = ArmGeometry()
    node.active_yaw = grasp_yaw_for_target(2.532, -.965, -.044 + node.pick_settings['tcp_x'])
    node.pick_execute = False
    node.predicted = None
    node.make_goal = lambda point, execute: ApproachNode.make_goal(node, point, execute)
    return node


def test_speed_and_plan_mode():
    node = fake_node()
    goal = node.make_goal(node.pick_target, False)
    assert goal.request.max_velocity_scaling_factor == .6
    assert goal.request.max_acceleration_scaling_factor == .4
    assert goal.planning_options.plan_only


@pytest.mark.parametrize('value', [0., -1., 1.1, float('nan'), float('inf')])
def test_invalid_runtime_speed(value):
    node = fake_node()
    node.busy = False
    response = ApproachNode.update_speed(node, [Parameter('velocity_scaling', value=value)])
    assert not response.successful
    assert node.settings['velocity_scaling'] == .6


def test_speed_change_during_motion_rejected():
    node = fake_node()
    node.busy = True
    assert not ApproachNode.update_speed(node, [Parameter('velocity_scaling', value=.8)]).successful


def test_grasp_goals_and_payload():
    node = fake_node()
    pre = PickNode.stage_goal(node, 'pregrasp')
    assert pre.request.goal_constraints
    for candidate in pre.request.goal_constraints:
        joints = {c.joint_name: c.position for c in candidate.joint_constraints}
        tcp = [node.pick_settings['tcp_' + axis] for axis in ('x', 'y', 'z')]
        point, _ = node.arm_geometry.forward(joints, tcp)
        assert np.allclose(point, [2.532, -.965, .641], atol=1e-7)
    close = PickNode.stage_goal(node, 'close')
    assert close.request.group_name == 'gripper'
    assert close.request.goal_constraints[0].joint_constraints[0].joint_name == 'joint_5'
    assert close.request.goal_constraints[0].joint_constraints[0].position == -.4
    assert close.request.max_velocity_scaling_factor == .1
    lift = PickNode.stage_goal(node, 'lift')
    attached = lift.planning_options.planning_scene_diff.robot_state.attached_collision_objects[0]
    assert attached.link_name == 'claw_link'
    assert list(attached.object.primitives[0].dimensions) == [.14]


def test_preview_propagates_gripper_and_mimic():
    result = MoveGroup.Result()
    result.trajectory_start.joint_state.name = ['joint_1', 'joint_5', 'joint_6']
    result.trajectory_start.joint_state.position = [.2, 0., 0.]
    from trajectory_msgs.msg import JointTrajectoryPoint
    result.planned_trajectory.joint_trajectory.joint_names = ['joint_5']
    result.planned_trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[-.8])]
    state = predicted_state(result)
    assert dict(zip(state.joint_state.name, state.joint_state.position)) == {
        'joint_1': .2, 'joint_5': -.8, 'joint_6': .8}


@pytest.mark.parametrize('cancel,status,error', [(False, 6, -1), (True, 4, 1), (False, 5, 1)])
def test_failure_or_cancel_does_not_advance(cancel, status, error):
    node = fake_node()
    node.sequence = STAGES
    node.index = 3  # Closing must succeed before lifting.
    node.cancel_pending = cancel
    events = []
    node.finish_pick = events.append
    node.next_stage = lambda: pytest.fail('Must not proceed to lift')
    result = MoveGroup.Result()
    result.error_code.val = error
    future = SimpleNamespace(result=lambda: SimpleNamespace(status=status, result=result))
    PickNode.on_result(node, future)
    assert len(events) == 1 and node.index == 3


@pytest.mark.parametrize('execute', [False, True])
def test_every_stage_respects_preview_or_execute(execute):
    node = fake_node()
    node.pick_execute = execute
    for stage in STAGES:
        assert PickNode.stage_goal(node, stage).planning_options.plan_only is not execute


def test_valid_runtime_speed_change():
    node = fake_node()
    node.busy = False
    result = ApproachNode.update_speed(node, [Parameter('velocity_scaling', value=.8)])
    assert result.successful and node.settings['velocity_scaling'] == .8


def test_approach_targets_offset_not_claw_origin():
    node = fake_node()
    node.approach_tcp = (-.00923864, .71493633, .04545)
    goal = node.make_goal(node.pick_target, False)
    offset = goal.request.goal_constraints[0].position_constraints[0].target_point_offset
    assert (offset.x, offset.y, offset.z) == node.approach_tcp


def test_execution_preflight_completes_before_open():
    node = fake_node()
    node.index = len(STAGES)
    node.pick_execute = False
    node.execution_requested = True
    node.cancel_pending = False
    events = []
    node.status = events.append
    node.next_stage = lambda: events.append('execute_next')
    node.finish_pick = lambda text: pytest.fail('Must start validated execution')
    PickNode.next_stage(node)
    assert node.pick_execute and node.index == 0 and node.predicted is None
    assert events == ['PREFLIGHT COMPLETE; starting execution', 'execute_next']


def test_pick_request_starts_with_planning_not_physical_open():
    from std_srvs.srv import Trigger
    node = fake_node()
    node.busy = False
    node.client = SimpleNamespace(server_is_ready=lambda: True)
    node.resolve_target = lambda: copy.deepcopy(node.pick_target)
    node.auto_grasp_yaw = False
    node.get_logger = lambda: SimpleNamespace(info=lambda text: None)
    node.poses_pub = SimpleNamespace(publish=lambda msg: None)
    for axis in ('x', 'y', 'z'):
        node.pick_settings['grasp_offset_' + axis] = 0.
    events = []
    node.next_stage = lambda: events.append(node.pick_execute)
    response = PickNode.start_pick(node, Trigger.Response(), True)
    assert response.success
    assert node.execution_requested and events == [False]
