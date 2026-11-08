"""Explicit, asynchronous open/approach/descend/close/lift demonstration."""
import copy
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes)
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from .approach_node import ApproachNode
from .arm_ik import ArmGeometry
from .gripper_geometry import load_finger_pads, fingertip_geometry, grasp_yaw_for_target, inset_grasp_point


STAGES = ('open', 'pregrasp', 'descend', 'close', 'lift')


def grasp_quaternion(yaw, roll=-0.3):
    # Rz(yaw) Rx(roll): local Y follows the fingers; negative roll tilts down.
    sy, cy = math.sin(yaw / 2), math.cos(yaw / 2)
    sr, cr = math.sin(roll / 2), math.cos(roll / 2)
    return Quaternion(x=sr * cy, y=sr * sy, z=cr * sy, w=cr * cy)


def predicted_state(result):
    """Propagate a plan endpoint so the next preview starts at that state."""
    state = copy.deepcopy(result.trajectory_start)
    trajectory = result.planned_trajectory.joint_trajectory
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    if trajectory.points:
        values.update(zip(trajectory.joint_names, trajectory.points[-1].positions))
    if 'joint_5' in values:
        values['joint_6'] = -values['joint_5']
    state.joint_state.name = list(values)
    state.joint_state.position = list(values.values())
    state.joint_state.velocity = []
    state.joint_state.effort = []
    state.joint_state.header.stamp.sec = 0
    state.joint_state.header.stamp.nanosec = 0
    state.is_diff = True
    return state


class PickNode(ApproachNode):
    def __init__(self):
        super().__init__('pick_node')
        defaults = {
            'grasp_offset_x': 0.0, 'grasp_offset_y': 0.0, 'grasp_offset_z': 0.0,
            'tcp_x': -0.00923864, 'tcp_y': 0.71493633, 'tcp_z': 0.04545,
            'grasp_yaw': 0.0, 'grasp_roll': -0.3, 'orientation_tolerance': 0.15,
            'pregrasp_height': 0.20, 'lift_height': 0.20,
            'gripper_open': -0.8, 'gripper_closed': -0.2406546887035444,
            'contact_velocity_scaling': 0.1, 'object_radius': 0.14,
            'grasp_insertion_depth': 0.03,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.pick_settings = {name: float(self.get_parameter(name).value) for name in defaults}
        if not all(math.isfinite(v) for v in self.pick_settings.values()):
            raise ValueError('Pick parameters must be finite')
        for name in ('pregrasp_height', 'lift_height', 'orientation_tolerance', 'object_radius'):
            if self.pick_settings[name] <= 0:
                raise ValueError(f'{name} must be positive')
        if not 0 < self.pick_settings['contact_velocity_scaling'] <= 1:
            raise ValueError('contact_velocity_scaling must be in (0, 1]')
        for name in ('gripper_open', 'gripper_closed'):
            if not -math.pi / 2 <= self.pick_settings[name] <= 0:
                raise ValueError(f'{name} is outside joint_5 URDF limits [-pi/2, 0]')
        if self.pick_settings['gripper_open'] >= self.pick_settings['gripper_closed']:
            raise ValueError('gripper_open must be more negative than gripper_closed')
        self.declare_parameter('auto_tcp', True)
        self.declare_parameter('auto_grasp_yaw', True)
        self.auto_tcp = self.get_parameter('auto_tcp').value
        self.auto_grasp_yaw = self.get_parameter('auto_grasp_yaw').value
        pads, self.arm_lateral_offset = load_finger_pads()
        midpoint, gap, _ = fingertip_geometry(pads, self.pick_settings['gripper_closed'])
        if self.auto_tcp:
            for axis, value in zip(('x', 'y', 'z'), midpoint):
                self.pick_settings['tcp_' + axis] = float(value)
        inset = inset_grasp_point(
            [self.pick_settings['tcp_' + axis] for axis in ('x', 'y', 'z')],
            self.pick_settings['grasp_insertion_depth'])
        for axis, value in zip(('x', 'y', 'z'), inset):
            self.pick_settings['tcp_' + axis] = float(value)
        self.active_yaw = self.pick_settings['grasp_yaw']
        self.get_logger().info(
            'Grasp TCP in claw_link: '
            + ', '.join(f'{axis}={self.pick_settings["tcp_" + axis]:.6f}' for axis in ('x', 'y', 'z'))
            + f' m; insertion={self.pick_settings["grasp_insertion_depth"]:.3f} m; mesh tip inner gap at configured close angle={gap:.4f} m. '
            'This is a geometric estimate, not force/contact calibration.')
        self.arm_geometry = ArmGeometry()
        self.sequence = None
        self.index = 0
        self.predicted = None
        self.pick_execute = False
        self.pick_target = None
        self.previews = []
        self.preview_start = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.poses_pub = self.create_publisher(PoseArray, '~/grasp_poses', latched)
        self.create_service(Trigger, '~/plan_pick', self.on_plan_pick)
        self.create_service(Trigger, '~/pick', self.on_pick)
        self.get_logger().info(
            'Pick demo ready: /pick_node/plan_pick previews all stages; /pick_node/pick executes. '
            'TCP is derived from finger geometry by default; verify object-center offsets and physical contact before execution.')

    def status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_plan_pick(self, request, response):
        return self.start_pick(response, False)

    def on_pick(self, request, response):
        return self.start_pick(response, True)

    def start_pick(self, response, execute):
        if self.busy:
            response.message = 'A motion request is active'
            return response
        if not self.client.server_is_ready():
            response.message = 'MoveIt /move_action is not ready'
            return response
        try:
            self.pick_target = self.resolve_target()
        except (ValueError, TransformException) as error:
            response.message = str(error)
            return response
        for axis in ('x', 'y', 'z'):
            setattr(self.pick_target.point, axis, getattr(self.pick_target.point, axis)
                    + self.pick_settings['grasp_offset_' + axis])
        try:
            if self.auto_grasp_yaw:
                if self.settings['target_frame'] != 'base_link':
                    raise ValueError('Automatic yaw requires target_frame=base_link')
                self.active_yaw = grasp_yaw_for_target(
                    self.pick_target.point.x, self.pick_target.point.y,
                    self.arm_lateral_offset + self.pick_settings['tcp_x'])
            else:
                self.active_yaw = self.pick_settings['grasp_yaw']
        except ValueError as error:
            response.message = str(error)
            return response
        self.get_logger().info(
            f'Finger-contact center target: ({self.pick_target.point.x:.3f}, '
            f'{self.pick_target.point.y:.3f}, {self.pick_target.point.z:.3f}); '
            f'claw yaw={self.active_yaw:.3f} rad')
        self.sequence = STAGES
        self.index = 0
        self.predicted = None
        self.previews = []
        self.preview_start = None
        self.execution_requested = execute
        self.pick_execute = False  # Validate all five stages before any physical motion.
        self.busy = True
        self.cancel_pending = False
        poses = PoseArray(header=self.pick_target.header)
        for height in (self.pick_settings['pregrasp_height'], 0., self.pick_settings['lift_height']):
            pose = Pose(position=copy.deepcopy(self.pick_target.point),
                        orientation=grasp_quaternion(getattr(self, 'active_yaw', self.pick_settings['grasp_yaw']),
                        self.pick_settings.get('grasp_roll', -0.3)))
            pose.position.z += height
            poses.poses.append(pose)
        self.poses_pub.publish(poses)
        self.next_stage()
        response.success = self.busy
        response.message = ('Submitted open/pregrasp/descend/close/lift '
                            + ('execution' if execute else 'plan-only preview')
                            + '; execution first validates all stages without motion. '
                            'Monitor /pick_node/status; submission is not completion.')
        return response

    def stage_goal(self, stage):
        point = copy.deepcopy(self.pick_target)
        if stage == 'pregrasp':
            point.point.z += self.pick_settings['pregrasp_height']
        elif stage == 'lift':
            point.point.z += self.pick_settings['lift_height']
        goal = self.make_goal(point, self.pick_execute)
        request = goal.request
        if self.predicted is not None and not self.pick_execute:
            request.start_state = copy.deepcopy(self.predicted)
        if stage in ('open', 'close'):
            request.group_name = 'gripper'
            angle = self.pick_settings['gripper_open' if stage == 'open' else 'gripper_closed']
            constraint = JointConstraint(joint_name='joint_5', position=angle,
                                         tolerance_above=0.01, tolerance_below=0.01, weight=1.)
            request.goal_constraints = [Constraints(joint_constraints=[constraint])]
        else:
            geometry = getattr(self, 'arm_geometry', None)
            if geometry is None:
                raise ValueError('Arm geometry is not initialized')
            tcp = [self.pick_settings['tcp_' + axis] for axis in ('x', 'y', 'z')]
            target = [point.point.x, point.point.y, point.point.z]
            seed = None
            if self.predicted is not None:
                seed = dict(zip(self.predicted.joint_state.name, self.predicted.joint_state.position))
            candidates = geometry.solve(
                target, tcp, self.active_yaw, self.pick_settings['grasp_roll'], seed=seed)
            if not candidates:
                raise ValueError(f'No joint-limit-valid IK for {stage} at {target}; '
                                 'check target, grasp_roll and fingertip geometry')
            # Each alternative is an exact URDF-verified joint goal. MoveIt still
            # checks collisions and plans a trajectory; pose sampling is bypassed.
            request.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=name, position=value,
                                tolerance_above=0.0001, tolerance_below=0.0001, weight=1.)
                for name, value in candidate.items()]) for candidate in candidates]
        if stage in ('descend', 'close', 'lift'):
            request.max_velocity_scaling_factor = min(
                request.max_velocity_scaling_factor, self.pick_settings['contact_velocity_scaling'])
        if stage == 'lift':
            # Collision representation only: this does NOT weld anything in Gazebo.
            attached = AttachedCollisionObject()
            attached.link_name = self.settings['tip_link']
            attached.touch_links = ['claw_link', 'gripper_left', 'gripper_right']
            attached.object.id = 'pick_payload_estimate'
            attached.object.header.frame_id = self.settings['tip_link']
            attached.object.operation = CollisionObject.ADD
            sphere = SolidPrimitive(type=SolidPrimitive.SPHERE,
                                    dimensions=[self.pick_settings['object_radius']])
            pose = Pose()
            pose.position.x = self.pick_settings['tcp_x']
            pose.position.y = self.pick_settings['tcp_y']
            pose.position.z = self.pick_settings['tcp_z']
            pose.orientation.w = 1.
            attached.object.primitives = [sphere]
            attached.object.primitive_poses = [pose]
            goal.planning_options.planning_scene_diff.robot_state.attached_collision_objects = [attached]
        return goal

    def next_stage(self):
        if self.cancel_pending:
            self.finish_pick('CANCELED; no further stages submitted')
            return
        if self.index == len(STAGES):
            if not self.pick_execute and getattr(self, 'execution_requested', False):
                # Keep the validated spatial snapshot, but execute by replanning
                # each action against the actual current robot state and scene.
                self.status('PREFLIGHT COMPLETE; starting execution')
                self.pick_execute = True
                self.index = 0
                self.predicted = None
                self.previews = []
                self.preview_start = None
                self.next_stage()
                return
            self.finish_pick('SEQUENCE COMPLETE; object contact/retention unverified'
                             if self.pick_execute else 'PREVIEW COMPLETE; no motion executed')
            return
        stage = STAGES[self.index]
        label = 'EXECUTE ' if self.pick_execute else ('PREFLIGHT ' if getattr(self, 'execution_requested', False) else 'PLAN ')
        self.status(label + stage)
        try:
            self.goal_handle = None
            self.client.send_goal_async(self.stage_goal(stage)).add_done_callback(self.on_goal)
        except Exception as error:
            self.finish_pick(f'FAILED {stage}: {error}')

    def on_goal(self, future):
        super().on_goal(future)
        if self.sequence is not None and not self.busy:
            self.finish_pick(f'FAILED {STAGES[self.index]}: goal rejected or unavailable')

    def on_result(self, future):
        if self.sequence is None:
            return super().on_result(future)
        try:
            wrapped = future.result()
            result = wrapped.result
            if self.cancel_pending or wrapped.status != 4 or result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.finish_pick(f'STOPPED {STAGES[self.index]}: action={wrapped.status}, '
                                 f'MoveIt={result.error_code.val}; no further stages submitted')
                return
            if not self.pick_execute:
                self.predicted = predicted_state(result)
            if result.planned_trajectory.joint_trajectory.points:
                if self.preview_start is None:
                    self.preview_start = copy.deepcopy(result.trajectory_start)
                self.previews.append(copy.deepcopy(result.planned_trajectory))
                from moveit_msgs.msg import DisplayTrajectory
                preview = DisplayTrajectory(model_id='Robot_Arm',
                                            trajectory_start=copy.deepcopy(self.preview_start),
                                            trajectory=copy.deepcopy(self.previews))
                self.preview_pub.publish(preview)
            self.goal_handle = None
            self.index += 1
            self.next_stage()
        except Exception as error:
            self.finish_pick(f'FAILED action result: {error}')

    def finish_pick(self, message):
        self.status(message)
        self.busy = False
        self.goal_handle = None
        self.cancel_pending = False
        self.sequence = None


def main(args=None):
    rclpy.init(args=args)
    node = PickNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
