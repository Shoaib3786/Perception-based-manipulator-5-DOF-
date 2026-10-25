"""Snapshot a cloud/point, transform at capture time, request one MoveIt motion.

Services: ~/plan and ~/move (Trigger). A service response acknowledges submission;
MoveIt's eventual result is logged. ~/cancel requests cancellation of an active goal.
Cloud mode assumes ONE object on the selected color topic, aligned RGB/depth,
and valid calibration. Point mode accepts a final link target without an offset.
"""
import copy
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, DisplayTrajectory, MoveItErrorCodes
from visualization_msgs.msg import Marker


class ApproachNode(Node):
    def __init__(self):
        super().__init__('approach_node')
        defaults = {
            'input_mode': 'cloud', 
            'cloud_topic': '/perception/red/points',
            'target_topic': '/perception/target_xyz', 
            'target_frame': 'base_link',
            'group': 'arm', 
            'tip_link': 'claw_link', 
            'approach_height': 0.20,
            'max_target_age': 2.0, 
            'position_tolerance': 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        if self.settings['input_mode'] not in ('cloud', 'point'):
            raise ValueError('input_mode must be cloud or point')
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.latest = None
        self.busy = False
        self.goal_handle = None
        self.cancel_pending = False
        self.client = ActionClient(self, MoveGroup, '/move_action')
        if self.settings['input_mode'] == 'cloud':
            self.input_sub = self.create_subscription(
                PointCloud2, self.settings['cloud_topic'], self.on_cloud,
                qos_profile_sensor_data)
        else:
            self.input_sub = self.create_subscription(
                PointStamped, self.settings['target_topic'], self.on_point,
                qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, '~/target_marker', latched)
        self.preview_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', latched)
        self.plan_srv = self.create_service(Trigger, '~/plan', self.on_plan)
        self.move_srv = self.create_service(Trigger, '~/move', self.on_move)
        self.cancel_srv = self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.get_logger().info(
            f"Input mode={self.settings['input_mode']}. Waiting for input; "
            'call /approach_node/plan or /approach_node/move. No automatic motion.')

    def on_cloud(self, msg):
        # Empty clouds invalidate old targets rather than retaining a vanished object.
        self.latest = None
        try:
            xyz = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            if len(xyz) == 0:
                return
            mean = xyz.mean(axis=0)
            point = PointStamped()
            point.header = copy.deepcopy(msg.header)
            point.point.x, point.point.y, point.point.z = map(float, mean)
            self.latest = point
        except (ValueError, AssertionError, TypeError) as exc:
            self.get_logger().error(f'Invalid XYZ cloud: {exc}', throttle_duration_sec=2.0)

    def on_point(self, msg):
        self.latest = None
        if np.isfinite([msg.point.x, msg.point.y, msg.point.z]).all():
            self.latest = copy.deepcopy(msg)

    def on_plan(self, request, response):
        return self.submit(response, execute=False)

    def on_move(self, request, response):
        return self.submit(response, execute=True)

    def submit(self, response, execute):
        if self.busy:
            response.message = 'A request is already active.'
            return response
        if self.latest is None or not self.latest.header.frame_id:
            response.message = 'No valid current target. Check input topic and frame_id.'
            return response
        if not self.client.server_is_ready():
            response.message = 'MoveIt /move_action is not ready.'
            return response
        point = copy.deepcopy(self.latest)
        age = (self.get_clock().now() - Time.from_msg(point.header.stamp)).nanoseconds / 1e9
        if age < -0.1 or age > self.settings['max_target_age']:
            response.message = f'Target age {age:.3f}s is invalid. Republish with capture time.'
            return response
        try:
            # Nonblocking: ROS can continue processing TF callbacks. Retry the service
            # if transform data for the capture time has not arrived yet.
            if point.header.frame_id != self.settings['target_frame']:
                transform = self.buffer.lookup_transform(
                    self.settings['target_frame'], point.header.frame_id,
                    Time.from_msg(point.header.stamp))
                point = do_transform_point(point, transform)
        except TransformException as exc:
            response.message = f'TF unavailable at capture time; retry: {exc}'
            return response
        if self.settings['input_mode'] == 'cloud':
            point.point.z += self.settings['approach_height']
        point.header.frame_id = self.settings['target_frame']
        # It is now a fixed spatial goal in the base frame, not an old camera goal.
        point.header.stamp = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header = point.header
        marker.ns = 'approach_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04
        marker.color.r = marker.color.g = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
        goal = self.make_goal(point, execute)
        self.busy = True
        self.cancel_pending = False
        try:
            future = self.client.send_goal_async(goal)
            future.add_done_callback(self.on_goal)
        except Exception as exc:
            self.busy = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = (
            f"Submitted {'plan + execute' if execute else 'plan only'}: "
            f"{self.settings['tip_link']} -> "
            f'({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f}) '
            f"in {point.header.frame_id}. Submission is not completion.")
        self.get_logger().info(response.message)
        return response

    def make_goal(self, point, execute):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.settings['group']
        request.pipeline_id = 'ompl'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.start_state.is_diff = True  # Use current monitored joint state.
        request.workspace_parameters.header.frame_id = self.settings['target_frame']
        request.workspace_parameters.min_corner.x = -10.0
        request.workspace_parameters.min_corner.y = -10.0
        request.workspace_parameters.min_corner.z = -10.0
        request.workspace_parameters.max_corner.x = 10.0
        request.workspace_parameters.max_corner.y = 10.0
        request.workspace_parameters.max_corner.z = 10.0
        constraint = PositionConstraint()
        constraint.header = point.header
        constraint.link_name = self.settings['tip_link']
        constraint.weight = 1.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [float(self.settings['position_tolerance'])]
        pose = Pose()
        pose.position = point.point
        pose.orientation.w = 1.0
        constraint.constraint_region.primitives = [region]
        constraint.constraint_region.primitive_poses = [pose]
        constraints = Constraints()
        constraints.name = 'position_only_approach'
        constraints.position_constraints = [constraint]
        request.goal_constraints = [constraints]
        # No orientation constraint: the five-axis arm chooses a feasible orientation.
        goal.planning_options.plan_only = not execute
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def on_goal(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                raise RuntimeError('MoveIt rejected the goal.')
            self.goal_handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_pending:
                self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.busy = False
            self.goal_handle = None

    def on_result(self, future):
        try:
            result = future.result().result
            if result.planned_trajectory.joint_trajectory.points:
                preview = DisplayTrajectory()
                preview.model_id = 'Robot_Arm'
                preview.trajectory_start = result.trajectory_start
                preview.trajectory = [result.planned_trajectory]
                self.preview_pub.publish(preview)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('MoveIt request SUCCEEDED (plan-only or execution as requested).')
            else:
                self.get_logger().error(f'MoveIt failed: error code {result.error_code.val}')
        except Exception as exc:
            self.get_logger().error(f'Action result error: {exc}')
        finally:
            self.busy = False
            self.goal_handle = None
            self.cancel_pending = False

    def on_cancel(self, request, response):
        if not self.busy:
            response.message = 'No active request.'
            return response
        self.cancel_pending = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        response.success = True
        response.message = 'Cancellation requested; wait for the action result.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
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
