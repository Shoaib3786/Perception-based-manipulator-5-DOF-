"""Run AFTER the existing Gazebo launch. No mock hardware or extra RSP."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    own = get_package_share_directory('roboarm_motion')
    description = get_package_share_directory('roboarm_urdf_description')
    original = get_package_share_directory('roboarm_movit_config')
    config = (
        MoveItConfigsBuilder('Robot_Arm', package_name='roboarm_movit_config')
        .robot_description(file_path=os.path.join(description, 'urdf', 'roboArm_urdf.urdf.xacro'))
        .robot_description_kinematics(file_path=os.path.join(own, 'config', 'kinematics.yaml'))
        .trajectory_execution(file_path=os.path.join(own, 'config', 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    parameters = config.to_dict()
    parameters['robot_description_kinematics']['arm']['position_only_ik'] = ParameterValue(
        LaunchConfiguration('position_only_ik'), value_type=bool)
    # The generated config references a nonexistent Kinect. Do not start those updaters.
    # This does NOT add the Gazebo table or boxes to the MoveIt planning scene.
    parameters.pop('sensors', None)
    parameters.pop('default_sensor', None)
    parameters.pop('kinect_depthimage', None)
    parameters['use_sim_time'] = True
    # Demonstration acceleration limits; tune to the simulated mechanism.
    limits = parameters['robot_description_planning']['joint_limits']
    for name, values in limits.items():
        if 'max_velocity' in values:
            values['max_velocity'] = float(values['max_velocity'])
        values['has_acceleration_limits'] = True
        values['max_acceleration'] = 0.02 if name == 'joint_prism' else 0.5
    return LaunchDescription([
        DeclareLaunchArgument('use_fingertip_target', default_value='true'),
        DeclareLaunchArgument('reference_gripper_angle', default_value='-0.2406546887035444'),
        DeclareLaunchArgument('start_approach', default_value='true'),
        DeclareLaunchArgument('position_only_ik', default_value='true'),
        DeclareLaunchArgument('target_topic', default_value='/perception/target_xyz'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/perception/red/points'),
        DeclareLaunchArgument('input_mode', default_value='cloud'),
        DeclareLaunchArgument('approach_height', default_value='0.20'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.1'),
        DeclareLaunchArgument('acceleration_scaling', default_value='0.1'),
        Node(package='moveit_ros_move_group', executable='move_group',
             output='screen', parameters=[parameters]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(original, 'config', 'moveit.rviz')],
             parameters=[parameters]),
        Node(package='roboarm_motion', executable='approach_node', output='screen',
             condition=IfCondition(LaunchConfiguration('start_approach')),
             parameters=[{
                 'use_sim_time': True,
                 'use_fingertip_target': ParameterValue(LaunchConfiguration('use_fingertip_target'), value_type=bool),
                 'reference_gripper_angle': ParameterValue(LaunchConfiguration('reference_gripper_angle'), value_type=float),
                 'velocity_scaling': ParameterValue(LaunchConfiguration('velocity_scaling'), value_type=float),
                 'acceleration_scaling': ParameterValue(LaunchConfiguration('acceleration_scaling'), value_type=float),
                 'cloud_topic': LaunchConfiguration('cloud_topic'),
                 'input_mode': LaunchConfiguration('input_mode'),
                 'target_topic': LaunchConfiguration('target_topic'),
                 'approach_height': ParameterValue(LaunchConfiguration('approach_height'), value_type=float),
             }]),
    ])
