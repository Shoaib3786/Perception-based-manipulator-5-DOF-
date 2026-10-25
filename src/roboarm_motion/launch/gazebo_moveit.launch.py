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
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/perception/red/points'),
        DeclareLaunchArgument('input_mode', default_value='cloud'),
        DeclareLaunchArgument('approach_height', default_value='0.20'),
        Node(package='moveit_ros_move_group', executable='move_group',
             output='screen', parameters=[parameters]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(original, 'config', 'moveit.rviz')],
             parameters=[parameters]),
        Node(package='roboarm_motion', executable='approach_node', output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'cloud_topic': LaunchConfiguration('cloud_topic'),
                 'input_mode': LaunchConfiguration('input_mode'),
                 'approach_height': ParameterValue(LaunchConfiguration('approach_height'), value_type=float),
             }]),
    ])
