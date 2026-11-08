"""Start MoveIt and the explicit grasp-and-lift demonstration after Gazebo."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    floats = {
        'velocity_scaling': '0.1', 'acceleration_scaling': '0.1',
        'grasp_offset_x': '0.0', 'grasp_offset_y': '0.0', 'grasp_offset_z': '0.0',
        'tcp_x': '-0.00923864', 'tcp_y': '0.71493633', 'tcp_z': '0.04545',
        'grasp_yaw': '0.0', 'grasp_roll': '-0.3', 'orientation_tolerance': '0.15',
        'pregrasp_height': '0.20', 'lift_height': '0.20',
        'gripper_open': '-0.8', 'gripper_closed': '-0.2406546887035444',
        'contact_velocity_scaling': '0.1', 'object_radius': '0.14',
        'grasp_insertion_depth': '0.03',
    }
    strings = {'cloud_topic': '/perception/red/points', 'input_mode': 'cloud',
               'target_topic': '/perception/target_xyz', 'rviz': 'false'}
    declarations = [DeclareLaunchArgument(name, default_value=value)
                    for name, value in (floats | strings).items()]
    declarations += [DeclareLaunchArgument('auto_tcp', default_value='true'),
                     DeclareLaunchArgument('auto_grasp_yaw', default_value='true')]
    own = Path(get_package_share_directory('roboarm_motion'))
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(own / 'launch/gazebo_moveit.launch.py')),
        launch_arguments={'start_approach': 'false', 'position_only_ik': 'false',
                          'rviz': LaunchConfiguration('rviz')}.items())
    parameters = {'use_sim_time': True,
                  'auto_tcp': ParameterValue(LaunchConfiguration('auto_tcp'), value_type=bool),
                  'auto_grasp_yaw': ParameterValue(LaunchConfiguration('auto_grasp_yaw'), value_type=bool)}
    parameters.update({name: ParameterValue(LaunchConfiguration(name), value_type=float)
                       for name in floats})
    parameters.update({name: LaunchConfiguration(name) for name in strings if name != 'rviz'})
    return LaunchDescription(declarations + [moveit, Node(
        package='roboarm_motion', executable='pick_node', output='screen', parameters=[parameters])])
