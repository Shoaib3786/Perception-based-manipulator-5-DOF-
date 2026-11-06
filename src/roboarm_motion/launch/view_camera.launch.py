"""View the running simulation with robot semantics and sensor-compatible QoS."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(get_package_share_directory('roboarm_urdf_description'))
    moveit = Path(get_package_share_directory('roboarm_movit_config'))
    robot = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        str(description / 'urdf' / 'roboArm_urdf.urdf.xacro'),
    ]), value_type=str)
    semantic = (moveit / 'config' / 'Robot_Arm.srdf').read_text()
    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(description / 'rviz' / 'rviz_config_display.rviz')],
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot,
                'robot_description_semantic': ParameterValue(semantic, value_type=str),
            }],
        ),
    ])
