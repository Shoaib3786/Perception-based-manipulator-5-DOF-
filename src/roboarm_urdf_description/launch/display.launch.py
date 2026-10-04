
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

   # model_path = "/home/shoaib-ubuntu/roboArm_rosws/src/roboArm_urdf_description/urdf/roboArm_urdf.urdf.xacro"
    roboarm_urdf_description = get_package_share_directory("roboarm_urdf_description")
    model_arg = DeclareLaunchArgument(
        name="model",    # this the name of the varaible/argument for being used as varaible in CLI           
        default_value=os.path.join(roboarm_urdf_description, "urdf", "roboArm_urdf.urdf.xacro"),
        # default_value=model_path,
        description="Absolute path to the URDF file"
        )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),      #LaunchConfiguration() it takes the CLI varaible created by DeclareLaunchArgument
        value_type=str
        )   #it converts xacro file to pure urdf file
    
    robot_state_publisher = Node(  #robot_state_publisher is the inbuilt package for being node by ros2 to get the access of the state of the robot and publishes its content to TF topic which handles the transformation of frames of robot for forward/inverse kinematics
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]      # robot_description is exact paramter name robot_state_publisher requires for passing the robot urdf
    )

    joint_state_publisher_gui = Node( # gui -- for providing slider that gives values to the joints that is used for testing the joint movement in rviz
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",        # ros2 pkg name in which node .py file is present
        executable="rviz2",     # talker node name given in the setup.py entry point.
        name="rviz2",           # DISCLAIMER: talker node name as per launch file otherwise it will assign random name to the node.
        output="screen",        # output screen means displaying the logs on the terminal instead of logging them in log file.
        arguments=["-d", os.path.join(roboarm_urdf_description, "rviz", "rviz_config_display.rviz")] # '-d' is directory to pass
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])

