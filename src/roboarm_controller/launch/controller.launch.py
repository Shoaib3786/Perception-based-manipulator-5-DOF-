
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    """
    # LaunchConfiguration() it takes the CLI varaible created by DeclareLaunchArgument --> that is, it takes the path of xacro file.
    --> COULD LOOK LIKE THIS: xacro /path/to/roboArm.urdf.xacro
    """
    robot_description = ParameterValue(
        # command() - run shell commands at launch, xacro is the shell command that need xacro file to convert into urdf xml string this is given as OUTPUT
        Command(["xacro ", os.path.join(get_package_share_directory("roboarm_urdf_description"),"urdf","roboArm_urdf.urdf.xacro")]), # it takes the path of xacro file.
        value_type=str,
        )   #it converts xacro file to pure urdf file


    """
    1. robot_state_publisher is the inbuilt ros package its a node to get the access of the state of the robot.
    2. it takes the urdf file "robot_description" and then send on /TF topic TF2 tool is provided by ros that handles transformation matrixes for exach frames for fwd and Inv kinematics
    3. The parameter "robot_description" globally accessible which is later used for spawning the robot
    """
    robot_state_publisher_node = Node(     
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],    # robot_description is exact paramter name robot_state_publisher requires for passing the robot urdf
        
    )


    # controller_manager=Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {"robot_description": robot_description},
    #         os.path.join(
    #             get_package_share_directory("roboarm_controller"),
    #             "config",
    #             "roboarm_controllers.yaml"
    #             ), 
    #         ],
    # )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription(
        [

            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
