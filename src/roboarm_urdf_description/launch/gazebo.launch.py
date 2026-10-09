
"""
1. get robot urdf path
2. convert xacro file to urdfxml string
3. Give robot_state_publisher node:
    3.1 robot urdf for knowing the state of the robot.
    3.2 Gazebo simulation has its own clock than realtime system clock hence need to provide simulation clock to the Node, Gazebo clock is known by the topic "/clock"
4. Give gazebo the robot urdf directory through special environment variable
5. Launching gazebo launch file within this launch file from the gazebo package "ros_gz_sim" and providing the path of it and the necessary arguments.
6. Spawning Robot object
7. bridging the ROS and Gazebo messages by remapping the Clock sync
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # model_path = "/home/shoaib-ubuntu/roboArm_rosws/src/roboArm_urdf_description/urdf/roboArm_urdf.urdf.xacro"
    robot_description_dir = get_package_share_directory("roboarm_urdf_description")
    
    """
    LaunchArgument => makes the variable that you can even pass through CLI, here the variable name is "model"
    --> COULD LOOK LIKE THIS: ros2 launch my_pkg my_launch.py model:=/path/to/other_robot.urdf.xacro
    """
    model_arg = DeclareLaunchArgument(
        name="model",    # varaible/argument name used for overwritting CLI command           
        default_value=os.path.join(robot_description_dir, "urdf", "roboArm_urdf.urdf.xacro"),
        description="Absolute path to the URDF file"
    )
    model = LaunchConfiguration("model") # making above path into python variable

    
    """
    # LaunchConfiguration() it takes the CLI varaible created by DeclareLaunchArgument --> that is, it takes the path of xacro file and convert to urdf
    --> COULD LOOK LIKE THIS: ros2 xacro xacro /path/to/roboArm.urdf.xacro
    """
    robot_description = ParameterValue( 
        # command() - run shell commands at launch, xacro is the shell command that need xacro file to convert into urdf xml string this is given as OUTPUT
        Command(["xacro ", LaunchConfiguration("model")]), # LaunchConfiguration() it is used to access the CLI varaible created by DeclareLaunchArgument --> that is, it takes the path of xacro file.
        value_type=str
    )   #it converts xacro file to pure urdf file


    """
    1. robot_state_publisher is the inbuilt ros package, its a node to get the access of the state of the robot.
    2. it takes the urdf file "robot_description" and then send on /TF topic TF2 tool is provided by ros that handles transformation matrixes for exach frames for fwd and Inv kinematics
    3. The parameter "robot_description" globally accessible which is later used for spawning the robot
    """
    robot_state_publisher = Node(     
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,   # robot_description is exact paramter name robot_state_publisher requires for passing the robot urdf
                     "use_sim_time": True}]      # it syncs the NODE to run according to the Gazebo simulation clock instead of real computer clock....therefore it hits the topic "/clock"
    )


    """
    Gazebo looks for the special Environment Variable that has stored the path of resources(in this case robot urdf)
    """
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
                str(Path(robot_description_dir).parent.resolve())
            ]
    )

    # ros_distro= os.environ["ROS_DISTRO"]

    """
    1. IncludeLaunchDescription() --> launches another launch file
    
    2. PythonLaunchDescriptionSource() --> launches launch file of type python from the provided path
        => passing the directory as: ros_gz_sim<package>/launch<directory>/gz_sim.launch.py<python launch file name>
        NOTE: "ros_gz_sim/launch/gz_sim.launch.py" --> gz_sim.launch.py is the GAZEBO's launch file that handles gui, loading server, plugin..etc
    
    3. launch_arguments --> takes argument which is needed by Gazebo launch file(gz_sim.launch.py).\
        => -v 4 : means verbosity at level 4 (0=silent, 1=errors, 2=warning, 3=info, 4=debug everything)
        => -r: run simulation immidiately
        => empty.sdf: load world file named empty.sdf -- here the world is empty, empty sky, empty ground, nothing else.
        =>

        NOTE: there 2 types of argument:
                1. The Declarelaunchargument--> which we built for CLI overwriting(say: "model")
                2. The launch_arguments--> the argument which we pass to another file here that is "gz_args"
    """


    """
    Getting the path of world (this contains plugins for sensor and future world background)
    """
    world_arg = DeclareLaunchArgument(
            name="world",    # varaible/argument name used for overwritting CLI command           
            default_value=os.path.join(robot_description_dir, "worlds", "perception_world2.sdf"),
            description="Absolute path to the world plugin sdf file"
        )
    # Read the launch argument value
    world = LaunchConfiguration("world")
    
    gazebo = IncludeLaunchDescription(
        #specify type and directory of the file we want to launch
        PythonLaunchDescriptionSource([  # type of the launch file is python launch file
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),  #ros_gz_sim is pre-installed ros2 packages just like robot_state_publisher
            "/gz_sim.launch.py"]),
        
        launch_arguments={"gz_args": ["-v 4 -r ", world]}.items()
    )


    """ 
    1. It spawns the robot in Gazebo simulation env on topic "/robot_description" and names the robot using "-name"
    2. Spawn cann't read URDF file directly instead it reads parameter robot_description(as argument flag -topic) hence uses global parameter created by robot_state_publisher
    """
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "roboArm",
                   "-z", "0.79"]
    )

    """
    1. bridging the msgs of Gazebo <--> ROS
    2. For bridging Ros and Gazebo for message transfering we need CLOCK of them to be synced
        - ROS run real system clock(wall clock), while Gazebo simulation clock run fast, slow, pause
    3. So synching clock we remap ROS clock topic with Gazebo clock topic through:
        /clock --> topic name
        @ --> remapping
        rosgraph_msgs/msg/Clock --> ROS message type
        gz.msgs.Clock --> GAZEBO message type
    """
    ros_bridge_config_file_path=os.path.join(robot_description_dir, 'config', 'ros_gz_bridge_config.yaml')
    config_file_arg = DeclareLaunchArgument(
        name="ros_bridge_config_file",
        default_value=ros_bridge_config_file_path # this path consist of sensor communication
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": LaunchConfiguration("ros_bridge_config_file")
        }],
        output="screen",
    )


    """
    Activate controller
    """
    joint_state_broadcaster_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
        "joint_state_broadcaster",
        "--controller-manager",
        "/controller_manager"
        ],
        output="screen"
    )
    joint_trajectory_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
        "joint_trajectory_controller",
        "--controller-manager",
        "/controller_manager"
        ],
        output="screen"
    )

    gripper_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "gripper_controller",
        "--controller-manager", "/controller_manager",
    ],
    output="screen",
    )
            

    return LaunchDescription([
        model_arg,
        world_arg,
        config_file_arg,
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        joint_trajectory_controller_spawner,
        gz_ros2_bridge,
    ])  