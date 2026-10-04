
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    
    #Launch Arguments (variables) for CLI
    #   SKIP!!

    # Just start executing the talker & listener NODE:
    talker_node = Node(
        package='roboArm_proj1',        # ros2 pkg name in which node .py file is present
        executable='my_Msgpublisher',   # talker node name given in the setup.py entry point.
        name='myTalker'     # DISCLAIMER: talker node name as per launch file otherwise it will assign random name to the node.
    )

    listner_node = Node(
        package="roboArm_proj1",       # ros2 pkg name in which your node code is present.
        executable="my_Msgsubscriber",  # listner node name given in the setup.py entry point.
        name="myListner"    # DISCLAIMER: listner node name as per launch file otherwise it will assign the random name to the node.
    )


    return LaunchDescription([
        talker_node,
        listner_node
    ])

