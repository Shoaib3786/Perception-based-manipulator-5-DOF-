#!/usr/bin/bash

ROBOARM_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"


# Source python venv
source ~/py_venv/ros2_roboArm_pyvenv/bin/activate



# For making ROS accessible from any termianl(without sourcing below command everytime)
source /opt/ros/jazzy/setup.bash

# ROS Autocompletion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash



# Source workspace roboArm
source "${ROBOARM_WORKSPACE}/install/setup.bash"
