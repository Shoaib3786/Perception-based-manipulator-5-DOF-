#!/usr/bin/env bash

ROBOARM_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${ROBOARM_WORKSPACE}/install/setup.bash"

ros2 run rqt_image_view rqt_image_view &
ros2 run rviz2 rviz2 -d "${ROBOARM_WORKSPACE}/install/roboarm_urdf_description/share/roboarm_urdf_description/rviz/rviz_config_display.rviz" --ros-args -p use_sim_time:=true   
