#!/usr/bin/env bash

ROBOARM_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${ROBOARM_WORKSPACE}/install/setup.bash"

ros2 run rqt_image_view rqt_image_view --ros-args -p use_sim_time:=true &
ros2 launch roboarm_motion view_camera.launch.py
