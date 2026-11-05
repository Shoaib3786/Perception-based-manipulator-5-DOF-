#!/usr/bin/env bash

ROBOARM_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${ROBOARM_WORKSPACE}/install/setup.bash"

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [
      'joint_1',
      'joint_2',
      'joint_3',
      'joint_4',
      'joint_prism'
    ],
    points: [
      {
        positions: [-1.75, 0.0, 2.0, -2.15, 0.0],
        time_from_start: {sec: 2}
      }
    ]
  }
}" --feedback & ros2 action send_goal \
  /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: ['joint_5'],
      points: [
        {
          positions: [0.25],
          time_from_start: {sec: 2}
        }
      ]
    }
  }" --feedback
