# Camera / XYZ to MoveIt in Gazebo

Built against repository snapshot c15d6e3. This package reuses the Setup Assistant's
SRDF and planning configuration. It overrides the controller mapping, uses the
Gazebo robot description directly, and enables KDL position-only IK. It does not
start Gazebo, a second controller manager, mock hardware, or another robot-state
publisher. Original packages are not modified.

## Scope and prerequisites

This is an approach demonstration for the origin of `claw_link`, not a grasp.
The orientation is unconstrained. Clouds must use valid optical-frame headers,
aligned RGB/depth, and correct calibration. Select a color topic containing ONE
object: the existing publisher merges all regions of the same color.

Gazebo must already publish `/joint_states`, `/tf`, `/tf_static`, and `/clock`.
Both controllers must be active, with action servers:

- `/joint_trajectory_controller/follow_joint_trajectory`
- `/gripper_controller/follow_joint_trajectory`

The package disables the generated Kinect Octomap configuration because its topic
does not exist. It does NOT insert Gazebo's environment into MoveIt's scene.
Add table/box collision geometry through RViz's Planning Scene tools or
ApplyPlanningScene before executing near them. Resolve the existing difference
between Gazebo spawn Z=0.79 and the URDF's world-to-base transform before using SDF
world positions. Do not assume that RViz point-cloud displays are collision objects.

Acceleration limits in the launch are demonstration values (0.5 rad/s^2 and
0.02 m/s^2 for joint_prism), with a further 0.1 request scaling; tune for your model.

## Install

Copy this `roboarm_motion` directory into your workspace's `src` directory.

```bash
cd ~/ros2_projects/roboArm_rosws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/roboarm_motion --ignore-src -r -y --skip-keys "roboarm_movit_config roboarm_urdf_description"
colcon build --symlink-install --packages-up-to roboarm_motion
source install/setup.bash
```

The skip keys are your existing workspace packages, built by colcon below.
In the old perception manifest, `cv2` should be the rosdep key `python3-opencv`,
not `cv2`, if you run rosdep over the entire workspace.

## Run with a point cloud

Use separate sourced terminals:

```bash
ros2 launch roboarm_urdf_description gazebo.launch.py
ros2 run perception_roboarm segmentation_node
ros2 run perception_roboarm depth_node
ros2 launch roboarm_motion gazebo_moveit.launch.py cloud_topic:=/perception/red/points approach_height:=0.20
```

Do not launch `demo.launch.py` simultaneously. The launch above starts move_group,
RViz, and approach_node. To keep an existing RViz use `rviz:=false`.

1. In RViz use Fixed Frame `base_link`, group `arm`, and the MotionPlanning display.
2. Add a Marker display on `/approach_node/target_marker` (transient-local durability).
3. Keep the arm still for the first test and select a reachable target.
4. Call plan only:

```bash
ros2 service call /approach_node/plan std_srvs/srv/Trigger '{}'
```

The yellow marker is the requested claw-link position: visible-surface mean,
transformed into base_link, plus 0.20 m in base Z. It is not the box center or a
finger-contact location. The offset is a configurable example, not guaranteed
clearance. Inspect the plan on `/display_planned_path` using MotionPlanning's
Planned Path or a Trajectory display. A small sphere defines 1 cm position tolerance.

5. After inspecting the scene and path, explicitly request motion:

```bash
ros2 service call /approach_node/move std_srvs/srv/Trigger '{}'
```

This snapshots the latest target and REPLANS before execution. It does not execute
the exact saved preview. Both calls return submission status, not motion completion.
The node logs the eventual MoveIt success/error code. No motion starts on camera
callbacks, and overlapping requests are rejected.

Cancellation:

```bash
ros2 service call /approach_node/cancel std_srvs/srv/Trigger '{}'
```

Keep the client alive until cancellation completes. Closing the client is not a
reliable way to cancel an already accepted server goal.

## Send XYZ directly

Restart only this integration launch in point mode (keep Gazebo running):

```bash
ros2 launch roboarm_motion gazebo_moveit.launch.py input_mode:=point
```

Publish a final target for the claw-link origin. Example numbers are illustrative;
choose a reachable, collision-free point in YOUR scene. This point mode does NOT
add approach_height.

```bash
ros2 topic pub -r 2 /perception/target_xyz geometry_msgs/msg/PointStamped "{header: {stamp: now, frame_id: base_link}, point: {x: 0.5, y: 0.5, z: 1.0}}" --ros-args -p use_sim_time:=true
```

Call the same plan/move services. Non-base points are transformed at their stamp.
Messages older than 2 seconds or significantly in the future are rejected; keep
publishing while requesting a plan. Both modes use the simulation clock.

## How the implementation works

- `on_cloud`: converts PointCloud2 to finite XYZ samples; calculates a surface mean.
- `submit`: checks age, copies one target, looks up the TF at capture time, applies
  a cloud-only base-Z offset, publishes a marker, and sends an asynchronous goal.
- `make_goal`: sets current-state planning, group arm, OMPL, a position constraint
  on claw_link, and plan_only depending on the requested service. No quaternion
  constraint is imposed, consistent with the five-axis position-only IK setup.
- `on_goal` / `on_result`: report acceptance and completion, publish trajectory,
  and allow the next request.
- The MoveIt server performs IK/path planning/time parameterization and dispatches
  execution to the existing Gazebo controller. The client never sends a cloud
  directly to the joint controller.

## Troubleshooting

- No target: verify the exact cloud topic and publishers; empty clouds clear targets.
- TF unavailable: check `tf2_echo base_link corrected_optical_frame`; retry after
  TF arrives. The callback does not block waiting for TF or use the latest pose
  in place of capture-time data.
- /move_action missing: inspect move_group logs; stop duplicate demo/move_group nodes.
- No current state: check `/joint_states`, advancing `/clock`, and use_sim_time.
- Planning failure: check reachability, start-state collisions and target link.
- Execution failure: check controller action names and joint lists.
- Marker looks wrong: verify RGB/depth alignment, calibration and camera optical TF.

## Validation

Python syntax and XML/configuration structure were checked outside ROS. ROS 2,
MoveIt, Gazebo and rclpy are not installed in the authoring environment, so colcon
build, runtime message construction, TF matching, planning and execution MUST be
validated in your Jazzy/Gazebo workspace. No successful robot execution is claimed.
