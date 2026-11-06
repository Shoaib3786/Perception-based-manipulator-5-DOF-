# Camera / XYZ to MoveIt in Gazebo

See the [workspace guide](../../README.md) for the repository layout.

This package reuses the Setup Assistant's
SRDF and planning configuration. It overrides the controller mapping, uses the
Gazebo robot description directly, and enables KDL position-only IK. It does not
start Gazebo, a second controller manager, mock hardware, or another robot-state
publisher. The integration uses the existing robot packages.

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

Run these commands from the existing workspace; the package is already in `src/`.

```bash
cd ~/ros2_projects/roboArm_rosws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/roboarm_motion --ignore-src -r -y --skip-keys "roboarm_movit_config roboarm_urdf_description"
colcon build --symlink-install --packages-up-to roboarm_motion
source install/setup.bash
```

The skip keys are your existing workspace packages, built by colcon below.
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

## Validation history

The following describes the original integration authoring environment, not a
current workspace test result.


Python syntax and XML/configuration structure were checked outside ROS. ROS 2,
MoveIt, Gazebo and rclpy are not installed in the authoring environment, so colcon
build, runtime message construction, TF matching, planning and execution MUST be
validated in your Jazzy/Gazebo workspace. No successful robot execution is claimed.

## Camera and cloud viewer

With Gazebo already running, use `bash scripts/view-camera.bash` from the
workspace root, or `ros2 launch roboarm_motion view_camera.launch.py` for RViz
alone. This supplies URDF and SRDF directly to RViz so its Trajectory display can
load the model. It starts no controller, state publisher, or MoveIt server.
The saved view subscribes to red, green and pink clouds with Best Effort QoS,
matching `depth_node`. If adding a new cloud display manually, set its Topic
Reliability Policy to Best Effort.

Run both perception nodes with `--ros-args -p use_sim_time:=true` when using Gazebo.
Old-message TF warnings can appear briefly at startup or after a simulation reset.
If they persist, verify `/clock` advances and restart the viewer and perception
nodes after a reset; check that image/cloud stamps and TF use the same simulation
clock. Do not replace capture timestamps with the current time to hide TF errors.

## Keyboard joint controller

Start Gazebo and its controllers, then in a separate interactive terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run roboarm_motion keyboard_teleop --ros-args -p use_sim_time:=true
```

| Joint | Increase | Decrease |
| --- | --- | --- |
| joint_1 | Q | A |
| joint_2 | W | S |
| joint_3 | E | D |
| joint_4 | R | F |
| joint_prism | T | G |
| joint_5 (gripper) | Y | H |

The other finger (`joint_6`) follows its mimic joint. Keys command numeric joint
directions, which depend on the URDF axes. Each key event jogs from the latest
measured joint position. Holding a key relies on your terminal's key repeat.
Default increments are 0.04 rad and 0.005 m; `+`/`-` change step size.
Space requests a hold at measured positions; X or Ctrl-C requests a hold and exits.
Releasing a key lets the last short trajectory finish. This is not an emergency
stop or a dead-man controller. Stale states prevent sending a hold as well as a jog.

The node reads limits from the robot URDF and requires fresh `/joint_states` and
a controller topic subscriber before publishing. It sends all joints of the
selected controller, keeping the others at their measured positions. Stop other
motion clients before jogging: topic trajectories can preempt MoveIt/action goals.
Direct joint control performs no collision checking. Use in simulation first.

Optional parameters: `angular_step`, `linear_step`, `duration` (minimum trajectory
time in seconds), and `state_timeout` (wall-time age of joint-state reception).
For example, append `-p angular_step:=0.02 -p linear_step:=0.002` for smaller steps.
