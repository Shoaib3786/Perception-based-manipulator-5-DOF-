# Camera / XYZ to MoveIt in Gazebo

See the [workspace guide](../../README.md) for the repository layout.

This package reuses the Setup Assistant's
SRDF and planning configuration. It overrides the controller mapping, uses the
Gazebo robot description directly, and enables KDL position-only IK. It does not
start Gazebo, a second controller manager, mock hardware, or another robot-state
publisher. The integration uses the existing robot packages.

## Scope and prerequisites

The approach targets the nominal fingertip midpoint by default, not the origin
of `claw_link`. The separate pick launch adds an experimental grasp sequence.
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
0.02 m/s^2 for joint_prism), with request scaling controlled by
`velocity_scaling` and `acceleration_scaling` (default 0.1); tune for your model.

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

Do not launch `demo.launch.py` simultaneously. The launch above starts move_group
and approach_node. RViz is optional and off by default; use `rviz:=true` to start it.

1. In RViz use Fixed Frame `base_link`, group `arm`, and the MotionPlanning display.
2. Add a Marker display on `/approach_node/target_marker` (transient-local durability).
3. Keep the arm still for the first test and select a reachable target.
4. Call plan only:

```bash
ros2 service call /approach_node/plan std_srvs/srv/Trigger '{}'
```

The yellow marker is the requested fingertip-midpoint position: visible-surface mean,
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

Publish a final target for the nominal fingertip midpoint. Example numbers are illustrative;
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

## CLI execution speed

Both planning and execution requests accept velocity and acceleration scaling
factors in `(0, 1]`. The original default remains 0.1. For faster approach motion:

```bash
ros2 launch roboarm_motion gazebo_moveit.launch.py \
  cloud_topic:=/perception/green/points \
  velocity_scaling:=0.5 acceleration_scaling:=0.5
```

Use the existing `/approach_node/plan` and `/approach_node/move` services. These
are fractions of the configured joint limits, not metres/second or a guaranteed
wall-time multiplier. The prismatic joint remains limited to 0.05 m/s before
scaling; the existing demonstration acceleration limits are unchanged. To change
speed between requests without relaunching:

```bash
ros2 param set /approach_node velocity_scaling 0.5
ros2 param set /approach_node acceleration_scaling 0.5
ros2 service call /approach_node/move std_srvs/srv/Trigger '{}'
```

Speed changes during an active request are rejected. PickNode supports the same
parameters through `/pick_node` instead.

## Mean point to grasp and lift (experimental)

The implemented sequence is **open → pregrasp → descend → close → lift**. It
snapshots one fresh input target, transforms it at capture time, then adds
`grasp_offset_x/y/z` in the base frame to estimate a grasp center. The cloud mean
is the mean of visible surfaces, not automatically the geometric center. For a
0.28 m cube seen from directly above, `grasp_offset_z:=-0.14` is a starting example,
not a universal correction for an oblique view or an apple.

Unlike the original approach, the pick goals constrain orientation and the
position of a finger-contact point offset from `claw_link`. With `auto_tcp:=true` (default), the node reads the scaled collision meshes and
URDF origins, extracts the distal 3 cm pad region, transforms each pad at the
configured `gripper_closed` angle, and uses their midpoint as the claw-frame TCP.
At the nominal close angle `-0.2406546887` rad this is approximately
`(-0.009239, 0.714936, 0.045450)` m, with an inner pad gap of 0.28 m.
The old 0.44 m offset was incorrect for the fingertips. The camera mount is not
used as the grasp point. Set `auto_tcp:=false` to supply manually calibrated
`tcp_x/y/z` instead.

`auto_grasp_yaw:=true` aligns the gripper with the target direction using the
current arm's yaw/parallel-pitch joint structure and lateral offsets. Use
`auto_grasp_yaw:=false grasp_yaw:=...` for manual yaw. `grasp_roll` controls the
finger tilt; default -0.3 rad is a shallow downward approach. The previous fixed
vertical (-pi/2) grasp is generally unreachable near the table because the wrist
and fingers are long relative to the upper arm. The new launch disables
position-only IK; orientation-infeasible requests fail rather than relaxing the
constraint. A five-axis arm cannot realize arbitrary orientations.

Stop any previous MoveIt integration launch before using the pick launch. Keep
Gazebo, segmentation and depth running. Do not command keyboard control or another
motion client concurrently. Example configuration for a top-view cube:

```bash
ros2 launch roboarm_motion pick.launch.py \
  cloud_topic:=/perception/green/points \
  velocity_scaling:=0.5 acceleration_scaling:=0.5 \
  contact_velocity_scaling:=0.1 \
  grasp_offset_z:=-0.14 pregrasp_height:=0.20 lift_height:=0.20 \
  gripper_open:=-0.8 gripper_closed:=-0.2406546887
```

The selected color cloud must represent one object. The current HSV pipeline
may merge a same-colored apple and block. It does not provide instance selection.
`input_mode:=point` with `target_topic:=/perception/target_xyz` can instead supply
an explicit estimated grasp center in a valid stamped frame.

First request a plan-only sequence:

```bash
ros2 service call /pick_node/plan_pick std_srvs/srv/Trigger '{}'
ros2 topic echo /pick_node/status
```

Inspect `/display_planned_path` in RViz and a PoseArray display on
`/pick_node/grasp_poses` (Transient Local durability). The preview propagates each
planned endpoint into the next stage, including the gripper mimic joint. It does
not open the physical gripper. Fix reachability, TCP alignment, object-center
estimation and collision-scene geometry before executing. The services return
submission status; final status is published on `/pick_node/status`.

```bash
ros2 service call /pick_node/pick std_srvs/srv/Trigger '{}'
```

Execution snapshots the current target and replans each stage from current joint
states; it does not execute a cached preview. A failed/rejected/canceled stage
stops the sequence, with no automatic open/release recovery. `/pick_node/cancel`
requests cancellation; keep the node running until the action terminates.
Approach/descent/lift are OMPL-planned motions to waypoints, not guaranteed
straight Cartesian segments. `pregrasp_height` and `lift_height` are along base Z.
Close, descent and lift velocity are capped by `contact_velocity_scaling`.

The gripper controller drives only `joint_5`; `joint_6` mimics it. The default close angle is derived from a nominal 0.28 m inner tip gap; it does
not automatically adapt to another object width. The open angle remains -0.8 rad.
Tune against actual contact, since the distal-pad estimate and collision meshes
are not force sensors. A successful close
trajectory only means the requested angle was reached: this implementation has
no force/contact confirmation or slip detection. Gazebo friction/contact must
actually retain the object for a physical pickup. No simulator weld is applied.

For lift collision planning, the request includes an attached sphere of radius
`object_radius` (default 0.14 m) at the estimated contact point, with the fingers
allowed to touch it. This is an approximate planning representation only, not a
Gazebo attachment or verification of a grasp. Populate the table and surrounding
obstacles in MoveIt's scene beforehand; Gazebo does not automatically do that.
The existing Gazebo spawn/world-to-base offset discrepancy must also be resolved
for correct table coordinates. No successful grasp or lift is claimed from the
non-moving validation performed here.

## Which node should run?

Use one of these alternatives, not both sequentially:

- `gazebo_moveit.launch.py`: starts MoveIt plus `approach_node`. This moves the
  nominal fingertip midpoint toward a target, but does not operate the gripper.
  `use_fingertip_target:=false` restores the old claw-origin target. The midpoint
  is evaluated at `reference_gripper_angle`, default -0.2406546887 rad; this
  standalone node does not command the fingers to that angle.
- `pick.launch.py`: starts one MoveIt server plus `pick_node` and disables the
  separate `approach_node`. It already includes opening, arm approach, descent,
  closing and lift. Each step waits for the previous action to succeed.

The common data flow is camera → segmentation → depth/point cloud → capture-time
TF conversion into base_link → motion request → MoveIt → existing arm/gripper
controllers. `PickNode` reuses `ApproachNode` code by Python inheritance; it does
not need a second approach process or a service call to `/approach_node/move`.

The TCP relation is `target = claw_position + claw_rotation * tcp_offset`.
MoveIt's `PositionConstraint.target_point_offset` handles this rotated offset;
it is not a constant subtraction along world Z. During picking, the target is
the desired midpoint at the configured final closing angle. Actual fingertips
move during opening/closing. The new `right_fingertip` and `left_fingertip` fixed
frames on the finger links let you inspect that movement using RViz TF or:

```bash
ros2 run tf2_ros tf2_echo claw_link right_fingertip
ros2 run tf2_ros tf2_echo claw_link left_fingertip
```

Restart Gazebo and MoveIt after rebuilding to load the added frames. The mesh
measurement corrects fingertip targeting; successful grasp retention still needs
contact/friction validation and a correct estimate of the object's center from
the visible point cloud. There is no automatic force-confirmed grasp.

### Joint-goal planning and execution preflight

The pick implementation now solves joint goals directly for this robot's current
URDF chain (joint_1 yaw; joint_2/3/4 parallel pitch axes; joint_prism slider).
It enumerates wrist extensions and analytic elbow configurations, rejects joint
limit violations, and independently verifies each candidate by composing the
URDF forward transforms. The fingertip midpoint and requested orientation are
therefore included in each joint target. MoveIt still checks collision validity
and plans the arm trajectory; collision checking is not bypassed. This replaces
pose-goal sampling, which failed on the logged green target despite reachable
joint configurations. Changes to axes or joint-origin layout outside the
supported chain are rejected rather than silently using the old equations.

`/pick_node/pick` first plans all five stages without moving. Status is reported
as `PREFLIGHT open`, `PREFLIGHT pregrasp`, etc. Execution starts only after
`PREFLIGHT COMPLETE`; it then replans each stage from actual monitored joint
states. A preflight failure leaves the physical gripper untouched. The separate
`/pick_node/plan_pick` service remains preview-only.

On 2026-11-07, an isolated MoveIt instance in a separate ROS domain validated the
logged target `(2.532, -0.965, 0.441)` m. Its pregrasp had 21 collision-valid joint
goals, and a plan-only request between distinct valid states succeeded with 226
trajectory waypoints. All five sequence stages also planned successfully from
synthetic start states. These checks executed no motion and do not validate a
physical grasp or the path from a different live starting state.

### Place the object deeper between the fingers

`grasp_insertion_depth` reduces the contact offset along claw-local Y after the
mesh-derived (or manual) TCP is calculated. Default `0.03` m places the object
3 cm farther inside the fingers than the distal-pad midpoint. For the default
closing angle, TCP Y changes from about 0.714936 m to 0.684936 m. The same inset
point is used throughout arm planning, descent and the lift payload estimate.
It does not change the cloud target, gripper angle, camera, or standalone approach.

```bash
ros2 launch roboarm_motion pick.launch.py \
  cloud_topic:=/perception/green/points \
  velocity_scaling:=0.5 acceleration_scaling:=0.5 \
  grasp_insertion_depth:=0.03
```

Set `grasp_insertion_depth:=0.0` to restore the previous fingertip alignment.
Increase cautiously to `0.04` or `0.05` m if the object still sits too near the tips;
accepted values are 0–0.10 m. Restart the pick launch after changing this startup
parameter, preview again, and inspect the gripper/object clearance. Insertion
alone does not guarantee grip force or retention, particularly because the gap
farther inside the curved fingers can differ from the nominal fingertip gap.
