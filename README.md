# Perception-based 5-DOF Manipulator

**An ongoing robotics project connecting RGB-D perception to motion planning in ROS 2 and Gazebo.**

I am building this project to understand the complete path from an object appearing in a camera image to a robot moving towards it. I started with a custom arm model and colour-based segmentation, then added depth processing, coordinate transformations and a MoveIt 2 interface. My next focus is learned instance segmentation and, eventually, reliable grasping.

The current arm has **four revolute joints and one prismatic joint**, with a separately controlled gripper. This is a simulation and learning project, not a completed autonomous pick-and-place system.

**Stack:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · Python · OpenCV · NumPy · MoveIt 2 · ros2_control

## Where to start

- [Current implementation](#current-status)
- [Build and launch commands](#build-and-run)
- [My development workflow](docs/WORKFLOW.md)
- [Detailed MoveIt interface](src/roboarm_motion/README.md)
- [Repository maintenance and verification](docs/REPOSITORY_NOTES.md)

## Current status

| Component | What is available |
| --- | --- |
| Robot and simulation | URDF/Xacro model, meshes, RGB-D sensor, table/block worlds, Gazebo and RViz launch files. |
| Joint control | Gazebo ros2_control integration, joint-state broadcaster, arm trajectory controller and separate gripper trajectory controller. |
| 2D perception | HSV segmentation of red, pink and green regions; masks, bounding boxes, pixel counts and 2D centroids published through custom ROS messages. |
| 3D perception | Exact timestamp matching of segmentation and depth, camera-intrinsic back-projection, invalid-depth filtering and colour-specific PointCloud2 outputs. |
| Motion interface | Capture-time TF transformation and explicit plan/move/cancel services using MoveIt's MoveGroup action. The goal constrains the position of `claw_link`; orientation is free. |
| Apple segmentation | Initial YOLO segmentation training on YCB apple images is documented in my development notes. Generalisation is still poor. Training scripts, weights, dataset conversion and a YOLO ROS node are not included here yet. |
| Grasping and evaluation | Still to implement: object orientation/6D pose, grasp selection, a complete pick-and-place sequence, environment collision-scene integration and quantitative evaluation. |

The supplied world contains coloured blocks; it does not yet include a packaged YCB apple asset. The launch sequence below runs the **HSV baseline**, not a YOLO apple pipeline.



## Build and run

### 1. Prepare the workspace

These commands assume ROS 2 Jazzy is already installed on Ubuntu 24.04 and `rosdep` is initialised. Use a ROS-compatible system Python environment; an unrelated virtual environment may hide ROS packages.

For a **new checkout**:

```bash
mkdir -p ~/ros2_projects
cd ~/ros2_projects
git clone https://github.com/Shoaib3786/Perception-based-manipulator-5-DOF-.git roboArm_rosws
cd roboArm_rosws
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-moveit
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

If this workspace already exists, use it instead of cloning over it. Review and commit local changes before updating from GitHub. The build requires the Python `setup.py` and `setup.cfg` files included in this revision.

**In every new terminal**, run:

```bash
cd ~/ros2_projects/roboArm_rosws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```
or
```bash
source script/setup-venv.bash
```

### 2. Terminal 1 — simulation and controllers
custom image size argument provided so that different kind of detection model who ask different input image size could be integrated with the captured images easily. In this it uses  camera5(Claw link mounted camera) only.
```bash
ros2 launch roboarm_urdf_description gazebo.launch.py image_width:=640 image_height:=480
```

This starts Gazebo with `perception_world2.sdf`, spawns the arm, starts `robot_state_publisher`, bridges the camera and clock topics, and spawns the three controllers. There is no need to launch another controller manager.

To select the other supplied world:

```bash
ros2 launch roboarm_urdf_description gazebo.launch.py \
  world:="$(ros2 pkg prefix --share roboarm_urdf_description)/worlds/perception_world.sdf"
```

Use only one of these simulation commands at a time.

### 3. Terminal 2 — 2D segmentation

```bash
ros2 run perception_roboarm segmentation_node --ros-args -p use_sim_time:=true
```

The OpenCV windows show the original image, accepted mask and labelled regions. This node requires a graphical desktop. Press `q` in an image window or Ctrl+C to stop it.

### 4. Terminal 3 — depth fusion

```bash
ros2 run perception_roboarm depth_node --ros-args -p use_sim_time:=true
```

The node waits for calibration and exactly matching depth/segmentation timestamps. It publishes `/perception/red/points`, `/perception/pink/points` and `/perception/green/points`, including empty clouds when a colour disappears.


### 5. Terminal 4 — Grasping and Reaching to Target location combined (motion_pick.launch.py)
```bash
ros2 launch roboarm_motion pick.launch.py   cloud_topic:=/perception/red/points   velocity_scaling:=0.5 acceleration_scaling:=0.5   grasp_insertion_depth:=0.03
```
Allows the velocity and acceleration control along with the controlling the offset of the object from the gripper for better grasping. The launch file handles motion planning & Execution for whole robotic arm along with gripper pick up operation.

Set `rviz:=false` to use an existing RViz window. Starting this launch **does not move the arm automatically**.

`velocity_scaling:=0.5` and `acceleration_scaling:=0.5` **to increases the speed of execution**, by defaults scaling factor is 0.1.
To change the speed of execution in between the robot being executed, the command is:
```bash
ros2 param set /pick_node velocity_scaling 0.5
ros2 param set /pick_node acceleration_scaling 0.5
```

For this baseline, select a colour topic containing **one object**. The depth node combines all regions of the same colour; their combined mean would not represent a particular object if several are present.

In RViz, do use Fixed Frame `base_link`, planning group `arm`, and inspect the planned path. Add a Marker display for `/approach_node/target_marker` with transient-local durability to see the target.


### 6. Terminal 5 — request motion
```bash
ros2 service call /pick_node/plan_pick std_srvs/srv/Trigger '{}'
```
Planning the trajectory, for Arm + Gripper.

```bash
ros2 service call /pick_node/pick std_srvs/srv/Trigger '{}'
```
Execute motion request after planning.

```bash
ros2 service call /pick_node/cancel std_srvs/srv/Trigger '{}'
```
cancel the execution request.

Keep the nodes running until the action result arrives. Closing its terminal is not a reliable cancellation mechanism.


**Known limitation:**
- The Gazebo table, blocks, and apples are not automatically added to MoveIt’s planning scene. MoveIt can therefore plan through obstacles visible in Gazebo.
- SRDF disables several robot-link collision pairs using reason="Never". Those exclusions needs working upon it.
- Descent and lift use planned joint motions; they are not guaranteed straight-line movements.
- The carried object is represented by an approximate sphere. Its actual attachment, contact, and slipping are to be worked upon.
- The gripper’s closing motion has no force/contact feedback yet.


### Optional tools

**1. To Collect images of the objects on the table for Dataset generation**, in a separate session close all gazebo simulations and run the below command. It also has custom image size argument for user defined image size capturing for testing with different pretrained models.
```bash
ros2 launch roboarm_urdf_description gazebo.launch.py \
  enable_cameras:=true record_dataset:=true \
  image_width:=1280 image_height:=720 \
  dataset_dir:="$PWD/dataset"
```

**2. Control robot joints using keyboard keys** in a separate session with Gazebo running:
```bash
ros2 run roboarm_motion keyboard_teleop --ros-args -p use_sim_time:=true
```

**3. Inspect only the robot model** in a separate session, with Gazebo stopped:

```bash
ros2 launch roboarm_urdf_description display.launch.py
```

This uses the joint-state GUI for visual inspection, not Gazebo control. Do not run it alongside the simulation because it introduces another robot-state/joint-state source.

**4. Open RViz without a second robot_state_publisher node starts:**

```bash
rviz2 -d "$(ros2 pkg prefix --share roboarm_urdf_description)/rviz/rviz_config_display.rviz" \
  --ros-args -p use_sim_time:=true
```

**5. Tune HSV thresholds** instead of running segmentation:

```bash
ros2 run perception_roboarm hsvTracking_node --ros-args -p use_sim_time:=true
```

**6. Inspect the ROS connections:**

```bash
ros2 topic list
ros2 topic hz /camera/color/image_raw
ros2 topic info /perception/segmentation_2d --verbose
ros2 run tf2_ros tf2_echo base_link corrected_optical_frame
```

For direct `PointStamped` XYZ targets, see the [motion package guide](src/roboarm_motion/README.md#send-xyz-directly).

## Package map

| Package | Responsibility |
| --- | --- |
| `roboarm_urdf_description` | Robot, sensor, world, bridge configuration and simulation launch. |
| `perception_interfaces` | `Segment2D` and `Segmentation2D` custom message definitions. |
| `perception_roboarm` | HSV tuning, segmentation and depth back-projection. |
| `roboarm_movit_config` | Setup Assistant's semantic model and planning configuration; the spelling is intentional here because it matches the package. |
| `roboarm_motion` | Gazebo-specific MoveIt launch and service-triggered approach client. |
| `roboarm_controller` | Earlier controller experiments; not required by the main launch sequence. |
| `roboarm_proj1` | Earlier publisher/subscriber learning exercises. |

## Topics used by this implementation

### 1. Claw-mounted RGB-D camera — camera5
| ROS topic | Message type | Gazebo bridge source | Purpose / consumers |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `/rgbd_camera/image` | Claw color images. Used by segmentation, HSV tuning, image viewers and dataset recording as `camera5_*.jpg`. |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `/rgbd_camera/depth_image` | Depth image consumed by `depth_node` with timestamp-matched segmentation results. |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `/rgbd_camera/camera_info` | Camera intrinsics (`fx`, `fy`, `cx`, `cy`) used to back-project pixels and depth into XYZ coordinates. |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | `/rgbd_camera/points` | Gazebo's full RGB-D point cloud. This is distinct from the color-filtered clouds generated by `depth_node`; the current fusion node uses depth images and camera calibration rather than subscribing to this full cloud. |

`camera5` is the dataset filename prefix for `/camera/color/image_raw`.
The claw camera remains available during normal simulation when dataset recording
and the four corner cameras are disabled.

### 2. Optional table-corner cameras
| ROS topic | Message type | Gazebo bridge source | Purpose / consumers |
| --- | --- | --- | --- |
| `/camera1/image_raw` | `sensor_msgs/msg/Image` | `/table_camera1/image` | Corner camera 1 images; recorder saves `camera1_*.jpg`. |
| `/camera2/image_raw` | `sensor_msgs/msg/Image` | `/table_camera2/image` | Corner camera 2 images; recorder saves `camera2_*.jpg`. |
| `/camera3/image_raw` | `sensor_msgs/msg/Image` | `/table_camera3/image` | Corner camera 3 images; recorder saves `camera3_*.jpg`. |
| `/camera4/image_raw` | `sensor_msgs/msg/Image` | `/table_camera4/image` | Corner camera 4 images; recorder saves `camera4_*.jpg`. |

The corner sensors are enabled with `enable_cameras:=true`. Their bridge entries
exist in the configuration even when the sensors are disabled, so discovery of a
ROS endpoint alone does not prove frames are arriving. Dataset recording is
controlled separately by `record_dataset:=true`.

`image_width` and `image_height` configure the actual camera sensor resolution;
in dataset mode all five cameras use the same requested dimensions. The recorder
subscribes to the image topics and writes JPEGs to `dataset_dir`; it does not
publish a separate image-download or dataset topic.

### 3. Perception results and target input
| Topic | Message type | Producer → consumer | Purpose / limitations |
| --- | --- | --- | --- |
| `/perception/segmentation_2d` | `perception_interfaces/msg/Segmentation2D` | `segmentation_node` → `depth_node` | Image-stamped array of detections containing color, centroid, bounding box, pixel count and binary mask. |
| `/perception/red/points` | `sensor_msgs/msg/PointCloud2` | `depth_node` → selected motion node, RViz | Valid depth-derived points for red regions. Default cloud input for approach/pick. |
| `/perception/green/points` | `sensor_msgs/msg/PointCloud2` | `depth_node` → selected motion node, RViz | Valid points for green regions. Select with `cloud_topic:=/perception/green/points`. |
| `/perception/pink/points` | `sensor_msgs/msg/PointCloud2` | `depth_node` → selected motion node, RViz | Valid points for pink regions. |
| `/perception/target_xyz` | `geometry_msgs/msg/PointStamped` | External publisher → approach or pick node | Optional stamped XYZ input when `input_mode:=point`. Topic can be overridden with `target_topic`. |

`Segment2D` is an element inside `Segmentation2D`, not a standalone topic. It
contains `name`, `color`, `centroid_u/v`, `bbox_xywh`, `pixel_count` and an image
mask. The outer message supplies the capture header used for synchronization.

Each color cloud currently combines all visible detections of that color.
The motion nodes use its mean point, which is a visible-surface mean rather than
a guaranteed object center. Use one object per selected color cloud or provide
an explicit point target. There is currently no dedicated apple-instance or
canary-yellow segmentation topic.


## Services — commands, not topics
| Service | Service type | Purpose |
| --- | --- | --- |
| `/approach_node/plan` | `std_srvs/srv/Trigger` | Submit an approach plan without execution. |
| `/approach_node/move` | `std_srvs/srv/Trigger` | Submit approach planning and execution. |
| `/approach_node/cancel` | `std_srvs/srv/Trigger` | Request cancellation of the active approach action. |
| `/pick_node/plan_pick` | `std_srvs/srv/Trigger` | Plan all pick stages without physical motion. |
| `/pick_node/pick` | `std_srvs/srv/Trigger` | Preflight the full sequence, then execute if planning succeeds. |
| `/pick_node/cancel` | `std_srvs/srv/Trigger` | Request cancellation; keep the node alive until the action ends. |
| `/pick_node/plan` and `/pick_node/move` | `std_srvs/srv/Trigger` | Inherited simple-approach services. Use `plan_pick` / `pick` for the full grasp-and-lift workflow. |


With the desired simulation and nodes running, inspect the actual graph:

```bash
ros2 topic list -t
ros2 topic list -t --include-hidden-topics
ros2 topic info /perception/green/points --verbose
ros2 topic hz /camera/color/image_raw
ros2 topic echo /pick_node/status
ros2 service list -t
ros2 action list -t
```

## What I am working on next
1. Adding more objects basically new variants of apples as an object in the scene.
2. Using the 4 new camera setup + 1 robot mounted camera to capture images of the objects in the scene for dataset collection.
3. Start building my Computer vision pipeline for implementing Object Segmentation for pick and place operation on this robotic arm.
4. Pipeline includes:
    - Dataset Preparation & Analysis
    - CNN Model Selection based on dataset analysis report.
    - Train the model over Google Colab
    - Perform Evaluation
    - Implement Deployment process

