# How I am building this project

This is my development record, organised around the decisions and problems that led to the current implementation. The project is still in progress. I have kept the HSV baseline because it lets me study the robotics pipeline before adding a learned perception model.

## 1. Start with the robot model

I began by describing the robot with URDF/Xacro: links, joint frames, mesh visuals, collision geometry and inertial properties. The current arm chain uses `joint_1` through `joint_4` and `joint_prism`. The fingers are separate from these five arm degrees of freedom.

I separated the main robot description, sensor description and ros2_control interfaces so I could change one part without rewriting the entire model. RViz helped me inspect the geometry and joint transforms; Gazebo exposed the physical behaviour that a visual-only model does not show.

One early problem was the arm falling under gravity. A valid URDF alone does not mean joints are actively controlled. I added the Gazebo ros2_control system and trajectory controllers, then investigated masses, inertia and effort limits when instability remained.

**What I learned:** controller activation and physical modelling are separate checks. An inertial origin must describe the centre of mass; it should not simply copy a mesh's visual origin unless that origin actually represents the centre of mass. Box inertia is an approximation for a link, not a measurement of the actual mechanism. Joint torque also depends on the downstream links, payload and configuration, so a single `m*g*d` calculation is only a starting estimate.

Relevant files:

- `src/roboarm_urdf_description/urdf/roboArm_urdf.urdf.xacro`
- `src/roboarm_urdf_description/urdf/roboArm_description.urdf.xacro`
- `src/roboarm_urdf_description/urdf/roboArm_ros2_control.urdf.xacro`
- `src/roboarm_urdf_description/config/controllers.yaml`

## 2. Connect simulation to ROS 2

I added an RGB-D sensor attached to `claw_link`, and the Gazebo Sensors system in the world file. The camera produces colour, depth, calibration and point-cloud data. A YAML bridge configuration maps the Gazebo messages to ROS topics and also bridges the simulation clock.

I use `use_sim_time` for the perception/motion sequence so data timestamps, TF and planning refer to the same clock. I also avoid starting the standalone display launch alongside Gazebo: both launch paths contain a robot-state publisher, and the display path introduces GUI joint states that do not represent simulated joint feedback.

Camera orientation was another important detail. The camera body axes and the ROS optical-frame axes use different conventions. I added `corrected_optical_frame` for optical coordinates: X right, Y down, Z forward. The actual image and cloud headers still need checking during a runtime test; assigning a frame name does not rotate numerical data.

Relevant files: `sensors_description.urdf.xacro`, `worlds/perception_world2.sdf`, `config/ros_gz_bridge_config.yaml`, and `launch/gazebo.launch.py` in the description package.

## 3. Build an understandable 2D segmentation baseline

My first detector uses HSV thresholds for red, pink and green objects. I chose this as a simple baseline so I could inspect masks and follow each stage of the pipeline.

In `segmentation.py`, I convert ROS images to OpenCV images with `cv_bridge`, convert BGR to HSV, threshold each colour, and apply morphological opening. I then find contours, reject small regions and create an individual mask for each accepted region. Red uses two hue ranges because its hue wraps around the ends of OpenCV's HSV scale.

For each region I record its mask, bounding box, pixel count and mean pixel location `(u, v)`. The displayed region number is assigned within a frame; it is not a persistent tracking ID. A 2D centroid is also not the physical centre of the object.

I created `Segment2D.msg` and `Segmentation2D.msg` to publish these results on `/perception/segmentation_2d`. The original image timestamp is retained, including in each mask, so the depth stage can match the correct frame. Empty detections are published too, so later stages know when an object disappears.

**Limitation observed:** the thresholds are sensitive to illumination and material appearance. A colour mask can recognise a colour without understanding what the object is.

## 4. Recover 3D points from masks and depth

The depth node subscribes to segmentation results and the depth image through `TimeSynchronizer`. Calibration arrives through a separate `CameraInfo` callback. Exact synchronization means the depth and segmentation headers must have identical timestamps; otherwise the paired callback will not run.

For every nonzero mask pixel, I read its depth and use the pinhole equations:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth in metres
```

Here `fx`, `fy`, `cx` and `cy` come from the camera intrinsic matrix. The current code treats `32FC1` depth as metres and converts `16UC1` millimetres to metres. It rejects non-finite and non-positive depths, and checks mask/depth dimensions.

These equations require depth registered to the colour image and intrinsics for that image geometry. Equal image dimensions alone do not prove registration.

The output is a set of **visible surface points in the camera optical frame**. It is not a complete 3D reconstruction or a 6D object pose. I publish the results as PointCloud2 messages and inspect them in RViz.

**Current simplification:** individual regions are reconstructed separately, but points are then combined by colour for publishing. Before handling several similar objects, I need to preserve instance identity through the 3D output and target-selection stage.

## 5. Connect a perceived target to MoveIt 2

The Setup Assistant package supplies the semantic model, planning group and related configuration. My `roboarm_motion` package reuses these while launching MoveIt against the existing Gazebo robot and its controllers.

The approach node performs the following steps:

1. Read the selected cloud and compute the mean of finite XYZ samples.
2. Keep its capture timestamp and frame, and reject missing/stale targets when a request arrives.
3. Transform the point into `base_link` using TF at the capture time.
4. Add the configured base-Z approach offset in cloud mode.
5. Publish a target marker and send a MoveGroup action goal after an explicit service call.
6. Report the eventual planning/execution result and display the planned trajectory.

The three services are `/approach_node/plan`, `/approach_node/move` and `/approach_node/cancel`. The `move` service plans again using a fresh target. The client does not directly send a point cloud to a joint controller; MoveIt calculates a joint trajectory and delegates execution to the configured controller action.

Because the arm has five controlled arm joints, the current goal specifies position only. KDL is configured for position-only IK; I do not impose an arbitrary end-effector orientation. The target is the origin of `claw_link`, which is different from a finger-contact location.

**Still unresolved:** Gazebo obstacles are not inserted automatically into MoveIt's scene. The launch also spawns the robot at Z=0.79; this must be reconciled with the URDF world/base transform before using Gazebo-world coordinates for collision objects. These are prerequisites for reliable environment-aware manipulation, not features I consider complete.

## 6. Begin learned apple segmentation

To move beyond colour rules, I started experiments with the Ultralytics YOLO26n segmentation model and YCB apple images. YCB is useful for my intended workflow because it supplies object imagery and 3D meshes that can later be used in simulation.

My initial dataset preparation involved converting raster `.pbm` masks into segmentation labels. For YOLO segmentation, each label row contains a class ID followed by normalised polygon vertices; it is not just a bounding-box label. Mask foreground polarity must be verified before extracting contours.

In my initial tests, the model recognised the YCB-style apple much better than unfamiliar apples, and sometimes labelled a pink block as an apple. These are qualitative observations, not a measured benchmark. Limited appearance/background diversity is a possible contributor; annotation quality, splitting and training settings also need checking. A single foreground class is not inherently a problem for an apple-only model.

My next experiment is to add varied apple images and negative scenes, including suitable COCO examples, then fine-tune and evaluate on held-out scenes. I will keep related views of the same capture session together when splitting data to reduce leakage. Fine-tuning means continuing optimisation from chosen weights; it does not mean merging two independently trained weight files.

The training and dataset-conversion work is not yet packaged in this repository. I still need to add the scripts, training configuration, dataset instructions, evaluation results and ROS inference integration. The committed launch sequence therefore remains the HSV pipeline.

## 7. Define what completion should mean

My next milestones are reproducible learned segmentation, instance-level 3D targets, a consistent planning scene and grasp selection. After that I can connect approach, gripper closure, lifting and placement into a task sequence.

I plan to evaluate each layer separately: held-out segmentation quality, false positives on no-apple scenes, perception latency, 3D target error, motion planning success and eventually grasp/task success. This will make it possible to explain whether a failure comes from perception, geometry, planning or contact rather than treating the whole system as one model.
