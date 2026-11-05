# Perception-based 5-DOF manipulator

ROS 2 workspace for a robot arm with Gazebo simulation, RGB-D color segmentation,
3D target estimation, and MoveIt approach planning. The workspace targets ROS 2
Jazzy and Gazebo Harmonic. Motion integration is an approach demonstration, not a
complete grasp pipeline.

## Repository layout

```text
roboArm_rosws/
├── README.md                    # Workspace overview and entry points
├── docs/
│   └── development-notes.md      # Historical experiments and design notes
├── scripts/                     # Optional local shell helpers
└── src/                         # Colcon packages
    ├── perception_interfaces/   # Custom segmentation messages
    ├── perception_roboarm/       # HSV segmentation and depth processing
    ├── roboarm_controller/       # Controller configuration and launch
    ├── roboarm_motion/           # Perception-to-MoveIt integration and guide
    ├── roboarm_movit_config/     # MoveIt Setup Assistant configuration
    ├── roboarm_proj1/            # Publisher/subscriber learning examples
    └── roboarm_urdf_description/ # Robot, meshes, worlds, RViz and simulation
```

Keep ROS packages directly under `src/`; keep each package's launch files,
configuration, meshes and resources inside that package. The existing spelling
`roboarm_movit_config` is intentional here because code refers to that package name.
Generated `build/`, `install/`, `log/` and Python caches are local build products.

## Clone

```bash
mkdir -p ~/ros2_projects
cd ~/ros2_projects
git clone https://github.com/Shoaib3786/Perception-based-manipulator-5-DOF-.git roboArm_rosws
cd roboArm_rosws
```

## Build

From the workspace root in a Bash terminal:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

These instructions require the ROS, Gazebo and MoveIt dependencies declared by the
packages. A successful build does not by itself validate simulation or robot motion.

## Run

Source ROS and `install/setup.bash` in each terminal. Start the simulation with:

```bash
ros2 launch roboarm_urdf_description gazebo.launch.py
```

For segmentation, depth processing, planning, execution and prerequisites, follow
the [motion integration guide](src/roboarm_motion/README.md). The segmentation
executable is currently spelled `segementation_node` in `setup.py`.

## Helper scripts

Run helpers from the workspace root after building:

- `bash scripts/view-camera.bash`: opens rqt Image View and the saved RViz view.
- `bash scripts/move-arm-example.bash`: **sends arm and gripper motion goals**;
  inspect its joint positions and your simulation scene before running it.
- `source scripts/setup-venv.bash`: sources the existing personal virtual
  environment at `~/py_venv/ros2_roboArm_pyvenv`, ROS and the workspace. Adjust the
  virtual-environment path for another machine.

The scripts locate the workspace relative to their own location.

## Development status

The current source contains HSV segmentation and depth processing. Apple
segmentation training experiments and next steps are recorded in the
[development journal](docs/development-notes.md); trained YOLO weights and a
YOLO runtime node are not included in this repository.

## Documentation and maintenance

- This README owns workspace setup, package layout and entry points.
- The [motion README](src/roboarm_motion/README.md) owns integration behavior and
  operating instructions; it is also installed with that package.
- The [development journal](docs/development-notes.md) preserves the project's
  history and unresolved observations; consult current code before reusing examples.
- Do not commit editor recovery files, Python bytecode or package export archives
  alongside the extracted source. Track `setup.py`, `setup.cfg` and `.gitignore`;
  broad `setup*` ignore rules can silently omit files needed by fresh clones.
- Local datasets should be placed and documented deliberately once their role is
  known. The existing root HDF5 files have been left untouched.
