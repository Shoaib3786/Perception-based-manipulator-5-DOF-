**Perception-based-manipulator-5-DOF**  
**Stage-1 Building 5R-Manipulator:**  
**1. Robot description URDF building.**  
- Learned about frames, joints, links collision, Inertial tags.  
- Build gazebo and rviz launch files.  
- Installed moveit config package to move my robot using assistant manager.  
- Created rviz and gazebo launch files for spawning and testing the robot.  
- **ISSUE:** Robot falls down in the gazebo simulation while base link stays static on the ground.  
- **SOLUTION**: The viable reason for this is absence of ros2_control setup in my URDF, because of which gazebo views the joints as hinges/passive joints instead of motors/active joints.  
   
**2. ros2_control setup:**  
- Make 2 new files  
  - **roboArm_ros2_control.urdf.xacro** file —> for providing controllers hardware interface (command and state interface) to each structural joints(Grippers not included YET) with in ros2_control tag.  
  - **roboArm_gazebo.urdf.xacro** file —> for adding gazeboSim plugin  
- Add config/controllers.yaml file —> which consist of selected controllers like joint_state_broadcaster, JTC.  
- In order to activate these two controller, start the controllers as a node so write it inside the gazebo launch file itself.  
- This **RESOLVED THE ISSUE OF** robot falling down in the gazebo simulation.  
- Motion Specific detailing w’ll do it once robot is fully settled up.  
   
**3. Adding rgbd_camera sensor:** **  
 **Now to fulfill the aim of the project is to get 3d coordinates of the object using object detection hence needed camera module first, therefore adding sensor plugin according to New Gazebo Harmonic method that uses ros_gz_bridge for communication unlike Classical Gazebo that just uses sensor specific gazebo plugins to start using sensors directly.  
- For adding the rgbd_camera sensor I first need to structure my urdf workspace properly, so:  
- 5R Manipulator robot description —> written in the separate URDF.  
- Sensors URDF —> consists of rgbd_camera links, joints, and gazebo/sensor tags.  
- Perception_world.sdf —> consist of the fundamental plugins that is needed according to the gazebo Harmonic documentation along with rgbd_camera sensor plugin.  
- ros_gz_bridge config.yaml file —> for proper structuring outputs of the rgbd_camera like Image_raw,camera_info,depth_camera,point-cloud sending from gazebo transport to the ros2 topics, thus needed proper structuring of message transportation instead of multiple CLI based ros_gz_bridge message transportation commands.  
- **ISSUE**: (To be Resolved later)  
  - Each links have been given random Inertia Matrix, important to resolve it before performing actual Motion Planning for accurate controller performance.  
   
**4. Testing Robot in the gazebo simulation and Camera feed:**  
- Unlike rviz2 which requires just the new potion to move the robot in the visualization, Gazebo simulation needs the controllers like JTC to perform motion because simulation doesn’t depends on position given by joint_state_publisher_gui. So instead of such stuffs, we have other way around to check how robot moves around in the simulation:  
  - **Method-1**   
 Use FollowJointTrajectory action interface CLI command and provide the position, it uses interpolation to produces the motion (Easy to implement just for testing purpose).  
 **CLI command:**   
 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory “{trajectory:{joint_name:[ ],point:[{position:[ ], time_from_start:{ }}]}”**  
 **  
  - **Method-2:**  
 Use Publisher node to publish directly on /joint_trajectory topic, JTC subscribes to this topic automatically to recieves the waypoint/poisition points of each joint to generate trajectory using interpolation just like above.  
 **CLI command:**  
 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory “{joint_name:[ ],point:[{position:[ ], time_from_start:{ }}]}”  
   
  - **Method-3:** Perform via moveIt2 (tedius for just testing purposes)  
   
  - **Method 4:** Use you on slider gui  
   
- Tested camera feed from rviz2 image  
   
- **ISSUES Found:**  
  - Unstable motion of the robot once it reaches the sent goal(Gripper not included in this evaluation).  
  - Camera outputs feed in totally different direction  
- **SOLUTION:**  
 1. Likely causes:   
  - is random assignment of Inertia matrix values for the links which has been assigned with different masses (This issue needs to be resolved now, instead of later implementation).  
 So update the masses and inertia matrix for each links thoughtfully.  
 When updating the masses and computing Inertia do also make changes tot he efforts for the each joints because if increased masses is made and inertia matrix is also changed accordingly but not changed efforts it will cause the robot to fall down because the efforts are not sufficient enough to hold the links weight thus compute efforts as well.  
 Effort/torque = m.g.d where d=distance between motor and the COM of link, g=gravity and m=mass of the link.  
 Also Could include damper it is meant for such unstable motion only(later on) if you want more stability.  
  - Less probable cause is: in gazebo simulation the robot is not magically teleported to new goal position upon providing it, it uses velocity command to perform motion v=Kp(x’-x)f, where Kp is position_proportional_gain , so if Kp is too high greater 1 or 2 it will start oscillation/unstability as well, Kp=0.1 is the default value.  
### 2. For Camera feed alter the camera joint’s orientation rpy to change the viewing direction of the camera feed, camera joint is of type fixed so altering the axis tag won’t make any difference hence alter origin tag’s rpy value to see the change, beecause fixed joint doesn’t move thus axis tag won’t influence anything but origin tag which cares about placement of the object initially needs to be altered.  
   
 **Better structure: use an optical frame**  
For ROS perception you should eventually have:  
claw_link  
    ↓  
camera_joint  
    ↓  
camera_link  
    ↓  
camera_optical_joint  
    ↓  
camera_optical_frame  
Because ROS camera optical convention is:  
Z forward  
X right  
Y down  
while Gazebo camera convention uses +X as viewing direction.  
Don't solve that by manually swapping XYZ values later. Encode it once as a TF transform.  
   
