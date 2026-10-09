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
  - Each links have been given random Inertia Matrix, important to resolve it before performing actual Motion Planning for accurate controller performance.**  
 **  
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
   
  1. Likely causes is:  
  - random assignment of Inertia matrix values for the links which has been assigned with different masses (This issue needs to be resolved now, instead of later implementation).  
 - So update the masses and inertia matrix for each links thoughtfully.  
 - For inertial matrix computaion of each elements we must also make sure the inertia frame must align with the object’s frame (location & origin) hence for the specific objects/links visual tag’s origin tag must be copied for inertial tag’s origin tag. Why? —>  
 Because link’s frame origin isn’t at the links COM if links visual origin isn’t  0 0 0, if its non zero that means the COM/link’s frame origin of the link is shifted from the actual link(remember the links actual position is dictated by its joint’s origin tag) therefore if the visual tag of link is shifted we must give exact shited location to the inertia origin tag so that inertia origin of the object/link must align with the links origin.  
 When updating the masses and computing Inertia do also make changes to the efforts for the each joints because if increased masses is made and inertia matrix is also changed accordingly but not changed efforts it will cause the robot to fall down because the efforts are not sufficient enough to hold the links weight thus compute efforts as well.  
   
  Effort/torque = m.g.d where d=distance between motor and the COM of link, g=gravity and m=mass of the link.  
 REMEBER: Effort is computed not just for the urrent link instead it should accommodate all the further links whos COM position and masses is putting influence on the current link’s joint/motor because that motor has to carry the weight of the further links and or additional object mass if its grapped by the gripper.  
   
 Also Could include damper it is meant for such unstable motion only(later on) if you want more stability.  
   
  - Less probable cause is: in gazebo simulation the robot is not magically teleported to new goal position upon providing it, it uses velocity command to perform motion v=Kp(x’-x)f, where Kp is position_proportional_gain , so if Kp is too high greater 1 or 2 it will start oscillation/unstability as well, Kp=0.1 is the default value.  
2. For Camera feed:  
 - alter the camera joint’s orientation rpy to change the viewing direction of the camera feed, camera joint is of type fixed so altering the axis tag won’t make any difference hence alter origin tag’s rpy value to see the change, because fixed joint doesn’t move thus axis tag won’t influence anything but origin tag which cares about placement of the object initially needs to be altered.  
   
 - Once you corrected camera view by rotation, the new problem of Gazebo and ROS2 viewing Image frame appears, its the Library issue of Gazebo and ROS2. The issue is Gazebo make camera viewing direction is X-forward, while ROS2 consider Z forward. Thus it is needed to add additional empty link infront fo the camera link name it “corrected_optical_frame” the joint of this link will correct the orientation of the camera frames. So camera_link will capture frames by gazebo (X-forward, Y:left, Z: Up) and then “corrected_optical_link” joint will coreect this to ROS2 acceptable frame orientation (X:right, Y:Down,Z:fwd).  
For camera rviz, we need to keep the rviz launch file not to open up because it consist of robot_state_publisher and gui publisher as well, which would use simulation time or not but robot_state_publisher will also be used in gazebo launch file which will create discrepancy. Therefore its needed to open saved rviz.config file instead with simulation time as true from CLE command:  
 rviz2   -d /home/shoaib-ubuntu/ros2_projects/roboArm_rosws/src/roboarm_urdf_description/rviz/rviz_config_display.rviz   --ros-args -p use_sim_time:=true   
