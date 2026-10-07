**Perception-based-manipulator-5-DOF**  
**Stage-1 Building 5R-Manipulator:**  
**1. Robot description URDF building.**  
- Learned about frames, joints, links collision, Inertial tags.  
- Build gazebo and Rviz launch files.  
- Installed moveit config package to move my robot using assistant manager.  
- Created rviz and gazebo launch files for spwaning and testing the robot.  
- **ISSUE:** Robot falls down in the gazebo simulation while base link stays static on the ground.  
 **SOLUTION**: The viable reason for this is absence of ros2_control setup in my URDF, because of which gazebo views the joints as hinges/passive joints instead of motors/active joints.  
**2. ros2_control setup:**  
- Make 2 new  
  1. **roboArm_ros2_control.urdf.xacro** file —> for providing controllers hardware interface (command and state interface) to each structural joints(Grippers not included YET) with in ros2_control tag.  
  2. **roboArm_gazebo.urdf.xacro **file —> for adding gazeboSim plugin  
- Add config/controllers.yaml file —> which consist of selected controllers like joint_state_broadcaster, JTC.  
- In order to activate these two controller, start the controllers as a node so write it inside the gazebo launch file itself.  
- **THIS FINALLY RESOLVES THE ISSUE OF** robot falling down in the gazebo simulation.  
- Motion Specific detailing w’ll do it once robot is fully settled up.  
**3. Adding rgbd_camera sensor:**  
Now to fulfill the aim of the project is to get 3d coordinates of the object using object detection hence needed camera module first, therefore adding sensor plugin according to New Gazebo Harmonic method that uses ros_gz_bridge for communication unlike Classical Gazebo that just uses sensor specific gazebo plugins to start using sensors directly.  
- For adding the rgbd_camera sensor I first need to structure my urdf workspace properly, so:  
- 5R Manipulator robot description —> written in the separate URDF.  
- Sensors URDF —> consists of rgbd_camera links, joints, and gazebo/sensor tags.  
- Perception_world.sdf —> consist of the fundamental plugins that is needed according to the gazebo Harmonic documentation along with rgbd_camera sensor plugin.  
- ros_gz_bridge config.yaml file —> for proper structuring outputs of the rgbd_camera like Image_raw,camera_info,depth_camera,pointcloud sending from gazebo transport to the ros2 topics, thus needed proper structuring of message transportation instead of multiple CLI based ros_gz_bridge message transporation commands.  
-  **ISSUE**: (To be Resolved later)  
  - Each links have been given random Inertia Matrix, important to resolve it before performing actual Motion Planning for accurate controller performance.  
