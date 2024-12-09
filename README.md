# Autonomous-Mobile-Robot

In the realm of indoor robotics, navigation poses a critical challenge across service robots, humanoids, and warehouse automation.Existing techniques like line following,RFID tracking, and Aruco marker-based systems exhibit limitations in autonomy and adaptability. This has created a need for the development of an advanced and sophisticated autonomous navigation system that encompasses self-reliant perception, planning, and control capabilities. The proposed system makes use of LiDAR technology for mapping and Robot operating system for planning and control which takes place on Jetson Nano GPU platform, providing real-time processing power for environmental data. LiDAR sensors offer accurate environmental perception, obstacle detection, and adaptability to dynamic indoor settings. The system employs BLDC wheels and an aluminium base for versatility, supporting loads up to 120 kg. Additionally, it incorporates an IMU sensor with EKF sensor fusion for accurately estimating the robot's position. This flexible design has applications in various indoor robotics scenarios, ranging from complex service tasks to efficient warehouse operations. This versatile design finds applications in various indoor robotics scenarios, from complex service tasks to efficient warehouse operations. 

﻿
****************** AUTONOMOUS NAVIGATION ROBOT***************************
REFERENCE LINK:https://automaticaddison.com/how-to-set-up-the-ros-navigation-stack-on-a-robot/ 

 (25-4-2023)
=========================================================================
1.DOWNLOAD AND FLASH THE  OPERATING SYSTEM ON  JETSON NANO DEVELOPER KIT (XUBUNTU IMAGE FROM NVIDIA WEBSITE [UBUNTU 20.04]) LINK :https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768

2.DOWNLOAD  ROS1,version name:Noetic.
LINK: https://automaticaddison.com/how-to-install-ros-noetic-ninjemys-on-ubuntu-linux/
LINK: https://www.youtube.com/watch?v=Qk4vLFhvfbI&list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q
roscore command to verify the installation 
Adding global and dewel source lines to gedit ~/.bashrc
rosrun rospy_tutorials talker AND rosrun rospy_tutorials listener AND rqt_graph
rosrun turtlesim turtlesim_node AND rosrun turtlesim turtle_teleop_key and rqt_graph

3.DOWNLOAD NAVIGATION STACK 
COMMAND: sudo apt-get install ros-noetic-navigation
LINK:https://automaticaddison.com/how-to-set-up-the-ros-navigation-stack-on-a-robot/
VERFICATION COMMAND: rospack find amcl to confirm installation 

4.DOWNLOAD RPLIDAR PACAKAGE
COMMAND: sudo git clone https://github.com/Slamtec/rplidar_ros.git 
LINK :https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/
 
5.DOWNLOAD ARDUINO IDE 
COMMAND:  git clone https://github.com/JetsonHacksNano/installArduinoIDE.git
LINK: https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/

6.DOWNLOAD ROS SERIAL PACKAGE
COMMAND:sudo apt-get install ros-noetic-rosserial-arduino
LINK:https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/

7.DOWLOAD HECTOR SLAM PACKAGE
COMMAND:git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
LINK:https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/

8.DOWLOAD GAZEBO: sudo apt-get install gazebo

9.DOWNLOAD RVIZ: sudo apt-get install rviz

10.DOWNLOAD TERMINATOR:sudo apt-get install terminator

11.DOWNLOAD VNC OR NO MACHINE.
LINK: https://developer.nvidia.com/embedded/learn/tutorials/vnc-setup

12.DOWNLOAD QT4: github https://github.com/qt-creator/qt-creator
                                   sudo apt install qtcreator
                                   qtcreator
13.DOWNLOAD  code-oss from github(paulmcwhorterchannels)
     Installing microsoft python package in code-oss

14.DOWNLOAD VISUAL STUDIO CODE 
COMMAND: sudo snap install code  --classic
VERFICATION COMMAND: code.
=========================================================================
IMPORTANT NOTES:

📌Global costmap: This costmap is used to generate long term plans over the entire environment….for example, to calculate the shortest path from point A to point B               
on  a map.
📌Local costmap: This costmap is used to generate short term plans over the environment….for example, to avoid obstacles.

📌map frame has its origin at some arbitrarily chosen point in the world. This coordinate frame is fixed in the world
📌odom frame has its origin at the point where the robot is initialized. This coordinate frame is fixed in the world.
📌base_footprint has its origin directly under the center of the robot. It is the 2D pose of the robot. This coordinate frame moves    as the robot moves.
📌base_link has its origin directly at the pivot point or center of the robot. This coordinate frame moves as the robot moves.
 📌laser_link has its origin at the center of the laser sensor (i.e. LIDAR). This coordinate frame remains fixed (i.e. “static”) relative to the base_link. 

📌 problem:To convert a detected object’s coordinates in the LIDAR coordinate frame to equivalent coordinates in the robot base’s frame. 
solution:ROS has a package called tf to handle all these coordinate transforms for us.

📌/right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
📌/left_ticks : Tick counts from the left motor encoder (std_msgs/Int16)
📌/initial_2d : The initial position and orientation of the robot.(geometry_msgs/PoseStamped)

The publisher will publish odometry data to the following topics:

📌/odom_data_euler : Position and velocity estimate. The orientation.z variable is an Euler angle representing the yaw angle. (nav_msgs/Odometry)
📌/odom_data_quat : Position and velocity estimate. The orientation is in quaternion format. (nav_msgs/Odometry)

📌2D Pose Estimate button:

Published Topic: initialpose
Type: geometry_msgs/PoseWithCovarianceStamped
Description: Allows the user to initialize the localization system used by the navigation stack by setting the pose of the robot in the world.

📌2D Nav Goal button:

Published Topic: move_base_simple/goal
Type: geometry_msgs/PoseStamped
Description: Allows the user to send a goal to the navigation by setting a desired pose for the robot to achieve.

📌/initial_2d topic (which the odometry publisher subscribes to) (geometry_msgs/PoseStamped)
📌/goal_2d topic (which our path planner node, which I’ll create in a later tutorial, will subscribe to). (geometry_msgs/PoseStamped)

📌/scan :  Laser scan messages from the LIDAR (sensor_msgs/LaserScan).
📌/tf : Coordinate frame transformations (tf/tfMessage).
📌/initialpose : The initial position and orientation of the robot using quaternions. (geometry_msgs/PoseWithCovarianceStamped) — RViz initial pose button publishes to this topic.
📌/map : The occupancy grid map created using gmapping, Hector SLAM, or manually using an image (nav_msgs/OccupancyGrid).

The amcl node will publish to the following topics:

📌/amcl_pose : Robot’s estimated pose in the map (geometry_msgs/PoseWithCovarianceStamped).
📌/particlecloud: The set of pose estimates being maintained by the filter (geometry_msgs/PoseArray).
📌/tf (tf/tfMessage): Publishes the transform from odom (which can be remapped via the ~odom_frame_id parameter) to map.
=========================================================================
                                                                                                             (25-04-2023)
CREATING BASE CONTROLLER NODE

LINK1: https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/
LINK2:https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/

TICKS_PER_REVOLUTION = 47
 WHEEL_RADIUS = 0.081 // Wheel radius in meters
 WHEEL_BASE = 0.279// Center of left tire to center of right tire
TICKS_PER_METER = 95

                                                                                                             (06-05-2023)
=========================================================================
INSTALLING IMU PACKAGE AND KALMAN FILTER PACKAGE (robot_ef_pkg):


KALMAN FILTER:sudo apt-get install ros-noetic-robot-pose-ekf
LINK: https://automaticaddison.com/sensor-fusion-using-the-ros-robot-pose-ekf-package/

IMU GITHUB LINK:git clone https://github.com/OSUrobotics/mpu_6050_driver.git
LINK: https://automaticaddison.com/visualize-imu-data-using-the-mpu6050-ros-and-jetson-nano/

=========================================================================
MOVING ROBOT  USING RQT STERRING GRAPH AND DOTTED LIDAR MAP MAKING:

LINK: https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/
LINK: //automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/

cd ~/catkin_ws
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
rosrun rqt_robot_steering rqt_robot_steering  
rosrun turtlesim turtle_teleop_key 
rostopic echo /turtle1/cmd_vel
rostopic echo /cmd_vel

roscore
cd ~/catkin_ws/
sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros rplidar.launch
rviz(select /scan topic press enter)
 
NOTE:CHANGE THE PUBLISHER FROM STEERING TO TELEOP_NODE OR VISE-VERSA
=========================================================================
VISUALIZATION CODES:

rosrun rqt_graph rqt_graph 
rosrun rqt_tf_tree rqt_tf_tree
=========================================================================
HECTOR SLAM MAP CREATION WITH LIDAR:

LINK: //automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/
LINK: https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/

cd ~/catkin_ws
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
rosrun rqt_robot_steering rqt_robot_steering  
rosrun turtlesim turtle_teleop_key 
rostopic echo /turtle1/cmd_vel
rostopic echo /cmd_vel

roscore
cd ~/catkin_ws/
roscore
sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros rplidar.launch
roslaunch hector_slam_launch tutorial.launch

SAVING THE MAP :

Before you launch the mapping process, you need to install map_server.
sudo apt-get install ros-noetic-map-server
Create a maps folder. This is where we will store our maps.
mkdir ~/catkin_ws/maps
cd ~/catkin_ws/maps
rosrun map_server map_saver -f my_map

                                                                                                     (21-06-2023)
=========================================================================
IMU NODE LAUNCHING AND VISUALIZATION:

LINK: https://automaticaddison.com/visualize-imu-data-using-the-mpu6050-ros-and-jetson-nano/

cd ~/catkin_ws/
roscore
smbus error solving command = sudo chmod a+rw /dev/i2c-*
NOTE:change python to python3(in both imu and broadcaster nodes)
rosrun mpu_6050_driver imu_node.py
rosrun mpu_6050_driver tf_broadcaster_imu.py
rosrun rviz rviz
=========================================================================
COMMAND TO ON JETSON NANO FAN:

sudo jetson_clocks
=========================================================================
                                                                                                        (26-06-2023)
1.SENSOR TRANSFORM NODE:
Input sensor transform node values in launch file:

(With respect to z-axis):

a.Base footprint to base link =0.08m
b.Base link to Laser link=0.25m
c.Base link to IMU=0.14m

(With respect to x-axis):

a.Base link to Laser link=0.20m
b.Y=0
========================================================================
2.Change IMU configuration for mpu6050:

ox68=104
=========================================================================
3.Input MAP path in launch file line 53 and 74 or 78
=========================================================================
4.Create navigation_data_pub package and saved the map folder there(rviz,png,pgm,yaml).
========================================================================
5. IN FOLDER navstack_pub create the following configuration:

LINK:https://automaticaddison.com/how-to-set-up-the-ros-navigation-stack-on-a-robot/

Common Configuration (Global and Local Costmap)
Global ConLocal Configuration (Local Costmap)
Local Configuration (Local Costmap)
Base Local Planner Configuration
=========================================================================
6. AFTER CREATING MAP AND LOADING,CHANGE THE ARDUINO BASE_CONTROLLER NODE.  
=========================================================================
                                                                                                           (27-06-2023)
1.Create localization_pub full package and Initial Pose and Goal Publisher in ROS
LINK: https://automaticaddison.com/how-to-create-an-initial-pose-and-goal-publisher-in-ros/

2. localization_pub full package integrating with kalman filter
LINK: https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/

3. Create an imu launch file using MAIN LAUNCH  file in the mpu_6050_driver,and in main launch file change (imu\data to imu\imu\data)

4. IN navstackpub/param/costmap_common_param.yaml change 
     a. obstacle range from 0.5 to 0.2
     b. ray trace from 0.5 to 0.2
     c.footprint from 0.14 to -+45 
=========================================================================
********************LAUNCH THE AUTONOMOUS MOBILE ROBOT*******************

   cd ~/catkin_ws/
   catkin_make --only-pkg-with-deps navstack_pub
   roslaunch navstack_pub jetson_nano_bot.launch
   rosrun rqt_graph rqt_graph 
   rosrun rqt_tf_tree rqt_tf_tree

=========================================================================

roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch

launch
 cd ~/catkin_ws/                                           rostopic list     rostopic echo 
catkin_make
sudo chmod a+rw /dev/i2c-*
catkin_make --only-pkg-with-deps navstack_pub      roslaunch navstack_pub nav_odom.launch
roslaunch navstack_pub jetson_nano_bot.launch      roslaunch navstack_pub without_imu.launch 


args="-d /home/techbots/catkin_ws/src/jetson_nano_bot/navstack_pub/robo_lab.rviz"

args="-d /home/techbots/catkin_ws/src/jetson_nano_bot/navigation_data_pub/maps/robo_lab.rviz"

add this line in main launch file 
 <node name="imu_filter_node_for_orientation" pkg="imu_complementary_filter" type="complementary_filter_node" >
  </node>


