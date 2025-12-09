# UGV ROS2 + Unity Simulation Environment

This repository contains a complete Unmanned Ground Vehicle (UGV) simulation built using **ROS2 Humble** and **Unity**. The system integrates SLAM, navigation (Nav2), simulated sensors, a MAVROS/MAVLink bridge to QGC, real-time video streaming, and full Unity physics.

##  Overview

This simulation allows a UGV to be driven, mapped, and navigated using ROS2 while running physically in Unity. It supports:

- Physics-based Unity UGV simulation  
- Simulated LIDAR & pointclouds  
- SLAM Toolbox for mapping  
- Nav2 for global / local planning  
- MAVROS + QGroundControl integration  
- Video streaming  
- A custom odometry publisher  
- ROS ↔ Unity networking via ros_tcp_endpoint

##  ROS2 Setup

I reccomend using the most recent version of Ubuntu, this package was developed on Ubuntu 22.04. Once you have your distrubution install ROS2 Humble with the following terminal commands, more comprehensive instructions can be found here: [ROS2 Humble Wiki](https://docs.ros.org/en/humble/index.html).

cd ros_tcp_ws  
source /opt/ros/humble/setup.bash  
colcon build --symlink-install  
source install/setup.bash  


##  Unity Setup

1. Open Unity Hub  
2. Click **Open**  
3. Select the `UnitySim/` folder  
4. Press **Play** to begin simulation  

Unity will automatically connect to ROS2 via the ROS-TCP Connector.

##  Running the Full System

### 1. Start ROS–TCP Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0


### 2. Start SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py


### 3. Start Nav2
ros2 launch nav2_bringup navigation_launch.py


### 4. Start MAVROS Bridge (QGroundControl)
python3 ros_tcp_ws/src/mavros_bridge/mavros_bridge/simple_mavlink_bridge.py


### 5. View camera feed (optional)
ros2 run rqt_image_view rqt_image_view


##  SLAM / Nav2 Notes

- LIDAR must remain fixed relative to `base_link`.  
- Use RViz to inspect TF (`odom`, `base_link`, `laser`).  

Clear costmaps when noise builds:  

ros2 service call /clear_entire_costmap std_srvs/srv/Empty {}  
ros2 service call /clear_entirely_local_costmap std_srvs/srv/Empty {}  
ros2 service call /clear_entirely_global_costmap std_srvs/srv/Empty {}  


Reset SLAM entirely:  
ros2 topic pub /slam_toolbox/clear_map std_msgs/msg/Empty {}  


##  Mission Planning (QGroundControl)

- QGC sends missions via MAVLink  
- MAVROS bridge converts missions → navigation goals  
- Unity robot executes paths realistically  

##  Quick Start (Minimal Commands)

Terminal 1  

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

Terminal 2

ros2 launch slam_toolbox online_async_launch.py

Terminal 3

ros2 launch nav2_bringup navigation_launch.py

Terminal 4

python3 ros_tcp_ws/src/mavros_bridge/mavros_bridge/simple_mavlink_bridge.py


Open Unity and press **Play**.

##  Notes

- Unity Library/Temp and ROS build/install/log are intentionally ignored via .gitignore.  
- Modular design allows easy extension with new sensors, controllers, or planners.  

##  Contributions

Feel free to fork or open issues. This project supports rapid robotics simulation and autonomous UGV research.

