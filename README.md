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

Downlaod the latest version of unity and in the unity hub select the UnitySim folder.

1. Open Unity Hub  
2. Click **Open**  
3. Select the `UnitySim/` folder
4. The Scene is called "UGVScene"
5. Press **Play** to begin simulation  

Unity will automatically connect to ROS2 via the ROS-TCP Connector.

##  Running the Full System

To start the system simply run ros_control_panel.py, to do this navigate to "cd ~/ugv_sim" (your project folder that contains this repo) in the terminal, then use "python3 ros_control_panel.py", the menu you will then see brings up all of the different nodes required to run the package. The essential ones to run are the Unity Ros2 Stack, Slam Toolbox stack, Nav2 Stack and RVIZ Nav2, this will give you full functionality. Display 1 in unity should be showing blue arrows to let you know you are connected to the endpoint, not red.

##  SLAM / Nav2 Notes

- LIDAR must remain fixed relative to `base_link`.  
- Use RViz to inspect TF (`odom`, `base_link`, `laser`).  

Clear costmaps when noise builds:  

ros2 service call /clear_entire_costmap std_srvs/srv/Empty {}  
ros2 service call /clear_entirely_local_costmap std_srvs/srv/Empty {}  
ros2 service call /clear_entirely_global_costmap std_srvs/srv/Empty {}  


Reset SLAM entirely:  
ros2 topic pub /slam_toolbox/clear_map std_msgs/msg/Empty {}  


##  Notes

- Unity Library/Temp and ROS build/install/log are intentionally ignored via .gitignore to reduce size of repo.  
- Modular design allows easy extension with new sensors, controllers, or planners.  

##  Contributions

Feel free to fork or open issues. This project supports rapid robotics simulation and autonomous UGV research.

Contact jamesknight557@gmail.com for any support or queries

