#!/usr/bin/env python3

import subprocess
import tkinter as tk
from functools import partial
import time

def run_in_terminal(command: str):
    """
    Open a new GNOME Terminal window and run the given command.
    """
    full_cmd = f'gnome-terminal -- bash -c "{command}; exec bash"'
    subprocess.Popen(full_cmd, shell=True)

# Common prefix for all ROS workspace commands
ROS_WS_PREFIX = "cd ~/ugv_sim/ros_tcp_ws && source install/setup.bash && "

def ros_ws_cmd(cmd: str) -> str:
    """
    Helper to build commands that should run from within the ROS2 workspace.
    """
    return ROS_WS_PREFIX + cmd

def run_slam_toolbox():
    cmd1 = ros_ws_cmd(
        "ros2 run unity_slam_bringup odom_to_tf_broadcaster"
    )
    cmd2 = ros_ws_cmd(
        "ros2 launch unity_slam_bringup unity_slam_bringup.launch.py"
    )

    run_in_terminal(cmd1)
    run_in_terminal(cmd2)

def run_unity_ros_stack():


    #ask for lat and long here!

    # 1) ROS TCP Endpoint
    endpoint_cmd = ros_ws_cmd(
        "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0"
    )
    run_in_terminal(endpoint_cmd)
	
    time.sleep(2)

    # 2) MediaMTX server (in Downloads)
    mediamtx_cmd = "cd ~/Downloads && ./mediamtx"
    run_in_terminal(mediamtx_cmd)

    time.sleep(2)
    # 3) MAVLink bridge
    mavros_cmd = ros_ws_cmd(
        "python3 src/mavros_bridge/mavros_bridge/simple_mavlink_bridge.py"
    )
    run_in_terminal(mavros_cmd)
    
    time.sleep(2)

    # 4) Camera RTSP bridge
    camera_bridge_cmd = ros_ws_cmd(
        "python3 src/mavros_bridge/mavros_bridge/camera_rtsp_bridge.py"
    )
    run_in_terminal(camera_bridge_cmd)
    
    time.sleep(2)

# ----------------- GUI SETUP -----------------

root = tk.Tk()
root.title("ROS / UGV Control Panel")
root.configure(bg="#ADD8E6")  # light blue background
root.geometry("500x600")

button_opts = {
    "width": 40,
    "height": 2,
    "bg": "white",
    "fg": "black",
}

# ---------------- BUTTONS --------------------

# Endpoint
endpoint_cmd_button = ros_ws_cmd(
    "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0"
)
tk.Button(root, text="Set up Endpoint",
          command=partial(run_in_terminal, endpoint_cmd_button),
          **button_opts).pack(pady=5)

# RVIZ2
rviz_cmd = ros_ws_cmd("rviz2")
tk.Button(root, text="Run RVIZ2",
          command=partial(run_in_terminal, rviz_cmd),
          **button_opts).pack(pady=5)

# RVIZ2 Nav2
rviz_nav2_cmd = ros_ws_cmd("ros2 launch nav2_bringup rviz_launch.py")
tk.Button(root, text="Run RVIZ2 with Nav2",
          command=partial(run_in_terminal, rviz_nav2_cmd),
          **button_opts).pack(pady=5)

# Camera viewer
camera_viewer_cmd = ros_ws_cmd("ros2 run rqt_image_view rqt_image_view")
tk.Button(root, text="Run Camera Viewer (rqt_image_view)",
          command=partial(run_in_terminal, camera_viewer_cmd),
          **button_opts).pack(pady=5)

# Teleop
teleop_cmd = ros_ws_cmd(
    "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
    "--ros-args -r /cmd_vel:=/cmd_vel"
)
tk.Button(root, text="Teleop Twist Keyboard",
          command=partial(run_in_terminal, teleop_cmd),
          **button_opts).pack(pady=5)

# Echo pose
pose_cmd = ros_ws_cmd("ros2 topic echo /rover/pose")
tk.Button(root, text="Echo Rover Pose",
          command=partial(run_in_terminal, pose_cmd),
          **button_opts).pack(pady=5)

# MAVROS bridge
mavros_cmd_button = ros_ws_cmd(
    "python3 src/mavros_bridge/mavros_bridge/simple_mavlink_bridge.py"
)
tk.Button(root, text="Start MAVROS Bridge",
          command=partial(run_in_terminal, mavros_cmd_button),
          **button_opts).pack(pady=5)

# Camera RTSP bridge
camera_bridge_cmd_button = ros_ws_cmd(
    "python3 src/mavros_bridge/mavros_bridge/camera_rtsp_bridge.py"
)
tk.Button(root, text="Start Camera RTSP Bridge",
          command=partial(run_in_terminal, camera_bridge_cmd_button),
          **button_opts).pack(pady=5)

# QGroundControl (still in Downloads)
tk.Button(root, text="Run QGroundControl",
          command=partial(run_in_terminal, "cd ~/Downloads && ./QGroundControl.AppImage"),
          **button_opts).pack(pady=5)

# RTSP server (mediamtx, still in Downloads)
tk.Button(root, text="Run RTSP Server (mediamtx)",
          command=partial(run_in_terminal, "cd ~/Downloads && ./mediamtx"),
          **button_opts).pack(pady=5)

# Unity ROS startup stack
tk.Button(root, text="Run Unity QGC Stack",
          command=run_unity_ros_stack,
          **button_opts).pack(pady=5)

# SLAM toolbox stack (Nav2 bringup)
tk.Button(root, text="Start SLAM Toolbox (RVIZ stack)",
          command=partial(run_in_terminal, "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false"),
          **button_opts).pack(pady=5)

# Nav2 + SLAM combo
tk.Button(root, text="Start Nav2",
          command=run_slam_toolbox,
          **button_opts).pack(pady=5)

# Quit button
tk.Button(root, text="Quit", command=root.destroy,
          bg="#FF6666", fg="white", width=20, height=2).pack(pady=15)

root.mainloop()

