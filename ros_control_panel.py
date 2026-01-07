#!/usr/bin/env python3

import subprocess
import tkinter as tk
from functools import partial
import time



# ---- HC-05 + cmd_vel bridge settings ----
HC05_MAC = "98:D3:21:F8:36:AB"
RFCOMM_DEV = 0
RFCOMM_CH = 1

CMDVEL_NODE_PATH = "~/ugv_sim/ros_tcp_ws/src/ps4_twist/ps4_twist/cmdvel_to_bt_serial.py"
BT_BAUD = 9600
TRACK_WIDTH = 0.32
MAX_LIN = 0.25
MIN_PWM = 110
DEADBAND = 0.0


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
    
def connect_bt():
    """
    1) Start a persistent rfcomm SPP connection to HC-05 (this blocks; keep terminal open)
    2) Start the ROS2 cmd_vel -> serial bridge node in another terminal
    """

    # Terminal A: keep rfcomm connected (you'll be prompted for sudo password)
    rfcomm_cmd = (
        f"sudo rfcomm release {RFCOMM_DEV} || true; "
        f"sudo rfcomm connect {RFCOMM_DEV} {HC05_MAC} {RFCOMM_CH}"
    )
    run_in_terminal(rfcomm_cmd)
    
def run_cmdvel_node():
    # Terminal 2: run the ROS2 python node in a clean ROS env (this is your known-working pattern)
    node_cmd = (
        "bash -lc '"
        "source /opt/ros/humble/setup.bash && "
        "source ~/ugv_sim/ros_tcp_ws/install/setup.bash && "
        f"python3 {CMDVEL_NODE_PATH} "
        "--ros-args "
        f"-p port:=/dev/rfcomm{RFCOMM_DEV} "
        f"-p baud:={BT_BAUD} "
        f"-p track_width:={TRACK_WIDTH} "
        f"-p max_lin:={MAX_LIN} "
        f"-p min_pwm:={MIN_PWM} "
        f"-p deadband:={DEADBAND}"
        "'"
    )
    run_in_terminal(node_cmd)



# ----------------- GUI SETUP -----------------

root = tk.Tk()
root.title("ROS / UGV Control Panel")
root.configure(bg="#ADD8E6")  # light blue background
root.geometry("500x600")

button_opts = {
    "width": 40,
    "height": 1,
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
tk.Button(root, text="Start Nav2",
          command=partial(run_in_terminal, "ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false"),
          **button_opts).pack(pady=5)

# Nav2 + SLAM combo
tk.Button(root, text="Start SLAM Toolbox (RVIZ stack)",
          command=run_slam_toolbox,
          **button_opts).pack(pady=5)
          
# controller 1
tk.Button(root, text="Controller 1",
	  command=partial(run_in_terminal, "ros2 run joy joy_node"),
          **button_opts).pack(pady=5)

# controller 2
tk.Button(root, text="Controller 2",
	  command=partial(run_in_terminal, "cd ~/ugv_sim/ros_tcp_ws/src/ps4_twist/ps4_twist && python3 ps4_to_twist.py"),
          **button_opts).pack(pady=5)
          
# robot Camera
tk.Button(root, text="Robot Camera",
	  command=partial(run_in_terminal, "cd ~/ugv_sim/ros_tcp_ws/src/ps4_twist/ps4_twist && python3 CameraStream.py"),
          **button_opts).pack(pady=5)


#Robot button 1
tk.Button(root, text="Robot BT Bridge",
          command=connect_bt,
          **button_opts).pack(pady=5) 
          
#Robot button 1
tk.Button(root, text="Robot BT cmd_vel Node",
          command=run_cmdvel_node,
          **button_opts).pack(pady=5)  
                  
# Quit button
tk.Button(root, text="Quit", command=root.destroy,
          bg="#FF6666", fg="white", width=20, height=2).pack(pady=15)
          



root.mainloop()

