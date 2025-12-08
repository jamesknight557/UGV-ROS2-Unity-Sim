#!/usr/bin/env python3

import subprocess
import tkinter as tk
from functools import partial

def run_in_terminal(command: str):
    """
    Open a new GNOME Terminal window and run the given command.
    """
    full_cmd = f'gnome-terminal -- bash -c "{command}; exec bash"'
    subprocess.Popen(full_cmd, shell=True)

def run_slam_toolbox():
    cmd1 = (
        "cd ~/ros_tcp_ws && "
        "source install/setup.bash && "
        "ros2 run unity_slam_bringup odom_to_tf_broadcaster"
    )
    cmd2 = (
        "cd ~/ros_tcp_ws && "
        "source install/setup.bash && "
        "ros2 launch unity_slam_bringup unity_slam_bringup.launch.py"
    )

    run_in_terminal(cmd1)
    run_in_terminal(cmd2)

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

endpoint_cmd = (
    "source ~/ros_tcp_ws/install/setup.bash && "
    "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0"
)
tk.Button(root, text="Set up Endpoint",
          command=partial(run_in_terminal, endpoint_cmd),
          **button_opts).pack(pady=5)

tk.Button(root, text="Run RVIZ2",
          command=partial(run_in_terminal, "rviz2"),
          **button_opts).pack(pady=5)

tk.Button(root, text="Run Camera Viewer (rqt_image_view)",
          command=partial(run_in_terminal, "ros2 run rqt_image_view rqt_image_view"),
          **button_opts).pack(pady=5)

teleop_cmd = (
    "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
    "--ros-args -r /cmd_vel:=/cmd_vel"
)
tk.Button(root, text="Teleop Twist Keyboard",
          command=partial(run_in_terminal, teleop_cmd),
          **button_opts).pack(pady=5)

tk.Button(root, text="Echo Rover Pose",
          command=partial(run_in_terminal, "ros2 topic echo /rover/pose"),
          **button_opts).pack(pady=5)

mavros_cmd = (
    "source ~/ros_tcp_ws/install/setup.bash && "
    "python3 ~/ros_tcp_ws/src/mavros_bridge/mavros_bridge/simple_mavlink_bridge.py"
)
tk.Button(root, text="Start MAVROS Bridge",
          command=partial(run_in_terminal, mavros_cmd),
          **button_opts).pack(pady=5)

camera_bridge_cmd = (
    "source ~/ros_tcp_ws/install/setup.bash && "
    "python3 ~/ros_tcp_ws/src/mavros_bridge/mavros_bridge/camera_rtsp_bridge.py"
)
tk.Button(root, text="Start Camera RTSP Bridge",
          command=partial(run_in_terminal, camera_bridge_cmd),
          **button_opts).pack(pady=5)

tk.Button(root, text="Run QGroundControl",
          command=partial(run_in_terminal, "cd ~/Downloads && ./QGroundControl.AppImage"),
          **button_opts).pack(pady=5)

tk.Button(root, text="Run RTSP Server (mediamtx)",
          command=partial(run_in_terminal, "cd ~/Downloads && ./mediamtx"),
          **button_opts).pack(pady=5)

tk.Button(root, text="Run Unity ROS Script",
          command=partial(run_in_terminal, "~/start_unity_ros.sh"),
          **button_opts).pack(pady=5)

tk.Button(root, text="Start SLAM Toolbox (RVIZ stack)",
          command=run_slam_toolbox,
          **button_opts).pack(pady=5)


# Quit button
tk.Button(root, text="Quit", command=root.destroy,
          bg="#FF6666", fg="white", width=20, height=2).pack(pady=15)

root.mainloop()

