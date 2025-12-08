#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from pymavlink import mavutil
import time
import threading
import sys

class SimpleMavlinkBridge(Node):
    def __init__(self, start_lat=51.5074, start_lon=-0.1278):
        super().__init__('simple_mavlink_bridge')
        
        # Create MAVLink connection - use udpout to actively send to QGC
        self.master = mavutil.mavlink_connection(
            'udpout:127.0.0.1:14550',
            source_system=1,
            source_component=1)
        
        # Subscribe to rover pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/rover/pose',
            self.pose_callback,
            10)
        
        # Publish to rover cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Store starting position (can be set via command line)
        self.home_lat = start_lat
        self.home_lon = start_lon
        self.lat = start_lat
        self.lon = start_lon
        self.alt = 0.0
        
        # Boot time for consistent timestamps
        self.boot_time = int(time.time())
        
        # Heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        self.heartbeat_thread.start()
        
        # MAVLink receive thread
        self.receive_thread = threading.Thread(target=self.receive_mavlink, daemon=True)
        self.receive_thread.start()
        
        self.get_logger().info(f'Simple MAVLink Bridge started')
        self.get_logger().info(f'Home Position: Lat={self.home_lat:.6f}, Lon={self.home_lon:.6f}')
        self.get_logger().info(f'Sending to 127.0.0.1:14550')
    
    def send_heartbeat(self):
        """Send heartbeat and other status messages"""
        while True:
            try:
                boot_ms = int((time.time() - self.boot_time) * 1000)
                
                # 1. HEARTBEAT (most important)
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE)
                
                # 2. SYSTEM_STATUS
                self.master.mav.sys_status_send(
                    0, 0, 0,      # sensors
                    500,          # load
                    11000,        # voltage (11V)
                    -1,           # current
                    90,           # battery remaining
                    0, 0, 0, 0, 0, 0)
                
                # 3. Send system time
                self.master.mav.system_time_send(
                    int(time.time() * 1000000),  # Unix time in microseconds
                    boot_ms)
                
                self.get_logger().info('Heartbeat sent', throttle_duration_sec=5.0)
                
            except Exception as e:
                self.get_logger().error(f'Heartbeat error: {e}')
            
            time.sleep(1)
    
    def pose_callback(self, msg):
        """Receive pose from Unity and send to QGC"""
        try:
            # Convert Unity position to GPS offset from home position
            self.lat = self.home_lat + (msg.pose.position.z * 0.00001)
            self.lon = self.home_lon + (msg.pose.position.x * 0.00001)
            self.alt = msg.pose.position.y
            
            boot_ms = int((time.time() - self.boot_time) * 1000)
            
            # Send GPS position
            self.master.mav.global_position_int_send(
                boot_ms,
                int(self.lat * 1e7),
                int(self.lon * 1e7),
                int(self.alt * 1000),
                int(self.alt * 1000),
                0, 0, 0,
                65535)
            
            # Also send local position
            self.master.mav.local_position_ned_send(
                boot_ms,
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(-msg.pose.position.z),
                0, 0, 0)
            
        except Exception as e:
            self.get_logger().error(f'Position send error: {e}')
    
    def receive_mavlink(self):
        """Receive commands from QGC"""
        while True:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1)
                if msg:
                    msg_type = msg.get_type()
                    
                    if msg_type == 'MANUAL_CONTROL':
                        twist = Twist()
                        twist.linear.x = msg.x / 1000.0
                        twist.angular.z = msg.r / 1000.0
                        self.cmd_vel_pub.publish(twist)
                        self.get_logger().info(f'Manual control received', throttle_duration_sec=2.0)
                        
            except Exception as e:
                pass

def main(args=None):
    # Parse command line arguments for starting GPS coordinates
    start_lat = 51.5074  # Default: London
    start_lon = -0.1278
    
    if len(sys.argv) >= 3:
        try:
            start_lat = float(sys.argv[1])
            start_lon = float(sys.argv[2])
            print(f"Using provided coordinates: Lat={start_lat}, Lon={start_lon}")
        except ValueError:
            print("Invalid coordinates provided. Using defaults (London).")
            print("Usage: python3 simple_mavlink_bridge.py [latitude] [longitude]")
    else:
        print("No coordinates provided. Using defaults (London: 51.5074, -0.1278)")
        print("Usage: python3 simple_mavlink_bridge.py [latitude] [longitude]")
    
    rclpy.init(args=args)
    bridge = SimpleMavlinkBridge(start_lat, start_lon)
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()