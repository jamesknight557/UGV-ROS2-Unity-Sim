#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Image, NavSatFix
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
import math

class RoverBridge(Node):
    def __init__(self):
        super().__init__('rover_bridge')
        
        # Subscribe to MAVROS manual control (from QGC)
        self.manual_control_sub = self.create_subscription(
            ManualControl,
            '/mavros/manual_control/control',
            self.manual_control_callback,
            10)
        
        # Subscribe to MAVROS setpoint velocity (from QGC missions)
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            self.velocity_callback,
            10)
        
        # Subscribe to rover pose (from Unity)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/rover/pose',
            self.pose_callback,
            10)
        
        # Publish to rover cmd_vel (to Unity)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publish to MAVROS local position (for QGC display)
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/local_position/pose',
            10)
        
        # Publish MAVROS state
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        
        # Publish fake GPS for QGC map display
        self.gps_pub = self.create_publisher(NavSatFix, '/mavros/global_position/global', 10)
        
        # State timer
        self.state_timer = self.create_timer(0.1, self.publish_state)
        
        # Store current pose
        self.current_pose = None
        
        # Initial "GPS" position (you can change this)
        self.home_lat = 51.5074  # London
        self.home_lon = -0.1278
        
        self.get_logger().info('Rover MAVROS Bridge started')
    
    def manual_control_callback(self, msg):
        """Convert QGC joystick/manual control to cmd_vel"""
        twist = Twist()
        
        # Map manual control axes (-1000 to 1000) to twist commands
        # x axis = forward/backward, r axis = rotation
        twist.linear.x = msg.x / 1000.0  # Normalize to -1 to 1
        twist.angular.z = msg.r / 1000.0
        
        self.cmd_vel_pub.publish(twist)
    
    def velocity_callback(self, msg):
        """Forward velocity commands from MAVROS missions to rover"""
        twist = Twist()
        twist.linear.x = msg.twist.linear.x
        twist.angular.z = msg.twist.angular.z
        
        self.cmd_vel_pub.publish(twist)
    
    def pose_callback(self, msg):
        """Receive pose from Unity and forward to MAVROS"""
        self.current_pose = msg
        
        # Publish to MAVROS local position
        self.local_pos_pub.publish(msg)
        
        # Convert to fake GPS coordinates for map display
        self.publish_fake_gps(msg)
    
    def publish_fake_gps(self, pose):
        """Convert local position to fake GPS for QGC map"""
        gps_msg = NavSatFix()
        gps_msg.header = pose.header
        
        # Simple conversion: 1 meter ≈ 0.00001 degrees (very rough)
        # You can improve this with proper coordinate transformation
        gps_msg.latitude = self.home_lat + (pose.pose.position.z * 0.00001)
        gps_msg.longitude = self.home_lon + (pose.pose.position.x * 0.00001)
        gps_msg.altitude = pose.pose.position.y
        
        gps_msg.status.status = 0  # GPS fix
        gps_msg.status.service = 1
        
        self.gps_pub.publish(gps_msg)
    
    def publish_state(self):
        """Publish MAVROS state for QGC connection"""
        state_msg = State()
        state_msg.connected = True
        state_msg.armed = True
        state_msg.guided = True
        state_msg.mode = "MANUAL"
        state_msg.system_status = 3  # MAV_STATE_STANDBY
        
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = RoverBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()