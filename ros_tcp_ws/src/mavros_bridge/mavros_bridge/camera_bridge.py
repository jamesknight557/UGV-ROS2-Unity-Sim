#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import socket

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        
        # Subscribe to Unity camera
        self.image_sub = self.create_subscription(
            Image,
            '/unity/camera/image_raw',
            self.image_callback,
            10)
        
        # Create UDP socket for sending video to QGC
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qgc_address = ('127.0.0.1', 5600)
        
        # JPEG encoding parameters
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        
        self.get_logger().info('Camera Bridge started - streaming JPEG to QGC on UDP 5600')
    
    def image_callback(self, msg):
        """Convert ROS image to JPEG and send to QGC"""
        try:
            # Convert ROS Image message to numpy array
            if msg.encoding == 'rgb8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            else:
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = img_array
            
            # Encode as JPEG
            _, jpeg = cv2.imencode('.jpg', bgr_image, self.encode_param)
            
            # Send via UDP (split into chunks if needed)
            data = jpeg.tobytes()
            
            # Send in chunks (max UDP packet size ~65k)
            chunk_size = 60000
            for i in range(0, len(data), chunk_size):
                chunk = data[i:i+chunk_size]
                self.sock.sendto(chunk, self.qgc_address)
            
            self.get_logger().info(f'Sent frame: {len(data)} bytes', throttle_duration_sec=1.0)
            
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
    
    def __del__(self):
        if hasattr(self, 'sock'):
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    bridge = CameraBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()