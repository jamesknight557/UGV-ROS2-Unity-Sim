#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import subprocess
import os
import signal

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        
        # Subscribe to Unity camera
        self.image_sub = self.create_subscription(
            Image,
            '/unity/camera/image_raw',
            self.image_callback,
            10)
        
        # Use opencv to show window and stream via gstreamer
        self.frame_count = 0
        
        # Start gstreamer as separate process
        gst_str = (
        "appsrc ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=800 speed-preset=superfast ! "
        "rtph264pay config-interval=10 pt=96 ! "
        "udpsink host=0.0.0.0 port=5600"
    )
        
        self.writer = cv2.VideoWriter(
            gst_str,
            cv2.CAP_GSTREAMER,
            0, 30, (640, 480), True)
        
        if not self.writer.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline')
        else:
            self.get_logger().info('Camera Bridge started - streaming to QGC on UDP 5600')
    
    def image_callback(self, msg):
        """Convert ROS image and write to video stream"""
        try:
            # Convert ROS Image to numpy array
            if msg.encoding == 'rgb8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            else:
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = img_array
            
            # Write frame
            self.writer.write(bgr_image)
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Streaming frame {self.frame_count}')
            
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
    
    def __del__(self):
        if hasattr(self, 'writer'):
            self.writer.release()

def main(args=None):
    rclpy.init(args=args)
    bridge = CameraBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()	  
