#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraRTSPBridge(Node):
    def __init__(self):
        super().__init__('camera_rtsp_bridge')
        
        # Subscribe to Unity camera
        self.image_sub = self.create_subscription(
            Image,
            '/unity/camera/image_raw',
            self.image_callback,
            10)
        
        # Stream to RTSP server
        rtsp_url = "rtsp://localhost:8554/mystream"
        
        gst_str = (
            f"appsrc ! "
            "videoconvert ! "
            "x264enc tune=zerolatency bitrate=800 ! "
            "rtspclientsink location=" + rtsp_url
        )
        
        self.writer = cv2.VideoWriter(
            gst_str,
            cv2.CAP_GSTREAMER,
            0, 30, (640, 480), True)
        
        if not self.writer.isOpened():
            self.get_logger().error('Failed to open GStreamer RTSP pipeline')
        else:
            self.get_logger().info(f'Camera RTSP Bridge started - streaming to {rtsp_url}')
    
    def image_callback(self, msg):
        """Convert ROS image and write to RTSP stream"""
        try:
            if msg.encoding == 'rgb8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            else:
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = img_array
            
            self.writer.write(bgr_image)
            
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
    
    def __del__(self):
        if hasattr(self, 'writer'):
            self.writer.release()

def main(args=None):
    rclpy.init(args=args)
    bridge = CameraRTSPBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
