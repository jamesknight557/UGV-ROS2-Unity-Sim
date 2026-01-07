#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# >>> CHANGE THIS to your Pi's IP <<<
STREAM_URL = "http://192.168.1.243:8080/"  # example


class HttpCameraNode(Node):
    def __init__(self):
        super().__init__("http_camera_node")
        self.bridge = CvBridge()

        self.declare_parameter("stream_url", STREAM_URL)
        self.stream_url = self.get_parameter("stream_url").get_parameter_value().string_value

        self.get_logger().info(f"Opening video stream: {self.stream_url}")
        # Open MJPEG-over-HTTP stream
        self.cap = cv2.VideoCapture(self.stream_url)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video stream.")
            raise SystemExit(1)

        # Publisher
        self.pub = self.create_publisher(Image, "camera/image_raw1", 10)

        # 10 Hz (adjust as needed)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame from stream")
            return

        # Convert OpenCV BGR → ROS Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HttpCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

