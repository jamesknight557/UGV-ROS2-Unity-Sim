#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import numpy as np
import subprocess
import threading


class CameraRTSPBridge(Node):
    def __init__(self):
        super().__init__('camera_rtsp_bridge')

        # ROS image subscription (Unity camera)
        self.image_sub = self.create_subscription(
            Image,
            '/unity/camera/image_raw',
            self.image_callback,
            10
        )

        # RTSP target (MediaMTX)
        self.rtsp_url = "rtsp://localhost:8554/mystream"

        # ffmpeg process and lock for thread safety
        self.ffmpeg_proc = None
        self.ffmpeg_lock = threading.Lock()

        self.get_logger().info(
            f"Camera RTSP Bridge initialised, will stream to {self.rtsp_url} "
            "(ffmpeg-based, no GStreamer)"
        )

    def start_ffmpeg(self, width: int, height: int):
        """
        Start an ffmpeg process that reads raw BGR frames from stdin
        and pushes them as H.264 over RTSP to MediaMTX.
        """
        with self.ffmpeg_lock:
            if self.ffmpeg_proc is not None:
                return  # already running

            self.get_logger().info(
                f"Starting ffmpeg for RTSP streaming to {self.rtsp_url} "
                f"({width}x{height}, BGR rawvideo)"
            )

            cmd = [
                "ffmpeg",
                "-re",                 # read input at native rate
                "-f", "rawvideo",
                "-pix_fmt", "bgr24",
                "-s", f"{width}x{height}",
                "-i", "pipe:0",        # read from stdin
                "-c:v", "libx264",
                "-preset", "veryfast",
                "-tune", "zerolatency",
                "-f", "rtsp",
                self.rtsp_url,
            ]

            try:
                self.ffmpeg_proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.get_logger().info("ffmpeg process started successfully")
            except Exception as e:
                self.ffmpeg_proc = None
                self.get_logger().error(f"Failed to start ffmpeg: {e}")

    def image_callback(self, msg: Image):
        """
        Convert ROS Image to BGR8 numpy array and feed it to ffmpeg.
        On first frame, start ffmpeg using the frame dimensions.
        """
        try:
            # Convert ROS image to BGR
            # Assume either rgb8 or bgr8-like 3-channel encodings
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )

            if msg.encoding == 'rgb8':
                bgr_image = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
                bgr_image = cv2.rotate(bgr_image, cv2.ROTATE_180)
            else:
                bgr_image = img_array
                bgr_image = cv2.rotate(bgr_image, cv2.ROTATE_180)


            # Lazily start ffmpeg when we know width/height
            if self.ffmpeg_proc is None:
                self.start_ffmpeg(msg.width, msg.height)

            # If ffmpeg is running, write frame
            with self.ffmpeg_lock:
                if self.ffmpeg_proc is not None and self.ffmpeg_proc.stdin:
                    try:
                        self.ffmpeg_proc.stdin.write(bgr_image.tobytes())
                    except BrokenPipeError:
                        # ffmpeg died – log once and stop writing
                        self.get_logger().error("ffmpeg pipe broken, stopping stream")
                        self.ffmpeg_proc = None

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def destroy_node(self):
        # Clean up ffmpeg process on node shutdown
        with self.ffmpeg_lock:
            if self.ffmpeg_proc is not None:
                self.get_logger().info("Terminating ffmpeg process")
                try:
                    self.ffmpeg_proc.stdin.close()
                except Exception:
                    pass
                self.ffmpeg_proc.terminate()
                self.ffmpeg_proc = None
        super().destroy_node()


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

