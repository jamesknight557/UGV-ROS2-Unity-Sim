#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import subprocess

class CameraFFmpegBridge(Node):
    def __init__(self):
        super().__init__('camera_ffmpeg_bridge')
        
        # Subscribe to Unity camera
        self.image_sub = self.create_subscription(
            Image,
            '/unity/camera/image_raw',
            self.image_callback,
            10)
        
        # FFmpeg command to stream to RTSP
        ffmpeg_cmd = [
            'ffmpeg',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', '640x480',
            '-r', '30',
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-b:v', '800k',
            '-f', 'rtsp',
            'rtsp://localhost:8554/mystream'
        ]
        
        self.ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        
        # Noise parameters
        self.noise_strength = 5  # Adjust 0-50 (higher = more noise)
        self.frame_count = 0
        
        self.get_logger().info('Camera FFmpeg Bridge started - streaming to rtsp://localhost:8554/mystream')
        self.get_logger().info(f'Video noise enabled (strength={self.noise_strength})')
    
    def add_realistic_noise(self, image):
        """Add realistic video noise and artifacts"""
        
        # 1. Gaussian noise (sensor noise)
        gaussian_noise = np.random.normal(0, self.noise_strength, image.shape).astype(np.int16)
        noisy_image = np.clip(image.astype(np.int16) + gaussian_noise, 0, 255).astype(np.uint8)
        
        # 2. Salt and pepper noise (occasional random pixels)
        if np.random.random() < 0.3:  # 30% chance per frame
            salt_pepper = np.random.random(image.shape[:2])
            noisy_image[salt_pepper < 0.001] = 255  # White specks
            noisy_image[salt_pepper > 0.999] = 0    # Black specks
        
        # 4. Color shift (simulating signal interference)
        if np.random.random() < 0.1:  # 10% chance
            shift = np.random.randint(-5, 5)
            noisy_image[:, :, np.random.randint(0, 3)] = np.clip(
                noisy_image[:, :, np.random.randint(0, 3)].astype(np.int16) + shift, 0, 255
            ).astype(np.uint8)
        
        # 5. Scanline artifacts (horizontal lines, like old analog video)
        if np.random.random() < 0.05:  # 5% chance
            line_y = np.random.randint(0, image.shape[0])
            noisy_image[line_y:line_y+2, :] = noisy_image[line_y:line_y+2, :] * 0.7
        
        self.frame_count += 1
        return noisy_image
    
    def image_callback(self, msg):
        """Convert ROS image and pipe to FFmpeg"""
        try:
            if msg.encoding == 'rgb8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            else:
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr_image = img_array

            # Rotate 180
            bgr_image = bgr_image[::-1, ::-1]
            
            # Add realistic video noise
            bgr_image = self.add_realistic_noise(bgr_image)
            
            # Write to FFmpeg stdin
            self.ffmpeg_process.stdin.write(bgr_image.tobytes())
            
        except BrokenPipeError:
            self.get_logger().error('FFmpeg process died')
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
    
    def __del__(self):
        if hasattr(self, 'ffmpeg_process'):
            self.ffmpeg_process.terminate()

def main(args=None):
    rclpy.init(args=args)
    bridge = CameraFFmpegBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()