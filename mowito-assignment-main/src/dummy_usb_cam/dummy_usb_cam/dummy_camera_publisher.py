#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DummyCameraPublisher(Node):
    def __init__(self):
        super().__init__('dummy_camera_publisher')
        
        # Publisher on /image_raw topic
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # Timer for 30Hz publishing
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Try to load a video or image from disk
        # If no video/image, create synthetic colored image
        self.cap = cv2.VideoCapture(0)  # Try webcam first
        
        if not self.cap.isOpened():
            self.get_logger().info('No camera found. Using synthetic image.')
            self.use_synthetic = True
            self.frame_count = 0
        else:
            self.use_synthetic = False
            self.get_logger().info('Camera opened successfully.')
        
        self.get_logger().info('Dummy Camera Publisher started on /image_raw at 30Hz')

    def timer_callback(self):
        if self.use_synthetic:
            # Create a synthetic colored image (640x480)
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            # Add some color variation based on frame count
            img[:, :, 0] = (self.frame_count * 2) % 255  # Blue channel
            img[:, :, 1] = (self.frame_count * 3) % 255  # Green channel
            img[:, :, 2] = (self.frame_count * 5) % 255  # Red channel
            self.frame_count += 1
            
            # Add text
            cv2.putText(img, f'Frame: {self.frame_count}', (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        else:
            # Read from camera
            ret, img = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to capture frame')
                return
        
        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_frame'
        
        # Publish
        self.publisher_.publish(ros_image)

    def destroy_node(self):
        if not self.use_synthetic:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

