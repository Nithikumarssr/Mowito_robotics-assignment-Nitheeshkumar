#!/usr/bin/env python3
"""
Auto Image Saver for VQ-BeT Task 2
Saves 10 grayscale + 10 color images automatically
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class AutoImageSaver(Node):
    def __init__(self):
        super().__init__('auto_image_saver')
        
        # Parameters
        self.declare_parameter('output_dir', '/tmp/ros2_images_auto')
        self.declare_parameter('images_per_mode', 10)
        
        self.output_dir = self.get_parameter('output_dir').value
        self.images_per_mode = self.get_parameter('images_per_mode').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f'Saving images to: {self.output_dir}')
        
        # State tracking
        self.image_count = 0
        self.mode = True  # Start with grayscale
        self.mode_switched = False
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscription to converted images
        self.subscription = self.create_subscription(
            Image,
            '/image_converted',
            self.image_callback,
            10
        )
        
        # Service client to control mode
        self.mode_client = self.create_client(SetBool, '/image_conversion/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /image_conversion/set_mode service...')
        
        self.get_logger().info('Auto Image Saver initialized. Saving first 10 grayscale images...')

    def image_callback(self, msg):
        """Save images and auto-switch modes"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Create filename with mode indicator
            mode_str = "GRAY" if self.mode else "COLOR"
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = f'img_{self.image_count:03d}_{mode_str}_{timestamp}.png'
            filepath = os.path.join(self.output_dir, filename)
            
            # Save image
            cv2.imwrite(filepath, cv_image)
            
            self.get_logger().info(f'Saved: {filename} ({os.path.getsize(filepath)} bytes)')
            self.image_count += 1
            
            # Check if we need to switch modes
            if self.image_count == self.images_per_mode and not self.mode_switched:
                self.switch_to_color()
            
            # Stop after collecting all images
            elif self.image_count >= self.images_per_mode * 2:
                self.get_logger().info('✓ Saved all images! 10 grayscale + 10 color')
                self.get_logger().info(f'Images saved to: {self.output_dir}')
                self.print_summary()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error saving image: {str(e)}')

    def switch_to_color(self):
        """Switch from grayscale to color mode"""
        self.get_logger().info('--- Switching to COLOR mode ---')
        
        request = SetBool.Request()
        request.data = False
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.mode_switch_callback)
        self.mode_switched = True

    def mode_switch_callback(self, future):
        """Handle service response"""
        try:
            response = future.result()
            if response.success:
                self.mode = False  # Now in color mode
                self.get_logger().info('✓ Successfully switched to COLOR mode')
                self.get_logger().info('Saving next 10 color images...')
            else:
                self.get_logger().error(f'Failed to switch mode: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def print_summary(self):
        """Print summary of saved images"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('IMAGE COLLECTION SUMMARY')
        self.get_logger().info('='*60)
        
        # Count files
        files = os.listdir(self.output_dir)
        gray_files = [f for f in files if 'GRAY' in f]
        color_files = [f for f in files if 'COLOR' in f]
        
        self.get_logger().info(f'Grayscale images: {len(gray_files)}')
        self.get_logger().info(f'Color images: {len(color_files)}')
        self.get_logger().info(f'Total: {len(files)}')
        self.get_logger()..info(f'Location: {self.output_dir}')
        self.get_logger().info('='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = AutoImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Auto Image Saver')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
