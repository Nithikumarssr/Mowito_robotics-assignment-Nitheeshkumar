#!/usr/bin/env python3
"""
VQ-BeT Task 2: Image Conversion Node
Subscribes to camera images and converts between color and grayscale modes.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2


class ImageConversionNode(Node):
    """
    Node that converts images between color and grayscale modes.
    
    Subscribes to: /image_raw (or configured input topic)
    Publishes to: /image_converted (or configured output topic)
    Service: /image_conversion/set_mode (SetBool)
    """

    def __init__(self):
        super().__init__('image_conversion')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/image_converted')
        self.declare_parameter('grayscale_mode', True)
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.grayscale_mode = self.get_parameter('grayscale_mode').value
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Create subscription to camera topic
        self.image_subscriber = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        # Create publisher for converted image
        self.image_publisher = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        # Create service for mode switching
        self.service = self.create_service(
            SetBool,
            'image_conversion/set_mode',
            self.set_mode_callback
        )
        
        # Statistics
        self.frame_count = 0
        
        self.get_logger().info(
            f'Image Conversion Node started\n'
            f'  Input topic: {self.input_topic}\n'
            f'  Output topic: {self.output_topic}\n'
            f'  Initial mode: {"Grayscale" if self.grayscale_mode else "Color"}\n'
            f'  Service: /image_conversion/set_mode'
        )

    def image_callback(self, msg: Image):
        """Callback function for image subscription."""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # DEBUG: Print current mode every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processing frame {self.frame_count}, mode={self.grayscale_mode}')
            
            # Apply conversion based on mode
            if self.grayscale_mode:
                # Convert BGR to Grayscale
                converted_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                # Convert back to 3-channel (BGR) for consistent output format
                converted_image = cv2.cvtColor(converted_image, cv2.COLOR_GRAY2BGR)
            else:
                # Keep original color image
                converted_image = cv_image
            
            # Convert back to ROS Image message
            output_msg = self.cv_bridge.cv2_to_imgmsg(converted_image, encoding='bgr8')
            output_msg.header = msg.header
            
            # Publish converted image
            self.image_publisher.publish(output_msg)
            
            self.frame_count += 1
            
            # Log every 30 frames (approximately 1 second at 30 FPS)
            if self.frame_count % 30 == 0:
                mode_str = "Grayscale" if self.grayscale_mode else "Color"
                self.get_logger().info(
                    f'Processed {self.frame_count} frames | Mode: {mode_str}'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def set_mode_callback(self, request: SetBool.Request, 
                         response: SetBool.Response) -> SetBool.Response:
        """Service callback to change image conversion mode."""
        try:
            # DEBUG: Log the request
            self.get_logger().info(f'Service called with data={request.data}')
            
            self.grayscale_mode = request.data
            mode_str = "Grayscale" if self.grayscale_mode else "Color"
            
            response.success = True
            response.message = f'Mode changed to: {mode_str}'
            
            self.get_logger().info(
                f'Mode changed to {mode_str} (frames processed: {self.frame_count})'
            )
            
            return response
        
        except Exception as e:
            response.success = False
            response.message = f'Error changing mode: {str(e)}'
            self.get_logger().error(f'Service error: {str(e)}')
            return response


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    node = ImageConversionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Image Conversion Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
