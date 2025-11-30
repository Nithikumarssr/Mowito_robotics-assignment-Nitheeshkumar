#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        os.makedirs('/tmp/ros2_images', exist_ok=True)
        self.count = 0
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/image_converted',
            self.callback,
            10
        )

        self.get_logger().info("ImageSaver running. Saving to /tmp/ros2_images")

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        filename = f'/tmp/ros2_images/img_{self.count:04d}.png'
        cv2.imwrite(filename, cv_image)
        self.count += 1

        if self.count % 10 == 0:
            self.get_logger().info(f"Saved {self.count} images")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

