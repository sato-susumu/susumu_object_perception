#!/usr/bin/env python3
"""Republish the omni camera image as a compressed JPEG topic."""

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class OmniImageCompressNode(Node):
    def __init__(self):
        super().__init__('omni_image_compress')
        self.bridge = CvBridge()
        self.declare_parameter('input_image', '/omni_camera/image_raw/image_color')
        self.declare_parameter('output_image', '/omni_camera/image_raw/compressed')
        self.declare_parameter('jpeg_quality', 80)

        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.pub = self.create_publisher(
            CompressedImage,
            self.get_parameter('output_image').value,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            self.get_parameter('input_image').value,
            self.on_image,
            qos_profile_sensor_data)
        self.get_logger().info('omni_image_compress started')

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'failed to decode omni image: {exc}')
            return

        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warning('failed to encode omni image')
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = encoded.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OmniImageCompressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
