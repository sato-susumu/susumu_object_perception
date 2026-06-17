#!/usr/bin/env python3
"""Publish CameraInfo metadata for the Webots equirectangular omni image."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image


class EquirectCameraInfoNode(Node):
    def __init__(self):
        super().__init__('equirect_camera_info')
        self.declare_parameter('input_image', '/omni_camera/image_raw/image_color')
        self.declare_parameter('output_camera_info', '/omni_camera/equirect/camera_info')
        self.declare_parameter('camera_frame', 'omni_camera_link')

        self.camera_frame = self.get_parameter('camera_frame').value
        self.pub = self.create_publisher(
            CameraInfo,
            self.get_parameter('output_camera_info').value,
            qos_profile_sensor_data)
        self.create_subscription(
            Image,
            self.get_parameter('input_image').value,
            self.on_image,
            qos_profile_sensor_data)
        self.get_logger().info('equirect_camera_info started')

    def on_image(self, msg):
        info = CameraInfo()
        info.header = msg.header
        info.header.frame_id = self.camera_frame
        info.width = msg.width
        info.height = msg.height
        # ROS CameraInfo has no standard equirectangular model. Keep the
        # explicit string for tools/users that inspect it, and pass
        # --camera_model equirectangular to calibration tools that support it.
        info.distortion_model = 'equirectangular'
        info.d = []
        info.k = [
            float(msg.width), 0.0, float(msg.width) * 0.5,
            0.0, float(msg.height), float(msg.height) * 0.5,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            float(msg.width), 0.0, float(msg.width) * 0.5, 0.0,
            0.0, float(msg.height), float(msg.height) * 0.5, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = EquirectCameraInfoNode()
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
