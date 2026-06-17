#!/usr/bin/env python3
"""Six rectilinear camera images -> equirectangular 360-degree image.

Gazebo Classic and Webots both handle normal camera sensors better than a
single omnidirectional camera. This node treats six 90-degree cameras as a
cube map and publishes a simulator-independent panoramic image.
"""

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


FACES = ('front', 'left', 'back', 'right', 'up', 'down')


class OmniImageNode(Node):
    def __init__(self):
        super().__init__('omni_image')
        self.bridge = CvBridge()

        self.declare_parameter('output_width', 2048)
        self.declare_parameter('output_height', 1024)
        self.declare_parameter('output_topic', '/omni_camera/image_raw')
        self.declare_parameter('rect_topic', '/omni_camera/image_rect')
        self.declare_parameter('camera_info_topic', '/omni_camera/camera_info')
        self.declare_parameter('frame_id', 'omni_camera_link')
        self.declare_parameter('publish_rate', 15.0)
        for face in FACES:
            self.declare_parameter(
                f'{face}_topic', f'/omni_camera/{face}/image_raw')

        self.out_w = int(self.get_parameter('output_width').value)
        self.out_h = int(self.get_parameter('output_height').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.images = {}
        self.maps = None
        self.last_shape = None

        self.pub = self.create_publisher(
            Image, self.get_parameter('output_topic').value,
            qos_profile_sensor_data)
        self.rect_pub = self.create_publisher(
            Image, self.get_parameter('rect_topic').value,
            qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, self.get_parameter('camera_info_topic').value, 10)

        for face in FACES:
            topic = self.get_parameter(f'{face}_topic').value
            self.create_subscription(
                Image, topic,
                lambda msg, f=face: self.on_image(f, msg),
                qos_profile_sensor_data)

        period = 1.0 / max(1.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(period, self.publish)
        self.get_logger().info(
            'omni_image started: six cube faces -> /omni_camera/image_raw')

    def on_image(self, face, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'failed to decode {face}: {exc}')
            return
        self.images[face] = (img, msg.header.stamp)

    def _ensure_maps(self, face_h, face_w):
        shape = (face_h, face_w, self.out_h, self.out_w)
        if self.maps is not None and self.last_shape == shape:
            return
        self.last_shape = shape

        xs = np.linspace(0.0, self.out_w - 1, self.out_w, dtype=np.float32)
        ys = np.linspace(0.0, self.out_h - 1, self.out_h, dtype=np.float32)
        uu, vv = np.meshgrid(xs, ys)
        lon = (uu / self.out_w) * (2.0 * math.pi) - math.pi
        lat = math.pi * 0.5 - (vv / self.out_h) * math.pi
        x = np.cos(lat) * np.cos(lon)
        y = np.cos(lat) * np.sin(lon)
        z = np.sin(lat)

        ax = np.abs(x)
        ay = np.abs(y)
        az = np.abs(z)
        face_idx = np.zeros((self.out_h, self.out_w), dtype=np.uint8)
        sc = np.maximum.reduce([ax, ay, az])

        # Face order: front(+x), left(+y), back(-x), right(-y), up(+z), down(-z).
        face_idx[(ax >= ay) & (ax >= az) & (x >= 0)] = 0
        face_idx[(ay > ax) & (ay >= az) & (y >= 0)] = 1
        face_idx[(ax >= ay) & (ax >= az) & (x < 0)] = 2
        face_idx[(ay > ax) & (ay >= az) & (y < 0)] = 3
        face_idx[(az > ax) & (az > ay) & (z >= 0)] = 4
        face_idx[(az > ax) & (az > ay) & (z < 0)] = 5

        local_u = np.zeros_like(x, dtype=np.float32)
        local_v = np.zeros_like(x, dtype=np.float32)

        m = face_idx == 0
        local_u[m] = -y[m] / sc[m]
        local_v[m] = -z[m] / sc[m]
        m = face_idx == 1
        local_u[m] = x[m] / sc[m]
        local_v[m] = -z[m] / sc[m]
        m = face_idx == 2
        local_u[m] = y[m] / sc[m]
        local_v[m] = -z[m] / sc[m]
        m = face_idx == 3
        local_u[m] = -x[m] / sc[m]
        local_v[m] = -z[m] / sc[m]
        m = face_idx == 4
        local_u[m] = -y[m] / sc[m]
        local_v[m] = x[m] / sc[m]
        m = face_idx == 5
        local_u[m] = -y[m] / sc[m]
        local_v[m] = -x[m] / sc[m]

        map_x = ((local_u + 1.0) * 0.5 * (face_w - 1)).astype(np.float32)
        map_y = ((local_v + 1.0) * 0.5 * (face_h - 1)).astype(np.float32)
        self.maps = (face_idx, map_x, map_y)

    def publish(self):
        if any(face not in self.images for face in FACES):
            return
        face_imgs = {face: self.images[face][0] for face in FACES}
        face_h, face_w = face_imgs['front'].shape[:2]
        self._ensure_maps(face_h, face_w)
        face_idx, map_x, map_y = self.maps

        pano = np.zeros((self.out_h, self.out_w, 3), dtype=np.uint8)
        for idx, face in enumerate(FACES):
            remapped = cv2.remap(
                face_imgs[face], map_x, map_y,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT)
            mask = face_idx == idx
            pano[mask] = remapped[mask]

        stamp = self.images['front'][1]
        msg = self.bridge.cv2_to_imgmsg(pano, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        # The cube faces are already rectilinear; the equirectangular output is
        # the distortion-corrected panoramic representation used downstream.
        rect = self.bridge.cv2_to_imgmsg(pano, encoding='bgr8')
        rect.header = msg.header
        self.rect_pub.publish(rect)

        info = CameraInfo()
        info.header = msg.header
        info.width = self.out_w
        info.height = self.out_h
        info.distortion_model = 'equidistant_equirectangular'
        info.d = [0.0, 0.0, 0.0, 0.0]
        info.k = [self.out_w / (2.0 * math.pi), 0.0, self.out_w / 2.0,
                  0.0, self.out_h / math.pi, self.out_h / 2.0,
                  0.0, 0.0, 1.0]
        info.p = [info.k[0], 0.0, info.k[2], 0.0,
                  0.0, info.k[4], info.k[5], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = OmniImageNode()
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
