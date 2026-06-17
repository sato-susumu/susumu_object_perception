#!/usr/bin/env python3
"""Colorize LiDAR points from an omnidirectional equirectangular camera image."""

import math
import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


FIELDS_XYZRGB = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ], dtype=np.float32)


def euler_xyz_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    return rz @ ry @ rx


WEBOTS_CYLINDRICAL_ROT = np.array([
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
], dtype=np.float32)


class ColorizedPointCloudNode(Node):
    def __init__(self):
        super().__init__('colorized_pointcloud')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('input_cloud', '/velodyne_points/point_cloud')
        self.declare_parameter('input_image', '/omni_camera/image_raw/image_color')
        self.declare_parameter('output_cloud', '/perception/colorized_points')
        self.declare_parameter('camera_frame', 'omni_camera_link')
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('pitch_offset_deg', 0.0)
        self.declare_parameter('calibration_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('projection_model', 'webots_cylindrical')

        self.camera_frame = self.get_parameter('camera_frame').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.yaw_offset = math.radians(float(
            self.get_parameter('yaw_offset_deg').value))
        self.pitch_offset = math.radians(float(
            self.get_parameter('pitch_offset_deg').value))
        self.projection_model = self.get_parameter('projection_model').value
        rpy = [float(v) for v in self.get_parameter('calibration_rpy_deg').value]
        self.calibration_rot = euler_xyz_to_matrix(
            math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2]))
        self.latest_image = None

        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_cloud').value,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('input_image').value,
            self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_cloud').value,
            self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info('colorized_pointcloud started')

    def on_image(self, msg):
        try:
            self.latest_image = (
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'),
                msg.header.stamp)
        except Exception as exc:
            self.get_logger().warning(f'failed to decode omni image: {exc}')

    def _project(self, pts_cam, width, height):
        if self.projection_model == 'webots_cylindrical':
            pts_proj = pts_cam @ WEBOTS_CYLINDRICAL_ROT.T
            x = pts_proj[:, 0]
            y = pts_proj[:, 1]
            z = pts_proj[:, 2]
            r = np.sqrt(x * x + y * y + z * z)
            valid = r > 1e-6
            yaw = np.arctan2(y, x) + self.yaw_offset
            z_unit = np.clip(z / np.maximum(r, 1e-6), -1.0, 1.0)
            # Inverse of Webots resources/wren/shaders/merge_spherical.frag
            # for cylindrical projection with fovX=2*pi and fovY=pi.
            v_angle = np.arccos(z_unit) - math.pi / 2.0
            pitch_offset = self.pitch_offset
            u = (0.5 - yaw / (2.0 * math.pi)) * width
            v = (0.5 + (v_angle + pitch_offset) / math.pi) * height
            valid &= (v >= 0.0) & (v < height)
            return (u % width).astype(np.int32), v.astype(np.int32), valid

        x = pts_cam[:, 0]
        y = pts_cam[:, 1]
        z = pts_cam[:, 2]
        r = np.sqrt(x * x + y * y + z * z)
        valid = r > 1e-6
        # Webots cylindrical camera image increases horizontally toward -Y in
        # the robot/LiDAR frame. Use atan2(-y, x) so +X is image center,
        # +Y is left of center, and -Y is right of center.
        yaw = np.arctan2(-y, x) + self.yaw_offset
        pitch = np.arcsin(np.clip(z / np.maximum(r, 1e-6), -1.0, 1.0))
        pitch = np.clip(pitch + self.pitch_offset, -math.pi / 2, math.pi / 2)
        u = ((yaw + math.pi) / (2.0 * math.pi) * width) % width
        # Webots cylindrical projection puts points below the camera in the
        # upper half of the image for this mounted orientation.
        v = (math.pi / 2.0 + pitch) / math.pi * height
        valid &= (v >= 0.0) & (v < height)
        return u.astype(np.int32), v.astype(np.int32), valid

    def on_cloud(self, msg):
        if self.latest_image is None:
            return
        image, _ = self.latest_image
        h, w = image.shape[:2]

        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        pts = pts.astype(np.float32)
        ranges = np.linalg.norm(pts, axis=1)
        keep = ranges <= self.max_range
        pts = pts[keep]
        if pts.shape[0] == 0:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warning(
                f'no transform {self.camera_frame} <- {msg.header.frame_id}: {exc}')
            return

        rot = quat_to_matrix(tf.transform.rotation)
        trans = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=np.float32)
        pts_cam = pts @ rot.T + trans
        pts_cam = pts_cam @ self.calibration_rot.T
        u, v, valid = self._project(pts_cam, w, h)

        colors = np.zeros((pts.shape[0], 3), dtype=np.uint8)
        colors[valid] = image[v[valid], u[valid]]
        b = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        r = colors[:, 2].astype(np.uint32)
        rgb_u32 = (r << 16) | (g << 8) | b
        rgb_f32 = rgb_u32.view(np.float32)

        structured = np.zeros(pts.shape[0], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('rgb', np.float32),
        ])
        structured['x'] = pts[:, 0]
        structured['y'] = pts[:, 1]
        structured['z'] = pts[:, 2]
        structured['rgb'] = rgb_f32

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = structured.shape[0]
        out.fields = FIELDS_XYZRGB
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.data = structured.tobytes()
        out.is_dense = True
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ColorizedPointCloudNode()
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
