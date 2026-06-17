#!/usr/bin/env python3
"""Create rectified perspective crops for LiDAR-detected objects."""

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

from susumu_object_perception.colorized_pointcloud_node import (
    WEBOTS_CYLINDRICAL_ROT, euler_xyz_to_matrix, quat_to_matrix)


class ObjectImageCropNode(Node):
    def __init__(self):
        super().__init__('object_image_crop')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('input_image', '/omni_camera/image_raw/image_color')
        self.declare_parameter('input_objects', '/perception/tracked_objects')
        self.declare_parameter('object_type', 'tracked')  # tracked or detected
        self.declare_parameter('output_image', '/perception/object_crops/image_rect')
        self.declare_parameter('camera_frame', 'omni_camera_link')
        self.declare_parameter('crop_width', 320)
        self.declare_parameter('crop_height', 240)
        self.declare_parameter('crop_fov_deg', 55.0)
        self.declare_parameter('max_objects', 8)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('pitch_offset_deg', 0.0)
        self.declare_parameter('calibration_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('projection_model', 'webots_cylindrical')

        self.camera_frame = self.get_parameter('camera_frame').value
        self.crop_w = int(self.get_parameter('crop_width').value)
        self.crop_h = int(self.get_parameter('crop_height').value)
        self.crop_fov = math.radians(float(self.get_parameter('crop_fov_deg').value))
        self.max_objects = int(self.get_parameter('max_objects').value)
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
            Image, self.get_parameter('output_image').value,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('input_image').value,
            self.on_image, qos_profile_sensor_data)
        object_type = self.get_parameter('object_type').value
        msg_type = TrackedObjects if object_type == 'tracked' else DetectedObjects
        self.create_subscription(
            msg_type, self.get_parameter('input_objects').value,
            self.on_objects, 10)
        self.get_logger().info('object_image_crop started')

    def on_image(self, msg):
        try:
            self.latest_image = (
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'),
                msg.header.stamp)
        except Exception as exc:
            self.get_logger().warning(f'failed to decode omni image: {exc}')

    def _pose_from_object(self, obj):
        return obj.kinematics.pose_with_covariance.pose

    def _object_label(self, obj, index):
        if hasattr(obj, 'object_id'):
            try:
                return str(obj.object_id.uuid[:4].hex())
            except Exception:
                return f'id{index}'
        return f'obj{index}'

    def _transform_point(self, point, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, source_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warning(
                f'no transform {self.camera_frame} <- {source_frame}: {exc}')
            return None
        rot = quat_to_matrix(tf.transform.rotation)
        trans = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=np.float32)
        p = np.array([point.x, point.y, point.z], dtype=np.float32)
        return self.calibration_rot @ (rot @ p + trans)

    def _perspective_crop(self, pano, direction):
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return None
        forward = direction / norm
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        if abs(float(np.dot(forward, world_up))) > 0.95:
            world_up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        right = np.cross(forward, world_up)
        right /= max(np.linalg.norm(right), 1e-6)
        up = np.cross(right, forward)
        up /= max(np.linalg.norm(up), 1e-6)

        xs = np.linspace(-1.0, 1.0, self.crop_w, dtype=np.float32)
        ys = np.linspace(-1.0, 1.0, self.crop_h, dtype=np.float32)
        xx, yy = np.meshgrid(xs, ys)
        tan_half = math.tan(self.crop_fov / 2.0)
        aspect = self.crop_w / float(self.crop_h)
        dirs = (forward.reshape(1, 1, 3) +
                right.reshape(1, 1, 3) * (xx[..., None] * tan_half * aspect) +
                up.reshape(1, 1, 3) * (-yy[..., None] * tan_half))
        dirs /= np.maximum(np.linalg.norm(dirs, axis=2, keepdims=True), 1e-6)

        h, w = pano.shape[:2]
        if self.projection_model == 'webots_cylindrical':
            dirs_proj = dirs @ WEBOTS_CYLINDRICAL_ROT.T
            yaw = np.arctan2(dirs_proj[:, :, 1], dirs_proj[:, :, 0]) + self.yaw_offset
            z_unit = np.clip(dirs_proj[:, :, 2], -1.0, 1.0)
            v_angle = np.arccos(z_unit) - math.pi / 2.0
            map_x = ((0.5 - yaw / (2.0 * math.pi)) * w % w).astype(np.float32)
            map_y = ((0.5 + (v_angle + self.pitch_offset) / math.pi) * h).astype(np.float32)
        else:
            yaw = np.arctan2(-dirs[:, :, 1], dirs[:, :, 0]) + self.yaw_offset
            pitch = np.arcsin(np.clip(dirs[:, :, 2], -1.0, 1.0)) + self.pitch_offset
            pitch = np.clip(pitch, -math.pi / 2.0, math.pi / 2.0)
            map_x = (((yaw + math.pi) / (2.0 * math.pi) * w) % w).astype(np.float32)
            map_y = ((math.pi / 2.0 + pitch) / math.pi * h).astype(np.float32)
        return cv2.remap(
            pano, map_x, map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_WRAP)

    def on_objects(self, msg):
        if self.latest_image is None or not msg.objects:
            return
        pano, stamp = self.latest_image
        crops = []
        for i, obj in enumerate(msg.objects[:self.max_objects]):
            pose = self._pose_from_object(obj)
            p_cam = self._transform_point(pose.position, msg.header.frame_id)
            if p_cam is None:
                return
            crop = self._perspective_crop(pano, p_cam)
            if crop is None:
                continue
            label = self._object_label(obj, i)
            distance = float(np.linalg.norm(p_cam))
            cv2.putText(
                crop, f'{label} {distance:.1f}m', (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
                cv2.LINE_AA)
            crops.append(crop)
        if not crops:
            return

        cols = min(4, len(crops))
        rows = int(math.ceil(len(crops) / cols))
        mosaic = np.zeros((rows * self.crop_h, cols * self.crop_w, 3),
                          dtype=np.uint8)
        for i, crop in enumerate(crops):
            r = i // cols
            c = i % cols
            mosaic[r * self.crop_h:(r + 1) * self.crop_h,
                   c * self.crop_w:(c + 1) * self.crop_w] = crop

        out = self.bridge.cv2_to_imgmsg(mosaic, encoding='bgr8')
        out.header.stamp = stamp
        out.header.frame_id = self.camera_frame
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectImageCropNode()
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
