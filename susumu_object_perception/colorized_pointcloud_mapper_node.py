#!/usr/bin/env python3
"""Accumulate colorized point clouds in a SLAM/world frame."""

from pathlib import Path
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from susumu_object_perception.omni_projection import quat_to_matrix


FIELDS_XYZRGB = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


def split_rgb(rgb_float):
    rgb_u = rgb_float.astype(np.float32).view(np.uint32)
    r = ((rgb_u >> 16) & 255).astype(np.float32)
    g = ((rgb_u >> 8) & 255).astype(np.float32)
    b = (rgb_u & 255).astype(np.float32)
    return np.stack([r, g, b], axis=1)


def pack_rgb(rgb):
    rgb = np.clip(rgb, 0.0, 255.0).astype(np.uint32)
    rgb_u = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    return rgb_u.view(np.float32)


class ColorizedPointCloudMapperNode(Node):
    def __init__(self):
        super().__init__('colorized_pointcloud_mapper')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('input_cloud', '/perception/colorized_points')
        self.declare_parameter('output_cloud', '/slam/colorized_points_map')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('fallback_frame', 'odom')
        self.declare_parameter('source_frame_override', '')
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_voxels', 250000)
        self.declare_parameter('max_range', 25.0)
        self.declare_parameter('min_z', -0.5)
        self.declare_parameter('max_z', 3.0)
        self.declare_parameter('update_alpha', 0.25)
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter(
            'save_dir', str(Path.home() / 'ros2_ws' / 'colorized_slam_maps'))
        self.declare_parameter('save_service', '/slam/save_colorized_map')
        # 【静止時のみ蓄積モード】docs/tasks/colorized_pointcloud.md の「未検証の有力策」。
        # 巡回中の SLAM 2D 姿勢誤差が点群のブレ・放射状の筋の真因。 連続する TF から
        # 並進・回転速度を推定し、両方とも閾値以下のときだけ integrate する。
        # 屋内で歩行ロボット (Webots Burger ~0.2 m/s) なら threshold 0.05 m/s が目安。
        # 0 以下で無効（既存動作と同じ）。
        # 一般的 robot SLAM の stationary detection 閾値 (VILENS 等で 0.2 m / 5°) より厳しめ。
        self.declare_parameter('stationary_only', False)
        self.declare_parameter('stationary_max_lin_velocity', 0.05)
        self.declare_parameter('stationary_max_ang_velocity', 0.2)
        self.declare_parameter('stationary_velocity_window_sec', 0.5)

        self.input_cloud = self.get_parameter('input_cloud').value
        self.output_cloud = self.get_parameter('output_cloud').value
        self.target_frame = self.get_parameter('target_frame').value
        self.fallback_frame = self.get_parameter('fallback_frame').value
        self.source_frame_override = self.get_parameter(
            'source_frame_override').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_voxels = int(self.get_parameter('max_voxels').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.update_alpha = float(self.get_parameter('update_alpha').value)
        self.save_dir = Path(self.get_parameter('save_dir').value)

        self.stationary_only = bool(
            self.get_parameter('stationary_only').value)
        self.stationary_max_lin = float(
            self.get_parameter('stationary_max_lin_velocity').value)
        self.stationary_max_ang = float(
            self.get_parameter('stationary_max_ang_velocity').value)
        self.stationary_window_sec = float(
            self.get_parameter('stationary_velocity_window_sec').value)

        self.voxels = {}
        self.current_frame = self.target_frame
        self.last_stamp = self.get_clock().now().to_msg()
        self.received_clouds = 0
        # 速度推定用の前回 TF キャッシュ (translation, quaternion, sec)。
        # stationary_only=False のときは未使用。
        self._prev_tf_xyz = None
        self._prev_tf_quat = None
        self._prev_tf_sec = None
        self._skipped_moving = 0
        self._accepted_stationary = 0

        self.pub = self.create_publisher(
            PointCloud2, self.output_cloud, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.input_cloud, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(
            float(self.get_parameter('publish_period_sec').value), self.publish_map)
        self.create_service(
            Trigger, self.get_parameter('save_service').value, self.on_save_map)
        self.get_logger().info(
            f'colorized_pointcloud_mapper started: {self.input_cloud} -> '
            f'{self.output_cloud}')

    def lookup_transform(self, source_frame):
        for target in [self.target_frame, self.fallback_frame]:
            if not target:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    target, source_frame, rclpy.time.Time())
                if target != self.current_frame and self.voxels:
                    self.get_logger().warning(
                        f'clearing accumulated map because target frame changed '
                        f'{self.current_frame} -> {target}')
                    self.voxels.clear()
                self.current_frame = target
                return tf
            except TransformException:
                continue
        return None

    def on_cloud(self, msg):
        source_frame = self.source_frame_override or msg.header.frame_id
        tf = self.lookup_transform(source_frame)
        if tf is None:
            self.get_logger().warning(
                f'no transform from {source_frame} to '
                f'{self.target_frame}/{self.fallback_frame}',
                throttle_duration_sec=5.0)
            return

        arr = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        if arr.shape[0] == 0:
            return

        pts = arr[:, :3].astype(np.float32)
        rgb = split_rgb(arr[:, 3])
        ranges = np.linalg.norm(pts, axis=1)
        keep = (
            np.isfinite(pts).all(axis=1) &
            (ranges <= self.max_range) &
            (pts[:, 2] >= self.min_z) &
            (pts[:, 2] <= self.max_z)
        )
        pts = pts[keep]
        rgb = rgb[keep]
        if pts.shape[0] == 0:
            return

        rot = quat_to_matrix(tf.transform.rotation)
        trans = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=np.float32)

        # 【静止時のみ蓄積】速度推定し、両方が閾値以下のときだけ integrate する。
        if self.stationary_only and self._prev_tf_xyz is not None:
            now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = now_sec - self._prev_tf_sec
            if dt > 0.0 and dt <= self.stationary_window_sec:
                lin_v = float(np.linalg.norm(trans - self._prev_tf_xyz)) / dt
                # quaternion 距離 -> 角度 -> 角速度
                q1 = self._prev_tf_quat
                q2 = np.array([
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                ], dtype=np.float32)
                # |dot| ≈ cos(theta/2) のため angle = 2*acos(|dot|)
                dot = float(np.clip(abs(np.dot(q1, q2)), -1.0, 1.0))
                ang = 2.0 * float(np.arccos(dot))
                ang_v = ang / dt
                if (lin_v > self.stationary_max_lin
                        or ang_v > self.stationary_max_ang):
                    self._skipped_moving += 1
                    if self._skipped_moving % 50 == 0:
                        self.get_logger().info(
                            f'stationary_only: skipped {self._skipped_moving} '
                            f'moving frames (last lin={lin_v:.3f} m/s '
                            f'ang={ang_v:.3f} rad/s)')
                    # TF キャッシュは更新する (速度推定のため)
                    self._prev_tf_xyz = trans.copy()
                    self._prev_tf_quat = q2
                    self._prev_tf_sec = now_sec
                    return
                self._accepted_stationary += 1
            self._prev_tf_xyz = trans.copy()
            self._prev_tf_quat = np.array([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ], dtype=np.float32)
            self._prev_tf_sec = now_sec
        elif self.stationary_only:
            # 初回: TF キャッシュ初期化のみ
            now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._prev_tf_xyz = trans.copy()
            self._prev_tf_quat = np.array([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ], dtype=np.float32)
            self._prev_tf_sec = now_sec
            return

        pts_map = pts @ rot.T + trans
        self.integrate(pts_map, rgb)
        self.last_stamp = msg.header.stamp
        self.received_clouds += 1

    def integrate(self, pts, rgb):
        keys = np.floor(pts / self.voxel_size).astype(np.int32)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        counts = np.bincount(inverse).astype(np.float32)
        sum_pts = np.zeros((unique.shape[0], 3), dtype=np.float32)
        sum_rgb = np.zeros((unique.shape[0], 3), dtype=np.float32)
        np.add.at(sum_pts, inverse, pts)
        np.add.at(sum_rgb, inverse, rgb)
        mean_pts = sum_pts / counts[:, None]
        mean_rgb = sum_rgb / counts[:, None]

        alpha = min(max(self.update_alpha, 0.0), 1.0)
        for key_arr, pt, color in zip(unique, mean_pts, mean_rgb):
            key = tuple(int(v) for v in key_arr)
            old = self.voxels.get(key)
            if old is None:
                self.voxels[key] = [pt, color]
            else:
                old[0] = old[0] * (1.0 - alpha) + pt * alpha
                old[1] = old[1] * (1.0 - alpha) + color * alpha

        overflow = len(self.voxels) - self.max_voxels
        if overflow > 0:
            for key in list(self.voxels.keys())[:overflow]:
                del self.voxels[key]

    def make_cloud_msg(self):
        values = list(self.voxels.values())
        pts = np.array([v[0] for v in values], dtype=np.float32)
        rgb = np.array([v[1] for v in values], dtype=np.float32)
        rgb_f32 = pack_rgb(rgb)

        structured = np.zeros(pts.shape[0], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('rgb', np.float32),
        ])
        structured['x'] = pts[:, 0]
        structured['y'] = pts[:, 1]
        structured['z'] = pts[:, 2]
        structured['rgb'] = rgb_f32

        out = PointCloud2()
        out.header.stamp = self.last_stamp
        out.header.frame_id = self.current_frame
        out.height = 1
        out.width = structured.shape[0]
        out.fields = FIELDS_XYZRGB
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.data = structured.tobytes()
        out.is_dense = True
        return out

    def publish_map(self):
        if not self.voxels:
            return
        self.pub.publish(self.make_cloud_msg())

    def on_save_map(self, request, response):
        del request
        if not self.voxels:
            response.success = False
            response.message = 'colorized map is empty'
            return response
        self.save_dir.mkdir(parents=True, exist_ok=True)
        path = self.save_dir / f'colorized_map_{int(time.time())}.ply'
        self.write_ply(path)
        response.success = True
        response.message = str(path)
        return response

    def write_ply(self, path):
        values = list(self.voxels.values())
        with path.open('w') as f:
            f.write('ply\nformat ascii 1.0\n')
            f.write(f'element vertex {len(values)}\n')
            f.write('property float x\nproperty float y\nproperty float z\n')
            f.write('property uchar red\nproperty uchar green\n')
            f.write('property uchar blue\nend_header\n')
            for pt, color in values:
                c = np.clip(color, 0.0, 255.0).astype(np.uint8)
                f.write(
                    f'{pt[0]:.5f} {pt[1]:.5f} {pt[2]:.5f} '
                    f'{int(c[0])} {int(c[1])} {int(c[2])}\n')


def main(args=None):
    rclpy.init(args=args)
    node = ColorizedPointCloudMapperNode()
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
