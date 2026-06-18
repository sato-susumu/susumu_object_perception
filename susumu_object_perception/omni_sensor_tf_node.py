#!/usr/bin/env python3
"""Publish LiDAR and omni-camera static TFs, optionally from vlcal calib.json."""

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def _normalize_quat(q):
    norm = sum(v * v for v in q) ** 0.5
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [float(v) / norm for v in q]


def _read_t_lidar_camera(path):
    data = json.loads(Path(path).read_text())
    candidates = [
        data.get('results', {}).get('T_lidar_camera'),
        data.get('T_lidar_camera'),
        data.get('init_T_lidar_camera'),
        data.get('results', {}).get('init_T_lidar_camera'),
    ]
    for candidate in candidates:
        if isinstance(candidate, list) and len(candidate) == 7:
            xyz = [float(v) for v in candidate[:3]]
            quat = _normalize_quat([float(v) for v in candidate[3:]])
            return xyz, quat
    raise ValueError(
        'calib json has no T_lidar_camera compatible 7-value transform')


class OmniSensorTfNode(Node):
    def __init__(self):
        super().__init__('omni_sensor_tf')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('camera_frame', 'omni_camera_link')
        self.declare_parameter('lidar_xyz', [0.0, 0.0, 0.20])
        self.declare_parameter('camera_xyz_initial', [0.0, 0.0, 0.75])
        self.declare_parameter('calibration_json', '')

        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.lidar_xyz = [
            float(v) for v in self.get_parameter('lidar_xyz').value]
        camera_xyz = [
            float(v) for v in self.get_parameter('camera_xyz_initial').value]
        self.initial_lidar_camera_xyz = [
            camera_xyz[i] - self.lidar_xyz[i] for i in range(3)]
        self.calibration_json = self.get_parameter('calibration_json').value

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tfs()

    def _make_tf(self, parent, child, xyz, quat):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        q = _normalize_quat(quat)
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        return msg

    def publish_static_tfs(self):
        lidar_tf = self._make_tf(
            self.base_frame, self.lidar_frame,
            self.lidar_xyz, [0.0, 0.0, 0.0, 1.0])

        source = 'initial'
        lidar_camera_xyz = self.initial_lidar_camera_xyz
        lidar_camera_quat = [0.0, 0.0, 0.0, 1.0]
        if self.calibration_json:
            try:
                lidar_camera_xyz, lidar_camera_quat = _read_t_lidar_camera(
                    self.calibration_json)
                source = self.calibration_json
            except Exception as exc:
                self.get_logger().error(
                    f'failed to read calibration_json; using initial TF: {exc}')

        camera_tf = self._make_tf(
            self.lidar_frame, self.camera_frame,
            lidar_camera_xyz, lidar_camera_quat)
        self.broadcaster.sendTransform([lidar_tf, camera_tf])
        self.get_logger().info(
            f'published {self.base_frame}->{self.lidar_frame} and '
            f'{self.lidar_frame}->{self.camera_frame} TFs from {source}')


def main(args=None):
    rclpy.init(args=args)
    node = OmniSensorTfNode()
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
