#!/usr/bin/env python3
"""Republish Webots PointCloud2 with a synthetic intensity channel."""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


FIELDS_XYZI = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


class PointCloudIntensityNode(Node):
    def __init__(self):
        super().__init__('pointcloud_intensity')
        self.declare_parameter('input_cloud', '/velodyne_points/point_cloud')
        self.declare_parameter('output_cloud', '/velodyne_points/point_cloud_intensity')
        self.declare_parameter('max_range', 30.0)

        self.max_range = float(self.get_parameter('max_range').value)
        self.pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('output_cloud').value,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2,
            self.get_parameter('input_cloud').value,
            self.on_cloud,
            qos_profile_sensor_data)
        self.get_logger().info('pointcloud_intensity started')

    def on_cloud(self, msg):
        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        pts = pts.astype(np.float32)
        ranges = np.linalg.norm(pts, axis=1)
        intensity = 1.0 - np.clip(ranges / max(self.max_range, 1e-3), 0.0, 1.0)

        structured = np.zeros(pts.shape[0], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32),
        ])
        structured['x'] = pts[:, 0]
        structured['y'] = pts[:, 1]
        structured['z'] = pts[:, 2]
        structured['intensity'] = intensity.astype(np.float32)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = structured.shape[0]
        out.fields = FIELDS_XYZI
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.data = structured.tobytes()
        out.is_dense = True
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudIntensityNode()
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
