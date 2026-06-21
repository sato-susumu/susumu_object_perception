#!/usr/bin/env python3
"""Save one PointCloud2 topic sample to an ASCII PLY file.

This is the automation bridge for the outdoor GLIM-first mapping route:
subscribe to `/slam/glim_colorized_points_map` (or another PointCloud2 topic),
write a PLY, then feed it to `glim_cloud_to_2d_map.py`.
"""

import argparse
import math
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def rgb_float_to_channels(value):
    arr = np.asarray([value], dtype=np.float32).view(np.uint32)
    packed = int(arr[0])
    return (packed >> 16) & 255, (packed >> 8) & 255, packed & 255


def rgb_int_to_channels(value):
    packed = int(value)
    return (packed >> 16) & 255, (packed >> 8) & 255, packed & 255


def detect_color_fields(msg):
    names = {f.name for f in msg.fields}
    if {'red', 'green', 'blue'} <= names:
        return ('red', 'green', 'blue')
    if {'r', 'g', 'b'} <= names:
        return ('r', 'g', 'b')
    if 'rgb' in names:
        return ('rgb',)
    if 'rgba' in names:
        return ('rgba',)
    return ()


def pointcloud_rows(msg):
    field_names = ['x', 'y', 'z']
    color_fields = detect_color_fields(msg)
    field_names.extend(color_fields)
    rows = []
    skipped = 0
    for p in pc2.read_points(msg, field_names=field_names, skip_nans=True):
        x = float(p[0])
        y = float(p[1])
        z = float(p[2])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            skipped += 1
            continue
        color = None
        if len(color_fields) == 3:
            color = tuple(max(0, min(255, int(round(float(c)))))
                          for c in p[3:6])
        elif len(color_fields) == 1:
            value = p[3]
            if isinstance(value, (float, np.floating)):
                color = rgb_float_to_channels(value)
            else:
                color = rgb_int_to_channels(value)
        rows.append((x, y, z, color))
    return rows, skipped


def write_ply(path, rows, frame_id, stamp_text):
    has_color = any(row[3] is not None for row in rows)
    with Path(path).open('w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'comment frame_id {frame_id}\n')
        f.write(f'comment stamp {stamp_text}\n')
        f.write(f'element vertex {len(rows)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        if has_color:
            f.write('property uchar red\nproperty uchar green\n')
            f.write('property uchar blue\n')
        f.write('end_header\n')
        for x, y, z, color in rows:
            if has_color:
                if color is None:
                    color = (255, 255, 255)
                f.write(
                    f'{x:.6f} {y:.6f} {z:.6f} '
                    f'{color[0]} {color[1]} {color[2]}\n')
            else:
                f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')


class PointCloudSaver(Node):
    def __init__(self, args):
        super().__init__('save_pointcloud2_to_ply')
        self.args = args
        self.done = False
        qos = self.make_qos(args.qos)
        self.create_subscription(PointCloud2, args.topic, self.on_cloud, qos)
        self.timer = self.create_timer(args.timeout_sec, self.on_timeout)
        self.get_logger().info(
            f'waiting for {args.topic} -> {args.out} '
            f'(min_points={args.min_points}, qos={args.qos})')

    @staticmethod
    def make_qos(name):
        if name == 'sensor_data':
            return qos_profile_sensor_data
        reliable = name == 'reliable'
        return QoSProfile(
            depth=1,
            reliability=(ReliabilityPolicy.RELIABLE if reliable
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)

    def on_timeout(self):
        if self.done:
            return
        self.get_logger().error(
            f'timed out waiting for {self.args.topic}')
        self.done = True

    def on_cloud(self, msg):
        if self.done:
            return
        rows, skipped = pointcloud_rows(msg)
        if len(rows) < self.args.min_points:
            self.get_logger().info(
                f'sample has {len(rows)} points (< {self.args.min_points}); '
                'waiting for a denser map',
                throttle_duration_sec=2.0)
            return
        out = Path(self.args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        stamp = msg.header.stamp
        stamp_text = f'{stamp.sec}.{stamp.nanosec:09d}'
        write_ply(out, rows, msg.header.frame_id, stamp_text)
        self.get_logger().info(
            f'wrote {out} points={len(rows)} skipped={skipped} '
            f'frame={msg.header.frame_id}')
        self.done = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/slam/glim_colorized_points_map')
    parser.add_argument('--out', required=True)
    parser.add_argument('--timeout-sec', type=float, default=20.0)
    parser.add_argument('--min-points', type=int, default=1)
    parser.add_argument('--qos', choices=('sensor_data', 'best_effort',
                                          'reliable'),
                        default='sensor_data')
    args = parser.parse_args()

    rclpy.init()
    node = PointCloudSaver(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        success = node.done and Path(args.out).exists()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
