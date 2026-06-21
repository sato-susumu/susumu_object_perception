#!/usr/bin/env python3
"""Live check of /scan and PointCloud2 visibility at fence probe waypoints."""

import csv
import json
import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_rotate(q, p):
    # Quaternion-vector rotation without external dependencies.
    x, y, z = p
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def apply_tf(tf, point):
    r = quat_rotate(tf.transform.rotation, point)
    t = tf.transform.translation
    return np.asarray([r[0] + t.x, r[1] + t.y, r[2] + t.z],
                      dtype=np.float64)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def read_xyz(msg):
    try:
        arr = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return np.asarray(arr, dtype=np.float64).reshape((-1, 3))
    except Exception:
        rows = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        return np.asarray(rows, dtype=np.float64).reshape((-1, 3))


class FenceProbeSensorCheck(Node):

    def __init__(self):
        super().__init__('fence_probe_sensor_check')
        self.declare_parameter('probe_file', '')
        self.declare_parameter('output_prefix', '/tmp/fence_probe_sensor_check')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/lidar/points/point_cloud')
        self.declare_parameter('capture_radius_m', 0.45)
        self.declare_parameter('capture_yaw_tolerance_deg', 35.0)
        self.declare_parameter('target_cone_deg', 5.0)
        self.declare_parameter('target_range_tolerance_m', 0.6)
        self.declare_parameter('cloud_min_z', -0.35)
        self.declare_parameter('cloud_max_z', 2.0)
        self.declare_parameter('p2l_min_height', 0.1)
        self.declare_parameter('p2l_max_height', 2.0)
        self.declare_parameter('p2l_range_min', 0.3)
        self.declare_parameter('p2l_range_max', 40.0)
        self.declare_parameter('min_cloud_points', 3)
        self.declare_parameter('min_samples_per_probe', 3)
        self.declare_parameter('timeout_sec', 0.0)

        self.probe_file = os.path.expanduser(
            self.get_parameter('probe_file').value)
        self.output_prefix = os.path.expanduser(
            self.get_parameter('output_prefix').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.capture_radius = float(
            self.get_parameter('capture_radius_m').value)
        self.capture_yaw_tol = math.radians(float(
            self.get_parameter('capture_yaw_tolerance_deg').value))
        self.target_cone = math.radians(float(
            self.get_parameter('target_cone_deg').value))
        self.range_tol = float(
            self.get_parameter('target_range_tolerance_m').value)
        self.cloud_min_z = float(self.get_parameter('cloud_min_z').value)
        self.cloud_max_z = float(self.get_parameter('cloud_max_z').value)
        self.p2l_min_height = float(
            self.get_parameter('p2l_min_height').value)
        self.p2l_max_height = float(
            self.get_parameter('p2l_max_height').value)
        self.p2l_range_min = float(
            self.get_parameter('p2l_range_min').value)
        self.p2l_range_max = float(
            self.get_parameter('p2l_range_max').value)
        self.min_cloud_points = int(
            self.get_parameter('min_cloud_points').value)
        self.min_samples = int(
            self.get_parameter('min_samples_per_probe').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.probes = self._load_probes(self.probe_file)
        self.results = []
        for i, probe in enumerate(self.probes):
            self.results.append({
                'index': i,
                'object_index': probe.get('object_index'),
                'object_name': probe.get('object_name', ''),
                'coverage_inside': probe.get('coverage_inside'),
                'pose': probe['pose'],
                'target': probe['target'],
                'nearest_pose_distance_m': None,
                'nearest_pose_yaw_error_deg': None,
                'nearest_pose_time_sec': None,
                'nearest_robot_xy': None,
                'samples': [],
            })

        self.last_scan = None
        self.last_cloud = None
        self.done = False
        self.report_written = False
        self.started = self.get_clock().now()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self._cloud_cb, qos_profile_sensor_data)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'loaded {len(self.probes)} fence probes from {self.probe_file}')

    def _load_probes(self, path):
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f'probe_file not found: {path}')
        data = yaml.safe_load(open(path))
        targets = data.get('targets', [])
        probes = []
        for i, wp in enumerate(data.get('waypoints', [])):
            target = targets[i] if i < len(targets) else {}
            pose = target.get('pose', {})
            if not pose:
                pose = {'x': float(wp[0]), 'y': float(wp[1]),
                        'yaw': float(wp[2]) if len(wp) >= 3 else None}
            probes.append({
                'object_index': target.get('object_index'),
                'object_name': target.get('object_name', ''),
                'coverage_inside': target.get('coverage_inside'),
                'pose': {
                    'x': float(pose['x']),
                    'y': float(pose['y']),
                    'yaw': None if pose.get('yaw') is None
                    else float(pose['yaw']),
                },
                'target': [
                    float(target.get('target', [wp[0], wp[1]])[0]),
                    float(target.get('target', [wp[0], wp[1]])[1]),
                ],
            })
        return probes

    def _scan_cb(self, msg):
        self.last_scan = msg

    def _cloud_cb(self, msg):
        self.last_cloud = msg

    def _robot_pose(self):
        tf = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, rclpy.time.Time())
        pos = apply_tf(tf, (0.0, 0.0, 0.0))
        yaw = quat_to_yaw(tf.transform.rotation)
        return float(pos[0]), float(pos[1]), yaw

    def _target_in_lidar(self, target_xy):
        tf = self.tf_buffer.lookup_transform(
            self.lidar_frame, self.map_frame, rclpy.time.Time())
        return apply_tf(tf, (target_xy[0], target_xy[1], 0.55))

    def _analyze_scan(self, target):
        scan = self.last_scan
        if scan is None:
            return {'available': False}
        angle = math.atan2(target[1], target[0])
        target_range = float(math.hypot(target[0], target[1]))
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        n = len(ranges)
        if n == 0:
            return {'available': False}
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        diffs = np.abs(np.arctan2(np.sin(angles - angle),
                                  np.cos(angles - angle)))
        valid = (
            (diffs <= self.target_cone) &
            np.isfinite(ranges) &
            (ranges >= scan.range_min) &
            (ranges <= scan.range_max)
        )
        selected = ranges[valid]
        if len(selected) == 0:
            return {
                'available': True,
                'finite_in_cone': 0,
                'target_hit': False,
                'min_range_m': None,
                'target_range_m': target_range,
            }
        near = selected[np.abs(selected - target_range) <= self.range_tol]
        return {
            'available': True,
            'finite_in_cone': int(len(selected)),
            'target_hit': bool(len(near) > 0),
            'min_range_m': float(np.min(selected)),
            'target_range_m': target_range,
            'near_target_count': int(len(near)),
        }

    def _analyze_cloud(self, target):
        cloud = self.last_cloud
        if cloud is None:
            return {'available': False}
        pts = read_xyz(cloud)
        if len(pts) == 0:
            return {'available': True, 'points_in_cone': 0,
                    'target_hit': False}
        angle = math.atan2(target[1], target[0])
        target_range = float(math.hypot(target[0], target[1]))
        xy_range = np.hypot(pts[:, 0], pts[:, 1])
        angles = np.arctan2(pts[:, 1], pts[:, 0])
        diffs = np.abs(np.arctan2(np.sin(angles - angle),
                                  np.cos(angles - angle)))
        mask = (
            (diffs <= self.target_cone) &
            (pts[:, 2] >= self.cloud_min_z) &
            (pts[:, 2] <= self.cloud_max_z)
        )
        selected = pts[mask]
        selected_range = xy_range[mask]
        if len(selected) == 0:
            return {
                'available': True,
                'points_in_cone': 0,
                'target_hit': False,
                'near_target_count': 0,
                'near_target_p2l_count': 0,
                'min_range_m': None,
                'target_range_m': target_range,
            }
        near_mask = np.abs(selected_range - target_range) <= self.range_tol
        near = selected[near_mask]
        near_range = selected_range[near_mask]
        p2l_mask = (
            (selected[:, 2] >= self.p2l_min_height) &
            (selected[:, 2] <= self.p2l_max_height) &
            (selected_range >= self.p2l_range_min) &
            (selected_range <= self.p2l_range_max)
        )
        near_p2l = selected[near_mask & p2l_mask]
        near_p2l_range = selected_range[near_mask & p2l_mask]

        def minmax(values):
            if len(values) == 0:
                return None, None
            return float(np.min(values)), float(np.max(values))

        near_z_min, near_z_max = minmax(near[:, 2] if len(near) else [])
        near_range_min, near_range_max = minmax(near_range)
        near_p2l_z_min, near_p2l_z_max = minmax(
            near_p2l[:, 2] if len(near_p2l) else [])
        near_p2l_range_min, near_p2l_range_max = minmax(near_p2l_range)
        return {
            'available': True,
            'points_in_cone': int(len(selected)),
            'target_hit': bool(len(near) >= self.min_cloud_points),
            'near_target_count': int(len(near)),
            'near_target_p2l_count': int(len(near_p2l)),
            'near_target_z_min': near_z_min,
            'near_target_z_max': near_z_max,
            'near_target_range_min': near_range_min,
            'near_target_range_max': near_range_max,
            'near_target_p2l_z_min': near_p2l_z_min,
            'near_target_p2l_z_max': near_p2l_z_max,
            'near_target_p2l_range_min': near_p2l_range_min,
            'near_target_p2l_range_max': near_p2l_range_max,
            'min_range_m': float(np.min(selected_range)),
            'target_range_m': target_range,
        }

    def _tick(self):
        if self.done:
            return
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        if self.timeout_sec > 0.0 and elapsed >= self.timeout_sec:
            self.get_logger().warn('timeout reached; writing partial report')
            self._finish()
            return
        try:
            rx, ry, ryaw = self._robot_pose()
        except TransformException:
            return
        for result, probe in zip(self.results, self.probes):
            pose = probe['pose']
            dist = math.hypot(rx - pose['x'], ry - pose['y'])
            yaw_ok = True
            yaw_err = None
            if pose['yaw'] is not None:
                yaw_err = abs(angle_diff(ryaw, pose['yaw']))
                yaw_ok = yaw_err <= self.capture_yaw_tol
            nearest = result['nearest_pose_distance_m']
            if nearest is None or dist < nearest:
                result['nearest_pose_distance_m'] = float(dist)
                result['nearest_pose_yaw_error_deg'] = (
                    None if yaw_err is None else math.degrees(yaw_err))
                result['nearest_pose_time_sec'] = float(elapsed)
                result['nearest_robot_xy'] = [float(rx), float(ry)]
            if len(result['samples']) >= self.min_samples:
                continue
            if dist > self.capture_radius or not yaw_ok:
                continue
            try:
                target_lidar = self._target_in_lidar(probe['target'])
            except TransformException:
                continue
            scan_stats = self._analyze_scan(target_lidar)
            cloud_stats = self._analyze_cloud(target_lidar)
            sample = {
                'time_sec': float(elapsed),
                'robot_xy': [float(rx), float(ry)],
                'robot_yaw_deg': math.degrees(float(ryaw)),
                'pose_distance_m': float(dist),
                'pose_yaw_error_deg': (
                    None if yaw_err is None else math.degrees(yaw_err)),
                'target_lidar_xyz': [float(v) for v in target_lidar],
                'scan': scan_stats,
                'cloud': cloud_stats,
            }
            result['samples'].append(sample)
            self.get_logger().info(
                f'probe {result["index"]}: scan_hit='
                f'{scan_stats.get("target_hit")} cloud_hit='
                f'{cloud_stats.get("target_hit")} samples='
                f'{len(result["samples"])}/{self.min_samples}')
        if all(len(r['samples']) >= self.min_samples for r in self.results):
            self._finish()

    def _summarize_result(self, result):
        samples = result['samples']
        scan_seen = sum(1 for s in samples if s['scan'].get('target_hit'))
        cloud_seen = sum(1 for s in samples if s['cloud'].get('target_hit'))
        p2l_near_seen = sum(
            1 for s in samples
            if s['cloud'].get('near_target_p2l_count', 0) > 0)
        n = len(samples)
        if n == 0:
            nearest = result.get('nearest_pose_distance_m')
            yaw_err = result.get('nearest_pose_yaw_error_deg')
            if nearest is None:
                diagnosis = 'no_live_samples'
            elif nearest > self.capture_radius:
                diagnosis = 'no_live_samples_not_reached'
            elif (
                    yaw_err is not None and
                    math.radians(yaw_err) > self.capture_yaw_tol):
                diagnosis = 'no_live_samples_yaw_not_met'
            else:
                diagnosis = 'no_live_samples'
        elif cloud_seen and scan_seen:
            diagnosis = 'scan_and_cloud_see_target'
        elif cloud_seen and not scan_seen:
            if p2l_near_seen == 0:
                diagnosis = 'cloud_only_points_outside_p2l_filters'
            else:
                diagnosis = 'cloud_only_check_scan_timing_or_angle_bin'
        elif not cloud_seen and scan_seen:
            diagnosis = 'scan_only_unexpected_check_cloud_or_tf'
        else:
            diagnosis = 'not_seen_by_cloud_or_scan'
        return {
            'index': result['index'],
            'object_index': result['object_index'],
            'object_name': result['object_name'],
            'coverage_inside': result['coverage_inside'],
            'samples': n,
            'scan_seen': scan_seen,
            'cloud_seen': cloud_seen,
            'p2l_near_seen': p2l_near_seen,
            'scan_seen_ratio': None if n == 0 else scan_seen / n,
            'cloud_seen_ratio': None if n == 0 else cloud_seen / n,
            'p2l_near_ratio': None if n == 0 else p2l_near_seen / n,
            'nearest_pose_distance_m': result.get('nearest_pose_distance_m'),
            'nearest_pose_yaw_error_deg': result.get(
                'nearest_pose_yaw_error_deg'),
            'nearest_pose_time_sec': result.get('nearest_pose_time_sec'),
            'diagnosis': diagnosis,
        }

    def _finish(self):
        self.done = True
        self.write_reports()

    def write_reports(self):
        if self.report_written:
            return
        self.report_written = True
        summary = [self._summarize_result(r) for r in self.results]
        data = {
            'probe_file': self.probe_file,
            'parameters': {
                'capture_radius_m': self.capture_radius,
                'capture_yaw_tolerance_deg': math.degrees(
                    self.capture_yaw_tol),
                'target_cone_deg': math.degrees(self.target_cone),
                'target_range_tolerance_m': self.range_tol,
                'p2l_min_height': self.p2l_min_height,
                'p2l_max_height': self.p2l_max_height,
                'p2l_range_min': self.p2l_range_min,
                'p2l_range_max': self.p2l_range_max,
                'min_cloud_points': self.min_cloud_points,
                'min_samples_per_probe': self.min_samples,
            },
            'summary': summary,
            'results': self.results,
        }
        os.makedirs(os.path.dirname(self.output_prefix) or '.', exist_ok=True)
        with open(self.output_prefix + '.json', 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write('\n')
        with open(self.output_prefix + '.csv', 'w', newline='') as f:
            fields = [
                'index', 'object_index', 'object_name', 'coverage_inside',
                'samples', 'scan_seen', 'cloud_seen', 'p2l_near_seen',
                'scan_seen_ratio', 'cloud_seen_ratio', 'p2l_near_ratio',
                'nearest_pose_distance_m', 'nearest_pose_yaw_error_deg',
                'nearest_pose_time_sec',
                'diagnosis',
            ]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(summary)
        with open(self.output_prefix + '.md', 'w') as f:
            f.write('# Fence Probe Sensor Check\n\n')
            f.write(
                '| # | object | coverage | samples | scan | cloud | p2l near | '
                'nearest m | yaw err deg | diagnosis |\n')
            f.write('|---:|---|---:|---:|---:|---:|---:|---:|---:|---|\n')
            for row in summary:
                nearest = row['nearest_pose_distance_m']
                yaw_err = row['nearest_pose_yaw_error_deg']
                nearest_text = '' if nearest is None else f'{nearest:.2f}'
                yaw_text = '' if yaw_err is None else f'{yaw_err:.1f}'
                f.write(
                    f"| {row['index']} | {row['object_name']} | "
                    f"{row['coverage_inside']} | {row['samples']} | "
                    f"{row['scan_seen']} | {row['cloud_seen']} | "
                    f"{row['p2l_near_seen']} | "
                    f"{nearest_text} | {yaw_text} | "
                    f"{row['diagnosis']} |\n")
        self.get_logger().info(f'wrote {self.output_prefix}.json/.csv/.md')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FenceProbeSensorCheck()
    except Exception as exc:
        if rclpy.ok():
            rclpy.shutdown()
        raise exc
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.write_reports()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
