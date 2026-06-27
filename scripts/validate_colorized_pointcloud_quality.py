#!/usr/bin/env python3
"""Validate live or saved colorized PointCloud2/PLY data."""

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


TARGETS = {
    'red_panel': {
        'world': np.array([0.0, -2.2, 0.55], dtype=np.float32),
        'radius': 0.55,
        'expect': 'red',
    },
    'yellow_panel': {
        'world': np.array([0.0, 2.2, 0.55], dtype=np.float32),
        'radius': 0.55,
        'expect': 'yellow',
    },
    'green_box': {
        'world': np.array([1.4, 1.2, 0.25], dtype=np.float32),
        'radius': 0.45,
        'expect': 'green',
    },
    'magenta_cylinder': {
        'world': np.array([-1.3, -1.4, 0.35], dtype=np.float32),
        'radius': 0.45,
        'expect': 'magenta',
    },
}


PLY_FLOAT_TYPES = {'float', 'float32', 'double', 'float64'}
PLY_UINT8_TYPES = {'uchar', 'uint8', 'uint8_t', 'unsigned_char'}


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


class CloudGrabber(Node):
    def __init__(self, topics):
        super().__init__('validate_colorized_pointcloud_quality')
        self.clouds = {}
        for topic in topics:
            self.create_subscription(
                PointCloud2, topic, self._make_cb(topic), qos_profile_sensor_data)

    def _make_cb(self, topic):
        def _on_cloud(msg):
            self.clouds[topic] = msg
        return _on_cloud


def unpack_rgb(values, field):
    if field.datatype == PointField.UINT32:
        rgb_u = values.astype(np.uint32)
    else:
        rgb_u = values.astype(np.float32).view(np.uint32)
    r = ((rgb_u >> 16) & 255).astype(np.uint8)
    g = ((rgb_u >> 8) & 255).astype(np.uint8)
    b = (rgb_u & 255).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def cloud_to_arrays(msg):
    fields = {field.name: field for field in msg.fields}
    missing = {'x', 'y', 'z', 'rgb'} - set(fields)
    if missing:
        raise ValueError(f'missing PointCloud2 fields: {sorted(missing)}')
    arr = pc2.read_points_numpy(
        msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
    xyz = arr[:, :3].astype(np.float32)
    rgb = unpack_rgb(arr[:, 3], fields['rgb'])
    return xyz, rgb


def color_score(rgb, expect):
    if len(rgb) == 0:
        return 0.0
    r = rgb[:, 0].astype(float)
    g = rgb[:, 1].astype(float)
    b = rgb[:, 2].astype(float)
    if expect == 'red':
        return float(np.mean((r > 150) & (r > g * 1.6) & (r > b * 1.6)))
    if expect == 'green':
        return float(np.mean((g > 130) & (g > r * 1.5) & (g > b * 1.5)))
    if expect == 'yellow':
        return float(np.mean((r > 90) & (g > 80) & (b < 100)))
    if expect == 'magenta':
        return float(np.mean((r > 100) & (b > 90) & (g < 100)))
    return 0.0


def world_to_lidar(point_world, yaw_deg):
    yaw = math.radians(yaw_deg)
    c = math.cos(-yaw)
    s = math.sin(-yaw)
    rot_inv = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
    point = rot_inv @ point_world
    point[2] -= 0.20
    return point


def summarize_cloud(name, msg, min_points, min_colored_ratio, min_channel_std):
    xyz, rgb = cloud_to_arrays(msg)
    failures = []
    points = int(xyz.shape[0])
    finite_ratio = float(np.mean(np.isfinite(xyz).all(axis=1))) if points else 0.0
    colored_ratio = float(np.mean(np.max(rgb, axis=1) > 10)) if points else 0.0
    channel_std = rgb.astype(np.float32).std(axis=0) if points else np.zeros(3)
    bounds_min = xyz.min(axis=0) if points else np.zeros(3)
    bounds_max = xyz.max(axis=0) if points else np.zeros(3)

    if points < min_points:
        failures.append(f'{name}: points {points} < {min_points}')
    if finite_ratio < 0.999:
        failures.append(f'{name}: finite ratio {finite_ratio:.3f} < 0.999')
    if colored_ratio < min_colored_ratio:
        failures.append(
            f'{name}: colored ratio {colored_ratio:.3f} < {min_colored_ratio:.3f}')
    if float(np.max(channel_std)) < min_channel_std:
        failures.append(
            f'{name}: max RGB std {np.max(channel_std):.2f} < {min_channel_std:.2f}')

    print(f'=== {name} ===')
    print(f'frame={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
    print(f'points={points} finite_ratio={finite_ratio:.4f} colored_ratio={colored_ratio:.4f}')
    print(
        'rgb_mean=[' + ','.join(f'{v:.1f}' for v in rgb.mean(axis=0)) + '] '
        'rgb_std=[' + ','.join(f'{v:.1f}' for v in channel_std) + ']')
    print(
        'bounds_min=[' + ','.join(f'{v:.2f}' for v in bounds_min) + '] '
        'bounds_max=[' + ','.join(f'{v:.2f}' for v in bounds_max) + ']')
    metrics = {
        'kind': 'pointcloud2',
        'name': name,
        'frame_id': msg.header.frame_id,
        'stamp_sec': int(msg.header.stamp.sec),
        'stamp_nanosec': int(msg.header.stamp.nanosec),
        'points': points,
        'finite_ratio': finite_ratio,
        'colored_ratio': colored_ratio,
        'rgb_mean_255': [round(float(v), 1) for v in rgb.mean(axis=0)]
        if points else [0.0, 0.0, 0.0],
        'rgb_std_255': [round(float(v), 1) for v in channel_std],
        'bounds_min_m': [round(float(v), 3) for v in bounds_min],
        'bounds_max_m': [round(float(v), 3) for v in bounds_max],
        'criteria': {
            'min_points': int(min_points),
            'min_colored_ratio': float(min_colored_ratio),
            'min_channel_std': float(min_channel_std),
        },
        'passed': not failures,
        'failures': failures,
    }
    return xyz, rgb, failures, metrics


def validate_targets(xyz, rgb, yaw_deg, min_score):
    failures = []
    print('=== calibration targets ===')
    for name, target in TARGETS.items():
        center = world_to_lidar(target['world'], yaw_deg)
        dist = np.linalg.norm(xyz - center.reshape(1, 3), axis=1)
        mask = dist < target['radius']
        target_rgb = rgb[mask]
        score = color_score(target_rgb, target['expect'])
        mean_rgb = target_rgb.mean(axis=0) if len(target_rgb) else np.array([0, 0, 0])
        print(
            f"{name}: pts={int(np.sum(mask))} score={score:.3f} "
            f"mean_rgb=[" + ','.join(f'{v:.1f}' for v in mean_rgb) + ']')
        if int(np.sum(mask)) <= 0:
            failures.append(f'{name}: no points')
        elif score < min_score:
            failures.append(f'{name}: score {score:.3f} < {min_score:.3f}')
    return failures


def read_ply(path):
    with open(path, 'r') as f:
        line = f.readline().strip()
        if line != 'ply':
            raise ValueError('not a PLY file')
        vertex_count = None
        property_defs = []
        current_element = None
        for line in f:
            line = line.strip()
            if line.startswith('element vertex '):
                current_element = 'vertex'
                vertex_count = int(line.split()[-1])
            elif line.startswith('element '):
                parts = line.split()
                current_element = parts[1] if len(parts) >= 2 else None
            elif line.startswith('property ') and current_element == 'vertex':
                parts = line.split()
                if len(parts) >= 3:
                    property_defs.append({'type': parts[-2], 'name': parts[-1]})
            elif line == 'end_header':
                break
        if vertex_count is None:
            raise ValueError('PLY vertex count missing')
        rows = []
        for _ in range(vertex_count):
            parts = f.readline().split()
            if len(parts) < len(property_defs):
                break
            rows.append(parts)
    prop_index = {prop['name']: idx for idx, prop in enumerate(property_defs)}
    required = {'x', 'y', 'z', 'red', 'green', 'blue'}
    missing = required - set(prop_index)
    if missing:
        raise ValueError(f'PLY missing properties: {sorted(missing)}')
    xyz = np.array([
        [float(row[prop_index['x']]), float(row[prop_index['y']]), float(row[prop_index['z']])]
        for row in rows
    ], dtype=np.float32)
    rgb_i = np.array([
        [int(row[prop_index['red']]), int(row[prop_index['green']]), int(row[prop_index['blue']])]
        for row in rows
    ], dtype=np.int32)
    return xyz, rgb_i, vertex_count, property_defs


def summarize_ply(path, min_points, min_colored_ratio, min_channel_std):
    xyz, rgb_i, vertex_count, property_defs = read_ply(path)
    ply_hash = file_sha256(path)
    failures = []
    prop_types = {prop['name']: str(prop['type']).lower()
                  for prop in property_defs}
    property_type_failures = []
    for name in ('x', 'y', 'z'):
        if prop_types.get(name) not in PLY_FLOAT_TYPES:
            property_type_failures.append(
                f'{name}:{prop_types.get(name, "missing")}')
    for name in ('red', 'green', 'blue'):
        if prop_types.get(name) not in PLY_UINT8_TYPES:
            property_type_failures.append(
                f'{name}:{prop_types.get(name, "missing")}')
    points = int(xyz.shape[0])
    finite_ratio = float(np.mean(np.isfinite(xyz).all(axis=1))) if points else 0.0
    rgb_in_range = bool(np.all((rgb_i >= 0) & (rgb_i <= 255))) if points else False
    rgb = np.clip(rgb_i, 0, 255).astype(np.uint8)
    colored_ratio = float(np.mean(np.max(rgb, axis=1) > 10)) if points else 0.0
    channel_std = rgb.astype(np.float32).std(axis=0) if points else np.zeros(3)
    bounds_min = xyz.min(axis=0) if points else np.zeros(3)
    bounds_max = xyz.max(axis=0) if points else np.zeros(3)
    if points != vertex_count:
        failures.append(f'{path}: read {points} vertices but header says {vertex_count}')
    if property_type_failures:
        failures.append(
            f'{path}: unsupported PLY property types '
            f'{", ".join(property_type_failures)}')
    if points < min_points:
        failures.append(f'{path}: points {points} < {min_points}')
    if finite_ratio < 0.999:
        failures.append(f'{path}: finite ratio {finite_ratio:.3f} < 0.999')
    if not rgb_in_range:
        failures.append(f'{path}: RGB values outside 0..255')
    if colored_ratio < min_colored_ratio:
        failures.append(
            f'{path}: colored ratio {colored_ratio:.3f} < {min_colored_ratio:.3f}')
    if float(np.max(channel_std)) < min_channel_std:
        failures.append(
            f'{path}: max RGB std {np.max(channel_std):.2f} < {min_channel_std:.2f}')
    print(f'=== {path} ===')
    print(
        f'points={points} header_vertices={vertex_count} '
        f'finite_ratio={finite_ratio:.4f} colored_ratio={colored_ratio:.4f}')
    print(
        'rgb_mean=[' + ','.join(f'{v:.1f}' for v in rgb.mean(axis=0)) + '] '
        'rgb_std=[' + ','.join(f'{v:.1f}' for v in channel_std) + ']')
    print(
        'bounds_min=[' + ','.join(f'{v:.2f}' for v in bounds_min) + '] '
        'bounds_max=[' + ','.join(f'{v:.2f}' for v in bounds_max) + ']')
    metrics = {
        'kind': 'ply',
        'name': path,
        'file_sha256': ply_hash,
        'file_size_bytes': int(Path(path).stat().st_size),
        'points': points,
        'header_vertices': int(vertex_count),
        'properties': property_defs,
        'property_types': prop_types,
        'property_types_valid': not property_type_failures,
        'property_type_failures': property_type_failures,
        'finite_ratio': finite_ratio,
        'rgb_values_in_range': rgb_in_range,
        'colored_ratio': colored_ratio,
        'rgb_mean_255': [round(float(v), 1) for v in rgb.mean(axis=0)]
        if points else [0.0, 0.0, 0.0],
        'rgb_std_255': [round(float(v), 1) for v in channel_std],
        'bounds_min_m': [round(float(v), 3) for v in bounds_min],
        'bounds_max_m': [round(float(v), 3) for v in bounds_max],
        'criteria': {
            'min_points': int(min_points),
            'min_colored_ratio': float(min_colored_ratio),
            'min_channel_std': float(min_channel_std),
        },
        'passed': not failures,
        'failures': failures,
    }
    return failures, metrics


def write_json(path, report):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_markdown(path, report):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    summary = report.get('summary', {})
    lines = [
        '# Colorized Point Cloud Quality Summary',
        '',
        f"- validation_passed: `{str(report['validation_passed']).lower()}`",
        f"- schema_version: `{report['schema_version']}`",
        f"- items: `{len(report['items'])}`",
        f"- unique_ply_count: `{summary.get('unique_ply_count', 0)}`",
        f"- duplicate_input_names: `{summary.get('duplicate_input_name_count', 0)}`",
        f"- duplicate_file_hashes: `{summary.get('duplicate_file_hash_count', 0)}`",
        f"- failures: `{len(report['failures'])}`",
        '',
        '| item | kind | points | colored ratio | max RGB std | sha256 | status |',
        '|---|---|---:|---:|---:|---|---|',
    ]
    for item in report['items']:
        max_std = max(item.get('rgb_std_255', [0.0]))
        status = 'PASS' if item.get('passed') else 'FAIL'
        digest = item.get('file_sha256', '')
        digest_short = digest[:12] if digest else ''
        lines.append(
            f"| `{item['name']}` | {item['kind']} | {item['points']} | "
            f"{item['colored_ratio']:.4f} | {max_std:.1f} | "
            f"`{digest_short}` | {status} |")
    if report['failures']:
        lines.extend(['', '## Failures'])
        lines.extend(f'- {failure}' for failure in report['failures'])
    out.write_text('\n'.join(lines) + '\n')


def duplicate_values(values):
    counts = {}
    for value in values:
        counts[value] = counts.get(value, 0) + 1
    return sorted(value for value, count in counts.items() if count > 1)


def grab_clouds(topics, timeout_sec):
    rclpy.init(args=None)
    node = CloudGrabber(topics)
    deadline = time.time() + timeout_sec
    while time.time() < deadline and any(topic not in node.clouds for topic in topics):
        rclpy.spin_once(node, timeout_sec=0.2)
    clouds = dict(node.clouds)
    node.destroy_node()
    rclpy.shutdown()
    return clouds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--cloud-topic', action='append', default=[],
        help='PointCloud2 topic to validate. Can be specified multiple times.')
    parser.add_argument('--ply', action='append', default=[],
                        help='ASCII PLY file to validate.')
    parser.add_argument('--timeout-sec', type=float, default=20.0)
    parser.add_argument('--min-points', type=int, default=1000)
    parser.add_argument('--min-colored-ratio', type=float, default=0.20)
    parser.add_argument('--min-channel-std', type=float, default=5.0)
    parser.add_argument('--calibration-targets', action='store_true')
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--min-target-score', type=float, default=0.45)
    parser.add_argument('--json-out', default='',
                        help='optional machine-readable summary path')
    parser.add_argument('--md-out', default='',
                        help='optional Markdown summary path')
    args = parser.parse_args()

    topics = args.cloud_topic or ([] if args.ply else ['/perception/colorized_points'])
    failures = []
    items = []

    if topics:
        clouds = grab_clouds(topics, args.timeout_sec)
        for topic in topics:
            msg = clouds.get(topic)
            if msg is None:
                failures.append(f'{topic}: no cloud received within {args.timeout_sec:.1f}s')
                continue
            xyz, rgb, cloud_failures, metrics = summarize_cloud(
                topic, msg, args.min_points,
                args.min_colored_ratio, args.min_channel_std)
            items.append(metrics)
            failures.extend(cloud_failures)
            if args.calibration_targets:
                failures.extend(validate_targets(
                    xyz, rgb, args.yaw_deg, args.min_target_score))

    for ply in args.ply:
        ply_failures, metrics = summarize_ply(
            ply, args.min_points, args.min_colored_ratio, args.min_channel_std)
        items.append(metrics)
        failures.extend(ply_failures)

    ply_items = [item for item in items if item.get('kind') == 'ply']
    ply_names = [item.get('name') for item in ply_items if item.get('name')]
    duplicate_input_names = duplicate_values(ply_names)
    hash_to_names = {}
    for item in ply_items:
        digest = item.get('file_sha256')
        if digest:
            hash_to_names.setdefault(digest, []).append(item.get('name'))
    duplicate_file_hashes = [
        {'file_sha256': digest, 'names': sorted(names)}
        for digest, names in sorted(hash_to_names.items())
        if len(names) > 1
    ]
    if duplicate_input_names:
        failures.append(
            'duplicate PLY input names: ' + ', '.join(duplicate_input_names))
    for duplicate in duplicate_file_hashes:
        failures.append(
            'duplicate PLY file hash '
            f"{duplicate['file_sha256']}: {', '.join(duplicate['names'])}")

    failure_set = set(failures)
    for item in items:
        item['passed'] = all(f not in failure_set for f in item.get('failures', []))

    passed = sum(1 for item in items if item.get('passed') is True)
    report = {
        'schema_version': 5,
        'validation_passed': not failures,
        'summary': {
            'item_count': len(items),
            'ply_item_count': len(ply_items),
            'unique_ply_count': len(set(ply_names)),
            'unique_ply_hash_count': len(hash_to_names),
            'duplicate_input_name_count': len(duplicate_input_names),
            'duplicate_file_hash_count': len(duplicate_file_hashes),
            'passed': passed,
            'failed': len(items) - passed,
            'failure_count': len(failures),
        },
        'items': items,
        'failures': failures,
        'duplicate_input_names': duplicate_input_names,
        'duplicate_file_hashes': duplicate_file_hashes,
        'inputs': {
            'cloud_topics': topics,
            'ply': args.ply,
            'timeout_sec': args.timeout_sec,
            'min_points': args.min_points,
            'min_colored_ratio': args.min_colored_ratio,
            'min_channel_std': args.min_channel_std,
            'calibration_targets': args.calibration_targets,
            'yaw_deg': args.yaw_deg,
            'min_target_score': args.min_target_score,
        },
        'inputs_hash': {
            'ply_sha256': {
                item['name']: item['file_sha256']
                for item in items
                if item.get('kind') == 'ply' and item.get('file_sha256')
            },
        },
    }

    if args.json_out:
        write_json(args.json_out, report)
        print(f'saved {args.json_out}')
    if args.md_out:
        write_markdown(args.md_out, report)
        print(f'saved {args.md_out}')

    if failures:
        print('=== validation failures ===')
        for failure in failures:
            print(f'- {failure}')
        return 2
    print('validation_passed=true')
    return 0


if __name__ == '__main__':
    sys.exit(main())
