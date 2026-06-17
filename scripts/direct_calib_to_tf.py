#!/usr/bin/env python3
"""Print ROS TF values from direct_visual_lidar_calibration calib.json."""

import argparse
import json
from pathlib import Path


def normalize_quat(q):
    norm = sum(v * v for v in q) ** 0.5
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [float(v) / norm for v in q]


def read_transform(path):
    data = json.loads(Path(path).read_text())
    candidates = [
        data.get('results', {}).get('T_lidar_camera'),
        data.get('T_lidar_camera'),
        data.get('init_T_lidar_camera'),
        data.get('results', {}).get('init_T_lidar_camera'),
    ]
    for candidate in candidates:
        if isinstance(candidate, list) and len(candidate) == 7:
            return [float(v) for v in candidate[:3]], normalize_quat(candidate[3:])
    raise RuntimeError('no T_lidar_camera transform found in calib.json')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('calib_json')
    parser.add_argument('--lidar-frame', default='velodyne_link')
    parser.add_argument('--camera-frame', default='omni_camera_link')
    args = parser.parse_args()

    xyz, quat = read_transform(args.calib_json)
    print('# T_lidar_camera maps p_camera to p_lidar.')
    print('# Publish it as parent=LiDAR, child=camera in ROS TF.')
    print(
        'ros2 run tf2_ros static_transform_publisher '
        f'--x {xyz[0]} --y {xyz[1]} --z {xyz[2]} '
        f'--qx {quat[0]} --qy {quat[1]} --qz {quat[2]} --qw {quat[3]} '
        f'--frame-id {args.lidar_frame} --child-frame-id {args.camera_frame}')
    print()
    print('launch argument:')
    print(f'  omni_calibration_json:={args.calib_json}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
