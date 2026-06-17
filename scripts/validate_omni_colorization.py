#!/usr/bin/env python3
"""Validate Webots omnidirectional colorized point cloud over robot yaw angles.

This script generates temporary calibration worlds with different robot yaw
angles, launches Webots, captures the omnidirectional image and colorized point
cloud, and scores known colored targets.
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


TARGETS = {
    'cyan_marker': {
        'world': np.array([1.8, 1.8, 0.85], dtype=np.float32),
        'radius': 0.28,
        'expect': 'cyan',
        'hsv': ((80, 50, 60), (100, 255, 255)),
    },
    'orange_marker': {
        'world': np.array([-1.8, 1.8, 0.85], dtype=np.float32),
        'radius': 0.28,
        'expect': 'orange',
        'hsv': ((5, 50, 60), (25, 255, 255)),
    },
    'blue_marker': {
        'world': np.array([-1.8, -1.8, 0.85], dtype=np.float32),
        'radius': 0.28,
        'expect': 'blue',
        'hsv': ((105, 50, 60), (130, 255, 255)),
    },
    'white_marker': {
        'world': np.array([1.8, -1.8, 0.85], dtype=np.float32),
        'radius': 0.28,
        'expect': 'white',
        'hsv': ((0, 0, 160), (179, 70, 255)),
    },
    'yellow_panel': {
        'world': np.array([0.0, 2.2, 0.55], dtype=np.float32),
        'radius': 0.55,
        'expect': 'yellow',
        'hsv': ((15, 40, 40), (45, 255, 255)),
    },
    'red_panel': {
        'world': np.array([0.0, -2.2, 0.55], dtype=np.float32),
        'radius': 0.55,
        'expect': 'red',
        'hsv': ((0, 60, 40), (12, 255, 255)),
    },
    'green_box': {
        'world': np.array([1.4, 1.2, 0.25], dtype=np.float32),
        'radius': 0.45,
        'expect': 'green',
        'hsv': ((40, 60, 40), (90, 255, 255)),
    },
    'magenta_cylinder': {
        'world': np.array([-1.3, -1.4, 0.35], dtype=np.float32),
        'radius': 0.45,
        'expect': 'magenta',
        'hsv': ((135, 40, 40), (179, 255, 255)),
    },
}


WEBOTS_CYLINDRICAL_ROT = np.array([
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
], dtype=np.float32)


class Grabber(Node):
    def __init__(self):
        super().__init__('validate_omni_colorization_grabber')
        self.bridge = CvBridge()
        self.image = None
        self.cloud = None
        self.create_subscription(
            Image, '/omni_camera/image_raw/image_color',
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/perception/colorized_points',
            self._on_cloud, qos_profile_sensor_data)

    def _on_image(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'image decode failed: {exc}')

    def _on_cloud(self, msg):
        self.cloud = msg


def package_root():
    share = Path(get_package_share_directory('susumu_object_perception'))
    # In symlink-install this resolves to the source tree. If not, writing under
    # share is still acceptable for generated validation worlds.
    return share


def make_world(base_world: Path, yaw_deg: float) -> str:
    text = base_world.read_text()
    yaw = math.radians(yaw_deg)
    replacement = f'TurtleBot3Burger {{\\n  translation 0 0 0\\n  rotation 0 0 1 {yaw:.9f}'
    text = re.sub(
        r'TurtleBot3Burger \{\s+translation 0 0 0(?:\s+rotation 0 0 1 [-0-9.eE]+)?',
        replacement, text, count=1)
    out_name = f'calibration_yaw_{int(round(yaw_deg)) % 360:03d}.wbt'
    out_path = base_world.parent / out_name
    out_path.write_text(text)
    return out_name


def launch_world(world_name: str):
    cmd = [
        'ros2', 'launch', 'susumu_object_perception', 'webots_simulation.launch.py',
        f'world:={world_name}', 'nav:=False', 'rviz:=False',
        'perception:=False', 'omni_perception:=True', 'mode:=fast',
    ]
    env = os.environ.copy()
    return subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, preexec_fn=os.setsid, env=env)


def stop_process(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def grab(timeout_sec: float):
    rclpy.init(args=None)
    node = Grabber()
    deadline = time.time() + timeout_sec
    while time.time() < deadline and (node.image is None or node.cloud is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    image = node.image
    cloud = node.cloud
    node.destroy_node()
    rclpy.shutdown()
    return image, cloud


def pointcloud_arrays(msg):
    arr = pc2.read_points_numpy(
        msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
    xyz = arr[:, :3].astype(np.float32)
    rgb_u = arr[:, 3].astype(np.float32).view(np.uint32)
    r = ((rgb_u >> 16) & 255).astype(np.uint8)
    g = ((rgb_u >> 8) & 255).astype(np.uint8)
    b = (rgb_u & 255).astype(np.uint8)
    rgb = np.stack([r, g, b], axis=1)
    return xyz, rgb


def world_to_lidar(point_world, yaw_deg):
    yaw = math.radians(yaw_deg)
    c, s = math.cos(-yaw), math.sin(-yaw)
    rot_inv = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
    p_base = rot_inv @ point_world
    p_base[2] -= 0.20
    return p_base


def project_webots(point_lidar):
    p_cam = point_lidar.copy()
    p_cam[2] += 0.20 - 0.75
    c = WEBOTS_CYLINDRICAL_ROT @ p_cam
    r = np.linalg.norm(c)
    if r < 1e-6:
        return None
    yaw = math.atan2(c[1], c[0])
    z_unit = float(np.clip(c[2] / r, -1.0, 1.0))
    u = (0.5 - yaw / (2.0 * math.pi)) * 2048.0
    v = (0.5 + (math.acos(z_unit) - math.pi / 2.0) / math.pi) * 1024.0
    return np.array([u % 2048.0, v], dtype=np.float32)


def color_score(rgb, expect):
    if len(rgb) == 0:
        return 0.0
    r, g, b = rgb[:, 0].astype(float), rgb[:, 1].astype(float), rgb[:, 2].astype(float)
    if expect == 'red':
        return float(np.mean((r > 150) & (r > g * 1.6) & (r > b * 1.6)))
    if expect == 'green':
        return float(np.mean((g > 130) & (g > r * 1.5) & (g > b * 1.5)))
    if expect == 'yellow':
        return float(np.mean((r > 90) & (g > 80) & (b < 100)))
    if expect == 'magenta':
        return float(np.mean((r > 100) & (b > 90) & (g < 100)))
    if expect == 'cyan':
        return float(np.mean((g > 120) & (b > 120) & (r < 100)))
    if expect == 'orange':
        return float(np.mean((r > 130) & (g > 70) & (g < 180) & (b < 100)))
    if expect == 'blue':
        return float(np.mean((b > 120) & (b > r * 1.5) & (b > g * 1.5)))
    if expect == 'white':
        return float(np.mean((r > 150) & (g > 150) & (b > 150)))
    return 0.0


def image_centroid(image, hsv_bounds, expected_uv):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lo, hi = hsv_bounds
    mask = cv2.inRange(hsv, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return None, 0
    # Use only the connected-looking color pixels near the projected location to
    # avoid counting unrelated red/yellow floor texture.
    du = ((xs.astype(float) - expected_uv[0] + 1024.0) % 2048.0) - 1024.0
    dv = ys.astype(float) - expected_uv[1]
    near = (du * du + dv * dv) < 160.0 * 160.0
    if int(np.sum(near)) < 10:
        near = (du * du + dv * dv) < 320.0 * 320.0
    if int(np.sum(near)) == 0:
        return None, 0
    xs2 = xs[near].astype(float)
    ys2 = ys[near].astype(float)
    du2 = ((xs2 - expected_uv[0] + 1024.0) % 2048.0) - 1024.0
    cx = (expected_uv[0] + float(np.mean(du2))) % 2048.0
    cy = float(np.mean(ys2))
    return np.array([cx, cy], dtype=np.float32), int(np.sum(near))


def score_capture(image, cloud, yaw_deg):
    xyz, rgb = pointcloud_arrays(cloud)
    rows = []
    for name, target in TARGETS.items():
        center = world_to_lidar(target['world'], yaw_deg)
        dist = np.linalg.norm(xyz - center.reshape(1, 3), axis=1)
        mask = dist < target['radius']
        target_rgb = rgb[mask]
        score = color_score(target_rgb, target['expect'])
        mean_rgb = target_rgb.mean(axis=0) if len(target_rgb) else np.array([0, 0, 0])
        expected_uv = project_webots(center)
        centroid, pixels = (None, 0)
        err_deg = None
        if expected_uv is not None:
            centroid, pixels = image_centroid(image, target['hsv'], expected_uv)
        if centroid is not None:
            du = ((centroid[0] - expected_uv[0] + 1024.0) % 2048.0) - 1024.0
            dv = centroid[1] - expected_uv[1]
            err_deg = math.hypot(du * 360.0 / 2048.0, dv * 180.0 / 1024.0)
        rows.append({
            'name': name,
            'points': int(np.sum(mask)),
            'score': score,
            'mean_rgb': mean_rgb,
            'project_error_deg': err_deg,
            'image_pixels': pixels,
        })
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaws', default='0,45,90,135,180,225,270,315')
    parser.add_argument('--startup-sec', type=float, default=45.0)
    parser.add_argument('--grab-timeout-sec', type=float, default=20.0)
    parser.add_argument('--require-pass', action='store_true')
    parser.add_argument('--min-large-target-score', type=float, default=0.45)
    parser.add_argument('--max-image-error-deg', type=float, default=20.0)
    args = parser.parse_args()

    share = package_root()
    base_world = share / 'webots_worlds' / 'calibration.wbt'
    yaws = [float(v) for v in args.yaws.split(',') if v.strip()]

    all_rows = []
    for yaw in yaws:
        world_name = make_world(base_world, yaw)
        print(f'=== yaw {yaw:.1f} deg world {world_name} ===', flush=True)
        proc = launch_world(world_name)
        try:
            time.sleep(args.startup_sec)
            image, cloud = grab(args.grab_timeout_sec)
            if image is None or cloud is None:
                print('capture failed: image/cloud missing')
                all_rows.append((yaw, False, []))
                continue
            rows = score_capture(image, cloud, yaw)
            all_rows.append((yaw, True, rows))
            for row in rows:
                err = row['project_error_deg']
                err_txt = 'n/a' if err is None else f'{err:.2f}deg'
                rgb_txt = ','.join(f'{v:.0f}' for v in row['mean_rgb'])
                print(
                    f"{row['name']}: pts={row['points']} score={row['score']:.2f} "
                    f"mean_rgb=[{rgb_txt}] img_err={err_txt} img_px={row['image_pixels']}")
        finally:
            stop_process(proc)
            subprocess.run(
                "ps aux | rg 'webots|webots_controller|colorized_pointcloud|object_image_crop' | rg -v rg | awk '{print $2}' | xargs -r kill -9",
                shell=True, check=False)
            time.sleep(2.0)

    scores = []
    errors = []
    for _, ok, rows in all_rows:
        if not ok:
            continue
        for row in rows:
            scores.append(row['score'])
            if row['project_error_deg'] is not None:
                errors.append(row['project_error_deg'])
    print('=== summary ===')
    if scores:
        print(f'color_score mean={np.mean(scores):.3f} min={np.min(scores):.3f}')
    if errors:
        print(f'image_projection_error_deg mean={np.mean(errors):.3f} max={np.max(errors):.3f}')

    large_targets = {'red_panel', 'yellow_panel', 'green_box', 'magenta_cylinder'}
    failures = []
    for yaw, ok, rows in all_rows:
        if not ok:
            failures.append(f'yaw {yaw:.1f}: capture failed')
            continue
        for row in rows:
            if row['name'] in large_targets:
                if row['points'] <= 0:
                    failures.append(f"yaw {yaw:.1f}: {row['name']} has no points")
                elif row['score'] < args.min_large_target_score:
                    failures.append(
                        f"yaw {yaw:.1f}: {row['name']} score "
                        f"{row['score']:.2f} < {args.min_large_target_score:.2f}")
            err = row['project_error_deg']
            if err is not None and err > args.max_image_error_deg:
                failures.append(
                    f"yaw {yaw:.1f}: {row['name']} image projection error "
                    f"{err:.2f}deg > {args.max_image_error_deg:.2f}deg")

    if failures:
        print('=== validation failures ===')
        for failure in failures:
            print(f'- {failure}')
        return 2 if args.require_pass else 0
    print('validation_passed=true')
    return 0


if __name__ == '__main__':
    sys.exit(main())
