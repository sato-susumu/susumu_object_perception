#!/usr/bin/env python3
"""Compare Webots GPS ground truth against TF pose during mapping/navigation.

The main use case is outdoor mapping: keep the mapping stack unchanged, record
GPS-derived ground-truth motion and the SLAM/Nav2 TF estimate, then write
JSON/CSV/Markdown reports so drift is reviewable after a run.
"""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformException, TransformListener


SUPPORTED_GPS_TYPES = {
    'sensor_msgs/msg/NavSatFix': NavSatFix,
    'geometry_msgs/msg/PointStamped': PointStamped,
    'geometry_msgs/msg/Vector3Stamped': Vector3Stamped,
    'nav_msgs/msg/Odometry': Odometry,
}

EARTH_RADIUS_M = 6378137.0


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def path_length(points):
    if len(points) < 2:
        return 0.0
    pts = np.asarray(points, dtype=np.float64)
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def error_stats(errors):
    if len(errors) == 0:
        return {}
    arr = np.asarray(errors, dtype=np.float64)
    return {
        'mean': float(np.mean(arr)),
        'max': float(np.max(arr)),
        'rmse': float(math.sqrt(np.mean(arr * arr))),
        'final': float(arr[-1]),
    }


def best_fit_2d(source_xy, target_xy):
    """Return source aligned to target by best-fit 2D rigid transform."""
    src = np.asarray(source_xy, dtype=np.float64)
    tgt = np.asarray(target_xy, dtype=np.float64)
    if src.shape[0] < 2:
        return None, None, None
    src_c = src.mean(axis=0)
    tgt_c = tgt.mean(axis=0)
    src0 = src - src_c
    tgt0 = tgt - tgt_c
    if np.linalg.norm(src0) < 1e-6 or np.linalg.norm(tgt0) < 1e-6:
        return None, None, None
    h = src0.T @ tgt0
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = tgt_c - src_c @ r.T
    aligned = src @ r.T + t
    yaw = math.atan2(r[1, 0], r[0, 0])
    return aligned, yaw, t


class GpsVsSlamEvaluator:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.gps_topic = None
        self.gps_type = None
        self.gps_sub = None
        self.latest_gps = None
        self.gps_origin = None
        self.latlon_origin = None
        self.rows = []
        self.last_sample_t = 0.0

    def _candidate_topics(self):
        if self.args.gps_topic != 'auto':
            return [self.args.gps_topic]
        names = [name for name, _ in self.node.get_topic_names_and_types()]
        preferred = ['/gps', '/TurtleBot3Burger/gps']
        out = [topic for topic in preferred if topic in names]
        out.extend(sorted(name for name in names
                          if name.endswith('/gps') and name not in out))
        return out

    def _try_attach_gps(self):
        if self.gps_sub is not None:
            return True
        topics = dict(self.node.get_topic_names_and_types())
        for topic in self._candidate_topics():
            for type_name in topics.get(topic, []):
                msg_type = SUPPORTED_GPS_TYPES.get(type_name)
                if msg_type is None:
                    continue
                self.gps_topic = topic
                self.gps_type = type_name
                self.gps_sub = self.node.create_subscription(
                    msg_type, topic, self._on_gps, 10)
                self.node.get_logger().info(
                    f'using GPS topic {topic} ({type_name})')
                return True
        return False

    def _on_gps(self, msg):
        xy = self._gps_xy(msg)
        if xy is None:
            return
        if self.gps_origin is None:
            self.gps_origin = np.array(xy, dtype=np.float64)
        self.latest_gps = np.array(xy, dtype=np.float64)

    def _gps_xy(self, msg):
        if isinstance(msg, NavSatFix):
            if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
                return None
            lat = math.radians(float(msg.latitude))
            lon = math.radians(float(msg.longitude))
            if self.latlon_origin is None:
                self.latlon_origin = (lat, lon)
            lat0, lon0 = self.latlon_origin
            x = (lon - lon0) * math.cos(lat0) * EARTH_RADIUS_M
            y = (lat - lat0) * EARTH_RADIUS_M
            return x, y
        if isinstance(msg, PointStamped):
            return float(msg.point.x), float(msg.point.y)
        if isinstance(msg, Vector3Stamped):
            return float(msg.vector.x), float(msg.vector.y)
        if isinstance(msg, Odometry):
            p = msg.pose.pose.position
            return float(p.x), float(p.y)
        return None

    def _estimate_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.estimate_frame,
                self.args.robot_frame,
                rclpy.time.Time())
        except TransformException as exc:
            self.node.get_logger().warning(
                f'no TF {self.args.estimate_frame} <- '
                f'{self.args.robot_frame}: {exc}')
            return None
        tr = tf.transform.translation
        return np.array([float(tr.x), float(tr.y)], dtype=np.float64), \
            stamp_to_float(tf.header.stamp)

    def spin(self):
        deadline = time.monotonic() + self.args.duration_sec
        next_graph_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._try_attach_gps()
            now = time.monotonic()
            if self.gps_sub is None and now >= next_graph_log:
                self.node.get_logger().info(
                    'waiting for GPS topic; candidates=%s' %
                    ','.join(self._candidate_topics() or ['<none>']))
                next_graph_log = now + 5.0
            if self.latest_gps is not None and \
                    (now - self.last_sample_t) >= self.args.sample_period:
                est = self._estimate_xy()
                if est is not None:
                    est_xy, tf_stamp = est
                    gps_rel = self.latest_gps - self.gps_origin
                    direct_error = float(np.linalg.norm(est_xy - gps_rel))
                    self.rows.append({
                        'wall_time': time.time(),
                        'tf_stamp': tf_stamp,
                        'gps_x': float(self.latest_gps[0]),
                        'gps_y': float(self.latest_gps[1]),
                        'gps_rel_x': float(gps_rel[0]),
                        'gps_rel_y': float(gps_rel[1]),
                        'estimate_x': float(est_xy[0]),
                        'estimate_y': float(est_xy[1]),
                        'direct_error_m': direct_error,
                    })
                    self.last_sample_t = now
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def summarize(self):
        gps = np.array([[r['gps_rel_x'], r['gps_rel_y']]
                        for r in self.rows], dtype=np.float64)
        est = np.array([[r['estimate_x'], r['estimate_y']]
                        for r in self.rows], dtype=np.float64)
        direct_errors = [r['direct_error_m'] for r in self.rows]
        aligned_errors = []
        align_yaw = None
        align_t = None
        aligned, align_yaw, align_t = best_fit_2d(gps, est)
        if aligned is not None:
            aligned_errors = np.linalg.norm(est - aligned, axis=1).tolist()
            for row, err in zip(self.rows, aligned_errors):
                row['aligned_error_m'] = float(err)
        summary = {
            'gps_topic': self.gps_topic,
            'gps_type': self.gps_type,
            'estimate_frame': self.args.estimate_frame,
            'robot_frame': self.args.robot_frame,
            'samples': len(self.rows),
            'gps_path_length_m': path_length(gps) if len(gps) else 0.0,
            'estimate_path_length_m': path_length(est) if len(est) else 0.0,
            'direct_error_m': error_stats(direct_errors),
            'aligned_error_m': error_stats(aligned_errors),
            'alignment_yaw_deg': (
                None if align_yaw is None else math.degrees(float(align_yaw))),
            'alignment_translation_xy': (
                None if align_t is None else [float(align_t[0]), float(align_t[1])]),
        }
        failures = []
        if len(self.rows) < self.args.min_samples:
            failures.append(
                f'samples {len(self.rows)} < {self.args.min_samples}')
        if summary['gps_path_length_m'] < self.args.min_path_length:
            failures.append(
                f'gps path length {summary["gps_path_length_m"]:.2f}m '
                f'< {self.args.min_path_length:.2f}m')
        direct_max = summary['direct_error_m'].get('max')
        if direct_max is not None and direct_max > self.args.max_direct_error:
            failures.append(
                f'direct max error {direct_max:.2f}m > '
                f'{self.args.max_direct_error:.2f}m')
        aligned_max = summary['aligned_error_m'].get('max')
        if aligned_max is not None and aligned_max > self.args.max_aligned_error:
            failures.append(
                f'aligned max error {aligned_max:.2f}m > '
                f'{self.args.max_aligned_error:.2f}m')
        if not summary['gps_topic']:
            failures.append('GPS topic was not found')
        return summary, failures


def write_reports(out_prefix, rows, summary, failures, args):
    if not out_prefix:
        return
    prefix = Path(out_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    report = {
        'args': vars(args),
        'summary': summary,
        'validation_passed': not failures,
        'failures': failures,
        'rows': rows,
    }
    json_path = prefix.with_suffix('.json')
    csv_path = prefix.with_suffix('.csv')
    md_path = prefix.with_suffix('.md')
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    fieldnames = [
        'wall_time', 'tf_stamp', 'gps_x', 'gps_y', 'gps_rel_x', 'gps_rel_y',
        'estimate_x', 'estimate_y', 'direct_error_m', 'aligned_error_m',
    ]
    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, '') for key in fieldnames})

    lines = [
        '# GPS vs SLAM Drift Evaluation',
        '',
        f"- gps_topic: `{summary.get('gps_topic')}`",
        f"- gps_type: `{summary.get('gps_type')}`",
        f"- estimate_frame: `{args.estimate_frame}`",
        f"- robot_frame: `{args.robot_frame}`",
        f"- validation_passed: `{str(not failures).lower()}`",
        '',
        '## Summary',
        '',
        f"- samples: `{summary['samples']}`",
        f"- gps_path_length_m: `{summary['gps_path_length_m']:.3f}`",
        f"- estimate_path_length_m: `{summary['estimate_path_length_m']:.3f}`",
    ]
    for label in ['direct_error_m', 'aligned_error_m']:
        stats = summary.get(label) or {}
        if stats:
            lines.append(
                f"- {label}: mean `{stats['mean']:.3f}`, "
                f"max `{stats['max']:.3f}`, rmse `{stats['rmse']:.3f}`, "
                f"final `{stats['final']:.3f}`")
    if summary.get('alignment_yaw_deg') is not None:
        lines.append(
            f"- alignment_yaw_deg: `{summary['alignment_yaw_deg']:.2f}`")
    if failures:
        lines.extend(['', '## Failures', ''])
        lines.extend(f'- {failure}' for failure in failures)
    lines.extend([
        '',
        '## Samples',
        '',
        '| idx | gps rel xy | estimate xy | direct err | aligned err |',
        '|---:|---|---|---:|---:|',
    ])
    for i, row in enumerate(rows):
        aligned = row.get('aligned_error_m')
        aligned_txt = '' if aligned is None else f'{aligned:.3f}'
        lines.append(
            f"| {i} | [{row['gps_rel_x']:.3f},{row['gps_rel_y']:.3f}] | "
            f"[{row['estimate_x']:.3f},{row['estimate_y']:.3f}] | "
            f"{row['direct_error_m']:.3f} | {aligned_txt} |")
    md_path.write_text('\n'.join(lines) + '\n')
    print(f'wrote {json_path}')
    print(f'wrote {csv_path}')
    print(f'wrote {md_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gps-topic', default='auto',
                        help='GPS topic, or auto to pick /gps or */gps.')
    parser.add_argument('--estimate-frame', default='map',
                        help='TF frame used as the SLAM/navigation estimate.')
    parser.add_argument('--robot-frame', default='base_footprint')
    parser.add_argument('--duration-sec', type=float, default=60.0)
    parser.add_argument('--sample-period', type=float, default=0.5)
    parser.add_argument('--min-samples', type=int, default=10)
    parser.add_argument('--min-path-length', type=float, default=1.0)
    parser.add_argument('--max-direct-error', type=float, default=2.0)
    parser.add_argument('--max-aligned-error', type=float, default=1.0)
    parser.add_argument('--out-prefix', default='')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('evaluate_gps_vs_slam')
    if args.use_sim_time:
        node.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
    evaluator = GpsVsSlamEvaluator(node, args)
    try:
        evaluator.spin()
        summary, failures = evaluator.summarize()
        print('=== summary ===')
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        if failures:
            print('=== validation failures ===')
            for failure in failures:
                print(f'- {failure}')
        else:
            print('validation_passed=true')
        write_reports(args.out_prefix, evaluator.rows, summary, failures, args)
        return 2 if failures and args.require_pass else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
