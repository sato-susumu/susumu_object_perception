#!/usr/bin/env python3
"""Live monitor comparing Webots GPS ground truth with SLAM/Nav2 TF pose.

The monitor is intentionally observational: it does not feed ground truth back
into SLAM or Nav2. It records where the simulator truth and the robot estimate
diverge, so mapping/navigation tuning can be based on concrete drift events.
"""

import csv
import json
import math
import time
from pathlib import Path

import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
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


def best_fit_2d(source_xy, target_xy):
    """Return source aligned to target by a 2D rigid transform."""
    src = np.asarray(source_xy, dtype=np.float64)
    tgt = np.asarray(target_xy, dtype=np.float64)
    if src.shape[0] < 3:
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


def angle_between_deg(a, b):
    la = float(np.linalg.norm(a))
    lb = float(np.linalg.norm(b))
    if la < 1e-6 or lb < 1e-6:
        return None
    cross = a[0] * b[1] - a[1] * b[0]
    dot = float(np.dot(a, b))
    return abs(math.degrees(math.atan2(cross, dot)))


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LiveSlamTruthMonitor(Node):

    def __init__(self):
        super().__init__('live_slam_truth_monitor')
        self.declare_parameter('gps_topic', 'auto')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('estimate_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('sample_period', 0.5)
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('min_align_samples', 8)
        self.declare_parameter('min_align_path_length', 1.0)
        self.declare_parameter('heading_window_samples', 8)
        self.declare_parameter('heading_min_baseline', 0.8)
        self.declare_parameter('max_aligned_error', 0.6)
        self.declare_parameter('max_heading_error_deg', 30.0)
        self.declare_parameter('max_yaw_error_deg', 8.0)
        self.declare_parameter('event_cooldown_sec', 5.0)
        self.declare_parameter('report_prefix', '')

        self.gps_topic_param = str(self.get_parameter('gps_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.estimate_frame = str(self.get_parameter('estimate_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.sample_period = float(self.get_parameter('sample_period').value)
        self.report_period = float(self.get_parameter('report_period').value)
        self.min_align_samples = int(
            self.get_parameter('min_align_samples').value)
        self.min_align_path_length = float(
            self.get_parameter('min_align_path_length').value)
        self.heading_window_samples = int(
            self.get_parameter('heading_window_samples').value)
        self.heading_min_baseline = float(
            self.get_parameter('heading_min_baseline').value)
        self.max_aligned_error = float(
            self.get_parameter('max_aligned_error').value)
        self.max_heading_error_deg = float(
            self.get_parameter('max_heading_error_deg').value)
        self.max_yaw_error_deg = float(
            self.get_parameter('max_yaw_error_deg').value)
        self.event_cooldown_sec = float(
            self.get_parameter('event_cooldown_sec').value)
        self.report_prefix = str(self.get_parameter('report_prefix').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gps_sub = None
        self.imu_sub = None
        self.gps_topic = None
        self.gps_type = None
        self.latest_gps = None
        self.latest_imu_yaw = None
        self.yaw_offset = None
        self.gps_origin = None
        self.latlon_origin = None
        self.rows = []
        self.events = []
        self.last_sample_wall = 0.0
        self.last_event_wall = 0.0

        self.status_pub = self.create_publisher(
            String, '/slam_truth_monitor/status', 10)
        self.event_pub = self.create_publisher(
            String, '/slam_truth_monitor/event', 10)
        if self.imu_topic:
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)
            self.imu_sub = self.create_subscription(
                Imu, self.imu_topic, self._on_imu, qos)
        self.create_timer(1.0, self._try_attach_gps)
        self.create_timer(max(0.1, self.sample_period), self._sample)
        self.create_timer(max(1.0, self.report_period), self._periodic_report)
        self._status('live SLAM truth monitor started')

    def _candidate_topics(self):
        if self.gps_topic_param != 'auto':
            return [self.gps_topic_param]
        names = [name for name, _ in self.get_topic_names_and_types()]
        preferred = ['/gps', '/TurtleBot3Burger/gps']
        out = [topic for topic in preferred if topic in names]
        out.extend(sorted(name for name in names
                          if name.endswith('/gps') and name not in out))
        return out

    def _try_attach_gps(self):
        if self.gps_sub is not None:
            return
        topics = dict(self.get_topic_names_and_types())
        for topic in self._candidate_topics():
            for type_name in topics.get(topic, []):
                msg_type = SUPPORTED_GPS_TYPES.get(type_name)
                if msg_type is None:
                    continue
                qos = QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST)
                self.gps_sub = self.create_subscription(
                    msg_type, topic, self._on_gps, qos)
                self.gps_topic = topic
                self.gps_type = type_name
                self._status(f'using GPS truth topic {topic} ({type_name})')
                return
        self._status('waiting for GPS truth topic')

    def _on_gps(self, msg):
        xy = self._gps_xy(msg)
        if xy is None:
            return
        if self.gps_origin is None:
            self.gps_origin = np.array(xy, dtype=np.float64)
        self.latest_gps = np.array(xy, dtype=np.float64)

    def _on_imu(self, msg):
        self.latest_imu_yaw = quat_to_yaw(msg.orientation)

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

    def _estimate_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.estimate_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().debug(
                f'no TF {self.estimate_frame} <- {self.robot_frame}: {exc}')
            return None
        tr = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        return (
            np.array([float(tr.x), float(tr.y)], dtype=np.float64),
            yaw,
            stamp_to_float(tf.header.stamp),
        )

    def _sample(self):
        now = time.monotonic()
        if now - self.last_sample_wall < self.sample_period:
            return
        if self.latest_gps is None or self.gps_origin is None:
            return
        estimate = self._estimate_pose()
        if estimate is None:
            return
        est_xy, est_yaw, tf_stamp = estimate
        gps_rel = self.latest_gps - self.gps_origin
        truth_yaw = ''
        yaw_error = ''
        if self.latest_imu_yaw is not None:
            if self.yaw_offset is None:
                self.yaw_offset = wrap_angle(est_yaw - self.latest_imu_yaw)
            truth_yaw_rad = wrap_angle(self.latest_imu_yaw + self.yaw_offset)
            truth_yaw = math.degrees(truth_yaw_rad)
            yaw_error = abs(math.degrees(wrap_angle(est_yaw - truth_yaw_rad)))
        row = {
            'idx': len(self.rows),
            'wall_time': time.time(),
            'tf_stamp': tf_stamp,
            'gps_rel_x': float(gps_rel[0]),
            'gps_rel_y': float(gps_rel[1]),
            'estimate_x': float(est_xy[0]),
            'estimate_y': float(est_xy[1]),
            'truth_yaw_deg': truth_yaw,
            'estimate_yaw_deg': float(math.degrees(est_yaw)),
            'direct_error_m': float(np.linalg.norm(est_xy - gps_rel)),
            'aligned_error_m': '',
            'heading_error_deg': '',
            'yaw_error_deg': yaw_error,
            'event': '',
        }
        self.rows.append(row)
        self.last_sample_wall = now
        self._update_alignment_and_events(now)

    def _update_alignment_and_events(self, now):
        if len(self.rows) < self.min_align_samples:
            return
        gps = np.array([[r['gps_rel_x'], r['gps_rel_y']]
                        for r in self.rows], dtype=np.float64)
        est = np.array([[r['estimate_x'], r['estimate_y']]
                        for r in self.rows], dtype=np.float64)
        if path_length(gps) < self.min_align_path_length:
            return
        aligned, _, _ = best_fit_2d(gps, est)
        if aligned is None:
            return
        aligned_errors = np.linalg.norm(est - aligned, axis=1)
        for row, err in zip(self.rows, aligned_errors):
            row['aligned_error_m'] = float(err)
        latest = self.rows[-1]
        heading_error = self._latest_heading_error(aligned, est)
        if heading_error is not None:
            latest['heading_error_deg'] = float(heading_error)

        reasons = []
        aligned_error = float(aligned_errors[-1])
        if aligned_error > self.max_aligned_error:
            reasons.append(
                f'aligned_error {aligned_error:.2f}m > '
                f'{self.max_aligned_error:.2f}m')
        if (heading_error is not None
                and heading_error > self.max_heading_error_deg):
            reasons.append(
                f'heading_error {heading_error:.1f}deg > '
                f'{self.max_heading_error_deg:.1f}deg')
        yaw_error = latest.get('yaw_error_deg')
        if yaw_error != '' and float(yaw_error) > self.max_yaw_error_deg:
            reasons.append(
                f'yaw_error {float(yaw_error):.1f}deg > '
                f'{self.max_yaw_error_deg:.1f}deg')
        if reasons and now - self.last_event_wall >= self.event_cooldown_sec:
            event = {
                'idx': latest['idx'],
                'wall_time': latest['wall_time'],
                'tf_stamp': latest['tf_stamp'],
                'gps_rel_x': latest['gps_rel_x'],
                'gps_rel_y': latest['gps_rel_y'],
                'estimate_x': latest['estimate_x'],
                'estimate_y': latest['estimate_y'],
                'truth_yaw_deg': latest.get('truth_yaw_deg'),
                'estimate_yaw_deg': latest.get('estimate_yaw_deg'),
                'aligned_error_m': aligned_error,
                'heading_error_deg': heading_error,
                'yaw_error_deg': (
                    None if yaw_error == '' else float(yaw_error)),
                'reasons': reasons,
            }
            latest['event'] = '; '.join(reasons)
            self.events.append(event)
            self.last_event_wall = now
            text = json.dumps(event, ensure_ascii=False)
            self.event_pub.publish(String(data=text))
            self.get_logger().warning('truth drift event: ' + text)
            self._write_reports()

    def _latest_heading_error(self, aligned_gps, estimate):
        if len(self.rows) < 2:
            return None
        latest = len(self.rows) - 1
        start = max(0, latest - self.heading_window_samples)
        for idx in range(start, latest):
            gps_vec = aligned_gps[latest] - aligned_gps[idx]
            est_vec = estimate[latest] - estimate[idx]
            if (np.linalg.norm(gps_vec) >= self.heading_min_baseline
                    and np.linalg.norm(est_vec) >= self.heading_min_baseline):
                return angle_between_deg(gps_vec, est_vec)
        return None

    def _periodic_report(self):
        if not self.rows:
            return
        latest = self.rows[-1]
        aligned = latest.get('aligned_error_m')
        heading = latest.get('heading_error_deg')
        yaw = latest.get('yaw_error_deg')
        aligned_txt = '' if aligned == '' else f' aligned={aligned:.2f}m'
        heading_txt = '' if heading == '' else f' heading={heading:.1f}deg'
        yaw_txt = '' if yaw == '' else f' yaw={float(yaw):.1f}deg'
        self._status(
            f'samples={len(self.rows)} events={len(self.events)}'
            f'{aligned_txt}{heading_txt}{yaw_txt}')
        self._write_reports()

    def _summary(self):
        gps = np.array([[r['gps_rel_x'], r['gps_rel_y']]
                        for r in self.rows], dtype=np.float64)
        est = np.array([[r['estimate_x'], r['estimate_y']]
                        for r in self.rows], dtype=np.float64)
        aligned_errors = [
            float(r['aligned_error_m']) for r in self.rows
            if r.get('aligned_error_m') != ''
        ]
        heading_errors = [
            float(r['heading_error_deg']) for r in self.rows
            if r.get('heading_error_deg') != ''
        ]
        yaw_errors = [
            float(r['yaw_error_deg']) for r in self.rows
            if r.get('yaw_error_deg') != ''
        ]
        return {
            'gps_topic': self.gps_topic,
            'gps_type': self.gps_type,
            'imu_topic': self.imu_topic,
            'estimate_frame': self.estimate_frame,
            'robot_frame': self.robot_frame,
            'samples': len(self.rows),
            'events': len(self.events),
            'gps_path_length_m': path_length(gps) if len(gps) else 0.0,
            'estimate_path_length_m': path_length(est) if len(est) else 0.0,
            'max_aligned_error_m': (
                max(aligned_errors) if aligned_errors else None),
            'max_heading_error_deg': (
                max(heading_errors) if heading_errors else None),
            'max_yaw_error_deg': (
                max(yaw_errors) if yaw_errors else None),
            'threshold_aligned_error_m': self.max_aligned_error,
            'threshold_heading_error_deg': self.max_heading_error_deg,
            'threshold_yaw_error_deg': self.max_yaw_error_deg,
        }

    def _write_reports(self):
        if not self.report_prefix:
            return
        prefix = Path(self.report_prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        summary = self._summary()
        report = {
            'summary': summary,
            'events': self.events,
            'rows': self.rows,
        }
        prefix.with_suffix('.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        fieldnames = [
            'idx', 'wall_time', 'tf_stamp',
            'gps_rel_x', 'gps_rel_y', 'estimate_x', 'estimate_y',
            'truth_yaw_deg', 'estimate_yaw_deg',
            'direct_error_m', 'aligned_error_m',
            'heading_error_deg', 'yaw_error_deg', 'event',
        ]
        with prefix.with_suffix('.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows:
                writer.writerow({key: row.get(key, '') for key in fieldnames})
        lines = [
            '# Live SLAM Truth Monitor',
            '',
            f"- gps_topic: `{summary['gps_topic']}`",
            f"- gps_type: `{summary['gps_type']}`",
            f"- imu_topic: `{summary['imu_topic']}`",
            f"- estimate_frame: `{summary['estimate_frame']}`",
            f"- robot_frame: `{summary['robot_frame']}`",
            f"- samples: `{summary['samples']}`",
            f"- events: `{summary['events']}`",
            f"- gps_path_length_m: `{summary['gps_path_length_m']:.3f}`",
            f"- estimate_path_length_m: `{summary['estimate_path_length_m']:.3f}`",
        ]
        if summary['max_aligned_error_m'] is not None:
            lines.append(
                f"- max_aligned_error_m: `{summary['max_aligned_error_m']:.3f}`")
        if summary['max_heading_error_deg'] is not None:
            lines.append(
                f"- max_heading_error_deg: `{summary['max_heading_error_deg']:.2f}`")
        if summary['max_yaw_error_deg'] is not None:
            lines.append(
                f"- max_yaw_error_deg: `{summary['max_yaw_error_deg']:.2f}`")
        lines.extend(['', '## Events', ''])
        if self.events:
            lines.extend([
                '| idx | gps rel xy | estimate xy | aligned err | heading err | yaw err | reason |',
                '|---:|---|---|---:|---:|---:|---|',
            ])
            for event in self.events:
                heading = event.get('heading_error_deg')
                heading_txt = '' if heading is None else f'{heading:.2f}'
                yaw = event.get('yaw_error_deg')
                yaw_txt = '' if yaw is None else f'{yaw:.2f}'
                lines.append(
                    f"| {event['idx']} | "
                    f"[{event['gps_rel_x']:.3f},{event['gps_rel_y']:.3f}] | "
                    f"[{event['estimate_x']:.3f},{event['estimate_y']:.3f}] | "
                    f"{event['aligned_error_m']:.3f} | {heading_txt} | "
                    f"{yaw_txt} | "
                    f"{'; '.join(event['reasons'])} |")
        else:
            lines.append('No drift events.')
        prefix.with_suffix('.md').write_text('\n'.join(lines) + '\n')

    def _status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = LiveSlamTruthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write_reports()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
