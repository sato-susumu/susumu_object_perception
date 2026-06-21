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
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('filtered_odom_topic', '')
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
        self.declare_parameter('stop_status_topic', '')
        self.declare_parameter('stop_status_patterns', '')

        self.gps_topic_param = str(self.get_parameter('gps_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.estimate_frame = str(self.get_parameter('estimate_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.filtered_odom_topic = str(
            self.get_parameter('filtered_odom_topic').value)
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
        self.stop_status_topic = str(
            self.get_parameter('stop_status_topic').value)
        self.stop_status_patterns = [
            item.strip() for item in str(
                self.get_parameter('stop_status_patterns').value).split(',')
            if item.strip()
        ]
        self.latest_status_text = ''
        self.current_waypoint_index = None
        self.stop_status_message = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gps_sub = None
        self.imu_sub = None
        self.gps_topic = None
        self.gps_type = None
        self.latest_gps = None
        self.latest_imu_yaw = None
        self.latest_filtered_odom = None
        self.yaw_offset = None
        self.odom_yaw_offset = None
        self.filtered_yaw_offset = None
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
        self.stop_status_sub = None
        if self.stop_status_topic and self.stop_status_patterns:
            self.stop_status_sub = self.create_subscription(
                String, self.stop_status_topic, self._on_status, 10)
        if self.imu_topic:
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)
            self.imu_sub = self.create_subscription(
                Imu, self.imu_topic, self._on_imu, qos)
        self.filtered_odom_sub = None
        if self.filtered_odom_topic:
            self.filtered_odom_sub = self.create_subscription(
                Odometry, self.filtered_odom_topic,
                self._on_filtered_odom, 10)
        self.create_timer(1.0, self._try_attach_gps)
        self.create_timer(max(0.1, self.sample_period), self._sample)
        self.create_timer(max(1.0, self.report_period), self._periodic_report)
        self._status('live SLAM truth monitor started')

    def _on_status(self, msg):
        text = str(msg.data)
        self.latest_status_text = text
        self._update_waypoint_context(text)
        if not any(pattern in text for pattern in self.stop_status_patterns):
            return
        self.stop_status_message = text
        self._status(f'stop status matched: {text}')
        self._write_reports()
        if rclpy.ok():
            rclpy.shutdown()

    def _update_waypoint_context(self, text):
        marker = 'heading to waypoint #'
        if marker not in text:
            return
        rest = text.split(marker, 1)[1]
        digits = []
        for ch in rest:
            if ch.isdigit():
                digits.append(ch)
            else:
                break
        if digits:
            self.current_waypoint_index = int(''.join(digits))

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

    def _on_filtered_odom(self, msg):
        self.latest_filtered_odom = msg

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

    def _pose_in_frame(self, frame):
        if not frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.robot_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().debug(
                f'no TF {frame} <- {self.robot_frame}: {exc}')
            return None
        tr = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        return (
            np.array([float(tr.x), float(tr.y)], dtype=np.float64),
            yaw,
            stamp_to_float(tf.header.stamp),
        )

    def _estimate_pose(self):
        return self._pose_in_frame(self.estimate_frame)

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
        odom = self._pose_in_frame(self.odom_frame)
        odom_xy = None
        odom_yaw = None
        odom_stamp = ''
        if odom is not None:
            odom_xy, odom_yaw, odom_stamp = odom
        filtered_xy = None
        filtered_yaw = None
        filtered_stamp = ''
        if self.latest_filtered_odom is not None:
            filtered_pose = self.latest_filtered_odom.pose.pose
            filtered_xy = np.array([
                float(filtered_pose.position.x),
                float(filtered_pose.position.y),
            ], dtype=np.float64)
            filtered_yaw = quat_to_yaw(filtered_pose.orientation)
            filtered_stamp = stamp_to_float(
                self.latest_filtered_odom.header.stamp)
        gps_rel = self.latest_gps - self.gps_origin
        truth_yaw = ''
        yaw_error = ''
        odom_yaw_error = ''
        filtered_yaw_error = ''
        if self.latest_imu_yaw is not None:
            if self.yaw_offset is None:
                self.yaw_offset = wrap_angle(est_yaw - self.latest_imu_yaw)
            truth_yaw_rad = wrap_angle(self.latest_imu_yaw + self.yaw_offset)
            truth_yaw = math.degrees(truth_yaw_rad)
            yaw_error = abs(math.degrees(wrap_angle(est_yaw - truth_yaw_rad)))
            if odom_yaw is not None:
                if self.odom_yaw_offset is None:
                    self.odom_yaw_offset = wrap_angle(
                        odom_yaw - self.latest_imu_yaw)
                odom_truth_yaw_rad = wrap_angle(
                    self.latest_imu_yaw + self.odom_yaw_offset)
                odom_yaw_error = abs(math.degrees(
                    wrap_angle(odom_yaw - odom_truth_yaw_rad)))
            if filtered_yaw is not None:
                if self.filtered_yaw_offset is None:
                    self.filtered_yaw_offset = wrap_angle(
                        filtered_yaw - self.latest_imu_yaw)
                filtered_truth_yaw_rad = wrap_angle(
                    self.latest_imu_yaw + self.filtered_yaw_offset)
                filtered_yaw_error = abs(math.degrees(
                    wrap_angle(filtered_yaw - filtered_truth_yaw_rad)))
        row = {
            'idx': len(self.rows),
            'wall_time': time.time(),
            'tf_stamp': tf_stamp,
            'odom_tf_stamp': odom_stamp,
            'filtered_odom_stamp': filtered_stamp,
            'gps_rel_x': float(gps_rel[0]),
            'gps_rel_y': float(gps_rel[1]),
            'estimate_x': float(est_xy[0]),
            'estimate_y': float(est_xy[1]),
            'odom_x': '' if odom_xy is None else float(odom_xy[0]),
            'odom_y': '' if odom_xy is None else float(odom_xy[1]),
            'filtered_x': (
                '' if filtered_xy is None else float(filtered_xy[0])),
            'filtered_y': (
                '' if filtered_xy is None else float(filtered_xy[1])),
            'truth_yaw_deg': truth_yaw,
            'estimate_yaw_deg': float(math.degrees(est_yaw)),
            'odom_yaw_deg': '' if odom_yaw is None else float(
                math.degrees(odom_yaw)),
            'filtered_yaw_deg': (
                '' if filtered_yaw is None
                else float(math.degrees(filtered_yaw))),
            'waypoint_index': (
                '' if self.current_waypoint_index is None
                else int(self.current_waypoint_index)),
            'status_text': self.latest_status_text,
            'direct_error_m': float(np.linalg.norm(est_xy - gps_rel)),
            'odom_direct_error_m': (
                '' if odom_xy is None
                else float(np.linalg.norm(odom_xy - gps_rel))),
            'filtered_direct_error_m': (
                '' if filtered_xy is None
                else float(np.linalg.norm(filtered_xy - gps_rel))),
            'aligned_error_m': '',
            'odom_aligned_error_m': '',
            'filtered_aligned_error_m': '',
            'heading_error_deg': '',
            'odom_heading_error_deg': '',
            'filtered_heading_error_deg': '',
            'yaw_error_deg': yaw_error,
            'odom_yaw_error_deg': odom_yaw_error,
            'filtered_yaw_error_deg': filtered_yaw_error,
            'event': '',
        }
        self.rows.append(row)
        self.last_sample_wall = now
        self._update_alignment_and_events(now)

    def _update_alignment_and_events(self, now):
        map_metrics = self._update_aligned_metrics(
            'estimate_x', 'estimate_y',
            'aligned_error_m', 'heading_error_deg')
        self._update_aligned_metrics(
            'odom_x', 'odom_y',
            'odom_aligned_error_m', 'odom_heading_error_deg')
        self._update_aligned_metrics(
            'filtered_x', 'filtered_y',
            'filtered_aligned_error_m', 'filtered_heading_error_deg')
        if map_metrics is None:
            return
        latest, aligned_error, heading_error = map_metrics

        reasons = []
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
                'waypoint_index': latest.get('waypoint_index'),
                'status_text': latest.get('status_text', ''),
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

    def _update_aligned_metrics(self, x_key, y_key, error_key, heading_key):
        valid_rows = [
            row for row in self.rows
            if row.get(x_key) != '' and row.get(y_key) != ''
        ]
        if len(valid_rows) < self.min_align_samples:
            return None
        gps = np.array([[r['gps_rel_x'], r['gps_rel_y']]
                        for r in valid_rows], dtype=np.float64)
        est = np.array([[r[x_key], r[y_key]]
                        for r in valid_rows], dtype=np.float64)
        if path_length(gps) < self.min_align_path_length:
            return None
        aligned, _, _ = best_fit_2d(gps, est)
        if aligned is None:
            return None
        aligned_errors = np.linalg.norm(est - aligned, axis=1)
        for row, err in zip(valid_rows, aligned_errors):
            row[error_key] = float(err)
        latest = valid_rows[-1]
        heading_error = self._latest_heading_error(valid_rows, aligned, est)
        if heading_error is not None:
            latest[heading_key] = float(heading_error)
        return latest, float(aligned_errors[-1]), heading_error

    def _latest_heading_error(self, rows, aligned_gps, estimate):
        if len(rows) < 2:
            return None
        latest = len(rows) - 1
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
        odom_aligned = latest.get('odom_aligned_error_m')
        filtered_aligned = latest.get('filtered_aligned_error_m')
        aligned_txt = '' if aligned == '' else f' aligned={aligned:.2f}m'
        heading_txt = '' if heading == '' else f' heading={heading:.1f}deg'
        yaw_txt = '' if yaw == '' else f' yaw={float(yaw):.1f}deg'
        odom_txt = (
            '' if odom_aligned == ''
            else f' odom_aligned={odom_aligned:.2f}m')
        filtered_txt = (
            '' if filtered_aligned == ''
            else f' filtered_aligned={filtered_aligned:.2f}m')
        self._status(
            f'samples={len(self.rows)} events={len(self.events)}'
            f'{aligned_txt}{heading_txt}{yaw_txt}{odom_txt}{filtered_txt}')
        self._write_reports()

    def _summary(self):
        gps = np.array([[r['gps_rel_x'], r['gps_rel_y']]
                        for r in self.rows], dtype=np.float64)
        est = np.array([[r['estimate_x'], r['estimate_y']]
                        for r in self.rows], dtype=np.float64)
        odom_pts = np.array(
            [[r['odom_x'], r['odom_y']] for r in self.rows
             if r.get('odom_x') != '' and r.get('odom_y') != ''],
            dtype=np.float64)
        filtered_pts = np.array(
            [[r['filtered_x'], r['filtered_y']] for r in self.rows
             if r.get('filtered_x') != '' and r.get('filtered_y') != ''],
            dtype=np.float64)
        aligned_errors = self._numeric_values('aligned_error_m')
        heading_errors = self._numeric_values('heading_error_deg')
        yaw_errors = self._numeric_values('yaw_error_deg')
        odom_aligned_errors = self._numeric_values('odom_aligned_error_m')
        odom_heading_errors = self._numeric_values('odom_heading_error_deg')
        odom_yaw_errors = self._numeric_values('odom_yaw_error_deg')
        filtered_aligned_errors = self._numeric_values(
            'filtered_aligned_error_m')
        filtered_heading_errors = self._numeric_values(
            'filtered_heading_error_deg')
        filtered_yaw_errors = self._numeric_values(
            'filtered_yaw_error_deg')
        return {
            'gps_topic': self.gps_topic,
            'gps_type': self.gps_type,
            'imu_topic': self.imu_topic,
            'estimate_frame': self.estimate_frame,
            'odom_frame': self.odom_frame,
            'filtered_odom_topic': self.filtered_odom_topic,
            'robot_frame': self.robot_frame,
            'samples': len(self.rows),
            'events': len(self.events),
            'gps_path_length_m': path_length(gps) if len(gps) else 0.0,
            'estimate_path_length_m': path_length(est) if len(est) else 0.0,
            'odom_path_length_m': (
                path_length(odom_pts) if len(odom_pts) else 0.0),
            'filtered_path_length_m': (
                path_length(filtered_pts) if len(filtered_pts) else 0.0),
            'max_aligned_error_m': (
                max(aligned_errors) if aligned_errors else None),
            'max_heading_error_deg': (
                max(heading_errors) if heading_errors else None),
            'max_yaw_error_deg': (
                max(yaw_errors) if yaw_errors else None),
            'max_odom_aligned_error_m': (
                max(odom_aligned_errors) if odom_aligned_errors else None),
            'max_odom_heading_error_deg': (
                max(odom_heading_errors) if odom_heading_errors else None),
            'max_odom_yaw_error_deg': (
                max(odom_yaw_errors) if odom_yaw_errors else None),
            'max_filtered_aligned_error_m': (
                max(filtered_aligned_errors)
                if filtered_aligned_errors else None),
            'max_filtered_heading_error_deg': (
                max(filtered_heading_errors)
                if filtered_heading_errors else None),
            'max_filtered_yaw_error_deg': (
                max(filtered_yaw_errors) if filtered_yaw_errors else None),
            'threshold_aligned_error_m': self.max_aligned_error,
            'threshold_heading_error_deg': self.max_heading_error_deg,
            'threshold_yaw_error_deg': self.max_yaw_error_deg,
            'stop_status_topic': self.stop_status_topic,
            'stop_status_patterns': list(self.stop_status_patterns),
            'stop_status_message': self.stop_status_message,
            'current_waypoint_index': self.current_waypoint_index,
            'worst_waypoint': self._worst_waypoint_summary(),
        }

    def _numeric_values(self, key):
        return [
            float(row[key]) for row in self.rows
            if row.get(key) not in ('', None)
        ]

    def _waypoint_summaries(self):
        summaries = {}
        for row in self.rows:
            waypoint = row.get('waypoint_index')
            if waypoint == '' or waypoint is None:
                continue
            key = str(int(waypoint))
            item = summaries.setdefault(key, {
                'waypoint_index': int(waypoint),
                'samples': 0,
                'events': 0,
                'max_aligned_error_m': None,
                'max_heading_error_deg': None,
                'max_yaw_error_deg': None,
                'max_odom_aligned_error_m': None,
                'max_odom_heading_error_deg': None,
                'max_odom_yaw_error_deg': None,
                'max_filtered_aligned_error_m': None,
                'max_filtered_heading_error_deg': None,
                'max_filtered_yaw_error_deg': None,
            })
            item['samples'] += 1
            if row.get('event'):
                item['events'] += 1
            aligned = row.get('aligned_error_m')
            if aligned != '':
                value = float(aligned)
                item['max_aligned_error_m'] = (
                    value if item['max_aligned_error_m'] is None
                    else max(item['max_aligned_error_m'], value))
            heading = row.get('heading_error_deg')
            if heading != '':
                value = float(heading)
                item['max_heading_error_deg'] = (
                    value if item['max_heading_error_deg'] is None
                    else max(item['max_heading_error_deg'], value))
            yaw = row.get('yaw_error_deg')
            if yaw != '':
                value = float(yaw)
                item['max_yaw_error_deg'] = (
                    value if item['max_yaw_error_deg'] is None
                    else max(item['max_yaw_error_deg'], value))
            odom_aligned = row.get('odom_aligned_error_m')
            if odom_aligned != '':
                value = float(odom_aligned)
                item['max_odom_aligned_error_m'] = (
                    value if item['max_odom_aligned_error_m'] is None
                    else max(item['max_odom_aligned_error_m'], value))
            odom_heading = row.get('odom_heading_error_deg')
            if odom_heading != '':
                value = float(odom_heading)
                item['max_odom_heading_error_deg'] = (
                    value if item['max_odom_heading_error_deg'] is None
                    else max(item['max_odom_heading_error_deg'], value))
            odom_yaw = row.get('odom_yaw_error_deg')
            if odom_yaw != '':
                value = float(odom_yaw)
                item['max_odom_yaw_error_deg'] = (
                    value if item['max_odom_yaw_error_deg'] is None
                    else max(item['max_odom_yaw_error_deg'], value))
            filtered_aligned = row.get('filtered_aligned_error_m')
            if filtered_aligned != '':
                value = float(filtered_aligned)
                item['max_filtered_aligned_error_m'] = (
                    value if item['max_filtered_aligned_error_m'] is None
                    else max(item['max_filtered_aligned_error_m'], value))
            filtered_heading = row.get('filtered_heading_error_deg')
            if filtered_heading != '':
                value = float(filtered_heading)
                item['max_filtered_heading_error_deg'] = (
                    value if item['max_filtered_heading_error_deg'] is None
                    else max(item['max_filtered_heading_error_deg'], value))
            filtered_yaw = row.get('filtered_yaw_error_deg')
            if filtered_yaw != '':
                value = float(filtered_yaw)
                item['max_filtered_yaw_error_deg'] = (
                    value if item['max_filtered_yaw_error_deg'] is None
                    else max(item['max_filtered_yaw_error_deg'], value))
        return [
            summaries[key] for key in sorted(summaries, key=lambda x: int(x))
        ]

    def _worst_waypoint_summary(self):
        candidates = [
            item for item in self._waypoint_summaries()
            if item['max_aligned_error_m'] is not None
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda item: item['max_aligned_error_m'])

    def _write_reports(self):
        if not self.report_prefix:
            return
        prefix = Path(self.report_prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        summary = self._summary()
        report = {
            'summary': summary,
            'events': self.events,
            'waypoints': self._waypoint_summaries(),
            'rows': self.rows,
        }
        prefix.with_suffix('.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        fieldnames = [
            'idx', 'wall_time', 'tf_stamp', 'odom_tf_stamp',
            'filtered_odom_stamp',
            'gps_rel_x', 'gps_rel_y', 'estimate_x', 'estimate_y',
            'odom_x', 'odom_y', 'filtered_x', 'filtered_y',
            'waypoint_index', 'status_text',
            'truth_yaw_deg', 'estimate_yaw_deg', 'odom_yaw_deg',
            'filtered_yaw_deg',
            'direct_error_m', 'odom_direct_error_m',
            'filtered_direct_error_m',
            'aligned_error_m', 'odom_aligned_error_m',
            'filtered_aligned_error_m',
            'heading_error_deg', 'odom_heading_error_deg',
            'filtered_heading_error_deg',
            'yaw_error_deg', 'odom_yaw_error_deg',
            'filtered_yaw_error_deg', 'event',
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
            f"- odom_frame: `{summary['odom_frame']}`",
            f"- filtered_odom_topic: `{summary['filtered_odom_topic']}`",
            f"- robot_frame: `{summary['robot_frame']}`",
            f"- samples: `{summary['samples']}`",
            f"- events: `{summary['events']}`",
            f"- gps_path_length_m: `{summary['gps_path_length_m']:.3f}`",
            f"- estimate_path_length_m: `{summary['estimate_path_length_m']:.3f}`",
            f"- odom_path_length_m: `{summary['odom_path_length_m']:.3f}`",
            f"- filtered_path_length_m: `{summary['filtered_path_length_m']:.3f}`",
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
        if summary['max_odom_aligned_error_m'] is not None:
            lines.append(
                f"- max_odom_aligned_error_m: "
                f"`{summary['max_odom_aligned_error_m']:.3f}`")
        if summary['max_odom_heading_error_deg'] is not None:
            lines.append(
                f"- max_odom_heading_error_deg: "
                f"`{summary['max_odom_heading_error_deg']:.2f}`")
        if summary['max_odom_yaw_error_deg'] is not None:
            lines.append(
                f"- max_odom_yaw_error_deg: "
                f"`{summary['max_odom_yaw_error_deg']:.2f}`")
        if summary['max_filtered_aligned_error_m'] is not None:
            lines.append(
                f"- max_filtered_aligned_error_m: "
                f"`{summary['max_filtered_aligned_error_m']:.3f}`")
        if summary['max_filtered_heading_error_deg'] is not None:
            lines.append(
                f"- max_filtered_heading_error_deg: "
                f"`{summary['max_filtered_heading_error_deg']:.2f}`")
        if summary['max_filtered_yaw_error_deg'] is not None:
            lines.append(
                f"- max_filtered_yaw_error_deg: "
                f"`{summary['max_filtered_yaw_error_deg']:.2f}`")
        worst = summary.get('worst_waypoint')
        if worst is not None:
            lines.append(
                f"- worst_waypoint: `#{worst['waypoint_index']}` "
                f"max_aligned_error_m=`{worst['max_aligned_error_m']:.3f}`")
        lines.extend(['', '## Waypoints', ''])
        waypoint_summaries = self._waypoint_summaries()
        if waypoint_summaries:
            lines.extend([
                '| waypoint | samples | events | max aligned | max yaw | max odom aligned | max odom yaw | max filtered aligned | max filtered yaw |',
                '|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
            ])
            for item in waypoint_summaries:
                aligned = item.get('max_aligned_error_m')
                yaw = item.get('max_yaw_error_deg')
                odom_aligned = item.get('max_odom_aligned_error_m')
                odom_yaw = item.get('max_odom_yaw_error_deg')
                filtered_aligned = item.get('max_filtered_aligned_error_m')
                filtered_yaw = item.get('max_filtered_yaw_error_deg')
                lines.append(
                    f"| {item['waypoint_index']} | {item['samples']} | "
                    f"{item['events']} | "
                    f"{'' if aligned is None else f'{aligned:.3f}'} | "
                    f"{'' if yaw is None else f'{yaw:.2f}'} | "
                    f"{'' if odom_aligned is None else f'{odom_aligned:.3f}'} | "
                    f"{'' if odom_yaw is None else f'{odom_yaw:.2f}'} | "
                    f"{'' if filtered_aligned is None else f'{filtered_aligned:.3f}'} | "
                    f"{'' if filtered_yaw is None else f'{filtered_yaw:.2f}'} |")
        else:
            lines.append('No waypoint context samples.')
        lines.extend(['', '## Events', ''])
        if self.events:
            lines.extend([
                '| idx | waypoint | gps rel xy | estimate xy | aligned err | heading err | yaw err | reason |',
                '|---:|---:|---|---|---:|---:|---:|---|',
            ])
            for event in self.events:
                heading = event.get('heading_error_deg')
                heading_txt = '' if heading is None else f'{heading:.2f}'
                yaw = event.get('yaw_error_deg')
                yaw_txt = '' if yaw is None else f'{yaw:.2f}'
                waypoint = event.get('waypoint_index')
                waypoint_txt = '' if waypoint in (None, '') else str(waypoint)
                lines.append(
                    f"| {event['idx']} | {waypoint_txt} | "
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
