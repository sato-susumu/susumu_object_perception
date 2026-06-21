#!/usr/bin/env python3
"""Mapless GPS/IMU waypoint follower for sparse outdoor Webots worlds."""

import csv
import json
import math
import os
import time
from pathlib import Path

import yaml

import rclpy
from geometry_msgs.msg import PointStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import String


SUPPORTED_GPS_TYPES = {
    'sensor_msgs/msg/NavSatFix': NavSatFix,
    'geometry_msgs/msg/PointStamped': PointStamped,
    'geometry_msgs/msg/Vector3Stamped': Vector3Stamped,
    'nav_msgs/msg/Odometry': Odometry,
}
EARTH_RADIUS_M = 6378137.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def path_length(points):
    if len(points) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(points[:-1], points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


class OutdoorGpsWaypointNav(Node):
    def __init__(self):
        super().__init__('outdoor_gps_waypoint_nav')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('gps_topic', 'auto')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_prefix', '/tmp/outdoor_gps_waypoint_nav')
        self.declare_parameter('waypoints_are_relative', True)
        self.declare_parameter('goal_tolerance_m', 0.35)
        self.declare_parameter('mission_timeout_sec', 90.0)
        self.declare_parameter('waypoint_timeout_sec', 35.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('max_linear_mps', 0.22)
        self.declare_parameter('max_angular_radps', 0.9)
        self.declare_parameter('slow_radius_m', 0.8)
        self.declare_parameter('yaw_kp', 1.8)
        self.declare_parameter('obstacle_stop_range_m', 0.55)
        self.declare_parameter('obstacle_cone_deg', 45.0)
        self.declare_parameter('min_scan_points_for_stop', 3)

        self.waypoints_file = os.path.expanduser(
            self.get_parameter('waypoints_file').value)
        self.gps_topic_param = self.get_parameter('gps_topic').value
        self.output_prefix = os.path.expanduser(
            self.get_parameter('output_prefix').value)
        self.relative_waypoints = bool(
            self.get_parameter('waypoints_are_relative').value)
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance_m').value)
        self.mission_timeout = float(
            self.get_parameter('mission_timeout_sec').value)
        self.waypoint_timeout = float(
            self.get_parameter('waypoint_timeout_sec').value)
        rate = float(self.get_parameter('control_rate_hz').value)
        self.max_linear = float(
            self.get_parameter('max_linear_mps').value)
        self.max_angular = float(
            self.get_parameter('max_angular_radps').value)
        self.slow_radius = float(
            self.get_parameter('slow_radius_m').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.obstacle_stop_range = float(
            self.get_parameter('obstacle_stop_range_m').value)
        self.obstacle_cone = math.radians(float(
            self.get_parameter('obstacle_cone_deg').value))
        self.min_scan_points_for_stop = int(
            self.get_parameter('min_scan_points_for_stop').value)

        self.waypoints = self._load_waypoints(self.waypoints_file)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.status_pub = self.create_publisher(
            String, '/outdoor_gps_nav/status', 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._scan_cb, qos_profile_sensor_data)

        self.gps_sub = None
        self.gps_topic = None
        self.gps_type = None
        self.latest_gps = None
        self.gps_origin = None
        self.latlon_origin = None
        self.latest_imu_yaw = None
        self.latest_odom_yaw = None
        self.latest_scan = None
        self.last_local_xy = None
        self.last_motion_yaw = None
        self.current_idx = 0
        self.reached = []
        self.missed = []
        self.samples = []
        self.done = False
        self.started = time.monotonic()
        self.waypoint_started = self.started

        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self._status(
            f'loaded {len(self.waypoints)} gps waypoints from '
            f'{self.waypoints_file or "<default>"}')

    @staticmethod
    def _load_waypoints(path):
        if path:
            data = yaml.safe_load(open(path))
            raw = data.get('waypoints', data)
        else:
            raw = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]
        waypoints = []
        for i, item in enumerate(raw):
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                raise ValueError(f'invalid waypoint #{i}: {item}')
            waypoints.append([float(item[0]), float(item[1])])
        if not waypoints:
            raise ValueError('waypoints are empty')
        return waypoints

    def _status(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

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
            return True
        topics = dict(self.get_topic_names_and_types())
        for topic in self._candidate_topics():
            for type_name in topics.get(topic, []):
                msg_type = SUPPORTED_GPS_TYPES.get(type_name)
                if msg_type is None:
                    continue
                self.gps_topic = topic
                self.gps_type = type_name
                self.gps_sub = self.create_subscription(
                    msg_type, topic, self._gps_cb, 10)
                self._status(f'using GPS topic {topic} ({type_name})')
                return True
        return False

    def _gps_cb(self, msg):
        xy = self._gps_xy(msg)
        if xy is None:
            return
        if self.gps_origin is None:
            self.gps_origin = [xy[0], xy[1]]
            self._status(
                f'gps origin set to ({xy[0]:.3f}, {xy[1]:.3f})')
        local = [xy[0] - self.gps_origin[0], xy[1] - self.gps_origin[1]]
        if self.last_local_xy is not None:
            dx = local[0] - self.last_local_xy[0]
            dy = local[1] - self.last_local_xy[1]
            if math.hypot(dx, dy) > 0.05:
                self.last_motion_yaw = math.atan2(dy, dx)
        self.latest_gps = local if self.relative_waypoints else [xy[0], xy[1]]
        self.last_local_xy = local

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

    def _imu_cb(self, msg):
        self.latest_imu_yaw = quat_to_yaw(msg.orientation)

    def _odom_cb(self, msg):
        self.latest_odom_yaw = quat_to_yaw(msg.pose.pose.orientation)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _heading_yaw(self):
        if self.latest_imu_yaw is not None:
            return self.latest_imu_yaw
        if self.latest_odom_yaw is not None:
            return self.latest_odom_yaw
        return self.last_motion_yaw

    def _obstacle_ahead(self):
        scan = self.latest_scan
        if scan is None or self.obstacle_stop_range <= 0.0:
            return False, None, 0
        count = 0
        min_range = None
        angle = scan.angle_min
        for value in scan.ranges:
            if abs(angle) <= self.obstacle_cone and math.isfinite(value):
                if scan.range_min <= value <= self.obstacle_stop_range:
                    count += 1
                    min_range = value if min_range is None else min(min_range, value)
            angle += scan.angle_increment
        return count >= self.min_scan_points_for_stop, min_range, count

    def _record_sample(self, target, dist, yaw_err, obstacle, obstacle_range):
        now = time.monotonic() - self.started
        pos = self.latest_gps or [None, None]
        self.samples.append({
            'time_sec': now,
            'waypoint_index': self.current_idx,
            'x': pos[0],
            'y': pos[1],
            'target_x': target[0],
            'target_y': target[1],
            'distance_m': dist,
            'yaw_error_deg': None if yaw_err is None else math.degrees(yaw_err),
            'obstacle_stop': obstacle,
            'obstacle_min_range_m': obstacle_range,
        })

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _tick(self):
        if self.done:
            return
        self._try_attach_gps()
        elapsed = time.monotonic() - self.started
        if elapsed >= self.mission_timeout:
            for idx in range(self.current_idx, len(self.waypoints)):
                if idx not in self.missed and idx not in self.reached:
                    self.missed.append(idx)
            self._finish('mission_timeout')
            return
        if self.latest_gps is None:
            self._stop()
            return
        if self.current_idx >= len(self.waypoints):
            self._finish('complete')
            return

        target = self.waypoints[self.current_idx]
        dx = target[0] - self.latest_gps[0]
        dy = target[1] - self.latest_gps[1]
        dist = math.hypot(dx, dy)
        heading = self._heading_yaw()
        yaw_err = None if heading is None else angle_diff(math.atan2(dy, dx), heading)
        obstacle, obstacle_range, _ = self._obstacle_ahead()
        self._record_sample(target, dist, yaw_err, obstacle, obstacle_range)

        if dist <= self.goal_tolerance:
            self.reached.append(self.current_idx)
            self._status(
                f'waypoint #{self.current_idx} reached at '
                f'{dist:.2f}m')
            self.current_idx += 1
            self.waypoint_started = time.monotonic()
            self._stop()
            return
        if time.monotonic() - self.waypoint_started >= self.waypoint_timeout:
            self.missed.append(self.current_idx)
            self._status(
                f'waypoint #{self.current_idx} timeout at {dist:.2f}m')
            self.current_idx += 1
            self.waypoint_started = time.monotonic()
            self._stop()
            return

        cmd = Twist()
        if obstacle:
            self._status(
                f'obstacle stop before waypoint #{self.current_idx} '
                f'range={obstacle_range}')
        else:
            if yaw_err is None:
                angular = 0.0
                linear_scale = 0.5
            else:
                angular = clamp(self.yaw_kp * yaw_err,
                                -self.max_angular, self.max_angular)
                linear_scale = max(0.0, math.cos(yaw_err))
            cmd.angular.z = angular
            cmd.linear.x = self.max_linear * min(1.0, dist / self.slow_radius)
            cmd.linear.x *= linear_scale
        self.cmd_pub.publish(cmd)

    def _finish(self, reason):
        self.done = True
        self._stop()
        self._status(
            f'gps nav finished reason={reason} '
            f'reached={len(self.reached)}/{len(self.waypoints)} '
            f'missed={self.missed}')
        self._write_reports(reason)

    def _write_reports(self, reason):
        prefix = Path(self.output_prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        positions = [(s['x'], s['y']) for s in self.samples
                     if s['x'] is not None and s['y'] is not None]
        final_error = None
        if self.samples:
            final_error = self.samples[-1]['distance_m']
        summary = {
            'reason': reason,
            'gps_topic': self.gps_topic,
            'gps_type': self.gps_type,
            'waypoints_file': self.waypoints_file,
            'waypoints_are_relative': self.relative_waypoints,
            'reached': self.reached,
            'missed': self.missed,
            'reached_count': len(self.reached),
            'waypoint_count': len(self.waypoints),
            'samples': len(self.samples),
            'gps_path_length_m': path_length(positions),
            'final_distance_m': final_error,
        }
        report = {
            'parameters': {
                'goal_tolerance_m': self.goal_tolerance,
                'mission_timeout_sec': self.mission_timeout,
                'waypoint_timeout_sec': self.waypoint_timeout,
                'max_linear_mps': self.max_linear,
                'obstacle_stop_range_m': self.obstacle_stop_range,
            },
            'summary': summary,
            'waypoints': self.waypoints,
            'samples': self.samples,
        }
        (prefix.with_suffix('.json')).write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        with (prefix.with_suffix('.csv')).open('w', newline='') as f:
            fields = [
                'time_sec', 'waypoint_index', 'x', 'y', 'target_x',
                'target_y', 'distance_m', 'yaw_error_deg', 'obstacle_stop',
                'obstacle_min_range_m',
            ]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for sample in self.samples:
                writer.writerow(sample)
        lines = [
            '# Outdoor GPS Waypoint Navigation',
            '',
            f"- reason: `{reason}`",
            f"- gps_topic: `{self.gps_topic}`",
            f"- reached: `{len(self.reached)}/{len(self.waypoints)}`",
            f"- missed: `{self.missed}`",
            f"- gps_path_length_m: `{summary['gps_path_length_m']:.3f}`",
        ]
        if final_error is not None:
            lines.append(f"- final_distance_m: `{final_error:.3f}`")
        lines.extend([
            '',
            '| idx | target xy | result |',
            '|---:|---|---|',
        ])
        for i, wp in enumerate(self.waypoints):
            if i in self.reached:
                result = 'reached'
            elif i in self.missed:
                result = 'missed'
            else:
                result = 'not_started'
            lines.append(f'| {i} | [{wp[0]:.2f}, {wp[1]:.2f}] | {result} |')
        (prefix.with_suffix('.md')).write_text('\n'.join(lines) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = OutdoorGpsWaypointNav()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._finish('keyboard_interrupt')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
