#!/usr/bin/env python3
"""Record pose/costmap consistency during Nav2 waypoint patrol.

This node is observational only. It does not publish TF, goals, or costmap
updates. It samples the robot pose against the static map, global costmap, local
costmap, latest global plan, and scan, then writes JSON/CSV/Markdown reports.

The immediate use case is outdoor saved-map patrol where Nav2 reports:

  - "Starting point in lethal space"
  - "None of the points of the global plan were in the local costmap"

Those errors can come from different layers. This monitor separates the cases:

  - robot pose is already lethal in the saved map
  - global costmap makes a saved-map-free pose lethal
  - local obstacle layer makes a saved-map-free pose lethal
  - the global plan is outside the local rolling window
  - the plan is inside the window but every visible point is blocked
"""

import csv
import json
import math
import os
import re
import time
from collections import Counter, deque

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy, ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


_WAYPOINT_RE = re.compile(r'waypoint #(\d+)')


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_xy(tf_msg, x, y):
    tr = tf_msg.transform.translation
    yaw = quat_to_yaw(tf_msg.transform.rotation)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return tr.x + c * x - s * y, tr.y + s * x + c * y


def grid_value_at(grid, x, y):
    if grid is None:
        return None, 'missing'
    res = float(grid.info.resolution)
    if res <= 0.0:
        return None, 'bad_resolution'
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    gx = int(math.floor((x - ox) / res))
    gy = int(math.floor((y - oy) / res))
    if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
        return None, 'out_of_bounds'
    idx = gy * grid.info.width + gx
    if idx < 0 or idx >= len(grid.data):
        return None, 'bad_index'
    return int(grid.data[idx]), 'ok'


def grid_array(grid):
    import numpy as np

    arr = np.asarray(grid.data, dtype=float).reshape(
        int(grid.info.height), int(grid.info.width))
    return arr


class Nav2PoseCostmapMonitor(Node):

    def __init__(self):
        super().__init__('nav2_pose_costmap_monitor')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('fallback_robot_frame', 'base_footprint')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('waypoint_status_topic', '/waypoint_nav/status')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('sample_period', 0.5)
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('event_cooldown_sec', 2.0)
        self.declare_parameter('max_samples', 5000)
        self.declare_parameter('max_events', 1000)
        self.declare_parameter('lethal_threshold', 90)
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('path_error_warn_m', 0.8)
        self.declare_parameter('scan_front_cone_deg', 35.0)
        self.declare_parameter('report_prefix', '')
        self.declare_parameter('write_png', True)

        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.fallback_robot_frame = str(
            self.get_parameter('fallback_robot_frame').value)
        self.sample_period = float(self.get_parameter('sample_period').value)
        self.report_period = float(self.get_parameter('report_period').value)
        self.event_cooldown_sec = float(
            self.get_parameter('event_cooldown_sec').value)
        self.max_samples = int(self.get_parameter('max_samples').value)
        self.max_events = int(self.get_parameter('max_events').value)
        self.lethal_threshold = int(self.get_parameter('lethal_threshold').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.path_error_warn_m = float(
            self.get_parameter('path_error_warn_m').value)
        self.scan_front_cone = math.radians(
            float(self.get_parameter('scan_front_cone_deg').value))
        self.report_prefix = str(self.get_parameter('report_prefix').value)
        self.write_png = bool(self.get_parameter('write_png').value)
        self.waypoints = self._load_waypoints(
            str(self.get_parameter('waypoints_file').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        volatile_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)

        self.static_map = None
        self.global_costmap = None
        self.local_costmap = None
        self.latest_plan = None
        self.latest_scan = None
        self.latest_amcl = None
        self.latest_odom = None
        self.current_waypoint = None
        self.last_status = ''
        self.rows = deque(maxlen=max(1, self.max_samples))
        self.events = deque(maxlen=max(1, self.max_events))
        self.event_counts = Counter()
        self.last_event_wall = 0.0
        self.started_wall = time.monotonic()

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._map_cb, map_qos)
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('global_costmap_topic').value),
            self._global_costmap_cb, map_qos)
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('local_costmap_topic').value),
            self._local_costmap_cb, map_qos)
        self.create_subscription(
            Path, str(self.get_parameter('plan_topic').value),
            self._plan_cb, reliable_qos)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('amcl_pose_topic').value),
            self._amcl_cb, reliable_qos)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._odom_cb, volatile_qos)
        self.create_subscription(
            String, str(self.get_parameter('waypoint_status_topic').value),
            self._status_cb, reliable_qos)
        self.event_pub = self.create_publisher(
            String, '/nav2_pose_costmap_monitor/event', 10)

        self.create_timer(max(0.1, self.sample_period), self._sample)
        self.create_timer(max(1.0, self.report_period), self._write_reports)
        self.get_logger().info(
            'nav2 pose/costmap monitor started '
            f'(report_prefix={self.report_prefix!r}, '
            f'waypoints={len(self.waypoints)})')

    def _load_waypoints(self, path):
        path = os.path.expanduser(str(path or '')).strip()
        if not path:
            return []
        candidates = [path]
        if not os.path.isabs(path):
            candidates.append(os.path.join(os.getcwd(), path))
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('susumu_object_perception')
                candidates.append(os.path.join(pkg, 'maps', path))
            except Exception:
                pass
        for candidate in candidates:
            if not candidate or not os.path.exists(candidate):
                continue
            try:
                with open(candidate) as f:
                    data = yaml.safe_load(f) or {}
                waypoints = []
                for item in data.get('waypoints', []):
                    if len(item) >= 2:
                        waypoints.append((float(item[0]), float(item[1])))
                return waypoints
            except Exception as exc:
                self.get_logger().warn(
                    f'failed to load waypoints file {candidate}: {exc}')
                return []
        self.get_logger().warn(f'waypoints_file not found: {path}')
        return []

    def _map_cb(self, msg):
        self.static_map = msg

    def _global_costmap_cb(self, msg):
        self.global_costmap = msg

    def _local_costmap_cb(self, msg):
        self.local_costmap = msg

    def _plan_cb(self, msg):
        self.latest_plan = msg

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _amcl_cb(self, msg):
        self.latest_amcl = msg

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _status_cb(self, msg):
        self.last_status = msg.data
        m = _WAYPOINT_RE.search(msg.data)
        if m:
            self.current_waypoint = int(m.group(1))

    def _lookup_robot_pose(self, frame_id):
        last_exc = None
        for robot_frame in (self.robot_frame, self.fallback_robot_frame):
            if not robot_frame:
                continue
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    frame_id, robot_frame, rclpy.time.Time())
                tr = tf_msg.transform.translation
                return {
                    'frame': frame_id,
                    'robot_frame': robot_frame,
                    'x': float(tr.x),
                    'y': float(tr.y),
                    'yaw': quat_to_yaw(tf_msg.transform.rotation),
                    'stamp': stamp_to_float(tf_msg.header.stamp),
                }
            except TransformException as exc:
                last_exc = exc
        return {'error': str(last_exc) if last_exc else 'no robot frame'}

    def _plan_stats_in_grid(self, grid):
        if grid is None:
            return {
                'points': 0,
                'inside': 0,
                'free': 0,
                'lethal': 0,
                'unknown': 0,
                'status': 'missing_costmap',
            }
        plan = self.latest_plan
        if plan is None or not plan.poses:
            return {
                'points': 0,
                'inside': 0,
                'free': 0,
                'lethal': 0,
                'unknown': 0,
                'status': 'missing_plan',
            }
        plan_frame = plan.header.frame_id or grid.header.frame_id
        tf_msg = None
        if plan_frame != grid.header.frame_id:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    grid.header.frame_id, plan_frame, rclpy.time.Time())
            except TransformException as exc:
                return {
                    'points': len(plan.poses),
                    'inside': 0,
                    'free': 0,
                    'lethal': 0,
                    'unknown': 0,
                    'status': f'tf_error:{exc}',
                }
        inside = free = lethal = unknown = 0
        for pose_stamped in plan.poses:
            p = pose_stamped.pose.position
            x, y = float(p.x), float(p.y)
            if tf_msg is not None:
                x, y = transform_xy(tf_msg, x, y)
            val, status = grid_value_at(grid, x, y)
            if status != 'ok':
                continue
            inside += 1
            if val < 0:
                unknown += 1
            elif val >= self.lethal_threshold:
                lethal += 1
            elif val <= self.free_threshold:
                free += 1
        return {
            'points': len(plan.poses),
            'inside': inside,
            'free': free,
            'lethal': lethal,
            'unknown': unknown,
            'status': 'ok',
        }

    def _scan_stats(self):
        scan = self.latest_scan
        if scan is None:
            return {'front_min_m': None, 'valid_count': 0}
        best = None
        valid = 0
        angle = float(scan.angle_min)
        for dist in scan.ranges:
            if math.isfinite(dist) and scan.range_min <= dist <= scan.range_max:
                valid += 1
                if abs(math.atan2(math.sin(angle), math.cos(angle))) <= \
                        self.scan_front_cone:
                    if best is None or dist < best:
                        best = float(dist)
            angle += float(scan.angle_increment)
        return {
            'front_min_m': round(best, 3) if best is not None else None,
            'valid_count': valid,
        }

    def _path_error_stats(self, map_pose):
        if 'x' not in map_pose:
            return {
                'path_error_m': None,
                'path_error_status': 'missing_pose',
            }
        xs, ys = self._plan_points_in_frame('map')
        if not xs:
            return {
                'path_error_m': None,
                'path_error_status': 'missing_plan',
            }
        px = float(map_pose['x'])
        py = float(map_pose['y'])
        best = min(math.hypot(px - x, py - y) for x, y in zip(xs, ys))
        return {
            'path_error_m': round(best, 3),
            'path_error_status': 'ok',
        }

    def _waypoint_edge_info(self):
        idx = self.current_waypoint
        info = {
            'edge_from': None,
            'edge_to': idx,
            'target_x': None,
            'target_y': None,
            'edge_start_x': None,
            'edge_start_y': None,
        }
        if idx is None or not self.waypoints:
            return info
        if 0 <= idx < len(self.waypoints):
            tx, ty = self.waypoints[idx]
            info['target_x'] = round(tx, 3)
            info['target_y'] = round(ty, 3)
        if idx > 0 and idx - 1 < len(self.waypoints):
            sx, sy = self.waypoints[idx - 1]
            info['edge_from'] = idx - 1
            info['edge_start_x'] = round(sx, 3)
            info['edge_start_y'] = round(sy, 3)
        return info

    def _sample(self):
        map_pose = self._lookup_robot_pose('map')
        odom_pose = self._lookup_robot_pose('odom')
        if 'error' in map_pose and 'error' in odom_pose:
            row = self._base_row()
            row.update({
                'diagnosis': 'missing_tf',
                'tf_error': map_pose.get('error') or odom_pose.get('error'),
            })
            self._record_row(row)
            return

        static_val = static_status = None
        global_val = global_status = None
        local_val = local_status = None
        if 'error' not in map_pose:
            static_val, static_status = grid_value_at(
                self.static_map, map_pose['x'], map_pose['y'])
            global_val, global_status = grid_value_at(
                self.global_costmap, map_pose['x'], map_pose['y'])
        if 'error' not in odom_pose:
            local_val, local_status = grid_value_at(
                self.local_costmap, odom_pose['x'], odom_pose['y'])

        local_plan = self._plan_stats_in_grid(self.local_costmap)
        global_plan = self._plan_stats_in_grid(self.global_costmap)
        scan = self._scan_stats()
        path_error = self._path_error_stats(map_pose)
        diagnosis, issues = self._diagnose(
            static_val, static_status, global_val, global_status,
            local_val, local_status, local_plan, global_plan, path_error)

        row = self._base_row()
        row.update({
            'diagnosis': diagnosis,
            'issues': ','.join(issues),
            'map_x': round(map_pose.get('x'), 3)
            if 'x' in map_pose else None,
            'map_y': round(map_pose.get('y'), 3)
            if 'y' in map_pose else None,
            'map_yaw_deg': round(math.degrees(map_pose.get('yaw', 0.0)), 1)
            if 'yaw' in map_pose else None,
            'odom_x': round(odom_pose.get('x'), 3)
            if 'x' in odom_pose else None,
            'odom_y': round(odom_pose.get('y'), 3)
            if 'y' in odom_pose else None,
            'static_value': static_val,
            'static_status': static_status,
            'global_value': global_val,
            'global_status': global_status,
            'local_value': local_val,
            'local_status': local_status,
            'local_plan_points': local_plan['points'],
            'local_plan_inside': local_plan['inside'],
            'local_plan_free': local_plan['free'],
            'local_plan_lethal': local_plan['lethal'],
            'local_plan_unknown': local_plan['unknown'],
            'local_plan_status': local_plan['status'],
            'global_plan_points': global_plan['points'],
            'global_plan_inside': global_plan['inside'],
            'global_plan_free': global_plan['free'],
            'global_plan_lethal': global_plan['lethal'],
            'global_plan_unknown': global_plan['unknown'],
            'global_plan_status': global_plan['status'],
            'scan_front_min_m': scan['front_min_m'],
            'scan_valid_count': scan['valid_count'],
            'path_error_m': path_error['path_error_m'],
            'path_error_status': path_error['path_error_status'],
        })
        row.update(self._waypoint_edge_info())
        self._record_row(row)
        if diagnosis != 'ok':
            self._record_event(row)

    def _base_row(self):
        return {
            'wall_elapsed_sec': round(time.monotonic() - self.started_wall, 3),
            'ros_time_sec': round(
                self.get_clock().now().nanoseconds * 1e-9, 3),
            'current_waypoint': self.current_waypoint,
            'last_status': self.last_status,
        }

    def _diagnose(self, static_val, static_status, global_val, global_status,
                  local_val, local_status, local_plan, global_plan,
                  path_error):
        issues = []
        static_free = static_status == 'ok' and static_val is not None and \
            0 <= static_val <= self.free_threshold
        if static_status not in (None, 'ok'):
            issues.append(f'static_{static_status}')
        if global_status not in (None, 'ok'):
            issues.append(f'global_{global_status}')
        if local_status not in (None, 'ok'):
            issues.append(f'local_{local_status}')

        if static_val is not None and static_val >= self.lethal_threshold:
            issues.append('pose_static_lethal')
        if global_val is not None and global_val >= self.lethal_threshold:
            issues.append('pose_global_lethal')
            if static_free:
                issues.append('pose_global_lethal_static_free')
        if local_val is not None and local_val >= self.lethal_threshold:
            issues.append('pose_local_lethal')
            if static_free:
                issues.append('pose_local_lethal_static_free')
        if local_plan['status'].startswith('tf_error'):
            issues.append('plan_local_tf_error')
        elif local_plan['points'] > 0 and local_plan['inside'] == 0:
            issues.append('plan_not_in_local_costmap')
        elif local_plan['inside'] > 0 and local_plan['free'] == 0:
            issues.append('plan_no_free_points_in_local_costmap')
        if global_plan['status'].startswith('tf_error'):
            issues.append('plan_global_tf_error')
        if path_error.get('path_error_status') == 'ok':
            err = path_error.get('path_error_m')
            if err is not None and err >= self.path_error_warn_m:
                issues.append('path_tracking_error')

        priority = [
            'pose_static_lethal',
            'pose_global_lethal_static_free',
            'pose_local_lethal_static_free',
            'pose_global_lethal',
            'pose_local_lethal',
            'plan_not_in_local_costmap',
            'plan_no_free_points_in_local_costmap',
            'local_out_of_bounds',
            'global_out_of_bounds',
            'static_out_of_bounds',
            'plan_local_tf_error',
            'plan_global_tf_error',
            'path_tracking_error',
        ]
        for key in priority:
            if key in issues:
                return key, issues
        if issues:
            return issues[0], issues
        return 'ok', []

    def _record_row(self, row):
        self.rows.append(row)

    def _record_event(self, row):
        now = time.monotonic()
        if now - self.last_event_wall < self.event_cooldown_sec:
            return
        self.last_event_wall = now
        event = dict(row)
        self.events.append(event)
        self.event_counts[row['diagnosis']] += 1
        self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
        self.get_logger().warn(
            f"diagnosis={row['diagnosis']} wp={row.get('current_waypoint')} "
            f"map=({row.get('map_x')},{row.get('map_y')}) "
            f"static/global/local="
            f"{row.get('static_value')}/{row.get('global_value')}/"
            f"{row.get('local_value')} plan_local="
            f"{row.get('local_plan_inside')}/{row.get('local_plan_free')}")

    def _write_reports(self):
        if not self.report_prefix:
            return
        prefix = os.path.expanduser(self.report_prefix)
        out_dir = os.path.dirname(prefix)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        rows = list(self.rows)
        events = list(self.events)
        diag_counts = Counter(row.get('diagnosis', '') for row in rows)
        summary = {
            'samples': len(rows),
            'events': len(events),
            'diagnosis_counts': dict(sorted(diag_counts.items())),
            'event_counts': dict(sorted(self.event_counts.items())),
            'trajectory': self._trajectory_summary(rows),
            'latest': rows[-1] if rows else None,
            'topics': {
                'map': self.get_parameter('map_topic').value,
                'global_costmap':
                    self.get_parameter('global_costmap_topic').value,
                'local_costmap':
                    self.get_parameter('local_costmap_topic').value,
                'plan': self.get_parameter('plan_topic').value,
                'scan': self.get_parameter('scan_topic').value,
            },
        }
        with open(prefix + '.json', 'w') as f:
            json.dump({'summary': summary, 'events': events, 'rows': rows},
                      f, indent=2, ensure_ascii=False)
            f.write('\n')
        if rows:
            keys = list(rows[0].keys())
            for row in rows[1:]:
                for key in row.keys():
                    if key not in keys:
                        keys.append(key)
            with open(prefix + '.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                for row in rows:
                    writer.writerow(row)
        lines = [
            '# Nav2 pose/costmap monitor report',
            '',
            f"- samples: `{len(rows)}`",
            f"- events: `{len(events)}`",
            f"- diagnosis_counts: `{dict(sorted(diag_counts.items()))}`",
            f"- event_counts: `{dict(sorted(self.event_counts.items()))}`",
            '',
            '| time | waypoint | diagnosis | map x | map y | '
            'static | global | local | local plan inside/free | path err | scan front |',
            '|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|',
        ]
        for row in events[-20:]:
            lines.append(
                f"| {row.get('wall_elapsed_sec')} | "
                f"{row.get('current_waypoint')} | {row.get('diagnosis')} | "
                f"{row.get('map_x')} | {row.get('map_y')} | "
                f"{row.get('static_value')} | {row.get('global_value')} | "
                f"{row.get('local_value')} | "
                f"{row.get('local_plan_inside')}/"
                f"{row.get('local_plan_free')} | "
                f"{row.get('path_error_m')} | "
                f"{row.get('scan_front_min_m')} |")
        with open(prefix + '.md', 'w') as f:
            f.write('\n'.join(lines) + '\n')
        if self.write_png:
            self._write_png(prefix)

    def _trajectory_summary(self, rows):
        valid = [
            row for row in rows
            if row.get('map_x') is not None and row.get('map_y') is not None
        ]
        if not valid:
            return {}
        by_waypoint = {}
        path_errors = []
        for row in valid:
            err = row.get('path_error_m')
            if err is not None:
                try:
                    path_errors.append(float(err))
                except (TypeError, ValueError):
                    pass
            wp = row.get('current_waypoint')
            if wp is None or wp == '':
                continue
            rec = by_waypoint.setdefault(str(wp), {
                'samples': 0,
                'diagnosis_counts': Counter(),
                'max_path_error_m': 0.0,
                'pose_static_lethal_samples': 0,
                'pose_global_lethal_static_free_samples': 0,
            })
            rec['samples'] += 1
            rec['diagnosis_counts'][row.get('diagnosis', '')] += 1
            if err is not None:
                rec['max_path_error_m'] = max(
                    rec['max_path_error_m'], float(err))
            if row.get('static_value') is not None and \
                    int(row.get('static_value')) >= self.lethal_threshold:
                rec['pose_static_lethal_samples'] += 1
            if row.get('diagnosis') == 'pose_global_lethal_static_free':
                rec['pose_global_lethal_static_free_samples'] += 1
        by_waypoint_out = {}
        for wp, rec in by_waypoint.items():
            out = dict(rec)
            out['diagnosis_counts'] = dict(
                sorted(rec['diagnosis_counts'].items()))
            out['max_path_error_m'] = round(out['max_path_error_m'], 3)
            by_waypoint_out[wp] = out
        return {
            'valid_pose_samples': len(valid),
            'max_path_error_m': round(max(path_errors), 3)
            if path_errors else None,
            'mean_path_error_m': round(
                sum(path_errors) / len(path_errors), 3)
            if path_errors else None,
            'waypoints': by_waypoint_out,
        }

    def _write_png(self, prefix):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().debug(f'png report disabled: {exc}')
            return
        grids = [
            ('static map', self.static_map, 'map'),
            ('global costmap', self.global_costmap, 'map'),
            ('local costmap', self.local_costmap, 'odom'),
        ]
        if not any(grid is not None for _, grid, _ in grids):
            return
        fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
        for ax, (title, grid, frame_id) in zip(axes, grids):
            ax.set_title(title)
            if grid is None:
                ax.text(0.5, 0.5, 'missing', transform=ax.transAxes,
                        ha='center', va='center')
                ax.set_axis_off()
                continue
            arr = grid_array(grid)
            extent = [
                grid.info.origin.position.x,
                grid.info.origin.position.x +
                grid.info.width * grid.info.resolution,
                grid.info.origin.position.y,
                grid.info.origin.position.y +
                grid.info.height * grid.info.resolution,
            ]
            ax.imshow(arr, origin='lower', extent=extent, cmap='gray_r',
                      vmin=-1, vmax=100)
            self._plot_trajectory(ax, grid.header.frame_id)
            pose = self._lookup_robot_pose(grid.header.frame_id)
            if 'x' in pose:
                ax.plot([pose['x']], [pose['y']], 'ro', markersize=4)
            plan = self.latest_plan
            if plan is not None and plan.poses:
                xs, ys = self._plan_points_in_frame(grid.header.frame_id)
                if xs:
                    ax.plot(xs, ys, 'c-', linewidth=1)
            ax.set_aspect('equal', adjustable='box')
        fig.savefig(prefix + '.png', dpi=140)
        plt.close(fig)

    def _plot_trajectory(self, ax, target_frame):
        rows = list(self.rows)
        if target_frame == 'map':
            xs = [row.get('map_x') for row in rows]
            ys = [row.get('map_y') for row in rows]
        elif target_frame == 'odom':
            xs = [row.get('odom_x') for row in rows]
            ys = [row.get('odom_y') for row in rows]
        else:
            return
        pts = [
            (float(x), float(y))
            for x, y in zip(xs, ys)
            if x is not None and y is not None
        ]
        if len(pts) >= 2:
            ax.plot([p[0] for p in pts], [p[1] for p in pts],
                    color='magenta', linewidth=1.2, alpha=0.8,
                    label='robot trace')
        event_pts = [
            (float(row.get('map_x')), float(row.get('map_y')))
            for row in self.events
            if target_frame == 'map' and row.get('map_x') is not None
            and row.get('map_y') is not None
        ]
        if event_pts:
            ax.plot([p[0] for p in event_pts], [p[1] for p in event_pts],
                    'rx', markersize=4, alpha=0.75, label='events')

    def _plan_points_in_frame(self, target_frame):
        plan = self.latest_plan
        if plan is None or not plan.poses:
            return [], []
        plan_frame = plan.header.frame_id or target_frame
        tf_msg = None
        if plan_frame != target_frame:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target_frame, plan_frame, rclpy.time.Time())
            except TransformException:
                return [], []
        xs = []
        ys = []
        for pose_stamped in plan.poses:
            p = pose_stamped.pose.position
            x, y = float(p.x), float(p.y)
            if tf_msg is not None:
                x, y = transform_xy(tf_msg, x, y)
            xs.append(x)
            ys.append(y)
        return xs, ys


def main():
    rclpy.init()
    node = Nav2PoseCostmapMonitor()
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
