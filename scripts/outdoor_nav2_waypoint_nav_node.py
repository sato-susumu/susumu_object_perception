#!/usr/bin/env python3
"""Send sparse outdoor map waypoints to Nav2 and write a mission report."""

import csv
import json
import math
import os
import time
from pathlib import Path

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def path_length(points):
    if len(points) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(points[:-1], points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


class OutdoorNav2WaypointNav(Node):
    def __init__(self):
        super().__init__('outdoor_nav2_waypoint_nav')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('output_prefix', '/tmp/outdoor_nav2_waypoint_nav')
        self.declare_parameter('start_delay_sec', 8.0)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('mission_timeout_sec', 120.0)
        self.declare_parameter('sample_period_sec', 0.5)
        self.declare_parameter('goal_reject_retries', 6)
        self.declare_parameter('goal_reject_retry_sec', 2.0)

        self.waypoints_file = os.path.expanduser(
            self.get_parameter('waypoints_file').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.output_prefix = os.path.expanduser(
            self.get_parameter('output_prefix').value)
        self.start_delay = float(self.get_parameter('start_delay_sec').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.mission_timeout = float(
            self.get_parameter('mission_timeout_sec').value)
        self.sample_period = float(
            self.get_parameter('sample_period_sec').value)
        self.goal_reject_retries = int(
            self.get_parameter('goal_reject_retries').value)
        self.goal_reject_retry_sec = float(
            self.get_parameter('goal_reject_retry_sec').value)

        self.waypoints = self._load_waypoints(self.waypoints_file)
        self.status_pub = self.create_publisher(
            String, '/outdoor_nav2_waypoint_nav/status', 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.started = time.monotonic()
        self.current_idx = 0
        self.reached = []
        self.missed = []
        self.samples = []
        self.goal_handle = None
        self.done = False
        self.token = 0
        self.reject_retry_count = 0
        self.goal_started_wall = None
        self.retry_timer = None
        self.sample_timer = self.create_timer(
            max(self.sample_period, 0.1), self._sample)
        self.start_timer = self.create_timer(self.start_delay, self._start_once)
        self._status(
            f'loaded {len(self.waypoints)} Nav2 outdoor waypoints from '
            f'{self.waypoints_file or "<default>"}')

    def _load_waypoints(self, path):
        if path:
            with open(path) as f:
                data = yaml.safe_load(f)
            if isinstance(data, dict):
                raw = data.get('waypoints', data)
                self.frame_id = data.get('frame_id', self.frame_id)
            else:
                raw = data
            if self.frame_id == 'gps_local':
                self.frame_id = 'map'
        else:
            raw = [[0.8, 0.0], [0.8, 0.8], [0.0, 0.8], [0.0, 0.0]]
        waypoints = []
        for i, item in enumerate(raw):
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                raise ValueError(f'invalid waypoint #{i}: {item}')
            yaw = None
            if len(item) >= 3 and item[2] is not None:
                yaw = float(item[2])
            waypoints.append([float(item[0]), float(item[1]), yaw])
        if not waypoints:
            raise ValueError('waypoints are empty')
        return waypoints

    def _status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        tr = tf.transform.translation
        return float(tr.x), float(tr.y)

    def _sample(self):
        if self.done:
            return
        xy = self._robot_xy()
        target = None
        dist = None
        if self.current_idx < len(self.waypoints):
            target = self.waypoints[self.current_idx]
        if xy is not None and target is not None:
            dist = math.hypot(target[0] - xy[0], target[1] - xy[1])
        self.samples.append({
            'time_sec': time.monotonic() - self.started,
            'waypoint_index': self.current_idx,
            'x': None if xy is None else xy[0],
            'y': None if xy is None else xy[1],
            'target_x': None if target is None else target[0],
            'target_y': None if target is None else target[1],
            'distance_m': dist,
        })
        if self.goal_started_wall is not None and \
                (time.monotonic() - self.goal_started_wall) >= self.goal_timeout:
            self._on_timeout(self.token)

    def _start_once(self):
        self.start_timer.cancel()
        if not self.client.wait_for_server(timeout_sec=20.0):
            self._finish('navigate_to_pose_unavailable')
            return
        self._go_next()

    def _go_next(self):
        if self.done:
            return
        if time.monotonic() - self.started >= self.mission_timeout:
            for idx in range(self.current_idx, len(self.waypoints)):
                if idx not in self.reached and idx not in self.missed:
                    self.missed.append(idx)
            self._finish('mission_timeout')
            return
        if self.current_idx >= len(self.waypoints):
            self._finish('complete')
            return

        x, y, yaw = self.waypoints[self.current_idx]
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(0.0 if yaw is None else yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.token += 1
        token = self.token
        self.goal_started_wall = time.monotonic()
        self._status(
            f'heading to Nav2 outdoor waypoint #{self.current_idx} '
            f'({x:.2f}, {y:.2f}) in {self.frame_id}')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, token))

    def _on_goal_response(self, future, token):
        if token != self.token or self.done:
            return
        gh = future.result()
        if not gh.accepted:
            self.goal_started_wall = None
            if self.reject_retry_count < self.goal_reject_retries:
                self.reject_retry_count += 1
                self._status(
                    f'Nav2 waypoint #{self.current_idx} rejected; retry '
                    f'{self.reject_retry_count}/{self.goal_reject_retries}')
                self.token += 1
                self._schedule_retry()
            else:
                self._status(
                    f'Nav2 waypoint #{self.current_idx} rejected; skip')
                self._advance(False)
            return
        self.reject_retry_count = 0
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_result(f, token))

    def _on_result(self, future, token):
        if token != self.token or self.done:
            return
        self.goal_started_wall = None
        self.goal_handle = None
        status = future.result().status
        self._advance(status == GoalStatus.STATUS_SUCCEEDED)

    def _on_timeout(self, token):
        if token != self.token or self.done:
            return
        self.token += 1
        self.goal_started_wall = None
        self._status(f'Nav2 waypoint #{self.current_idx} timeout; skip')
        gh = self.goal_handle
        self.goal_handle = None
        if gh is not None:
            gh.cancel_goal_async()
        self._advance(False)

    def _schedule_retry(self):
        self._cancel_retry_timer()
        self.retry_timer = self.create_timer(
            self.goal_reject_retry_sec, self._retry_current_once)

    def _retry_current_once(self):
        self._cancel_retry_timer()
        self._go_next()

    def _advance(self, reached):
        if reached:
            self.reached.append(self.current_idx)
            self._status(f'Nav2 waypoint #{self.current_idx} reached')
        else:
            if self.current_idx not in self.missed:
                self.missed.append(self.current_idx)
            self._status(f'Nav2 waypoint #{self.current_idx} missed')
        self.reject_retry_count = 0
        self._cancel_retry_timer()
        self.current_idx += 1
        self._go_next()

    def _cancel_retry_timer(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None

    def _finish(self, reason):
        if self.done:
            return
        self.done = True
        self._status(
            f'outdoor Nav2 nav finished reason={reason} '
            f'reached={len(self.reached)}/{len(self.waypoints)} '
            f'missed={self.missed}')
        self._write_reports(reason)

    def _write_reports(self, reason):
        prefix = Path(self.output_prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        positions = [(s['x'], s['y']) for s in self.samples
                     if s['x'] is not None and s['y'] is not None]
        final_distance = None
        if self.samples:
            final_distance = self.samples[-1]['distance_m']
        summary = {
            'reason': reason,
            'frame_id': self.frame_id,
            'robot_frame': self.robot_frame,
            'waypoints_file': self.waypoints_file,
            'reached': self.reached,
            'missed': self.missed,
            'reached_count': len(self.reached),
            'waypoint_count': len(self.waypoints),
            'samples': len(self.samples),
            'tf_path_length_m': path_length(positions),
            'final_distance_m': final_distance,
        }
        report = {
            'parameters': {
                'start_delay_sec': self.start_delay,
                'goal_timeout_sec': self.goal_timeout,
                'mission_timeout_sec': self.mission_timeout,
            },
            'summary': summary,
            'waypoints': self.waypoints,
            'samples': self.samples,
        }
        prefix.with_suffix('.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        with prefix.with_suffix('.csv').open('w', newline='') as f:
            fields = [
                'time_sec', 'waypoint_index', 'x', 'y', 'target_x',
                'target_y', 'distance_m',
            ]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for sample in self.samples:
                writer.writerow(sample)
        lines = [
            '# Outdoor Nav2 Waypoint Navigation',
            '',
            f"- reason: `{reason}`",
            f"- frame_id: `{self.frame_id}`",
            f"- reached: `{len(self.reached)}/{len(self.waypoints)}`",
            f"- missed: `{self.missed}`",
            f"- tf_path_length_m: `{summary['tf_path_length_m']:.3f}`",
        ]
        if final_distance is not None:
            lines.append(f"- final_distance_m: `{final_distance:.3f}`")
        lines.extend(['', '| idx | target xy | result |', '|---:|---|---|'])
        for i, wp in enumerate(self.waypoints):
            if i in self.reached:
                result = 'reached'
            elif i in self.missed:
                result = 'missed'
            else:
                result = 'not_started'
            lines.append(f'| {i} | [{wp[0]:.2f}, {wp[1]:.2f}] | {result} |')
        prefix.with_suffix('.md').write_text('\n'.join(lines) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = OutdoorNav2WaypointNav()
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
