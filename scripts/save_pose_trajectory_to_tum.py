#!/usr/bin/env python3
"""Save a PoseStamped trajectory topic as a TUM trajectory text file."""

import argparse
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)


class PoseTrajectorySaver(Node):
    def __init__(self, args):
        super().__init__('save_pose_trajectory_to_tum')
        self.args = args
        self.rows = []
        self.first_wall_time = None
        self.last_position = None
        self.done = False
        self.start_wall_time = time.monotonic()
        self.create_subscription(
            PoseStamped, args.topic, self.on_pose, self.make_qos(args.qos))
        self.create_timer(0.2, self.on_timer)
        self.get_logger().info(
            f'waiting for {args.topic} -> {args.out} '
            f'(duration={args.duration_sec}s, min_poses={args.min_poses}, '
            f'qos={args.qos})')

    @staticmethod
    def make_qos(name):
        if name == 'sensor_data':
            return qos_profile_sensor_data
        reliable = name == 'reliable'
        return QoSProfile(
            depth=20,
            reliability=(ReliabilityPolicy.RELIABLE if reliable
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)

    def on_pose(self, msg):
        if self.done:
            return
        if self.args.frame_id and msg.header.frame_id != self.args.frame_id:
            self.get_logger().warn(
                f'ignoring pose in frame {msg.header.frame_id!r}; '
                f'expected {self.args.frame_id!r}',
                throttle_duration_sec=2.0)
            return

        pos = msg.pose.position
        ori = msg.pose.orientation
        if not all(math.isfinite(v) for v in (
                pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)):
            return
        if self.last_position is not None and self.args.min_distance > 0.0:
            dx = pos.x - self.last_position[0]
            dy = pos.y - self.last_position[1]
            dz = pos.z - self.last_position[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.args.min_distance:
                return

        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_sec == 0.0:
            stamp_sec = time.time()
        self.rows.append((
            stamp_sec, pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w,
        ))
        self.last_position = (pos.x, pos.y, pos.z)
        if self.first_wall_time is None:
            self.first_wall_time = time.monotonic()

    def on_timer(self):
        if self.done:
            return
        now = time.monotonic()
        if now - self.start_wall_time >= self.args.timeout_sec:
            self.finish(success=len(self.rows) >= self.args.min_poses,
                        reason='timeout')
            return
        if self.first_wall_time is None:
            return
        duration_reached = (
            self.args.duration_sec <= 0.0
            or now - self.first_wall_time >= self.args.duration_sec)
        if duration_reached and len(self.rows) >= self.args.min_poses:
            self.finish(success=True, reason='duration')

    def finish(self, success, reason):
        if self.done:
            return
        self.done = True
        if not success:
            self.get_logger().error(
                f'{reason}: only collected {len(self.rows)} poses '
                f'(< {self.args.min_poses})')
            return
        out = Path(self.args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        rows = self.rows
        if self.args.max_poses > 0 and len(rows) > self.args.max_poses:
            step = max(1, math.ceil(len(rows) / self.args.max_poses))
            rows = rows[::step]
        with out.open('w') as f:
            f.write('# timestamp tx ty tz qx qy qz qw\n')
            f.write(f'# source_topic {self.args.topic}\n')
            f.write(f'# frame_id {self.args.frame_id or "*"}\n')
            for row in rows:
                f.write(' '.join(f'{v:.9f}' for v in row) + '\n')
        self.get_logger().info(
            f'wrote {out} poses={len(rows)} raw_poses={len(self.rows)} '
            f'reason={reason}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/glim_ros/pose_corrected')
    parser.add_argument('--out', required=True)
    parser.add_argument('--duration-sec', type=float, default=10.0,
                        help='Wall-clock seconds to record after first pose; '
                             '0 means stop as soon as min-poses is reached')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--min-poses', type=int, default=2)
    parser.add_argument('--max-poses', type=int, default=0,
                        help='Downsample output to at most this many poses; '
                             '0 disables downsampling')
    parser.add_argument('--min-distance', type=float, default=0.0,
                        help='Skip poses closer than this distance from the '
                             'last saved pose')
    parser.add_argument('--frame-id', default='glim_map',
                        help='Expected frame_id; empty accepts any frame')
    parser.add_argument('--qos', choices=('sensor_data', 'best_effort',
                                          'reliable'),
                        default='reliable')
    args = parser.parse_args()

    rclpy.init()
    node = PoseTrajectorySaver(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        success = node.done and Path(args.out).exists()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
