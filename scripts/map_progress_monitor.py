#!/usr/bin/env python3
"""地図作成（frontier 探索 + slam_toolbox）の進捗を監視するスクリプト（検証用）。

`/map`(OccupancyGrid) を一定間隔でサンプリングし、既知セル(free+occ)数・面積・壁数の
推移を表示する。frontier 探索が進んでいるか（既知面積が増えているか）を客観的に判定し、
停滞（一定回数連続で増えない）を検知して原因のヒントを出す。

使い方:
  python3 map_progress_monitor.py            # 既定 5 秒間隔・180 秒監視
  python3 map_progress_monitor.py --interval 10 --duration 600
  python3 map_progress_monitor.py --frontier  # /map に加え frontier ノードの存命も見る

注意: /scan(BEST_EFFORT) や /map(TRANSIENT_LOCAL) の QoS に合わせて購読する。
RMW は launch と揃える（CycloneDDS 推奨）。
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class MapProgressMonitor(Node):
    def __init__(self, interval, duration):
        super().__init__('map_progress_monitor')
        self.set_parameters(
            [rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.interval = interval
        self.duration = duration

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)

        self.last_map = None
        self.last_scan_n = None
        self.last_odom = None

    def _map_cb(self, m):
        d = np.array(m.data)
        free = int((d == 0).sum())
        occ = int((d == 100).sum())
        res = m.info.resolution
        self.last_map = dict(
            free=free, occ=occ, known=free + occ,
            area=(free + occ) * res * res,
            w=m.info.width, h=m.info.height, res=res)

    def _scan_cb(self, m):
        r = np.array(m.ranges)
        self.last_scan_n = int(np.isfinite(r).sum() & (r > 0).sum())
        self.last_scan_n = int((np.isfinite(r) & (r > 0)).sum())

    def _odom_cb(self, m):
        p = m.pose.pose.position
        self.last_odom = (round(p.x, 2), round(p.y, 2))

    def run(self):
        history = []  # (t, known_area)
        stall = 0
        t0 = time.time()
        print('時刻  既知面積[m2]  壁数  地図[m]  /scan有効点  odom  判定')
        print('-' * 78)
        while rclpy.ok() and time.time() - t0 < self.duration:
            tend = time.time() + self.interval
            while rclpy.ok() and time.time() < tend:
                rclpy.spin_once(self, timeout_sec=0.2)
            t = time.time() - t0
            if self.last_map is None:
                print(f'{t:5.0f}s  (no /map yet)  scan={self.last_scan_n}')
                continue
            m = self.last_map
            area = m['area']
            verdict = ''
            if history:
                d_area = area - history[-1][1]
                if d_area < 0.3:  # 0.3 m2 未満しか増えない=停滞
                    stall += 1
                    verdict = f'停滞{stall}（Δ{d_area:+.1f}m2）'
                else:
                    stall = 0
                    verdict = f'進行（Δ{d_area:+.1f}m2）'
            history.append((t, area))
            print(f'{t:5.0f}s  {area:9.1f}    {m["occ"]:5d}  '
                  f'{m["w"]*m["res"]:.1f}x{m["h"]*m["res"]:.1f}  '
                  f'scan={self.last_scan_n}  odom={self.last_odom}  {verdict}')
            if stall >= 5:
                print('--- 5 回連続で停滞。考えられる原因: '
                      'frontier が未踏領域を見つけられない / ゴールに到達できず '
                      'blacklist 連発 / scan が壁を捉えていない / ロボットが詰まっている')
        # 総括
        if len(history) >= 2:
            total = history[-1][1] - history[0][1]
            print('-' * 78)
            print(f'総括: {history[0][0]:.0f}s→{history[-1][0]:.0f}s で既知面積 '
                  f'{history[0][1]:.1f}→{history[-1][1]:.1f}m2 (Δ{total:+.1f}m2)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--interval', type=float, default=5.0)
    ap.add_argument('--duration', type=float, default=180.0)
    args = ap.parse_args()
    rclpy.init()
    node = MapProgressMonitor(args.interval, args.duration)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
