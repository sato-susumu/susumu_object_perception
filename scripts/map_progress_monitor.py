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
        self.travel = 0.0  # 累積移動距離 [m]

    def _map_cb(self, m):
        w, h = m.info.width, m.info.height
        d = np.array(m.data).reshape(h, w)
        free = int((d == 0).sum())
        occ = int((d == 100).sum())
        res = m.info.resolution
        # 既知セル(free/occ)の bounding box 内に残る unknown の割合＝「開拓余地」。
        # 探索が進むほど bbox 内の unknown が埋まり 0 に近づく。残っていれば未開拓あり。
        known = (d == 0) | (d == 100)
        frontier_unknown = 0
        if known.any():
            ys, xs = np.where(known)
            y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
            box = d[y0:y1, x0:x1]
            unk_in_box = int((box == -1).sum())
            tot_box = box.size
            frontier_unknown = 100.0 * unk_in_box / max(tot_box, 1)
        self.last_map = dict(
            free=free, occ=occ, known=free + occ,
            area=(free + occ) * res * res,
            w=w, h=h, res=res, unk_in_box=frontier_unknown)

    def _scan_cb(self, m):
        r = np.array(m.ranges)
        self.last_scan_n = int(np.isfinite(r).sum() & (r > 0).sum())
        self.last_scan_n = int((np.isfinite(r) & (r > 0)).sum())

    def _odom_cb(self, m):
        p = m.pose.pose.position
        # 累積移動距離を積算する（探索が機能しているかの一次指標。移動量が少ないのに
        # 地図が増えない＝frontier がゴールを出せていない/到達できないアルゴリズム問題）。
        if self.last_odom is not None:
            self.travel += ((p.x - self.last_odom[0]) ** 2
                            + (p.y - self.last_odom[1]) ** 2) ** 0.5
        self.last_odom = (round(p.x, 2), round(p.y, 2))

    def run(self):
        history = []  # (t, known_area, travel)
        stall = 0
        t0 = time.time()
        print('時刻 既知面積[m2] 壁 地図[m] /scan 区間移動 累積 開拓余地% 判定')
        print('-' * 88)
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
            d_move = self.travel - history[-1][2] if history else 0.0
            if history:
                d_area = area - history[-1][1]
                # 移動量と地図進捗の両方で診断する:
                #  - 移動少+進捗なし → frontier がゴールを出せていない/到達できない＝
                #    アルゴリズム問題（ユーザー指摘）。
                #  - 移動多+進捗なし → slam/scan が壁を捉えられず地図が広がらない問題。
                if d_area < 0.3:
                    stall += 1
                    if d_move < 0.15:
                        verdict = f'停滞{stall}：ほぼ動かず(移動{d_move:.2f}m)→探索アルゴリズム疑い'
                    else:
                        verdict = f'停滞{stall}：動くが地図増えず(移動{d_move:.2f}m)→slam/scan疑い'
                else:
                    stall = 0
                    verdict = f'進行（Δ{d_area:+.1f}m2 移動{d_move:.2f}m）'
            history.append((t, area, self.travel))
            print(f'{t:5.0f}s {area:8.1f} {m["occ"]:4d} '
                  f'{m["w"]*m["res"]:.1f}x{m["h"]*m["res"]:.1f} '
                  f'{self.last_scan_n} {d_move:6.2f} {self.travel:6.1f} '
                  f'{m["unk_in_box"]:6.1f}  {verdict}')
        # 総括
        if len(history) >= 2:
            total = history[-1][1] - history[0][1]
            moved = history[-1][2] - history[0][2]
            print('-' * 82)
            print(f'総括: {history[0][0]:.0f}s→{history[-1][0]:.0f}s で既知面積 '
                  f'{history[0][1]:.1f}→{history[-1][1]:.1f}m2 (Δ{total:+.1f}m2) '
                  f'/ ロボット移動 {moved:.1f}m')
            # 診断: 移動が多いのに地図が増えない＝slam、移動が少ない＝探索アルゴリズム。
            if total < 3.0:
                if moved < 1.0:
                    print('診断: ほぼ動かず地図も増えていない → 探索アルゴリズムが'
                          'ゴールを出せていない/到達できない（frontier の問題）')
                else:
                    print(f'診断: {moved:.1f}m 動いたのに地図がほぼ増えていない → '
                          'slam/scan が壁を捉えられていない（環境特徴不足 or scan の問題）')


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
