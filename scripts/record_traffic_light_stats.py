#!/usr/bin/env python3
"""TrafficSignalArray を一定時間購読して live stats JSON を保存する。"""

import argparse
from collections import Counter, defaultdict
import json
import os
import time

import rclpy
from rclpy.node import Node

from autoware_perception_msgs.msg import TrafficSignalArray


class TrafficLightStats:
    def __init__(self):
        self.frames = 0
        self.unique_ids = set()
        self.signal_id_hist = Counter()
        self.signal_id_color_hist = defaultdict(Counter)
        self.color_hist = Counter()
        self.shape_hist = Counter()
        self.status_hist = Counter()
        self.confidences = []

    def add_msg(self, msg):
        self.frames += 1
        for signal in msg.signals:
            signal_id = int(signal.traffic_signal_id)
            signal_id_key = str(signal_id)
            self.unique_ids.add(signal_id)
            self.signal_id_hist[signal_id_key] += 1
            for elem in signal.elements:
                color_key = str(int(elem.color))
                self.color_hist[color_key] += 1
                self.signal_id_color_hist[signal_id_key][color_key] += 1
                self.shape_hist[str(int(elem.shape))] += 1
                self.status_hist[str(int(elem.status))] += 1
                self.confidences.append(float(elem.confidence))

    def report(self, world, backend, omni_views, duration_sec, elapsed_sec):
        conf = sorted(self.confidences)
        n = len(conf)
        if n:
            mid = n // 2
            median = conf[mid] if n % 2 else 0.5 * (conf[mid - 1] + conf[mid])
            confidence = {
                'mean': sum(conf) / n,
                'median': median,
                'min': conf[0],
                'max': conf[-1],
            }
        else:
            confidence = {'mean': None, 'median': None, 'min': None, 'max': None}
        return {
            'world': world,
            'backend': backend,
            'omni_views': omni_views,
            'duration_sec': duration_sec,
            'elapsed_sec': elapsed_sec,
            'frames': self.frames,
            'rate_hz': self.frames / elapsed_sec if elapsed_sec > 0.0 else 0.0,
            'unique_signal_ids': len(self.unique_ids),
            'signal_id_hist': dict(sorted(
                self.signal_id_hist.items(), key=lambda kv: int(kv[0]))),
            'signal_id_color_hist': {
                signal_id: dict(sorted(hist.items(), key=lambda kv: int(kv[0])))
                for signal_id, hist in sorted(
                    self.signal_id_color_hist.items(), key=lambda kv: int(kv[0]))
            },
            'color_hist': dict(sorted(self.color_hist.items(), key=lambda kv: int(kv[0]))),
            'shape_hist': dict(sorted(self.shape_hist.items(), key=lambda kv: int(kv[0]))),
            'status_hist': dict(sorted(self.status_hist.items(), key=lambda kv: int(kv[0]))),
            'confidence': confidence,
        }


class Recorder(Node):
    def __init__(self, topic):
        super().__init__('traffic_light_stats_recorder')
        self.stats = TrafficLightStats()
        self.create_subscription(TrafficSignalArray, topic, self.on_msg, 10)

    def on_msg(self, msg):
        self.stats.add_msg(msg)


def write_json(path, data):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write('\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/perception/traffic_signals')
    ap.add_argument('--duration', type=float, default=20.0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='')
    ap.add_argument('--backend', default='')
    ap.add_argument('--omni-views', type=int, default=0)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.topic)
    start = time.monotonic()
    try:
        while rclpy.ok() and (time.monotonic() - start) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        elapsed = time.monotonic() - start
        report = node.stats.report(
            args.world, args.backend, args.omni_views, args.duration, elapsed)
        write_json(args.out, report)
        print(
            f"frames={report['frames']} rate_hz={report['rate_hz']:.2f} "
            f"unique_signal_ids={report['unique_signal_ids']} "
            f"color_hist={report['color_hist']}"
        )
        print(f"wrote {args.out}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
