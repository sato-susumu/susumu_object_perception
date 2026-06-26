#!/usr/bin/env python3
"""step_detector イベントの後追い可視化ツール (オフライン解析)。

frontier_explore_node や waypoint_nav_node の標準出力ログから
`step_detector event=<type> ... around (x, y)` パターンを抽出し、
保存地図に重ねて「どこで段差検知が何回起きたか」 を PNG に出す。

ノードを追加せず、 既存のログ出力をパースする後追い専用ツール。 屋外マッピング
や屋外巡回の後、 段差ハマりの空間分布をレビュー目的で確認するのに使う。

ログ例:
  [frontier_explore] step_detector event=tilt (tilt_deg=24.78);
    blacklist around (-6.97, -2.08) r=1.50m and cancel current goal

使い方:
  ros2 run susumu_object_perception visualize_step_events.py \\
      --map outputs/mapping_outdoor/village_park.yaml \\
      --log experiments/mapping_outdoor/.../iter18b_launch.log \\
      --out experiments/mapping_outdoor/.../step_events_overlay.png
"""

import argparse
import json
import os
import re
from collections import Counter
import yaml
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# `step_detector event=tilt (tilt_deg=24.78); blacklist around (-6.97, -2.08) r=1.50m`
# / `step_detector event=stuck (...); blacklist around (x, y) r=...`
LOG_RE = re.compile(
    r'step_detector event=(?P<type>\w+).*?'
    r'around\s*\(\s*(?P<x>-?\d+(?:\.\d+)?)\s*,\s*(?P<y>-?\d+(?:\.\d+)?)\s*\)'
    r'(?:\s*r=(?P<r>\d+(?:\.\d+)?)m)?'
)


def parse_log(log_path):
    events = []
    with open(log_path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = LOG_RE.search(line)
            if not m:
                continue
            events.append({
                'type': m.group('type'),
                'x': float(m.group('x')),
                'y': float(m.group('y')),
                'r': float(m.group('r')) if m.group('r') else 1.5,
            })
    return events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='保存地図 YAML')
    ap.add_argument('--log', required=True, help='launch ログファイル (テキスト)')
    ap.add_argument('--out', default='/tmp/step_events_overlay.png')
    ap.add_argument('--events-json', default='',
                    help='抽出 event を JSON で残す (空なら out と同名 .json)')
    ap.add_argument('--scale', type=int, default=4,
                    help='地図表示の拡大倍率 (PNG 視認性向上)')
    args = ap.parse_args()

    meta = yaml.safe_load(open(args.map))
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    pgm_path = os.path.join(os.path.dirname(args.map), meta['image'])
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f'cannot read pgm: {pgm_path}')
    h, w = img.shape

    events = parse_log(args.log)
    type_counter = Counter(e['type'] for e in events)

    print(f'events: {len(events)} (by type: {dict(type_counter)})')

    # 地図画像を準備 (グレースケール → RGB)
    rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if args.scale > 1:
        rgb = cv2.resize(rgb, (w * args.scale, h * args.scale),
                         interpolation=cv2.INTER_NEAREST)

    fig, ax = plt.subplots(figsize=(12, 12 * (h / max(w, 1))))
    ax.imshow(rgb, origin='upper')

    # event ごとに色分け
    color_map = {
        'tilt': ('red', 'tilt (段差傾き)'),
        'stuck': ('orange', 'stuck (車輪空回り)'),
        'accel_jolt': ('yellow', 'accel jolt (急変)'),
        'tilt_recover': ('green', 'tilt recover'),
    }
    legend_handles = []
    seen_types = set()
    for ev in events:
        color, label = color_map.get(ev['type'], ('magenta', ev['type']))
        # map 座標 → pixel (PGM の y は反転)
        px = (ev['x'] - ox) / res
        py = h - 1 - (ev['y'] - oy) / res
        px *= args.scale
        py *= args.scale
        radius_px = ev['r'] / res * args.scale
        ax.add_patch(plt.Circle((px, py), radius_px,
                                edgecolor=color, facecolor=color,
                                alpha=0.2, linewidth=1.5))
        ax.plot(px, py, 'o', color=color, markersize=8)
        if ev['type'] not in seen_types:
            seen_types.add(ev['type'])
            legend_handles.append(
                plt.Line2D([0], [0], marker='o', color='w',
                           markerfacecolor=color, markersize=10, label=label))

    counts_text = ', '.join(f'{k}={v}' for k, v in sorted(type_counter.items()))
    title = (f'Step detector events overlay\n'
             f'map: {os.path.basename(args.map)} '
             f'events: {len(events)} ({counts_text})')
    ax.set_title(title)
    ax.axis('off')
    if legend_handles:
        ax.legend(handles=legend_handles, loc='upper right')

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    fig.savefig(args.out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'saved PNG: {args.out}')

    # JSON 出力
    json_path = args.events_json
    if not json_path:
        base, _ = os.path.splitext(args.out)
        json_path = base + '.json'
    with open(json_path, 'w') as f:
        json.dump({
            'map': args.map,
            'log': args.log,
            'count': len(events),
            'by_type': dict(type_counter),
            'events': events,
        }, f, ensure_ascii=False, indent=2)
    print(f'saved JSON: {json_path}')


if __name__ == '__main__':
    main()
