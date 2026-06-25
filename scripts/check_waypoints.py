#!/usr/bin/env python3
"""生成済み waypoint YAML の「配置品質」を地図と照合して定量評価する。

ウェイポイント生成タスク（docs/tasks/waypoint_generation.md）の合格判定を、目視だけでなく
数値でも裏付けるための検査ツール。「reached=N/N で完走した」は巡回できたことしか示さず、
配置の良し悪し（壁に近すぎないか / カバレッジに偏りが無いか）は別に確認する必要がある。

評価する観点:
  - clearance: 各 waypoint の壁・未知からの距離[m]。小さいと走行中こすり/衝突のリスク。
  - coverage:  配置可能な自由空間の各点から最近傍 waypoint までの距離[m]。max が大きいと
               その辺りが手薄（点が離れすぎ）。

使い方:
  ros2 run susumu_object_perception check_waypoints.py \
      --map outputs/mapping_indoor/indoor.yaml --waypoints outputs/waypoint_generation/indoor_sparse_waypoints.yaml \
      --clearance 0.6
"""

import argparse

import numpy as np
import yaml
import cv2
from scipy import ndimage
from scipy.spatial import cKDTree


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--clearance', type=float, default=0.6,
                    help='配置基準の clearance[m]。これ未満の waypoint を警告')
    ap.add_argument('--coverage-warn', type=float, default=3.0,
                    help='この距離[m]より手薄な自由空間があれば警告')
    args = ap.parse_args()

    meta = yaml.safe_load(open(args.map))
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.25))
    import os
    pgm = os.path.join(os.path.dirname(args.map), meta['image'])
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    # map_server trinary: 205(unknown) は p=0.196 で free_t 未満になり free に誤分類される。
    # generate_waypoints.py と同じく free は「明確に白(>=250)」に限定する。
    p = (255.0 - img.astype(np.float32)) / 255.0
    occ = p >= occ_t
    free = (img >= 250) & ~occ
    unknown = ~free & ~occ
    dist = ndimage.distance_transform_edt(~(occ | unknown)) * res

    def to_cell(mx, my):
        cx = int(round((mx - ox) / res - 0.5))
        cy = int(round(h - 1 - ((my - oy) / res - 0.5)))
        return max(0, min(w - 1, cx)), max(0, min(h - 1, cy))

    wp = yaml.safe_load(open(args.waypoints))['waypoints']
    wpc = np.array([to_cell(mx, my) for mx, my in wp])
    clrs = np.array([dist[cy, cx] for cx, cy in wpc])

    placeable = free & (dist >= args.clearance)
    ys, xs = np.where(placeable)
    tree = cKDTree(wpc)
    cov, _ = tree.query(np.column_stack([xs, ys]))
    cov = cov * res

    print(f'waypoints: {len(wp)} 点')
    print(f'clearance[m]: min={clrs.min():.2f} mean={clrs.mean():.2f} '
          f'max={clrs.max():.2f}')
    near = int((clrs < args.clearance).sum())
    print(f'  壁に近い(<{args.clearance}m): {near} 点'
          + ('  ← 配置 clearance 未達' if near else '  OK'))
    print(f'coverage[m] (placeable 自由空間→最近傍WP): '
          f'mean={cov.mean():.2f} max={cov.max():.2f}')
    thin = int((cov > args.coverage_warn).sum())
    print(f'  {args.coverage_warn}m より手薄: {thin} セル '
          f'({thin / max(len(cov), 1) * 100:.1f}%)'
          + ('  ← カバレッジに偏り' if thin else '  OK'))

    ok = near == 0 and thin == 0
    print('判定: ' + ('OK' if ok else 'NG（上記の警告を解消）'))
    # 各点の clearance も列挙（小さい順）。
    order = np.argsort(clrs)
    print('clearance 小さい順:')
    for i in order[:min(8, len(order))]:
        print(f'  #{i}: clearance={clrs[i]:.2f}m  map=({wp[i][0]:.2f},{wp[i][1]:.2f})')


if __name__ == '__main__':
    main()
