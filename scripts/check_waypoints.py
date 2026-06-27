#!/usr/bin/env python3
"""生成済み waypoint YAML の「配置品質」を地図と照合して定量評価する。

ウェイポイント生成タスク（docs/tasks/waypoint_generation.md）の合格判定を、目視だけでなく
数値でも裏付けるための検査ツール。「reached=N/N で完走した」は巡回できたことしか示さず、
配置の良し悪し（壁に近すぎないか / カバレッジに偏りが無いか）は別に確認する必要がある。

評価する観点:
  - clearance: 各 waypoint の壁・未知からの距離[m]。小さいと走行中こすり/衝突のリスク。
  - coverage:  配置可能な自由空間の各点から最近傍 waypoint までの距離[m]。max が大きいと
               その辺りが手薄（点が離れすぎ）。
  - route:     連続 waypoint 間が route clearance 上で到達可能か、測地ジャンプが過大でないか。

使い方:
  ros2 run susumu_object_perception check_waypoints.py \
      --map outputs/mapping_indoor/indoor.yaml --waypoints outputs/waypoint_generation/indoor_sparse_waypoints.yaml \
      --clearance 0.6
"""

import argparse
import hashlib
import json
import heapq
import math
import os

import numpy as np
import yaml
import cv2
from scipy import ndimage
from scipy.spatial import cKDTree


_NEI = [(-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)]


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def _octile(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + (1.41421356 - 1.0) * min(dx, dy)


def _astar_len(passable, src, goal):
    """8近傍 passable 上の最短路長[セル]を A* で返す。到達不能なら inf。"""
    if src == goal:
        return 0.0
    h, w = passable.shape
    sx, sy = src
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return float('inf')
    if not passable[sy, sx] or not passable[gy, gx]:
        return float('inf')

    g_score = {(sx, sy): 0.0}
    pq = [(_octile((sx, sy), (gx, gy)), 0.0, sx, sy)]
    closed = set()
    while pq:
        _f, g, x, y = heapq.heappop(pq)
        if (x, y) in closed:
            continue
        if (x, y) == (gx, gy):
            return g
        closed.add((x, y))
        for dx, dy in _NEI:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h and passable[ny, nx]):
                continue
            step = 1.41421356 if dx and dy else 1.0
            ng = g + step
            old = g_score.get((nx, ny))
            if old is not None and ng >= old:
                continue
            g_score[(nx, ny)] = ng
            heapq.heappush(
                pq, (ng + _octile((nx, ny), (gx, gy)), ng, nx, ny))
    return float('inf')


def write_json(path, report):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_markdown(path, report):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    m = report['metrics']
    lines = [
        '# Waypoint Check',
        '',
        f"- schema_version: `{report['schema_version']}`",
        f"- validation_passed: `{str(report['validation_passed']).lower()}`",
        f"- map: `{report['inputs']['map']}`",
        f"- map_image: `{report['inputs'].get('map_image', '')}`",
        f"- waypoints: `{report['inputs']['waypoints']}`",
        f"- map_sha256: `{report.get('inputs_hash', {}).get('map_sha256')}`",
        f"- map_image_sha256: `{report.get('inputs_hash', {}).get('map_image_sha256')}`",
        f"- waypoints_sha256: `{report.get('inputs_hash', {}).get('waypoints_sha256')}`",
        f"- waypoint_count: `{m['waypoint_count']}`",
        '',
        '| metric | value |',
        '|---|---:|',
        f"| min_clearance_m | {m['clearance_min_m']:.3f} |",
        f"| mean_clearance_m | {m['clearance_mean_m']:.3f} |",
        f"| near_clearance_count | {m['near_clearance_count']} |",
        f"| coverage_mean_m | {m['coverage_mean_m']:.3f} |",
        f"| coverage_max_m | {m['coverage_max_m']:.3f} |",
        f"| thin_coverage_cells | {m['thin_coverage_cells']} |",
        f"| route_unreachable_edges | {m['route_unreachable_edges']} |",
        f"| route_over_jump_edges | {m['route_over_jump_edges']} |",
        f"| route_max_geodesic_m | {m['route_max_geodesic_m']:.3f} |",
    ]
    if report['issues']:
        lines.extend(['', '## Issues'])
        lines.extend(f'- {issue}' for issue in report['issues'])
    lines.extend(['', '## Worst Clearances', '',
                  '| index | clearance[m] | x | y |',
                  '|---:|---:|---:|---:|'])
    for item in report['worst_clearances']:
        lines.append(
            f"| {item['index']} | {item['clearance_m']:.3f} | "
            f"{item['x']:.3f} | {item['y']:.3f} |")
    lines.extend(['', '## Longest Route Edges', '',
                  '| edge | geodesic[m] | straight[m] | from | to |',
                  '|---|---:|---:|---|---|'])
    for item in report['longest_edges']:
        geo = item['geodesic_m']
        geo_text = geo if isinstance(geo, str) else f'{geo:.3f}'
        lines.append(
            f"| {item['from_index']}->{item['to_index']} | "
            f"{geo_text} | {item['straight_m']:.3f} | "
            f"({item['from_x']:.3f},{item['from_y']:.3f}) | "
            f"({item['to_x']:.3f},{item['to_y']:.3f}) |")
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--clearance', type=float, default=0.6,
                    help='配置基準の clearance[m]。これ未満の waypoint を警告')
    ap.add_argument('--connect-clearance', type=float, default=0.30,
                    help='route 連結性の clearance[m]')
    ap.add_argument('--route-clearance', type=float, default=None,
                    help='連続 edge の測地経路に使う clearance[m]。省略時は --connect-clearance')
    ap.add_argument('--coverage-warn', type=float, default=3.0,
                    help='この距離[m]より手薄な自由空間があれば警告')
    ap.add_argument('--max-jump-warn', type=float, default=8.0,
                    help='連続 waypoint 間の測地距離がこれを超えたら警告[m]。0 以下で無効')
    ap.add_argument('--json-out', default='',
                    help='optional machine-readable summary path')
    ap.add_argument('--md-out', default='',
                    help='optional Markdown summary path')
    ap.add_argument('--require-pass', action='store_true',
                    help='return non-zero when the check is NG')
    args = ap.parse_args()
    if args.route_clearance is None:
        args.route_clearance = args.connect_clearance

    meta = yaml.safe_load(open(args.map))
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    occ_t = float(meta.get('occupied_thresh', 0.65))
    pgm = os.path.join(os.path.dirname(args.map), meta['image'])
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    # map_server trinary: 205(unknown) は p=0.196 で meta['free_thresh'](0.25) 未満になり
    # free に誤分類される。 generate_waypoints.py と同じく free は「明確に白(>=250)」に限定する。
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
    if not wp:
        print('waypoints: 0 点')
        print('判定: NG（waypoint が空）')
        report = {
            'schema_version': 3,
            'validation_passed': False,
            'issues': ['waypoint list is empty'],
            'summary': {
                'waypoint_count': 0,
                'min_clearance_m': 0.0,
                'coverage_max_m': 0.0,
                'route_max_geodesic_m': 0.0,
                'route_unreachable_edges': 0,
                'route_over_jump_edges': 0,
            },
            'inputs': {
                'map': args.map,
                'map_image': pgm,
                'waypoints': args.waypoints,
            },
            'inputs_hash': {
                'map_sha256': file_sha256(args.map),
                'map_image_sha256': file_sha256(pgm),
                'waypoints_sha256': file_sha256(args.waypoints),
            },
            'metrics': {
                'waypoint_count': 0,
                'clearance_min_m': 0.0,
                'clearance_mean_m': 0.0,
                'near_clearance_count': 0,
                'coverage_mean_m': 0.0,
                'coverage_max_m': 0.0,
                'thin_coverage_cells': 0,
                'route_unreachable_edges': 0,
                'route_over_jump_edges': 0,
                'route_max_geodesic_m': 0.0,
            },
            'worst_clearances': [],
            'longest_edges': [],
        }
        if args.json_out:
            write_json(args.json_out, report)
        if args.md_out:
            write_markdown(args.md_out, report)
        return 2 if args.require_pass else 0
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

    route_passable = free & (dist >= args.route_clearance)
    labels, n_lab = ndimage.label(route_passable)
    wp_labels = [int(labels[cy, cx]) for cx, cy in wpc]
    off_route = sum(1 for lab in wp_labels if lab == 0)
    nonzero_labels = sorted(set(lab for lab in wp_labels if lab != 0))
    split_components = max(0, len(nonzero_labels) - 1)
    print(f'route clearance[m]: {args.route_clearance:.2f} '
          f'components={n_lab} wp_off_route={off_route} '
          f'wp_components={len(nonzero_labels)}'
          + ('  ← WP が route 領域外/別成分' if off_route or split_components else '  OK'))

    geo = []
    straight = []
    for i in range(1, len(wpc)):
        start = tuple(int(v) for v in wpc[i - 1])
        goal = tuple(int(v) for v in wpc[i])
        length = _astar_len(route_passable, start, goal) * res
        geo.append(length)
        straight.append(
            math.hypot(wp[i][0] - wp[i - 1][0], wp[i][1] - wp[i - 1][1]))
    unreachable = sum(1 for g in geo if not math.isfinite(g))
    over_jump = (
        sum(1 for g in geo if math.isfinite(g) and g > args.max_jump_warn)
        if args.max_jump_warn > 0.0 else 0)
    if geo:
        finite_geo = [g for g in geo if math.isfinite(g)]
        max_geo = max(finite_geo) if finite_geo else float('inf')
        mean_geo = float(np.mean(finite_geo)) if finite_geo else float('inf')
        max_straight = max(straight) if straight else 0.0
        print('route edge geodesic[m]: '
              f'mean={mean_geo:.2f} max={max_geo:.2f} '
              f'(straight max={max_straight:.2f}) '
              f'unreachable={unreachable}')
        if args.max_jump_warn > 0.0:
            print(f'  {args.max_jump_warn}m より長い連続edge: {over_jump} 本'
                  + ('  ← 1 goal が長すぎる可能性' if over_jump else '  OK'))

    ok = (near == 0 and thin == 0 and off_route == 0 and
          split_components == 0 and unreachable == 0 and over_jump == 0)
    issues = []
    if near:
        issues.append(
            f'{near} waypoints are closer than clearance {args.clearance:.2f}m')
    if thin:
        issues.append(
            f'{thin} placeable cells exceed coverage warning {args.coverage_warn:.2f}m')
    if off_route:
        issues.append(f'{off_route} waypoints are outside route-passable cells')
    if split_components:
        issues.append(f'waypoints span {len(nonzero_labels)} route components')
    if unreachable:
        issues.append(f'{unreachable} route edges are unreachable')
    if over_jump:
        issues.append(f'{over_jump} route edges exceed {args.max_jump_warn:.2f}m')
    print('判定: ' + ('OK' if ok else 'NG（上記の警告を解消）'))
    # 各点の clearance も列挙（小さい順）。
    order = np.argsort(clrs)
    worst_clearances = []
    print('clearance 小さい順:')
    for i in order[:min(8, len(order))]:
        worst_clearances.append({
            'index': int(i),
            'clearance_m': float(clrs[i]),
            'x': float(wp[i][0]),
            'y': float(wp[i][1]),
        })
        print(f'  #{i}: clearance={clrs[i]:.2f}m  map=({wp[i][0]:.2f},{wp[i][1]:.2f})')
    longest_edges = []
    if geo:
        worst_edges = sorted(
            range(len(geo)),
            key=lambda i: geo[i] if math.isfinite(geo[i]) else float('inf'),
            reverse=True)
        print('route edge 長い順:')
        for i in worst_edges[:min(8, len(worst_edges))]:
            g = geo[i]
            g_text = f'{g:.2f}m' if math.isfinite(g) else 'inf'
            longest_edges.append({
                'from_index': int(i),
                'to_index': int(i + 1),
                'geodesic_m': float(g) if math.isfinite(g) else 'inf',
                'straight_m': float(straight[i]),
                'from_x': float(wp[i][0]),
                'from_y': float(wp[i][1]),
                'to_x': float(wp[i + 1][0]),
                'to_y': float(wp[i + 1][1]),
            })
            print(f'  #{i}->{i + 1}: geodesic={g_text} '
                  f'straight={straight[i]:.2f}m  '
                  f'from=({wp[i][0]:.2f},{wp[i][1]:.2f}) '
                  f'to=({wp[i + 1][0]:.2f},{wp[i + 1][1]:.2f})')
    report = {
        'schema_version': 3,
        'validation_passed': bool(ok),
        'issues': issues,
        'summary': {
            'waypoint_count': int(len(wp)),
            'min_clearance_m': float(clrs.min()),
            'coverage_max_m': float(cov.max()),
            'route_max_geodesic_m': (
                float(max_geo) if geo and math.isfinite(max_geo) else 0.0),
            'route_unreachable_edges': int(unreachable),
            'route_over_jump_edges': int(over_jump),
        },
        'inputs': {
            'map': args.map,
            'map_image': pgm,
            'waypoints': args.waypoints,
            'clearance_m': args.clearance,
            'connect_clearance_m': args.connect_clearance,
            'route_clearance_m': args.route_clearance,
            'coverage_warn_m': args.coverage_warn,
            'max_jump_warn_m': args.max_jump_warn,
        },
        'inputs_hash': {
            'map_sha256': file_sha256(args.map),
            'map_image_sha256': file_sha256(pgm),
            'waypoints_sha256': file_sha256(args.waypoints),
        },
        'metrics': {
            'waypoint_count': int(len(wp)),
            'clearance_min_m': float(clrs.min()),
            'clearance_mean_m': float(clrs.mean()),
            'clearance_max_m': float(clrs.max()),
            'near_clearance_count': int(near),
            'coverage_mean_m': float(cov.mean()),
            'coverage_max_m': float(cov.max()),
            'thin_coverage_cells': int(thin),
            'thin_coverage_percent': float(thin / max(len(cov), 1) * 100.0),
            'route_components_total': int(n_lab),
            'route_waypoint_off_route_count': int(off_route),
            'route_waypoint_component_count': int(len(nonzero_labels)),
            'route_unreachable_edges': int(unreachable),
            'route_over_jump_edges': int(over_jump),
            'route_mean_geodesic_m': float(mean_geo) if geo else 0.0,
            'route_max_geodesic_m': (
                float(max_geo) if geo and math.isfinite(max_geo) else 0.0),
            'route_max_straight_m': float(max_straight) if geo else 0.0,
        },
        'worst_clearances': worst_clearances,
        'longest_edges': longest_edges,
    }
    if args.json_out:
        write_json(args.json_out, report)
        print(f'JSON: {args.json_out}')
    if args.md_out:
        write_markdown(args.md_out, report)
        print(f'MD: {args.md_out}')
    return 0 if ok or not args.require_pass else 2


if __name__ == '__main__':
    raise SystemExit(main())
