#!/usr/bin/env python3
"""色付き点群 PLY の「ブレ」を定量＋目視で検査する。

カラー点群出力タスク（docs/tasks/colorized_pointcloud.md）の合格確認用。SLAM 姿勢のブレや
外れ点で点群が放射状に散る／壁が二重化すると不合格。本スクリプトは:
  - 主要部(5-95%)の寸法（真値と比べて膨らんでいないか）
  - 床下点 z<thr の割合（散乱の指標）
  - 壁面のシャープさ（占有セルの厚み代理: XY 占有グリッドで「壁が単一線か」を見る）
を数値で出し、XY 俯瞰・XZ 側面の投影図 PNG を保存する。

使い方:
  ros2 run susumu_object_perception check_colorized_cloud.py \
      outputs/colorized_pointcloud/<name>.ply --true-x 5 --true-y 10 --out /tmp/<name>_check.png
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def load_ply(path):
    with open(path) as f:
        line = f.readline().strip()
        if line != 'ply':
            raise ValueError('not a PLY file')
        vertex_count = None
        property_defs = []
        current_element = None
        for line in f:
            line = line.strip()
            if line.startswith('element '):
                parts = line.split()
                current_element = parts[1] if len(parts) >= 2 else None
                if current_element == 'vertex':
                    vertex_count = int(parts[-1])
            elif line.startswith('property ') and current_element == 'vertex':
                parts = line.split()
                if len(parts) >= 3:
                    property_defs.append({'type': parts[-2], 'name': parts[-1]})
            elif line == 'end_header':
                break
        if vertex_count is None:
            raise ValueError('PLY vertex count missing')
        prop_index = {prop['name']: idx for idx, prop in enumerate(property_defs)}
        required = {'x', 'y', 'z', 'red', 'green', 'blue'}
        missing = required - set(prop_index)
        if missing:
            raise ValueError(f'PLY missing vertex properties: {sorted(missing)}')
        pts, rgb = [], []
        for _ in range(vertex_count):
            row = f.readline().split()
            if len(row) < len(property_defs):
                break
            pts.append([
                float(row[prop_index['x']]),
                float(row[prop_index['y']]),
                float(row[prop_index['z']]),
            ])
            rgb.append([
                int(row[prop_index['red']]),
                int(row[prop_index['green']]),
                int(row[prop_index['blue']]),
            ])
    return (
        np.array(pts, dtype=np.float32),
        np.array(rgb, dtype=np.float32) / 255.0,
        int(vertex_count),
        property_defs,
    )


def write_json(path, metrics):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w') as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_markdown(path, metrics):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        '# Colorized Point Cloud Check',
        '',
        f"- validation_passed: `{str(metrics.get('validation_passed', False)).lower()}`",
        f"- ply: `{metrics['ply']}`",
        f"- ply_sha256: `{metrics['ply_sha256']}`",
        f"- points: `{metrics['points']}`",
        f"- header_vertices: `{metrics['header_vertices']}`",
        f"- colored_ratio: `{metrics['colored_ratio']:.4f}`",
        f"- main_extent_m: `{metrics['main_extent_x_m']:.2f}` x "
        f"`{metrics['main_extent_y_m']:.2f}`",
        f"- below_floor_percent: `{metrics['below_floor_percent']:.2f}`",
        f"- occupancy_cells_5cm_z_ge_0: `{metrics['occupancy_cells_5cm_z_ge_0']}`",
        f"- z_range_m: `{metrics['z_min_m']:.2f}` .. `{metrics['z_max_m']:.2f}`",
        f"- rgb_mean_255: `{metrics['rgb_mean_255']}`",
        f"- rgb_std_255: `{metrics['rgb_std_255']}`",
    ]
    if metrics.get('true_x_m') and metrics.get('true_y_m'):
        lines.extend([
            f"- true_extent_m: `{metrics['true_x_m']:.2f}` x "
            f"`{metrics['true_y_m']:.2f}`",
            f"- expansion_ratio: `{metrics['expansion_x']:.3f}` x "
            f"`{metrics['expansion_y']:.3f}`",
        ])
    if metrics.get('png'):
        lines.append(f"- png: `{metrics['png']}`")
    if metrics.get('failures'):
        lines.extend(['', '## Failures'])
        lines.extend(f"- {failure}" for failure in metrics['failures'])
    out.write_text('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--true-x', type=float, default=0.0,
                    help='真値の x 寸法[m]（0 で判定しない）')
    ap.add_argument('--true-y', type=float, default=0.0,
                    help='真値の y 寸法[m]（0 で判定しない）')
    ap.add_argument('--floor-z', type=float, default=-0.05,
                    help='これ未満を床下散乱とみなす z[m]')
    ap.add_argument('--min-points', type=int, default=1000)
    ap.add_argument('--min-colored-ratio', type=float, default=0.95)
    ap.add_argument('--max-below-floor-percent', type=float, default=1.0)
    ap.add_argument('--max-expansion-ratio', type=float, default=1.15,
                    help='true-x/true-y 指定時の主要部寸法膨張率の上限')
    ap.add_argument('--out', default='/tmp/colorized_cloud_check.png')
    ap.add_argument('--json-out', default='',
                    help='optional JSON metrics output path')
    ap.add_argument('--md-out', default='',
                    help='optional Markdown metrics output path')
    ap.add_argument('--require-pass', action='store_true',
                    help='return non-zero when validation_passed is false')
    args = ap.parse_args()

    pts, rgb, header_vertices, property_defs = load_ply(args.ply)
    ply_hash = file_sha256(args.ply)
    n = len(pts)
    if n == 0:
        print('ERROR: PLY に点がありません')
        return 2
    x5, x95 = np.percentile(pts[:, 0], [5, 95])
    y5, y95 = np.percentile(pts[:, 1], [5, 95])
    mx, my = x95 - x5, y95 - y5
    below = (pts[:, 2] < args.floor_z).mean() * 100.0
    colored_count = int((rgb.sum(1) > 0).sum())
    colored_ratio = colored_count / max(n, 1)
    rgb255 = rgb * 255.0
    rgb_mean = [round(float(v), 1) for v in rgb255.mean(axis=0)]
    rgb_std = [round(float(v), 1) for v in rgb255.std(axis=0)]

    print(f'点数: {n}, 色付き(非黒): {colored_count} '
          f'({colored_ratio * 100.0:.1f}%)')
    print(f'主要部(5-95%): x={mx:.2f}m y={my:.2f}m', end='')
    if args.true_x or args.true_y:
        print(f'  (真値 {args.true_x:.1f}x{args.true_y:.1f}m, '
              f'膨張率 x={mx/args.true_x:.2f} y={my/args.true_y:.2f})'
              if args.true_x and args.true_y else '')
    else:
        print()
    print(f'床下点 z<{args.floor_z}: {below:.1f}%')
    print(f'z 範囲[{pts[:,2].min():.2f},{pts[:,2].max():.2f}] z_std={pts[:,2].std():.2f}')

    # 壁シャープさ: z>=0 の点を XY 5cm グリッドに占有化し、占有セルの面積を見る。
    # ブレると壁が厚くなり占有セルが増える（同じ構造でも面積が膨らむ）。
    m = pts[:, 2] >= 0.0
    g = np.floor(pts[m, :2] / 0.05).astype(np.int64)
    occ = len(set(map(tuple, g)))
    print(f'占有セル(5cm,z>=0): {occ}（ブレるほど増える＝壁が厚い目安）')
    print('rgb_mean_255=[' + ','.join(f'{v:.1f}' for v in rgb255.mean(axis=0)) +
          '] rgb_std_255=[' +
          ','.join(f'{v:.1f}' for v in rgb255.std(axis=0)) + ']')

    failures = []
    if n < args.min_points:
        failures.append(f'points {n} < {args.min_points}')
    if n != header_vertices:
        failures.append(f'points {n} != header_vertices {header_vertices}')
    if colored_ratio < args.min_colored_ratio:
        failures.append(
            f'colored_ratio {colored_ratio:.4f} < {args.min_colored_ratio:.4f}')
    if below > args.max_below_floor_percent:
        failures.append(
            f'below_floor_percent {below:.2f} > {args.max_below_floor_percent:.2f}')
    if args.true_x and mx / args.true_x > args.max_expansion_ratio:
        failures.append(
            f'expansion_x {mx / args.true_x:.3f} > {args.max_expansion_ratio:.3f}')
    if args.true_y and my / args.true_y > args.max_expansion_ratio:
        failures.append(
            f'expansion_y {my / args.true_y:.3f} > {args.max_expansion_ratio:.3f}')

    metrics = {
        'schema_version': 4,
        'validation_passed': not failures,
        'failures': failures,
        'summary': {
            'points': int(n),
            'colored_ratio': colored_ratio,
            'below_floor_percent': float(below),
            'main_extent_x_m': float(mx),
            'main_extent_y_m': float(my),
            'occupancy_cells_5cm_z_ge_0': int(occ),
        },
        'ply': args.ply,
        'ply_sha256': ply_hash,
        'header_vertices': int(header_vertices),
        'properties': property_defs,
        'points': int(n),
        'colored_count': colored_count,
        'colored_ratio': colored_ratio,
        'main_extent_x_m': float(mx),
        'main_extent_y_m': float(my),
        'true_x_m': float(args.true_x),
        'true_y_m': float(args.true_y),
        'expansion_x': float(mx / args.true_x) if args.true_x else None,
        'expansion_y': float(my / args.true_y) if args.true_y else None,
        'floor_z_m': float(args.floor_z),
        'below_floor_percent': float(below),
        'z_min_m': float(pts[:, 2].min()),
        'z_max_m': float(pts[:, 2].max()),
        'z_std_m': float(pts[:, 2].std()),
        'occupancy_cells_5cm_z_ge_0': int(occ),
        'rgb_mean_255': rgb_mean,
        'rgb_std_255': rgb_std,
        'criteria': {
            'min_points': int(args.min_points),
            'min_colored_ratio': float(args.min_colored_ratio),
            'max_below_floor_percent': float(args.max_below_floor_percent),
            'max_expansion_ratio': float(args.max_expansion_ratio),
        },
        'png': args.out,
    }

    fig, ax = plt.subplots(1, 2, figsize=(12, 6))
    ax[0].scatter(pts[:, 0], pts[:, 1], s=0.4, c=rgb)
    ax[0].set_title('XY top-view (wall double = blur)')
    ax[0].set_xlabel('x'); ax[0].set_ylabel('y')
    ax[0].set_aspect('equal'); ax[0].grid(True, alpha=0.3)
    ax[1].scatter(pts[:, 0], pts[:, 2], s=0.4, c=rgb)
    ax[1].set_title('XZ side-view (radial scatter = blur)')
    ax[1].set_xlabel('x'); ax[1].set_ylabel('z')
    ax[1].set_aspect('equal'); ax[1].grid(True, alpha=0.3)
    plt.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(args.out, dpi=80)
    print(f'saved {args.out}')
    if args.json_out:
        write_json(args.json_out, metrics)
        print(f'saved {args.json_out}')
    if args.md_out:
        write_markdown(args.md_out, metrics)
        print(f'saved {args.md_out}')
    if failures:
        print('validation_passed=false')
        for failure in failures:
            print(f'- {failure}')
    else:
        print('validation_passed=true')
    return 2 if failures and args.require_pass else 0


if __name__ == '__main__':
    raise SystemExit(main())
