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

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_ply(path):
    pts, rgb = [], []
    with open(path) as f:
        line = f.readline()
        while 'end_header' not in line:
            line = f.readline()
        for line in f:
            v = line.split()
            if len(v) >= 6:
                pts.append([float(v[0]), float(v[1]), float(v[2])])
                rgb.append([int(v[3]), int(v[4]), int(v[5])])
    return np.array(pts), np.array(rgb, dtype=np.float32) / 255.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--true-x', type=float, default=0.0,
                    help='真値の x 寸法[m]（0 で判定しない）')
    ap.add_argument('--true-y', type=float, default=0.0,
                    help='真値の y 寸法[m]（0 で判定しない）')
    ap.add_argument('--floor-z', type=float, default=-0.05,
                    help='これ未満を床下散乱とみなす z[m]')
    ap.add_argument('--out', default='/tmp/colorized_cloud_check.png')
    args = ap.parse_args()

    pts, rgb = load_ply(args.ply)
    n = len(pts)
    x5, x95 = np.percentile(pts[:, 0], [5, 95])
    y5, y95 = np.percentile(pts[:, 1], [5, 95])
    mx, my = x95 - x5, y95 - y5
    below = (pts[:, 2] < args.floor_z).mean() * 100.0

    print(f'点数: {n}, 色付き(非黒): {int((rgb.sum(1) > 0).sum())}')
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
    plt.savefig(args.out, dpi=80)
    print(f'saved {args.out}')


if __name__ == '__main__':
    main()
