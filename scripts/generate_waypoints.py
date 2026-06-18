#!/usr/bin/env python3
"""保存済み占有格子地図から巡回ウェイポイントを生成する（オフライン）。

frontier 探索で作った maps/<world>.pgm/.yaml を読み、自由空間のうち壁から十分
離れた点を、地図全体をカバーするように間引いて巡回順に並べ、YAML に保存する。
生成した YAML は waypoint_nav_node.py（FollowWaypoints 巡回）と
waypoint_viz_node.py（RViz 可視化）が読む。

アルゴリズム:
  1. PGM を読み free(255 付近)/occupied(0 付近)/unknown(205) に分類。
  2. 占有・未知セルからの距離変換(distance transform)を計算。robot_radius +
     margin より壁から離れた free セルだけを候補にする（壁際を避ける）。
  3. 候補を spacing[m] グリッドに間引いてウェイポイント点群にする。
  4. 最近傍貪欲法で巡回順に並べる（簡易 TSP）。始点は地図中心に最も近い候補。
  5. map 座標(yaml の resolution/origin で変換)に直して YAML 保存。

使い方:
  ros2 run susumu_object_perception generate_waypoints.py \
    --map ~/ros2_ws/src/susumu_object_perception/maps/city.yaml \
    --out ~/ros2_ws/src/susumu_object_perception/maps/city_waypoints.yaml \
    --spacing 1.5 --clearance 0.4
"""

import argparse
import os

import numpy as np
import yaml
from scipy import ndimage


def load_pgm(path):
    """P5(binary) / P2(ascii) PGM を numpy 配列で読む。"""
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        # コメント行をスキップ。
        def read_token():
            tok = b''
            while True:
                c = f.read(1)
                if c in (b' ', b'\t', b'\n', b'\r'):
                    if tok:
                        return tok
                elif c == b'#':
                    f.readline()
                else:
                    tok += c
        w = int(read_token())
        h = int(read_token())
        maxv = int(read_token())
        if magic == b'P5':
            data = np.frombuffer(f.read(w * h), dtype=np.uint8)
        else:  # P2
            vals = f.read().split()
            data = np.array([int(v) for v in vals[:w * h]], dtype=np.uint8)
    return data.reshape(h, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='地図 yaml のパス')
    ap.add_argument('--out', required=True, help='出力 waypoint yaml のパス')
    # ウェイポイント間隔 [m]。大きいほど点が疎。
    ap.add_argument('--spacing', type=float, default=1.5)
    # 壁から最低この距離 [m] 離れた点だけ採用（ロボット半径+余裕）。
    ap.add_argument('--clearance', type=float, default=0.4)
    # 巡回の最大ウェイポイント数（多すぎると 1 周が長い）。
    ap.add_argument('--max-waypoints', type=int, default=40)
    args = ap.parse_args()

    with open(args.map) as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.25))
    pgm_path = os.path.join(os.path.dirname(args.map), meta['image'])

    img = load_pgm(pgm_path)
    h, w = img.shape
    # map_server 流: trinary。occ=黒(0), free=白(254/255), unknown=205。
    # 占有確率 p = (255 - value)/255。p>=occ_thresh が占有、p<=free_thresh が空き。
    p = (255.0 - img.astype(np.float32)) / 255.0
    free = p <= free_thresh
    occ = p >= occ_thresh
    unknown = ~free & ~occ
    # 通行不可（占有 or 未知）からの距離 [セル]。free セルが壁/未知からどれだけ
    # 離れているか。
    blocked = occ | unknown
    dist_cells = ndimage.distance_transform_edt(~blocked)
    clearance_cells = args.clearance / res

    # 候補: free かつ 壁/未知から clearance 以上離れている。
    cand_mask = free & (dist_cells >= clearance_cells)
    ys, xs = np.where(cand_mask)
    if len(xs) == 0:
        print('ERROR: clearance を満たす自由空間が無い。--clearance を下げて。')
        return

    # spacing グリッドに間引く（各セルを spacing で量子化し代表点を1つ）。
    step = max(1, int(round(args.spacing / res)))
    seen = set()
    pts_cell = []
    # 壁から遠い点を優先採用するため dist 降順で走査。
    order = np.argsort(-dist_cells[ys, xs])
    for i in order:
        cx, cy = xs[i], ys[i]
        key = (cx // step, cy // step)
        if key in seen:
            continue
        seen.add(key)
        pts_cell.append((cx, cy))

    # ピクセル座標 → map 座標。pgm は上下反転（行0が上端=y最大）。
    def cell_to_map(cx, cy):
        mx = ox + (cx + 0.5) * res
        my = oy + (h - 1 - cy + 0.5) * res
        return (mx, my)

    pts_map = [cell_to_map(cx, cy) for (cx, cy) in pts_cell]

    # 最近傍貪欲法で巡回順に並べる。始点は地図中心に最も近い点。
    cx_map = ox + (w * 0.5) * res
    cy_map = oy + (h * 0.5) * res
    remaining = list(pts_map)
    ordered = []
    cur = min(remaining, key=lambda pp: (pp[0] - cx_map) ** 2 + (pp[1] - cy_map) ** 2)
    remaining.remove(cur)
    ordered.append(cur)
    while remaining:
        nxt = min(remaining, key=lambda pp: (pp[0] - cur[0]) ** 2 + (pp[1] - cur[1]) ** 2)
        remaining.remove(nxt)
        ordered.append(nxt)
        cur = nxt

    if len(ordered) > args.max_waypoints:
        stepf = len(ordered) / float(args.max_waypoints)
        ordered = [ordered[int(i * stepf)] for i in range(args.max_waypoints)]

    out = {
        'map': os.path.basename(args.map),
        'frame_id': 'map',
        'waypoints': [[round(float(x), 3), round(float(y), 3)] for (x, y) in ordered],
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump(out, f, default_flow_style=None, sort_keys=False)

    print(f'生成: {len(ordered)} waypoints -> {args.out}')
    print(f'  地図 {w}x{h} ({round(w*res,1)}x{round(h*res,1)}m) '
          f'clearance={args.clearance}m spacing={args.spacing}m')


if __name__ == '__main__':
    main()
