#!/usr/bin/env python3
"""保存済み占有格子地図から巡回ウェイポイントを生成する（オフライン）。

frontier 探索で作った maps/<world>.pgm/.yaml を読み、自由空間のうち壁から十分
離れた点を、地図全体をカバーするように間引いて巡回順に並べ、YAML に保存する。
生成した YAML は waypoint_nav_node.py（FollowWaypoints 巡回）と
waypoint_viz_node.py（RViz 可視化）が読む。

アルゴリズム:
  1. PGM を読み free(255 付近)/occupied(0 付近)/unknown(205) に分類。
  2. 占有・未知セルからの距離変換(distance transform)を計算。
  3. 連結用 clearance（緩い閾値）で「通れる領域」を作り、その**最大連結成分**を
     巡回対象にする。連結用を緩くするのは、幅 1.2m 未満の通路（ドア・家具の隙間）で
     部屋が分断されカバーが落ちるのを防ぐため。最大成分を採るのは「探索で作った地図
     の最も広い通行可能空間」をなるべく沢山まわるため。
  4. ウェイポイントは配置用 clearance（壁から余裕）を満たすセルだけを、最大連結成分
     の中から spacing[m] グリッドに間引いて作る。配置と連結で clearance を分けるのが
     要点（連結＝繋がるか / 配置＝壁から離すか は別要件）。
  5. **点間の測地距離（passable 上の最短路長）行列**を作り、NN+2-opt で巡回順を解く。
     直線距離 NN だと「直線では近いが壁越しで実際は遠い」点へ大ジャンプして
     goal_timeout を超えスキップされる。測地距離なら連続点間が必ず通行可能で近い。
  6. map 座標(yaml の resolution/origin で変換)に直して YAML 保存。

「なるべく沢山まわる」と「Nav2 で完走できる」を両立する設計: カバーは連結領域を
spacing で漏れなく拾い、完走は測地巡回順で連続点間の大ジャンプを無くして担保する。

使い方:
  ros2 run susumu_object_perception generate_waypoints.py \
    --map ~/ros2_ws/src/susumu_object_perception/maps/city.yaml \
    --out ~/ros2_ws/src/susumu_object_perception/maps/city_waypoints.yaml \
    --spacing 1.5 --clearance 0.4
"""

import argparse
import heapq
import math
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


_NEI = [(-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)]


def _dijkstra(passable, src):
    """passable(bool 2D) 上を 8 近傍で動くダイクストラ。src(cx,cy) からの距離 [セル]。

    到達不能セルは inf のまま。対角は √2 でコスト付けする。
    """
    h, w = passable.shape
    dist = np.full((h, w), np.inf)
    sx, sy = src
    dist[sy, sx] = 0.0
    pq = [(0.0, sx, sy)]
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[y, x]:
            continue
        for dx, dy in _NEI:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and passable[ny, nx]:
                nd = d + (1.41421356 if dx and dy else 1.0)
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))
    return dist


def _geodesic_matrix(pts, passable):
    """全ウェイポイント間の測地距離（passable 上の最短路長 [セル]）行列。

    各点を始点に 1 回ずつダイクストラする（点数 N に対し N 回）。同じ連結成分内なら
    全ペア有限。点数は間引き済みで数十程度なので実用的に高速。
    """
    n = len(pts)
    dm = np.full((n, n), np.inf)
    for s in range(n):
        d = _dijkstra(passable, pts[s])
        for t in range(n):
            dm[s, t] = d[pts[t][1], pts[t][0]]
    return dm


def _nn_two_opt(dm, start_idx):
    """測地距離行列 dm 上で巡回順を解く。最近傍貪欲法→2-opt 改善（オープンパス）。

    始点 start_idx を固定し、残りを測地最近傍でつなぎ、2-opt で経路長を縮める。
    端点固定なので 2-opt は始点以降の区間反転のみ行う。
    """
    n = len(dm)
    remaining = set(range(n))
    cur = start_idx
    order = [cur]
    remaining.discard(cur)
    while remaining:
        nxt = min(remaining, key=lambda i: dm[cur, i])
        order.append(nxt)
        remaining.discard(nxt)
        cur = nxt

    improved = True
    passes = 0
    while improved and passes < 60:
        improved = False
        passes += 1
        for i in range(1, n - 1):
            for k in range(i + 1, n):
                a, b = order[i - 1], order[i]
                c = order[k]
                if k + 1 < n:
                    d = order[k + 1]
                    old = dm[a, b] + dm[c, d]
                    new = dm[a, c] + dm[b, d]
                else:
                    old = dm[a, b]
                    new = dm[a, c]
                if new + 1e-9 < old:
                    order[i:k + 1] = order[i:k + 1][::-1]
                    improved = True
    return order


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='地図 yaml のパス')
    ap.add_argument('--out', required=True, help='出力 waypoint yaml のパス')
    # ウェイポイント間隔 [m]。大きいほど点が疎。
    ap.add_argument('--spacing', type=float, default=1.5)
    # 【配置用 clearance】壁/未知から最低この距離 [m] 離れた点だけウェイポイントに
    # する（ロボット半径+余裕）。近いとロボットが乗り上げ/衝突で転倒するため余裕を
    # 多めに（0.6m）。屋外の段差が多い world では更に上げる。
    ap.add_argument('--clearance', type=float, default=0.6)
    # 【連結用 clearance】部屋同士を繋ぐ通路の「通れる最小幅」の半分 [m]。連結成分
    # 判定をこの緩い閾値で行い、幅 1.2m 未満の通路（ドア・家具の隙間）で部屋が分断
    # されてカバーが落ちるのを防ぐ。ロボット半径(~0.22m)+α。配置 clearance より緩く。
    ap.add_argument('--connect-clearance', type=float, default=0.30)
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
    connect_cells = args.connect_clearance / res
    place_cells = args.clearance / res

    # === 連結用領域 = 巡回路が通れる領域（緩い clearance）===
    # 連結用 clearance で「ロボットが通れる幅のある領域」を作る。配置用より緩いので
    # 幅 1.2m 未満の通路でも繋がり、部屋同士が分断されない。巡回路（測地距離の最短路）
    # もこの領域上を走る前提。
    connectable = free & (dist_cells >= connect_cells)
    if not connectable.any():
        print('ERROR: connect_clearance を満たす自由空間が無い。'
              '--connect-clearance を下げて。')
        return

    # === 最大連結成分を巡回対象にする ===
    # 探索で作った地図の「最も広い通行可能空間」をなるべく沢山まわるため、最大の
    # 連結成分を採る。残った全点は互いに通行可能で測地距離が必ず有限になり、巡回順の
    # 連続点間が壁越しの大ジャンプ（goal_timeout でスキップされる原因）にならない。
    labels, n_lab = ndimage.label(connectable)
    if n_lab == 0:
        print('ERROR: 連結成分が無い。')
        return
    sizes = ndimage.sum(np.ones_like(labels), labels, range(1, n_lab + 1))
    main_label = int(np.argmax(sizes)) + 1
    connectable = (labels == main_label)
    cys, cxs = np.where(connectable)
    # 巡回の始点（最大成分の重心に最も近いセル）。
    cy0 = int(round(cys.mean()))
    cx0 = int(round(cxs.mean()))
    if not connectable[cy0, cx0]:
        k = int(np.argmin((cxs - cx0) ** 2 + (cys - cy0) ** 2))
        cy0, cx0 = int(cys[k]), int(cxs[k])

    # === ウェイポイント候補 = 最大連結成分 ∩ 配置 clearance を満たすセル ===
    place = connectable & (dist_cells >= place_cells)
    ys, xs = np.where(place)
    if len(xs) == 0:
        print('ERROR: 配置 clearance を満たす候補が無い。--clearance を下げて。')
        return

    # === spacing グリッドで間引いて候補ウェイポイントを作る ===
    # 各 spacing セルにつき「壁から最も遠い1点」を代表点に採る（地図全体を spacing
    # 間隔で漏れなくカバー）。
    step = max(1, int(round(args.spacing / res)))
    best_in_cell = {}
    for cx, cy in zip(xs, ys):
        key = (int(cx) // step, int(cy) // step)
        d = dist_cells[cy, cx]
        if key not in best_in_cell or d > best_in_cell[key][2]:
            best_in_cell[key] = (int(cx), int(cy), float(d))
    pts_cell = [(c[0], c[1]) for c in best_in_cell.values()]

    if len(pts_cell) > args.max_waypoints:
        print(f'  注意: 候補 {len(pts_cell)} > max_waypoints {args.max_waypoints}。'
              f'--spacing を上げると点数が減り 1 周が短くなります（今回は全点出力）。')

    # === 測地距離で巡回順を解く（NN + 2-opt）===
    # 連結成分(connectable)上の最短路長を点間距離に使う。直線距離だと「直線では近いが
    # 壁越しで実は遠い」点へ大ジャンプして goal_timeout でスキップされる。測地距離なら
    # 連続点間が必ず通行可能で短く、止まらず一巡（完走）できる。
    dm_cells = _geodesic_matrix(pts_cell, connectable)
    start_idx = min(range(len(pts_cell)),
                    key=lambda i: (pts_cell[i][0] - cx0) ** 2
                    + (pts_cell[i][1] - cy0) ** 2)
    order = _nn_two_opt(dm_cells, start_idx)

    # ピクセル座標 → map 座標。pgm は上下反転（行0が上端=y最大）。
    def cell_to_map(cx, cy):
        mx = ox + (cx + 0.5) * res
        my = oy + (h - 1 - cy + 0.5) * res
        return (mx, my)

    ordered = [cell_to_map(*pts_cell[i]) for i in order]

    out = {
        'map': os.path.basename(args.map),
        'frame_id': 'map',
        'waypoints': [[round(float(x), 3), round(float(y), 3)] for (x, y) in ordered],
    }
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump(out, f, default_flow_style=None, sort_keys=False)

    # 巡回路の指標を報告（測地経路長と最大ジャンプ＝完走のしやすさの目安）。
    geo = [dm_cells[order[i - 1], order[i]] * res for i in range(1, len(order))]
    straight = [math.hypot(ordered[i][0] - ordered[i - 1][0],
                           ordered[i][1] - ordered[i - 1][1])
                for i in range(1, len(ordered))]
    print(f'生成: {len(ordered)} waypoints -> {args.out}')
    cov_m2 = int(connectable.sum()) * res * res
    print(f'  地図 {w}x{h} ({round(w*res,1)}x{round(h*res,1)}m) '
          f'clearance={args.clearance}m connect={args.connect_clearance}m '
          f'spacing={args.spacing}m カバー領域={cov_m2:.0f}m²')
    if geo:
        print(f'  測地経路長={sum(geo):.1f}m 最大測地ジャンプ={max(geo):.1f}m '
              f'(直線最大={max(straight):.1f}m)')


if __name__ == '__main__':
    main()
