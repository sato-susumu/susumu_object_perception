#!/usr/bin/env python3
"""保存済み占有格子地図から巡回ウェイポイントを生成する（オフライン）。

frontier 探索で作った outputs/mapping_*/<world>.pgm/.yaml を読み、自由空間のうち壁から十分
離れた点を、地図全体をカバーするように間引いて巡回順に並べ、YAML に保存する。
生成した YAML は waypoint_nav_node.py（NavigateToPose 巡回）と
waypoint_viz_node.py（RViz 可視化）が読む。併せて、地図にウェイポイント・巡回経路・
起点・clearance 領域を重ねた確認用 PNG（<out> と同名 .png）も出力する（--no-png で抑止）。

アルゴリズム:
  1. PGM を読み free(255 付近)/occupied(0 付近)/unknown(205) に分類。
  2. 占有・未知セルからの距離変換(distance transform)を計算。
  3. 連結用 clearance（緩い閾値）で「通れる領域」を作り、巡回 edge に使う
     route 用 clearance（未指定なら連結用と同じ）で絞った**最大連結成分**を
     巡回対象にする。連結用を緩くするのは、幅 1.2m 未満の通路（ドア・家具の隙間）で
     部屋が分断されカバーが落ちるのを防ぐため。屋外では route 用を高めにし、Nav2 の
     inflation で足元が lethal になる壁際 edge を避ける。最大成分を採るのは
     「探索で作った地図の最も広い通行可能空間」をなるべく沢山まわるため。
  4. ウェイポイント候補を作る（--candidate-mode）。
     - grid（既定・従来）: 配置用 clearance を満たすセルを spacing[m] グリッドに間引く。
       配置と連結で clearance を分けるのが要点（連結＝繋がるか / 配置＝壁から離すか は別要件）。
     - sparse_graph（SWAGGER 着想）: 距離変換の局所最大 + NMS で疎なノードを作る。点数が
       地図面積でなく通路・部屋・分岐の複雑さに近づく。**アイデアの出典は SWAGGER（Sparse
       WAypoint Graph Generation for Efficient Routing）。考え方のみを借り、SWAGGER 本体の
       コードや cuCIM/CUDA/scikit-image には依存しない**（詳細は _sparse_graph_candidates の
       docstring）。
  5. **点間の測地距離（passable 上の最短路長）行列**を作り、NN+2-opt で巡回順を解く。
     直線距離 NN だと「直線では近いが壁越しで実際は遠い」点へ大ジャンプして
     goal_timeout を超えスキップされる。測地距離なら連続点間が必ず通行可能で近い。
     屋外では任意で edge clearance cost を使い、通行可能ではあるが inflation に近い
     狭い corridor を route graph 全体で選びにくくする。
  6. `--max-segment-length` を指定した場合、長い測地区間だけ最短経路上の中間点で
     分割する。中間点は可能な限り通常 waypoint と同じ配置 clearance を満たすセルに置く。
  7. 認識巡回向けに `--object-viewpoints` を指定した場合、壁ではない小〜中サイズの
     occupied 成分を安全距離から見る追加点を入れる（既定 OFF）。追加点の clearance は
     未指定なら通常ウェイポイントの配置用 clearance と同じ値にする。
  8. map 座標(yaml の resolution/origin で変換)に直して YAML 保存。

「なるべく沢山まわる」と「Nav2 で完走できる」を両立する設計: カバーは連結領域を
spacing で漏れなく拾い、完走は測地巡回順で連続点間の大ジャンプを無くして担保する。

使い方:
  ros2 run susumu_object_perception generate_waypoints.py \
    --map ~/ros2_ws/src/susumu_object_perception/outputs/mapping_outdoor/city.yaml \
    --out ~/ros2_ws/src/susumu_object_perception/outputs/waypoint_generation/city_waypoints.yaml \
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
        read_token()  # consume maxv (P5/P2 ヘッダ消費、 8bit 前提)
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


def _dijkstra_clearance_weighted(passable, src, dist_cells,
                                 desired_clearance_cells, weight):
    """clearance 不足を移動コストに乗せる 8 近傍ダイクストラ。

    `route-clearance` は二値の通行可否だが、屋外では「通れるが inflation や
    実軌跡では危ない」細い corridor が残る。ここでは障害物/unknown からの
    距離場を使い、desired_clearance に満たないセルほど高コストにする。
    通行可否は変えず、route graph の順序だけを広い通路寄りへ寄せる。
    """
    if weight <= 0.0 or desired_clearance_cells <= 0.0:
        return _dijkstra(passable, src)

    h, w = passable.shape
    dist = np.full((h, w), np.inf)
    sx, sy = src
    dist[sy, sx] = 0.0
    pq = [(0.0, sx, sy)]
    denom = max(1e-6, float(desired_clearance_cells))
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[y, x]:
            continue
        for dx, dy in _NEI:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and passable[ny, nx]:
                step = 1.41421356 if dx and dy else 1.0
                shortfall = max(
                    0.0, (desired_clearance_cells - dist_cells[ny, nx])
                    / denom)
                nd = d + step * (1.0 + weight * shortfall * shortfall)
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))
    return dist


def _shortest_path(passable, src, goal):
    """passable 上の最短経路をセル列で返す。到達不能なら空リスト。

    全ペア距離行列は既存の _dijkstra で作るが、実際に経路セルが必要なのは
    「長すぎる区間を分割する」少数の edge だけなので、親ポインタ付き探索は
    ここで個別に実行する。
    """
    if src == goal:
        return [src]
    h, w = passable.shape
    sx, sy = src
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return []
    if not passable[sy, sx] or not passable[gy, gx]:
        return []

    dist = np.full((h, w), np.inf)
    parent = {}
    dist[sy, sx] = 0.0
    pq = [(0.0, sx, sy)]
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[y, x]:
            continue
        if (x, y) == (gx, gy):
            break
        for dx, dy in _NEI:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and passable[ny, nx]:
                nd = d + (1.41421356 if dx and dy else 1.0)
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    parent[(nx, ny)] = (x, y)
                    heapq.heappush(pq, (nd, nx, ny))

    if not math.isfinite(dist[gy, gx]):
        return []
    path = [(gx, gy)]
    cur = (gx, gy)
    while cur != (sx, sy):
        cur = parent.get(cur)
        if cur is None:
            return []
        path.append(cur)
    path.reverse()
    return path


def _shortest_path_clearance_weighted(passable, src, goal, dist_cells,
                                      desired_clearance_cells, weight):
    """clearance weighted cost の最短経路をセル列で返す。"""
    if weight <= 0.0 or desired_clearance_cells <= 0.0:
        return _shortest_path(passable, src, goal)
    if src == goal:
        return [src]
    h, w = passable.shape
    sx, sy = src
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return []
    if not passable[sy, sx] or not passable[gy, gx]:
        return []

    dist = np.full((h, w), np.inf)
    parent = {}
    dist[sy, sx] = 0.0
    pq = [(0.0, sx, sy)]
    denom = max(1e-6, float(desired_clearance_cells))
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[y, x]:
            continue
        if (x, y) == (gx, gy):
            break
        for dx, dy in _NEI:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and passable[ny, nx]:
                step = 1.41421356 if dx and dy else 1.0
                shortfall = max(
                    0.0, (desired_clearance_cells - dist_cells[ny, nx])
                    / denom)
                nd = d + step * (1.0 + weight * shortfall * shortfall)
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    parent[(nx, ny)] = (x, y)
                    heapq.heappush(pq, (nd, nx, ny))

    if not math.isfinite(dist[gy, gx]):
        return []
    path = [(gx, gy)]
    cur = (gx, gy)
    while cur != (sx, sy):
        cur = parent.get(cur)
        if cur is None:
            return []
        path.append(cur)
    path.reverse()
    return path


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


def _clearance_weighted_matrix(pts, passable, dist_cells,
                               desired_clearance_cells, weight):
    """全点間の clearance weighted route cost 行列。"""
    n = len(pts)
    dm = np.full((n, n), np.inf)
    for s in range(n):
        d = _dijkstra_clearance_weighted(
            passable, pts[s], dist_cells, desired_clearance_cells, weight)
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


def _split_long_segments(ordered_cells, passable, res, max_segment_length,
                         waypoint_mask=None, shortest_path_func=None):
    """長い測地区間を最短経路上の中間 waypoint で分割する。

    waypoint 同士が地図上では近く見えても、測地経路が長いと Nav2 の 1 goal が
    長距離になり、局所的な再計画や costmap の lethal 化で失敗したときの影響が大きい。
    既存の巡回順は保ち、長い edge だけを passable 上の最短経路に沿って分割する。
    """
    if max_segment_length <= 0.0 or len(ordered_cells) < 2:
        return ordered_cells, [], 0

    if waypoint_mask is None:
        waypoint_mask = passable
    if shortest_path_func is None:
        shortest_path_func = lambda start, goal: _shortest_path(
            passable, start, goal)
    expanded = [ordered_cells[0]]
    segment_lengths = []
    inserted = 0
    for start, goal in zip(ordered_cells[:-1], ordered_cells[1:]):
        path = shortest_path_func(start, goal)
        if not path:
            expanded.append(goal)
            segment_lengths.append(float('inf'))
            continue
        cumulative = [0.0]
        for i in range(1, len(path)):
            dx = abs(path[i][0] - path[i - 1][0])
            dy = abs(path[i][1] - path[i - 1][1])
            step = (1.41421356 if dx and dy else 1.0) * res
            cumulative.append(cumulative[-1] + step)
        last_idx = 0
        total = cumulative[-1]
        while total - cumulative[last_idx] > max_segment_length:
            target = cumulative[last_idx] + max_segment_length
            target_idx = next(
                i for i in range(last_idx + 1, len(cumulative))
                if cumulative[i] >= target)
            candidates = [
                i for i in range(last_idx + 1, len(path) - 1)
                if waypoint_mask[path[i][1], path[i][0]]
            ]
            if candidates:
                split_idx = min(
                    candidates, key=lambda i: abs(cumulative[i] - target))
            else:
                split_idx = min(target_idx, len(path) - 2)
            cell = path[split_idx]
            if expanded[-1] != cell:
                expanded.append(cell)
                segment_lengths.append(cumulative[split_idx] -
                                       cumulative[last_idx])
                inserted += 1
            last_idx = split_idx
        if expanded[-1] != goal:
            expanded.append(goal)
            segment_lengths.append(total - cumulative[last_idx])
    return expanded, segment_lengths, inserted


def _relink_long_jumps(ordered_cells, all_cells, passable, res,
                       max_jump_m, shortest_path_func):
    """巡回順の長い区間を、既存ウェイポイントを経由して短いジャンプの連続に置き換える。

    NN+2-opt は各点を 1 回ずつ訪れる順序を作るが、什器などを大きく迂回する区間では連続 2 点の
    測地距離が長くなる（例: #5→#6 が 6m）。ここで「既に訪れた点と重なってもよいので、その区間の
    測地経路の近くにある既存ウェイポイントを経由点として挿入」し、1 つの長いジャンプを複数の短い
    ジャンプに割る。これにより Nav2 の 1 ゴールあたり移動距離が短くなり、局所 planner が長い迂回で
    詰まりにくくなる（ユーザー提案。SWAGGER の「疎グラフの edge 上を辿る」発想に近い）。

    ordered_cells: 巡回順のセル列。all_cells: 全ウェイポイントのセル（経由候補）。
    戻り値は経由点を挿入した新しいセル列（既存点との重複を許す）。
    """
    max_jump_cells = max_jump_m / res
    out = [ordered_cells[0]]
    for a, b in zip(ordered_cells[:-1], ordered_cells[1:]):
        path = shortest_path_func(a, b)
        glen = _path_len_cells(path) if path else float('inf')
        if glen <= max_jump_cells or not path:
            out.append(b)
            continue
        # この区間の経路の近く(経路から via_dist 以内)にある、a,b 以外の既存点を集める。
        via_dist2 = (max_jump_m / res) ** 2
        path_arr = np.array(path)
        vias = []
        for c in all_cells:
            if c == a or c == b:
                continue
            d2 = np.min((path_arr[:, 0] - c[0]) ** 2 + (path_arr[:, 1] - c[1]) ** 2)
            if d2 <= via_dist2:
                # 経路上で c に最も近い点の経路インデックス（経由順を決めるため）。
                k = int(np.argmin((path_arr[:, 0] - c[0]) ** 2
                                  + (path_arr[:, 1] - c[1]) ** 2))
                vias.append((k, c))
        vias.sort()  # 経路に沿った順に経由
        # 経由点は「直前に出した点と一定距離以上離れている」ものだけ採る。relink は経路を
        # 短いジャンプに割るのが目的で、同座標・近接の重複点を入れると waypoint_nav が
        # 「既に居る場所」と見なして skip(missed)するため、重複を作らない。
        min_sep_cells2 = (max(0.5, max_jump_m * 0.3) / res) ** 2
        for _k, c in vias:
            px, py = out[-1]
            if (c[0] - px) ** 2 + (c[1] - py) ** 2 >= min_sep_cells2:
                out.append(c)
        bx, by = out[-1]
        if (b[0] - bx) ** 2 + (b[1] - by) ** 2 >= min_sep_cells2 or out[-1] == ordered_cells[0]:
            out.append(b)
        elif out[-1] != b:
            # b が直前と近接でも、巡回点 b 自体は必ず残す（カウント対象）。直前の経由点を b に置換。
            out[-1] = b
    return out


def _path_len_cells(path):
    if not path or len(path) < 2:
        return 0.0
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        total += 1.41421356 if (x0 != x1 and y0 != y1) else 1.0
    return total


def _route_edge_stats(ordered_cells, passable, dist_cells, res,
                      desired_clearance_m, shortest_path_func):
    """巡回 edge ごとの clearance リスクを出す。"""
    rows = []
    for idx, (start, goal) in enumerate(
            zip(ordered_cells[:-1], ordered_cells[1:])):
        path = shortest_path_func(start, goal)
        if not path:
            rows.append({
                'edge': idx,
                'from_cell': start,
                'to_cell': goal,
                'path_cells': 0,
                'geodesic_m': float('inf'),
                'min_clearance_m': 0.0,
                'shortfall_integral_m2': float('inf'),
                'low_clearance_cells': 0,
            })
            continue
        geodesic = 0.0
        shortfall = 0.0
        low_cells = 0
        min_clearance = float('inf')
        prev = None
        for cell in path:
            x, y = cell
            clr = float(dist_cells[y, x]) * res
            min_clearance = min(min_clearance, clr)
            if clr < desired_clearance_m:
                low_cells += 1
                shortfall += (desired_clearance_m - clr) * res
            if prev is not None:
                dx = abs(x - prev[0])
                dy = abs(y - prev[1])
                geodesic += (1.41421356 if dx and dy else 1.0) * res
            prev = cell
        rows.append({
            'edge': idx,
            'from_cell': start,
            'to_cell': goal,
            'path_cells': len(path),
            'geodesic_m': geodesic,
            'min_clearance_m': min_clearance,
            'shortfall_integral_m2': shortfall,
            'low_clearance_cells': low_cells,
        })
    return rows


def _write_edge_risk_report(prefix, edge_stats, cell_to_map,
                            edge_clearance_m, edge_clearance_weight):
    """edge clearance risk の CSV/Markdown を保存する。"""
    import csv
    import json

    out_dir = os.path.dirname(prefix)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    rows = []
    for stat in edge_stats:
        sx, sy = stat['from_cell']
        gx, gy = stat['to_cell']
        smx, smy = cell_to_map(sx, sy)
        gmx, gmy = cell_to_map(gx, gy)
        row = dict(stat)
        row.pop('from_cell', None)
        row.pop('to_cell', None)
        row.update({
            'from_x': round(smx, 3),
            'from_y': round(smy, 3),
            'to_x': round(gmx, 3),
            'to_y': round(gmy, 3),
            'min_clearance_m': round(stat['min_clearance_m'], 3),
            'geodesic_m': round(stat['geodesic_m'], 3)
            if math.isfinite(stat['geodesic_m']) else 'inf',
            'shortfall_integral_m2':
                round(stat['shortfall_integral_m2'], 5)
                if math.isfinite(stat['shortfall_integral_m2']) else 'inf',
        })
        rows.append(row)

    keys = [
        'edge', 'from_x', 'from_y', 'to_x', 'to_y', 'path_cells',
        'geodesic_m', 'min_clearance_m', 'shortfall_integral_m2',
        'low_clearance_cells',
    ]
    with open(prefix + '.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in keys})

    worst = sorted(
        rows, key=lambda r: float(r['shortfall_integral_m2'])
        if r['shortfall_integral_m2'] != 'inf' else float('inf'),
        reverse=True)
    summary = {
        'edge_clearance_m': edge_clearance_m,
        'edge_clearance_weight': edge_clearance_weight,
        'edges': len(rows),
        'worst_edge': worst[0] if worst else None,
    }
    with open(prefix + '.json', 'w') as f:
        json.dump({'summary': summary, 'edges': rows},
                  f, indent=2, ensure_ascii=False)
        f.write('\n')

    lines = [
        '# Waypoint edge clearance risk',
        '',
        f'- edge_clearance_m: `{edge_clearance_m}`',
        f'- edge_clearance_weight: `{edge_clearance_weight}`',
        f'- edges: `{len(rows)}`',
        '',
        '| edge | from | to | geodesic | min clearance | shortfall | low cells |',
        '|---:|---|---|---:|---:|---:|---:|',
    ]
    for row in worst[:20]:
        lines.append(
            f"| {row['edge']} | "
            f"({row['from_x']}, {row['from_y']}) | "
            f"({row['to_x']}, {row['to_y']}) | "
            f"{row['geodesic_m']} | {row['min_clearance_m']} | "
            f"{row['shortfall_integral_m2']} | "
            f"{row['low_clearance_cells']} |")
    with open(prefix + '.md', 'w') as f:
        f.write('\n'.join(lines) + '\n')


def _coverage_fill(kept, place, res, cover_radius):
    """疎ノードだけだとカバレッジが手薄な place 領域に点を追加する。

    距離変換の局所最大（中心線）だけでは、広い部屋の壁際など「最近傍ノードから遠い自由空間」
    が残る（SWAGGER でも free-space sampling でこれを補う）。既採用ノードから cover_radius[m]
    より遠い place セルを farthest-point sampling で拾い、カバレッジの偏りを埋める。
    """
    ys, xs = np.where(place)
    if len(xs) == 0:
        return kept
    pts = np.column_stack([xs, ys]).astype(np.float64)
    cov_cells = cover_radius / res
    kept = list(kept)
    # 既採用ノードからの最近傍距離（セル）を初期化。
    if kept:
        kx = np.array([k[0] for k in kept]); ky = np.array([k[1] for k in kept])
        d2 = np.min((pts[:, 0:1] - kx) ** 2 + (pts[:, 1:2] - ky) ** 2, axis=1)
    else:
        d2 = np.full(len(pts), np.inf)
    while True:
        i = int(np.argmax(d2))
        if d2[i] <= cov_cells ** 2:
            break  # 最も手薄な点でも cover_radius 内＝もう穴は無い
        nx, ny = int(pts[i, 0]), int(pts[i, 1])
        kept.append((nx, ny))
        nd2 = (pts[:, 0] - nx) ** 2 + (pts[:, 1] - ny) ** 2
        d2 = np.minimum(d2, nd2)
    return kept


def _sparse_graph_candidates(place, dist_cells, res, node_spacing,
                             cover_radius=0.0):
    """SWAGGER-lite: 距離変換の局所最大から疎な候補ノードを作る。

    アイデアの出典（attribution）:
      占有格子地図の自由空間に「疎な waypoint ノード」を置く着想は SWAGGER
      （Sparse WAypoint Graph Generation for Efficient Routing）に着想を得ている。
      SWAGGER は距離変換・skeleton・boundary node・free-space local maxima・Delaunay
      triangulation・pruning でグラフを作る手法。本実装はその**考え方のみ**を借り、
      コードは一切流用していない（SWAGGER 本体・cuCIM・scikit-image・CUDA には依存しない）。
      本リポの目的は global planner 用グラフではなく「保存地図を偏りなく巡回する route」
      なので、SWAGGER の構成要素のうち「free-space の local maxima にノードを置く」だけを
      scipy.ndimage + numpy で最小実装し、接続・巡回順は既存の測地距離 TSP を流用する。
      参考: SWAGGER (occupancy grid -> sparse waypoint graph for route planning)。

    処理:
      1. place(最大連結成分 ∩ 配置 clearance)上で、距離変換 dist_cells の局所最大を取る。
         local maxima は部屋の中心・通路の中心に出やすく、広い空間ほど点が密にならない。
      2. 近すぎる候補（node_spacing[m] 未満）は壁から遠い方を残して間引く（NMS / pruning 相当）。

    こうすると候補点数が「地図面積」でなく「通路・部屋・分岐の複雑さ」に近づく。
    skeleton / Delaunay / scikit-image は使わず scipy.ndimage と numpy だけで済む（追加依存なし）。
    返り値は grid モードと同じ ``[(cx, cy), ...]`` で、下流の測地距離 TSP / YAML / PNG をそのまま使える。
    """
    from scipy.ndimage import maximum_filter

    # 局所最大の探索窓。node_spacing 程度の半径で見れば、近接ピークが 1 つにまとまる。
    win = max(3, int(round(node_spacing / res)))
    if win % 2 == 0:
        win += 1
    local_max = (dist_cells == maximum_filter(dist_cells, size=win)) & place
    ys, xs = np.where(local_max)
    cand = [(int(x), int(y), float(dist_cells[y, x])) for x, y in zip(xs, ys)]
    if not cand:
        return []
    # 壁から遠い順に貪欲採用し、既採用点から node_spacing 未満の候補は捨てる（NMS）。
    cand.sort(key=lambda c: c[2], reverse=True)
    sep_cells2 = (node_spacing / res) ** 2
    kept = []
    for cx, cy, _d in cand:
        ok = True
        for kx, ky in kept:
            if (cx - kx) ** 2 + (cy - ky) ** 2 < sep_cells2:
                ok = False
                break
        if ok:
            kept.append((cx, cy))
    # 局所最大（中心線）だけだと壁際の広い空間が手薄になるので、cover_radius を超えて
    # 手薄な place 領域に点を足してカバレッジの偏りを埋める。
    if cover_radius > 0.0:
        kept = _coverage_fill(kept, place, res, cover_radius)
    return kept


def _limit_points_by_farthest_sampling(pts, max_points, seed):
    """候補点を max_points 個まで、空間的に散るように間引く。

    max-waypoints は「1 周を現実的な長さに抑える」ための上限なので、候補を
    先頭から切ると地図の一部だけに偏る。重心付近の点を起点に、既に選んだ
    点群から最も遠い点を順に追加する farthest-point sampling で粗いカバーを
    維持する。
    """
    if max_points <= 0 or len(pts) <= max_points:
        return pts

    sx, sy = seed
    start = min(range(len(pts)),
                key=lambda i: (pts[i][0] - sx) ** 2 + (pts[i][1] - sy) ** 2)
    selected_idx = [start]
    selected = [pts[start]]
    remaining = [i for i in range(len(pts)) if i != start]
    min_d2 = {}
    for i in remaining:
        min_d2[i] = ((pts[i][0] - pts[start][0]) ** 2
                     + (pts[i][1] - pts[start][1]) ** 2)

    while remaining and len(selected) < max_points:
        nxt = max(remaining, key=lambda i: min_d2[i])
        remaining.remove(nxt)
        selected_idx.append(nxt)
        selected.append(pts[nxt])
        nx, ny = pts[nxt]
        for i in remaining:
            d2 = (pts[i][0] - nx) ** 2 + (pts[i][1] - ny) ** 2
            if d2 < min_d2[i]:
                min_d2[i] = d2
    return selected


def _append_object_viewpoints(pts_cell, occ, connectable, dist_cells, res,
                              object_viewpoints, view_clearance,
                              object_min_area, object_max_area,
                              min_separation, map_border_margin):
    """認識巡回向けに occupied 小〜中サイズ成分の近くへ追加視点を入れる。

    通常の spacing 代表点は「壁から遠い通路中央」を選ぶため、家具や植物の前を
    十分近く通らないことがある。ここでは world 真値を使わず、保存地図上の occupied
    成分だけを見て、指定面積範囲の成分に近い安全セルを追加する。
    """
    if object_viewpoints <= 0:
        return pts_cell, []
    h, w = connectable.shape
    border_cells = max(0, int(math.ceil(float(map_border_margin) / res)))
    interior = np.ones_like(connectable, dtype=bool)
    if border_cells > 0:
        interior[:border_cells, :] = False
        interior[-border_cells:, :] = False
        interior[:, :border_cells] = False
        interior[:, -border_cells:] = False
    view_place = connectable & interior & (dist_cells >= (view_clearance / res))
    ys_view, xs_view = np.where(view_place)
    if len(xs_view) == 0:
        return pts_cell, []

    labels, n_lab = ndimage.label(occ)
    sizes = ndimage.sum(np.ones_like(labels), labels, range(1, n_lab + 1))
    existing = list(pts_cell)
    min_sep_cells = max(1.0, float(min_separation) / res)
    candidates = []
    for lab, size in enumerate(sizes, 1):
        area = float(size) * res * res
        if area < object_min_area or area > object_max_area:
            continue
        ys, xs = np.where(labels == lab)
        if len(xs) == 0:
            continue
        if border_cells > 0 and (
                xs.min() < border_cells or xs.max() >= w - border_cells or
                ys.min() < border_cells or ys.max() >= h - border_cells):
            continue
        cx = float(xs.mean())
        cy = float(ys.mean())
        d2 = (xs_view.astype(np.float32) - cx) ** 2 + \
            (ys_view.astype(np.float32) - cy) ** 2
        k = int(np.argmin(d2))
        vx, vy = int(xs_view[k]), int(ys_view[k])
        if existing:
            cover = min(math.hypot(vx - px, vy - py) for px, py in existing)
        else:
            cover = float('inf')
        # すでに通常巡回点が十分近い成分は追加しない。
        if cover < min_sep_cells:
            continue
        candidates.append((cover, area, vx, vy, lab))

    candidates.sort(reverse=True)
    added = []
    for _, area, vx, vy, lab in candidates:
        if len(added) >= object_viewpoints:
            break
        if existing and min(math.hypot(vx - px, vy - py)
                            for px, py in existing) < min_sep_cells:
            continue
        existing.append((vx, vy))
        added.append((vx, vy, area, lab))
    return existing, added


def save_overlay(png_path, img, place, connectable, pts_cell, order,
                 start_cell, place_cells, title):
    """地図にウェイポイント・巡回経路・起点・clearance 領域を重ねた PNG を保存。

    RViz を立てなくても巡回路が壁を跨がないか・カバー範囲を一目で確認できる。
    ピクセル座標系のまま描く（imshow の行=y下向き）。matplotlib は Agg
    バックエンドでヘッドレスでも動く。
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    h, w = img.shape
    fig, ax = plt.subplots(figsize=(w / 50.0, h / 50.0), dpi=120)
    # 背景: 地図(白=free/灰=unknown/黒=occ)。
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper')
    # 巡回対象の最大連結成分を薄青、配置 clearance を満たす領域を薄緑で塗る。
    conn_rgba = np.zeros((h, w, 4))
    conn_rgba[connectable] = (0.3, 0.6, 1.0, 0.18)   # 通行可能領域(連結成分)
    ax.imshow(conn_rgba, origin='upper')
    place_rgba = np.zeros((h, w, 4))
    place_rgba[place] = (0.2, 0.8, 0.2, 0.18)         # 配置可能領域(clearance)
    ax.imshow(place_rgba, origin='upper')

    # 巡回順に並べたセル座標。
    xs = [pts_cell[i][0] for i in order]
    ys = [pts_cell[i][1] for i in order]
    # 巡回経路線（順に点をつなぐ）。
    ax.plot(xs, ys, '-', color='orange', linewidth=1.5, alpha=0.9, zorder=2,
            label='patrol path')
    # 番号付き点（巡回開始点=赤、他=青）。番号は巡回順。
    ax.plot([], [], 'o', color='royalblue', markersize=5, label='waypoint')
    ax.plot([], [], 'o', color='red', markersize=7, label='waypoint #0 (first)')
    for rank, (cx, cy) in enumerate(zip(xs, ys)):
        is_first = (rank == 0)
        ax.plot(cx, cy, 'o', color='red' if is_first else 'royalblue',
                markersize=7 if is_first else 5, zorder=3)
        ax.annotate(str(rank), (cx, cy), color='black', fontsize=6,
                    fontweight='bold', xytext=(3, 3),
                    textcoords='offset points', zorder=4)
    # ロボット起点（最大連結成分の重心）を×で明示。ラベルは日本語フォント非依存
    # にするため英字（matplotlib 既定フォントに日本語グリフが無い環境向け）。
    ax.plot(start_cell[0], start_cell[1], 'x', color='magenta',
            markersize=11, markeredgewidth=2.5, zorder=5,
            label='start (centroid)')

    ax.set_title(title, fontsize=9)
    ax.set_xlabel('x [cell]   (green=placeable area, blue tint=connected area)',
                  fontsize=8)
    ax.set_ylabel('y [cell]')
    # 凡例は地図に被らないよう軸の外（下）へ。xlabel とも重ならないよう離す。
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13),
              ncol=2, fontsize=7, framealpha=0.9)
    # 小さい地図では tight_layout が legend を収めきれず警告を出すため、
    # savefig 側の bbox_inches='tight' で余白を自動確保する。
    fig.savefig(png_path, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='地図 yaml のパス')
    ap.add_argument('--out', required=True, help='出力 waypoint yaml のパス')
    # ウェイポイント間隔 [m]。大きいほど点が疎。
    ap.add_argument('--spacing', type=float, default=1.5)
    # 【候補生成モード】grid: 既存（spacing グリッド代表点。地図面積に比例して点が増える）。
    # sparse_graph: SWAGGER-lite（距離変換の局所最大 + NMS。点数が通路・部屋・分岐の複雑さに
    # 近づき、広い空間で無駄に点が増えない）。下流の測地距離 TSP / YAML / PNG は共通。
    ap.add_argument('--candidate-mode', choices=('grid', 'sparse_graph'),
                    default='grid', help='候補点生成方式（既定 grid＝従来互換）')
    # sparse_graph のノード最小間隔 [m]。0 以下なら --spacing を流用。
    ap.add_argument('--graph-node-spacing', type=float, default=0.0,
                    help='sparse_graph のノード最小間隔 [m]（0 で --spacing を使う）')
    # sparse_graph のカバレッジ補完半径 [m]。局所最大だけだと壁際が手薄になるので、
    # 既存ノードからこの距離より遠い自由空間に点を足す。0 以下なら 2*node_spacing。
    ap.add_argument('--graph-cover-radius', type=float, default=0.0,
                    help='sparse_graph で手薄な自由空間を埋める半径 [m]（0 で 2*node_spacing）')
    # 【配置用 clearance】壁/未知から最低この距離 [m] 離れた点だけウェイポイントに
    # する（ロボット半径+余裕）。近いとロボットが乗り上げ/衝突で転倒するため余裕を
    # 多めに（0.6m）。屋外の段差が多い world では更に上げる。
    ap.add_argument('--clearance', type=float, default=0.6)
    # 【連結用 clearance】部屋同士を繋ぐ通路の「通れる最小幅」の半分 [m]。連結成分
    # 判定をこの緩い閾値で行い、幅 1.2m 未満の通路（ドア・家具の隙間）で部屋が分断
    # されてカバーが落ちるのを防ぐ。ロボット半径(~0.22m)+α。配置 clearance より緩く。
    ap.add_argument('--connect-clearance', type=float, default=0.30)
    # 【経路用 clearance】測地距離と edge 最短路に使う passable 領域。
    # 未指定なら connect_clearance と同じで従来互換。屋外では costmap inflation と
    # 整合させるため、配置 clearance と同程度に上げる。
    ap.add_argument('--route-clearance', type=float, default=None,
                    help='clearance for geodesic route edges [m] (default: --connect-clearance)')
    # route-clearance は「そのセルを通れるか」の二値判定。edge-clearance は
    # 通れるセルの中でも obstacle/unknown に近い経路を巡回順で避けるための
    # soft cost。既定 0.0 で完全に従来互換。
    ap.add_argument('--edge-clearance', type=float, default=None,
                    help='desired clearance for soft edge cost [m] (default: --route-clearance)')
    ap.add_argument('--edge-clearance-weight', type=float, default=0.0,
                    help='soft penalty weight for low-clearance route edges')
    ap.add_argument('--edge-risk-report', default='',
                    help='optional prefix for edge risk CSV/MD reports')
    # 巡回の最大ウェイポイント数（多すぎると 1 周が長い）。
    ap.add_argument('--max-waypoints', type=int, default=40)
    # 長い測地区間を中間 waypoint で分割する閾値。0 以下なら従来通り分割しない。
    # 巡回順の長い区間（測地距離がこの値[m]超）を、既存ウェイポイントを経由して短いジャンプの
    # 連続に置き換える（既訪点との重複を許す）。0 で無効。什器の大迂回で 1 ゴールの移動が長く
    # なり Nav2 が詰まるのを防ぐ。--max-segment-length と併用可（relink で経由→残りを分割）。
    ap.add_argument('--relink-long-jumps', type=float, default=0.0,
                    help='測地距離がこの値[m]超の区間を既存点経由で短い連続ジャンプにする（0で無効）')
    ap.add_argument('--max-segment-length', type=float, default=0.0,
                    help='split route edges longer than this geodesic length [m]')
    # Consecutive duplicate-like waypoints (e.g. 5cm apart pairs created by relink /
    # split) are mapped to "already at goal" by Nav2 and just consume goal_timeout
    # per WP. xy_goal_tolerance is typically 0.25m so points closer than ~0.3m are
    # practically the same goal. Merge them in post-processing.
    ap.add_argument('--dedupe-min-separation', type=float, default=0.3,
                    help='merge consecutive waypoints closer than this [m] (0 disables)')
    # Grid モードでも疎な分布を実現する Adaptive NMS (ANMS 系の簡易版)。
    # 既定値 0.6 は spacing 1.5m との比 0.4 で、 1.5m × 0.4 = 0.6m を最小間隔とする。
    # これは Nav2 の xy_goal_tolerance (0.25m) を 2 倍以上超える距離で「実質同じゴール」
    # にならない設計。 0 で無効。
    ap.add_argument('--grid-nms-separation-ratio', type=float, default=0.4,
                    help='grid NMS で最小許容距離 = spacing * ratio。0 で無効 (既定 0.4)')
    # 屋外のように LiDAR が通行止めの向こう側を free として広く観測する地図では、
    # 探索対象外の free セルまで巡回点にすると Nav2 が区画外へ向かう。0 以下なら無制限。
    ap.add_argument('--limit-radius', type=float, default=0.0,
                    help='only use cells within this radius [m] from limit center')
    ap.add_argument('--limit-center-x', type=float, default=0.0,
                    help='map-frame x center for --limit-radius')
    ap.add_argument('--limit-center-y', type=float, default=0.0,
                    help='map-frame y center for --limit-radius')
    # 認識巡回向けの追加視点。通常の巡回点は通路中央寄りなので、家具・植物など
    # occupied 小〜中サイズ成分の近くを通りたい場合だけ有効にする。
    ap.add_argument('--object-viewpoints', type=int, default=0,
                    help='occupied object-like components to add as recognition viewpoints')
    ap.add_argument('--view-clearance', type=float, default=None,
                    help='clearance for object-viewpoints [m] (default: same as --clearance)')
    ap.add_argument('--object-min-area', type=float, default=0.005,
                    help='minimum occupied component area for object-viewpoints [m^2]')
    ap.add_argument('--object-max-area', type=float, default=0.40,
                    help='maximum occupied component area for object-viewpoints [m^2]')
    ap.add_argument('--viewpoint-min-separation', type=float, default=0.75,
                    help='minimum distance from existing waypoints for added viewpoints [m]')
    ap.add_argument('--view-map-border-margin', type=float, default=0.25,
                    help='do not add object-viewpoints for map-border components [m]')
    # 地図にウェイポイントを重ねた確認用 PNG を出さない（既定は出す）。
    ap.add_argument('--no-png', action='store_true',
                    help='<out> と同名の確認用 .png を生成しない')
    args = ap.parse_args()
    if args.view_clearance is None:
        args.view_clearance = args.clearance
    if args.route_clearance is None:
        args.route_clearance = args.connect_clearance
    if args.edge_clearance is None:
        args.edge_clearance = args.route_clearance

    with open(args.map) as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    pgm_path = os.path.join(os.path.dirname(args.map), meta['image'])

    img = load_pgm(pgm_path)
    h, w = img.shape
    # map_server 流: trinary。occ=黒(0 付近), free=白(254/255), unknown=205。
    # 占有確率 p = (255 - value)/255。p>=occ_thresh が占有、p<=meta['free_thresh'] が空き。
    # ただし unknown(205)は p=0.196 で meta['free_thresh'](既定 0.25)未満になり free に誤分類される。
    # map_server の trinary 規約どおり「205 は unknown」を厳守するため、free は
    # 「明確に白(>=250)」に限定する。これを外すと地図端の unknown 帯が free 扱いされ、
    # 距離変換が地図境界を壁とみなさず、ウェイポイントが unknown 領域（PNG では地図端の
    # 灰色＝壁の外側）に置かれてしまう。
    p = (255.0 - img.astype(np.float32)) / 255.0
    occ = p >= occ_thresh
    free = (img >= 250) & ~occ
    unknown = ~free & ~occ
    # 通行不可（占有 or 未知）からの距離 [セル]。free セルが壁/未知からどれだけ
    # 離れているか。
    blocked = occ | unknown
    dist_cells = ndimage.distance_transform_edt(~blocked)
    connect_cells = args.connect_clearance / res
    route_cells_clearance = args.route_clearance / res
    place_cells = args.clearance / res
    edge_clearance_cells = args.edge_clearance / res

    limit_mask = np.ones_like(free, dtype=bool)
    if args.limit_radius > 0.0:
        grid_y, grid_x = np.indices((h, w))
        map_x = ox + (grid_x + 0.5) * res
        map_y = oy + (h - 1 - grid_y + 0.5) * res
        limit_mask = ((map_x - args.limit_center_x) ** 2
                      + (map_y - args.limit_center_y) ** 2
                      <= args.limit_radius ** 2)

    # === 連結用領域 = 巡回路候補が通れる領域（緩い clearance）===
    # 連結用 clearance で「ロボットが通れる幅のある領域」を作る。配置用より緩いので
    # 幅 1.2m 未満の通路でも繋がり、部屋同士が分断されない。巡回路（測地距離の最短路）
    # もこの領域上を走る前提。
    connectable = free & limit_mask & (dist_cells >= connect_cells)
    if not connectable.any():
        print('ERROR: connect_clearance を満たす自由空間が無い。'
              '--connect-clearance を下げて。')
        return
    # route 用 clearance は測地距離・edge 最短路の通行領域。屋内の既定は connectable と
    # 同じだが、屋外では inflation に食い込む細い edge を避けるため高めにする。
    route_passable = connectable & (dist_cells >= route_cells_clearance)
    if not route_passable.any():
        print('ERROR: route_clearance を満たす自由空間が無い。'
              '--route-clearance を下げて。')
        return

    # === route 用最大連結成分を巡回対象にする ===
    # 探索で作った地図の「最も広い通行可能空間」をなるべく沢山まわるため、最大の
    # route clearance 連結成分を採る。残った全点は互いに通行可能で測地距離が必ず有限になり、巡回順の
    # 連続点間が壁越しの大ジャンプ（goal_timeout でスキップされる原因）にならない。
    labels, n_lab = ndimage.label(route_passable)
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

    # === 候補ウェイポイント生成（grid / sparse_graph）===
    if args.candidate_mode == 'sparse_graph':
        # SWAGGER-lite: 距離変換の局所最大 + NMS で疎なノードを作る。点数が地図面積でなく
        # 通路・部屋・分岐の複雑さに近づく。node_spacing 既定は spacing を流用。
        node_spacing = (args.graph_node_spacing
                        if args.graph_node_spacing > 0.0 else args.spacing)
        cover_radius = (args.graph_cover_radius
                        if args.graph_cover_radius > 0.0 else 2.0 * node_spacing)
        pts_cell = _sparse_graph_candidates(
            place, dist_cells, res, node_spacing, cover_radius)
        print(f'  candidate-mode=sparse_graph: 局所最大 {len(pts_cell)} 点 '
              f'(node_spacing={node_spacing:.2f}m)')
        if not pts_cell:
            print('  注意: 局所最大が取れなかった。grid にフォールバックします。')
    if args.candidate_mode != 'sparse_graph' or not pts_cell:
        # 既定 grid: 各 spacing セルにつき「壁から最も遠い1点」を代表点に採る
        # （地図全体を spacing 間隔で漏れなくカバー）。
        step = max(1, int(round(args.spacing / res)))
        best_in_cell = {}
        for cx, cy in zip(xs, ys):
            key = (int(cx) // step, int(cy) // step)
            d = dist_cells[cy, cx]
            if key not in best_in_cell or d > best_in_cell[key][2]:
                best_in_cell[key] = (int(cx), int(cy), float(d))
        # NMS: grid 境界付近で隣 grid と近接する WP ペアを統合する。 grid モード単独だと
        # 「各 1.5m 四方に最大 1 点」だが、境界がたまたま壁近くだと隣との距離が 0.1m
        # 程度になることがある（iter1 で確認した 5cm ペア問題の根因の一つ）。
        # 距離変換が大きい (壁から遠い) ものを優先して残し、min_separation 未満の近接候補は
        # 捨てる。 min_separation = spacing * grid_nms_separation_ratio。 0 で無効。
        nms_ratio = float(args.grid_nms_separation_ratio)
        if nms_ratio > 0.0:
            cells = sorted(best_in_cell.values(), key=lambda c: -c[2])
            min_sep_cells2 = (args.spacing * nms_ratio / res) ** 2
            kept = []
            for cx, cy, _d in cells:
                ok = True
                for kx, ky in kept:
                    if (cx - kx) ** 2 + (cy - ky) ** 2 < min_sep_cells2:
                        ok = False
                        break
                if ok:
                    kept.append((cx, cy))
            before = len(best_in_cell)
            pts_cell = kept
            if len(pts_cell) < before:
                print(f'  grid NMS: {before} -> {len(pts_cell)} '
                      f'(min_separation={args.spacing*nms_ratio:.2f}m)')
        else:
            pts_cell = [(c[0], c[1]) for c in best_in_cell.values()]
    pts_cell, added_viewpoints = _append_object_viewpoints(
        pts_cell, occ, connectable, dist_cells, res,
        args.object_viewpoints, args.view_clearance,
        args.object_min_area, args.object_max_area,
        args.viewpoint_min_separation, args.view_map_border_margin)

    raw_count = len(pts_cell)
    pts_cell = _limit_points_by_farthest_sampling(
        pts_cell, args.max_waypoints, (cx0, cy0))
    if raw_count > len(pts_cell):
        print(f'  注意: 候補 {raw_count} > max_waypoints {args.max_waypoints}。'
              f'空間的に散る {len(pts_cell)} 点へ間引きました。'
              f'カバー不足なら --max-waypoints か --spacing を調整してください。')

    # === 測地距離で巡回順を解く（NN + 2-opt）===
    # 連結成分(connectable)上の最短路長を点間距離に使う。直線距離だと「直線では近いが
    # 壁越しで実は遠い」点へ大ジャンプして goal_timeout でスキップされる。測地距離なら
    # 連続点間が必ず通行可能で短く、止まらず一巡（完走）できる。
    if args.edge_clearance_weight > 0.0:
        dm_cells = _clearance_weighted_matrix(
            pts_cell, connectable, dist_cells,
            edge_clearance_cells, args.edge_clearance_weight)
    else:
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

    ordered_cells = [pts_cell[i] for i in order]
    if args.edge_clearance_weight > 0.0:
        shortest_path_func = lambda start, goal: _shortest_path_clearance_weighted(
            connectable, start, goal, dist_cells, edge_clearance_cells,
            args.edge_clearance_weight)
    else:
        shortest_path_func = lambda start, goal: _shortest_path(
            connectable, start, goal)
    # 長い区間を既存点経由で短いジャンプの連続に置き換える（重複を許す）。
    if args.relink_long_jumps > 0.0:
        ordered_cells = _relink_long_jumps(
            ordered_cells, pts_cell, connectable, res,
            args.relink_long_jumps, shortest_path_func)
    route_cells, route_geo, inserted_segments = _split_long_segments(
        ordered_cells, connectable, res, args.max_segment_length,
        waypoint_mask=place, shortest_path_func=shortest_path_func)
    ordered = [cell_to_map(*cell) for cell in route_cells]
    # Dedupe consecutive near-duplicate waypoints (within --dedupe-min-separation [m]).
    # Nav2 controllers' xy_goal_tolerance is typically 0.25m, so points closer than
    # ~0.3m are practically the same goal and just consume goal_timeout per WP.
    # See ros-planning/navigation2#3107 for related "waypoint skipping" issue.
    dedupe_min = float(args.dedupe_min_separation)
    if dedupe_min > 0.0 and len(ordered) > 1:
        before = len(ordered)
        dedup_min2 = dedupe_min ** 2
        deduped = [ordered[0]]
        for (px, py) in ordered[1:]:
            qx, qy = deduped[-1]
            if (px - qx) ** 2 + (py - qy) ** 2 >= dedup_min2:
                deduped.append((px, py))
        ordered = deduped
        if before > len(ordered):
            print(f'  dedupe: {before} -> {len(ordered)} '
                  f'(連続近接 < {dedupe_min:.2f}m を統合)')
    edge_stats = _route_edge_stats(
        route_cells, connectable, dist_cells, res, args.edge_clearance,
        shortest_path_func)

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
    if route_geo:
        geo = route_geo
    else:
        geo = [
            dm_cells[order[i - 1], order[i]] * res
            for i in range(1, len(order))
        ]
    straight = [math.hypot(ordered[i][0] - ordered[i - 1][0],
                           ordered[i][1] - ordered[i - 1][1])
                for i in range(1, len(ordered))]
    print(f'生成: {len(ordered)} waypoints -> {args.out}')
    cov_m2 = int(connectable.sum()) * res * res
    print(f'  地図 {w}x{h} ({round(w*res,1)}x{round(h*res,1)}m) '
          f'clearance={args.clearance}m connect={args.connect_clearance}m '
          f'route={args.route_clearance}m spacing={args.spacing}m '
          f'カバー領域={cov_m2:.0f}m²')
    if args.edge_clearance_weight > 0.0:
        print(f'  edge clearance cost: desired={args.edge_clearance}m '
              f'weight={args.edge_clearance_weight}')
    if args.limit_radius > 0.0:
        print(f'  制限半径={args.limit_radius}m '
              f'center=({args.limit_center_x},{args.limit_center_y})')
    if inserted_segments:
        print(f'  長距離区間分割: max_segment_length='
              f'{args.max_segment_length}m inserted={inserted_segments}')
    if added_viewpoints:
        areas = ','.join(f'{a:.3f}' for _, _, a, _ in added_viewpoints[:8])
        print(f'  認識用 object-viewpoints={len(added_viewpoints)} '
              f'view_clearance={args.view_clearance}m '
              f'border_margin={args.view_map_border_margin}m areas=[{areas}]')
    if geo:
        print(f'  測地経路長={sum(geo):.1f}m 最大測地ジャンプ={max(geo):.1f}m '
              f'(直線最大={max(straight):.1f}m)')
    if edge_stats:
        worst = max(edge_stats, key=lambda r: r['shortfall_integral_m2'])
        print('  edge risk worst='
              f"#{worst['edge']} min_clearance="
              f"{worst['min_clearance_m']:.2f}m "
              f"shortfall={worst['shortfall_integral_m2']:.3f}")
    if args.edge_risk_report:
        _write_edge_risk_report(
            args.edge_risk_report, edge_stats, cell_to_map,
            args.edge_clearance, args.edge_clearance_weight)

    # 地図にウェイポイント・巡回経路を重ねた確認用 PNG を生成（--no-png で抑止）。
    if not args.no_png:
        png_path = os.path.splitext(args.out)[0] + '.png'
        title = (f'{os.path.basename(args.out)}  {len(ordered)}pts  '
                 f'len={sum(geo):.0f}m  maxjump={max(geo):.1f}m'
                 if geo else os.path.basename(args.out))
        try:
            overlay_cells = route_cells if inserted_segments else pts_cell
            overlay_order = (
                list(range(len(route_cells))) if inserted_segments else order)
            save_overlay(png_path, img, place, connectable,
                         overlay_cells, overlay_order,
                         (cx0, cy0), place_cells, title)
            print(f'  オーバーレイ画像 -> {png_path}')
        except Exception as e:  # 画像生成失敗で yaml 出力までは無駄にしない。
            print(f'  注意: オーバーレイ画像の生成に失敗（yaml は出力済み）: {e}')


if __name__ == '__main__':
    main()
