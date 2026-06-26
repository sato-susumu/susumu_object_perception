#!/usr/bin/env python3
"""出来上がった地図(PGM/YAML)を Webots world(wbt)の真の構造と照合する（検証用）。

地図内部の整合（連結成分・壁率）だけでは「地図が実 world と合っているか」は分からない。
このスクリプトは wbt の Floor / Wall / 障害物(PottedTree, SimpleBuilding 等)の
translation・size をパースし、地図 PNG に **真の壁(赤線)・床範囲(青枠)・障害物(緑)** を
重ねて出力する。地図の occ(黒)が赤線に沿い、free(白)が床範囲に収まっていれば world と一致。
ズレ・歪み・星形（実構造を成さない）なら不合格。

使い方:
  python3 check_map_vs_world.py --wbt webots_worlds/indoor.wbt --map outputs/mapping_indoor/indoor.yaml \
    --out /tmp/indoor_check.png

注意: Webots は y-up でなく z-up、地図(map)は SLAM 起点が原点。wbt の world 座標と
map 座標には「ロボット初期位置ぶんのオフセット」がある。ロボット初期 translation を
--robot で渡すと map 原点(=ロボット初期位置)に合わせて wbt 座標を平行移動して重ねる。
"""

import argparse
import csv
import json
import re
from collections import Counter

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from scipy import ndimage


NODE_TYPES = (
    'Floor', 'Wall', 'RectangleArena', 'Pose',
    'PottedTree', 'SimpleBuilding', 'SolidBox',
    'StreetLight', 'PicketFence', 'PicketFenceWithDoor',
    'Cypress', 'PalmTree', 'BigSassafras', 'Sassafras', 'Pine',
    'StoneBench', 'Bench', 'OldBench', 'PublicBin', 'TrashBin',
    'MetallicTrash', 'TrashContainer', 'SwingCouch', 'Pergolas',
    'TrafficCone', 'WoodenPallet', 'WoodenPalletStack', 'OilBarrel',
    'Church', 'ComposedHouse', 'HouseWithGarage', 'ModernSuburbanHouse',
    'SmallManor', 'SuburbanHouse', 'Windmill', 'Warehouse',
)

POINT_OBSTACLES = {
    'PottedTree', 'StreetLight', 'Cypress', 'PalmTree', 'BigSassafras',
    'Sassafras', 'Pine', 'StoneBench', 'Bench', 'OldBench', 'PublicBin',
    'TrashBin', 'MetallicTrash', 'TrashContainer', 'SwingCouch',
    'Pergolas', 'TrafficCone', 'WoodenPallet', 'WoodenPalletStack',
    'OilBarrel', 'Church', 'ComposedHouse', 'HouseWithGarage',
    'ModernSuburbanHouse', 'SmallManor', 'SuburbanHouse', 'Windmill',
    'Warehouse',
}

BUILDING_TYPES = {
    'SimpleBuilding', 'Church', 'ComposedHouse', 'HouseWithGarage',
    'ModernSuburbanHouse', 'SmallManor', 'SuburbanHouse', 'Windmill',
    'Warehouse',
}

CELL_CLASSES = ('occupied', 'free', 'unknown', 'outside')


def load_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()

        def rt():
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
        w = int(rt())
        h = int(rt())
        int(rt())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8)
    return data.reshape(h, w)


def node_body(text, start):
    i = start - 1
    depth = 0
    j = i
    while j < len(text):
        if text[j] == '{':
            depth += 1
        elif text[j] == '}':
            depth -= 1
            if depth == 0:
                break
        j += 1
    return text[i:j]


def parse_floats(block):
    return [float(x) for x in re.findall(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', block)]


def rotation_yaw(body):
    rot = re.search(r'rotation\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', body)
    if not rot:
        return 0.0
    z = float(rot.group(3))
    angle = float(rot.group(4))
    return angle if abs(z) > 0.5 else 0.0


def rotate_xy(x, y, yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)
    return c * x - s * y, s * x + c * y


def transform_points(points, origin, yaw):
    out = []
    ox, oy = origin
    for x, y in points:
        rx, ry = rotate_xy(x, y, yaw)
        out.append((ox + rx, oy + ry))
    return out


def rect_points(center, size, yaw):
    sx, sy = size
    pts = [
        (-sx / 2.0, -sy / 2.0),
        (sx / 2.0, -sy / 2.0),
        (sx / 2.0, sy / 2.0),
        (-sx / 2.0, sy / 2.0),
    ]
    return transform_points(pts, center, yaw)


def centroid(points):
    if not points:
        return None
    arr = np.asarray(points, dtype=np.float64)
    return float(arr[:, 0].mean()), float(arr[:, 1].mean())


def parse_point_list(body, key, dims):
    m = re.search(rf'\b{key}\s*\[([^\]]+)\]', body, re.S)
    if not m:
        return []
    vals = parse_floats(m.group(1))
    return [tuple(vals[i:i + dims][:2])
            for i in range(0, len(vals) - dims + 1, dims)]


def add_obj(objs, typ, xy, size=None, yaw=0.0, points=None,
            sample_points=None, kind='obstacle', name=''):
    objs.append({
        'type': typ,
        'name': name,
        'xy': xy,
        'size': size,
        'yaw': yaw,
        'points': points or [],
        'sample_points': sample_points or [],
        'kind': kind,
    })


def line_samples_local(center, length, yaw=0.0, step=0.25):
    count = max(2, int(np.ceil(length / step)) + 1)
    out = []
    for y in np.linspace(-length / 2.0, length / 2.0, count):
        x, yy = rotate_xy(0.0, y, yaw)
        out.append((center[0] + x, center[1] + yy))
    return out


def box_points_from_proto(xy, yaw, local_center, size, local_yaw=0.0):
    local_poly = rect_points(local_center, size, local_yaw)
    return transform_points(local_poly, xy, yaw)


def box_samples_from_proto(xy, yaw, local_center, size, local_yaw=0.0):
    # Fence boxes are very thin in X and long in Y. Sampling the centerline is
    # closer to the occupied trace than using only corners.
    local_samples = line_samples_local(local_center, size[1], local_yaw)
    return transform_points(local_samples, xy, yaw)


def parse_wbt(path):
    """wbt から Floor / Wall / 障害物の translation, size, rotation を雑にパースする。

    Webots の node は `Type { ... translation x y z ... size sx sy sz ... }` 形式。
    ネストを完全には追わず、既知の world で使う主な PROTO と Pose/Shape の代表形状を拾う。
    village_center のような大きい world では完全な WBT パーサではなく、地図範囲の真値目安を
    重ねるための実用パーサとして使う。
    """
    text = open(path).read()
    objs = []
    type_re = '|'.join(re.escape(t) for t in NODE_TYPES)
    node_re = re.compile(rf'(?:DEF\s+([A-Za-z0-9_]+)\s+)?\b({type_re})\b\s*\{{')
    for m in node_re.finditer(text):
        name = m.group(1) or ''
        typ = m.group(2)
        body = node_body(text, m.end())
        name_field = re.search(r'\bname\s+"([^"]+)"', body)
        if not name and name_field:
            name = name_field.group(1)
        tr = re.search(r'translation\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', body)
        sz = re.search(r'\bsize\s+([-\d.eE+]+)\s+([-\d.eE+]+)(?:\s+([-\d.eE+]+))?', body)
        xy = (float(tr.group(1)), float(tr.group(2))) if tr else (0.0, 0.0)
        size = (float(sz.group(1)), float(sz.group(2))) if sz else None
        yaw = rotation_yaw(body)

        if typ == 'Pose':
            if name == 'DELIMITER' and size:
                add_obj(objs, 'Delimiter', xy, size=size, yaw=yaw,
                        kind='marking', name=name)
                continue
            if name and 'IndexedFaceSet' in body:
                points = parse_point_list(body, 'point', 3)
                if points:
                    add_obj(objs, name, xy, yaw=yaw,
                            points=transform_points(points, xy, yaw),
                            kind='floor', name=name)
                continue
            continue

        if typ in ('Floor', 'RectangleArena') and size:
            add_obj(objs, typ, xy, size=size, yaw=yaw, kind='floor', name=name)
            continue
        if typ == 'Wall' and size:
            add_obj(objs, typ, xy, size=size, yaw=yaw, kind='wall', name=name)
            continue
        if typ == 'SimpleBuilding':
            corners = parse_point_list(body, 'corners', 2)
            add_obj(objs, typ, xy, yaw=yaw,
                    points=transform_points(corners, xy, yaw) if corners else [],
                    kind='building', name=name)
            continue
        if typ in ('PicketFence', 'PicketFenceWithDoor'):
            if typ == 'PicketFence':
                seg = re.search(r'numberOfSegments\s+(\d+)', body)
                segments = int(seg.group(1)) if seg else 1
                # Official boundingObject:
                # Pose translation 0 (-0.85 * numberOfSegments) 0.55
                # Box size 0.04 (1.7 * numberOfSegments) 1.1
                size = (0.04, 1.7 * segments)
                center = (0.0, -0.85 * segments)
                add_obj(objs, typ, xy, size=size, yaw=yaw,
                        points=box_points_from_proto(xy, yaw, center, size),
                        sample_points=box_samples_from_proto(xy, yaw, center, size),
                        kind='fence', name=name)
            else:
                # Official PicketFenceWithDoor boundingObject consists of
                # three thin boxes: two straight spans and one diagonal span.
                boxes = [
                    ((0.0, 0.8700000000000002), (0.04, 1.75), 0.0),
                    ((0.0, 3.689999999999992), (0.04, 1.75), 0.0),
                    ((-0.3975640202570147, 2.0529991572942476),
                     (0.04, 1.13), 0.85),
                ]
                for i, (center, size, local_yaw) in enumerate(boxes):
                    add_obj(
                        objs, typ, xy, size=size, yaw=yaw + local_yaw,
                        points=box_points_from_proto(xy, yaw, center, size,
                                                     local_yaw),
                        sample_points=box_samples_from_proto(
                            xy, yaw, center, size, local_yaw),
                        kind='fence', name=f'{name}:{i}')
            continue
        if typ == 'SolidBox' and size:
            add_obj(objs, typ, xy, size=size, yaw=yaw,
                    kind='obstacle', name=name)
            continue
        if typ in POINT_OBSTACLES:
            kind = 'building' if typ in BUILDING_TYPES else 'obstacle'
            add_obj(objs, typ, xy, yaw=yaw, kind=kind, name=name)
    return objs


def object_sample_points(obj):
    if obj.get('sample_points'):
        return obj['sample_points']
    if obj['points']:
        pts = list(obj['points'])
        c = centroid(pts)
        if c is not None:
            pts.append(c)
        return pts
    if obj['size']:
        pts = rect_points(obj['xy'], obj['size'], obj['yaw'])
        pts.append(obj['xy'])
        return pts
    return [obj['xy']]


def cell_class(value):
    if value is None:
        return 'outside'
    if value < 50:
        return 'occupied'
    if value >= 250:
        return 'free'
    return 'unknown'


def empty_cell_counts():
    return {cls: 0 for cls in CELL_CLASSES}


def summarize_alignment(objs, img_display, world_to_map_px, res, threshold_m):
    occ = img_display < 50
    dist = ndimage.distance_transform_edt(~occ) * res
    h, w = occ.shape
    rows = []
    by_kind = {}
    objects = []

    def bucket(kind):
        if kind not in by_kind:
            by_kind[kind] = {
                'objects': 0,
                'objects_inside': 0,
                'objects_near_any': 0,
                'samples': 0,
                'inside': 0,
                'near_occupied': 0,
                'distances_m': [],
                'object_coverages': [],
                'cell_counts': empty_cell_counts(),
            }
        return by_kind[kind]

    for obj_idx, obj in enumerate(objs):
        if obj['kind'] in ('floor', 'marking'):
            continue
        b = bucket(obj['kind'])
        b['objects'] += 1
        obj_samples = object_sample_points(obj)
        obj_inside = 0
        obj_near = 0
        obj_distances = []
        obj_cell_counts = empty_cell_counts()
        for wx, wy in obj_samples:
            px, py = world_to_map_px(wx, wy)
            ix = int(round(px))
            iy = int(round(py))
            b['samples'] += 1
            inside = 0 <= ix < w and 0 <= iy < h
            value = int(img_display[iy, ix]) if inside else None
            cls = cell_class(value)
            b['cell_counts'][cls] += 1
            obj_cell_counts[cls] += 1
            row = {
                'object_index': obj_idx,
                'object_name': obj.get('name', ''),
                'type': obj['type'],
                'kind': obj['kind'],
                'world_xy': [float(wx), float(wy)],
                'pixel_xy': [float(px), float(py)],
                'inside_map': inside,
                'cell_value': value,
                'cell_class': cls,
                'distance_to_occupied_m': None,
            }
            if inside:
                d = float(dist[iy, ix])
                row['distance_to_occupied_m'] = d
                b['inside'] += 1
                obj_inside += 1
                b['distances_m'].append(d)
                obj_distances.append(d)
                if d <= threshold_m:
                    b['near_occupied'] += 1
                    obj_near += 1
            rows.append(row)
        if obj_inside:
            b['objects_inside'] += 1
            coverage = float(obj_near / obj_inside)
            b['object_coverages'].append(coverage)
            if obj_near:
                b['objects_near_any'] += 1
        else:
            coverage = None
        objects.append({
            'object_index': obj_idx,
            'object_name': obj.get('name', ''),
            'type': obj['type'],
            'kind': obj['kind'],
            'samples': len(obj_samples),
            'inside': obj_inside,
            'near_occupied': obj_near,
            'coverage_inside': coverage,
            'cell_counts': obj_cell_counts,
            'mean_distance_to_occupied_m': (
                None if not obj_distances else float(np.mean(obj_distances))),
            'max_distance_to_occupied_m': (
                None if not obj_distances else float(np.max(obj_distances))),
        })

    summary = {
        'threshold_m': threshold_m,
        'samples': len(rows),
        'inside': sum(v['inside'] for v in by_kind.values()),
        'near_occupied': sum(v['near_occupied'] for v in by_kind.values()),
        'by_kind': {},
    }
    for kind, value in sorted(by_kind.items()):
        distances = np.asarray(value['distances_m'], dtype=np.float64)
        coverages = np.asarray(value['object_coverages'], dtype=np.float64)
        stats = {
            'objects': value['objects'],
            'objects_inside': value['objects_inside'],
            'objects_near_any': value['objects_near_any'],
            'object_near_any_ratio_inside': (
                None if value['objects_inside'] == 0
                else float(value['objects_near_any'] / value['objects_inside'])),
            'mean_object_coverage_inside': (
                None if len(coverages) == 0 else float(coverages.mean())),
            'samples': value['samples'],
            'inside': value['inside'],
            'near_occupied': value['near_occupied'],
            'cell_counts': value['cell_counts'],
            'near_ratio_inside': (
                None if value['inside'] == 0
                else float(value['near_occupied'] / value['inside'])),
            'mean_distance_to_occupied_m': (
                None if len(distances) == 0 else float(distances.mean())),
            'max_distance_to_occupied_m': (
                None if len(distances) == 0 else float(distances.max())),
        }
        summary['by_kind'][kind] = stats
    summary['near_ratio_inside'] = (
        None if summary['inside'] == 0
        else float(summary['near_occupied'] / summary['inside']))
    return {'summary': summary, 'objects': objects, 'samples': rows}


def write_object_csv(report, path):
    objects = sorted(
        report['objects'],
        key=lambda obj: (
            obj['kind'],
            2.0 if obj['coverage_inside'] is None else obj['coverage_inside'],
            -obj['inside'],
            obj['object_index'],
        ))
    fields = [
        'object_index', 'object_name', 'type', 'kind', 'samples', 'inside',
        'near_occupied', 'coverage_inside', 'mean_distance_to_occupied_m',
        'max_distance_to_occupied_m', 'occupied_samples', 'free_samples',
        'unknown_samples', 'outside_samples',
    ]
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for obj in objects:
            counts = obj['cell_counts']
            writer.writerow({
                'object_index': obj['object_index'],
                'object_name': obj.get('object_name', ''),
                'type': obj['type'],
                'kind': obj['kind'],
                'samples': obj['samples'],
                'inside': obj['inside'],
                'near_occupied': obj['near_occupied'],
                'coverage_inside': obj['coverage_inside'],
                'mean_distance_to_occupied_m': obj['mean_distance_to_occupied_m'],
                'max_distance_to_occupied_m': obj['max_distance_to_occupied_m'],
                'occupied_samples': counts['occupied'],
                'free_samples': counts['free'],
                'unknown_samples': counts['unknown'],
                'outside_samples': counts['outside'],
            })


def worst_objects(report, kind='', limit=8):
    objects = [
        obj for obj in report['objects']
        if obj['inside'] > 0 and obj['coverage_inside'] is not None
    ]
    if kind:
        objects = [obj for obj in objects if obj['kind'] == kind]
    objects.sort(key=lambda obj: (
        obj['coverage_inside'],
        obj['cell_counts']['free'],
        -obj['max_distance_to_occupied_m'],
        obj['object_index'],
    ))
    return objects[:limit]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wbt', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    # ロボット初期位置(world座標)。map 原点(=起点)に合わせるため wbt 座標から引く。
    ap.add_argument('--robot', nargs=2, type=float, default=None,
                    help='ロボット初期 translation x y（省略時 wbt から TurtleBot を探す）')
    ap.add_argument('--show-all-world', action='store_true',
                    help='地図範囲外の wbt オブジェクトも含めて全体表示する')
    ap.add_argument('--margin-m', type=float, default=1.0,
                    help='通常表示で地図範囲の外に足す余白[m]')
    ap.add_argument('--report', default='',
                    help='wbt sample と occupied の距離評価 JSON 出力先')
    ap.add_argument('--object-report', default='',
                    help='object 単位の coverage / cell class CSV 出力先')
    ap.add_argument('--worst-kind', default='fence',
                    help='低 coverage object を標準出力へ出す kind（空文字で全 kind）')
    ap.add_argument('--worst-limit', type=int, default=8,
                    help='標準出力へ出す低 coverage object の最大数')
    ap.add_argument('--occupied-distance-threshold-m', type=float, default=0.5,
                    help='wbt sample が occupied に近いとみなす距離[m]')
    args = ap.parse_args()

    meta = yaml.safe_load(open(args.map))
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    import os
    img = load_pgm(os.path.join(os.path.dirname(args.map), meta['image']))
    h, w = img.shape

    objs = parse_wbt(args.wbt)
    # ロボット初期位置（world→map のオフセット）。
    if args.robot:
        rxw, ryw = args.robot
    else:
        txt = open(args.wbt).read()
        m = re.search(r'TurtleBot\w*\s*\{[^}]*?translation\s+([-\d.]+)\s+([-\d.]+)', txt)
        rxw, ryw = (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)
    print(f'robot world pos = ({rxw}, {ryw}) を map 原点に合わせる')

    def world_to_map_px(wx, wy):
        # world → map座標(ロボット初期位置を原点に) → ピクセル
        mx = wx - rxw
        my = wy - ryw
        px = (mx - ox) / res
        py = (my - oy) / res  # PGM は上下反転だが imshow origin=lower で合わせる
        return px, py

    # 自動拡大 (iter41): 固定スケール `figsize=(w/40, h/40)` だと indoor (200x100 cell)
    # で 5x2.5 inch しか無く、 ラベルや真値境界が小さくなる。 render_recognition_overlay.py
    # 同パターンで小さい地図でも読めるサイズへ拡大。 ユーザー指示「小さければ自動拡大」。
    scale = max(1.0, 520.0 / max(w, h))
    fig_w = max(7.0, (w * scale) / 80.0)
    fig_h = max(7.0, (h * scale) / 80.0)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    # PGM(map_server規約)は img[0] が画像最上行＝map y 最大。world点の py は origin からの
    # 上方向距離 (my-oy)/res なので、PGM をそのまま origin='lower' で出すと上下が反転する
    # （break_room で 180度ズレて見えた原因）。img を上下反転して y 向きを world と一致させる。
    img_display = img[::-1]
    ax.imshow(img_display, cmap='gray', vmin=0, vmax=255, origin='lower')
    def draw_poly(points, edgecolor, facecolor='none', lw=1.5, alpha=1.0):
        pix = [world_to_map_px(x, y) for x, y in points]
        ax.add_patch(Polygon(pix, closed=True, fill=facecolor != 'none',
                             edgecolor=edgecolor, facecolor=facecolor,
                             lw=lw, alpha=alpha))

    for obj in objs:
        typ = obj['type']
        tx, ty = obj['xy']
        s = obj['size']
        yaw = obj['yaw']
        px, py = world_to_map_px(tx, ty)
        if obj['points']:
            if obj['kind'] == 'floor':
                draw_poly(obj['points'], edgecolor='blue', facecolor='none', lw=2)
            elif obj['kind'] == 'building':
                draw_poly(obj['points'], edgecolor='red', facecolor='none', lw=2)
            elif obj['kind'] == 'fence':
                draw_poly(obj['points'], edgecolor='orange', facecolor='none', lw=1.5)
            else:
                draw_poly(obj['points'], edgecolor='green', facecolor='none', lw=1.5)
        elif obj['kind'] == 'floor' and s:
            wpx, hpx = s[0] / res, s[1] / res
            # Floor の rotation(z軸 90deg)を反映。size はローカル座標なので、90度回転
            # した床は world 上で幅高さが入れ替わる（break_room は Floor が 90度回転して
            # おり、これを無視すると照合が 90度ズレて見えた）。
            if abs(abs(yaw) - 1.5708) < 0.3:
                wpx, hpx = hpx, wpx
            ax.add_patch(Rectangle((px - wpx / 2, py - hpx / 2), wpx, hpx,
                         fill=False, edgecolor='blue', lw=2, label='floor'))
        elif obj['kind'] in ('wall', 'building') and s:
            draw_poly(rect_points((tx, ty), s, yaw), edgecolor='red', lw=2)
        elif obj['kind'] == 'marking' and s:
            draw_poly(rect_points((tx, ty), s, yaw), edgecolor='cyan', lw=1.5)
        elif obj['kind'] == 'fence' and s:
            draw_poly(rect_points((tx, ty), s, yaw), edgecolor='orange', lw=1.5)
        elif s:
            draw_poly(rect_points((tx, ty), s, yaw), edgecolor='green', lw=1.5)
        else:
            marker = 'rx' if obj['kind'] == 'building' else 'g+'
            ax.plot(px, py, marker, ms=8, mew=1.8)

    if not args.show_all_world:
        margin_px = args.margin_m / res
        ax.set_xlim(-margin_px, w + margin_px)
        ax.set_ylim(-margin_px, h + margin_px)
    ax.set_title(f'{os.path.basename(args.map)} vs {os.path.basename(args.wbt)}\n'
                 'red=wall/building, blue=floor/ground, orange=fence, '
                 'green=object, cyan=low marking')
    fig.savefig(args.out, dpi=80, bbox_inches='tight')
    print(f'saved {args.out}')
    report = summarize_alignment(
        objs, img_display, world_to_map_px, res,
        args.occupied_distance_threshold_m)
    summary = report['summary']
    print(
        'alignment: samples={samples} inside={inside} '
        'near_occupied={near_occupied} near_ratio_inside={ratio}'.format(
            samples=summary['samples'],
            inside=summary['inside'],
            near_occupied=summary['near_occupied'],
            ratio=(
                'n/a' if summary['near_ratio_inside'] is None
                else f'{summary["near_ratio_inside"]:.3f}')))
    for kind, stats in summary['by_kind'].items():
        mean_d = stats['mean_distance_to_occupied_m']
        max_d = stats['max_distance_to_occupied_m']
        print(
            f'  {kind}: objects_inside={stats["objects_inside"]}/'
            f'{stats["objects"]} objects_near_any='
            f'{stats["objects_near_any"]} '
            f'mean_object_coverage={stats["mean_object_coverage_inside"]} '
            f'inside={stats["inside"]}/{stats["samples"]} '
            f'near={stats["near_occupied"]} '
            f'cells={stats["cell_counts"]} '
            f'ratio={stats["near_ratio_inside"]} '
            f'mean_dist={mean_d} max_dist={max_d}')
    if args.worst_limit > 0:
        print('worst objects:')
        for obj in worst_objects(report, args.worst_kind, args.worst_limit):
            counts = obj['cell_counts']
            print(
                f'  #{obj["object_index"]} {obj["type"]} '
                f'{obj.get("object_name", "")} kind={obj["kind"]} '
                f'coverage={obj["coverage_inside"]:.3f} '
                f'inside={obj["inside"]} near={obj["near_occupied"]} '
                f'cells={counts} '
                f'max_dist={obj["max_distance_to_occupied_m"]}')
    if args.report:
        with open(args.report, 'w') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
            f.write('\n')
        print(f'wrote {args.report}')
    if args.object_report:
        write_object_csv(report, args.object_report)
        print(f'wrote {args.object_report}')
    # 寸法サマリ
    kinds = Counter(o['kind'] for o in objs)
    types = Counter(o['type'] for o in objs)
    print('wbt parsed: ' + ', '.join(
        f'{kind}={count}' for kind, count in sorted(kinds.items())))
    print('wbt types: ' + ', '.join(
        f'{typ}={count}' for typ, count in sorted(types.items())))
    for obj in objs:
        if obj['kind'] == 'floor':
            t = obj['xy']
            print(f'  Floor/ground {obj["type"]} at world{t} '
                  f'(map原点基準 {t[0]-rxw:.1f},{t[1]-ryw:.1f})')
    print(f'地図実寸 {w*res:.1f}x{h*res:.1f}m')


if __name__ == '__main__':
    main()
