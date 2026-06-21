#!/usr/bin/env python3
"""Filter object_memory SQLite results before final recognition artifacts.

This is part of the recognition task pipeline, not a world-comparison
postprocessor. It uses only the saved map and recognition DB to remove static
object candidates that do not have enough map support or plausible class
geometry.
"""

import argparse
import math
import os
import shutil
import sqlite3

import numpy as np
import yaml


MEMORY_CLASS_NORMALIZATION = {
    'vase': 'potted plant',
    'person': 'pedestrian',
    'sofa': 'couch',
    'table': 'dining table',
    'fridge': 'refrigerator',
    'umbrella': 'potted plant',
}

SEMANTIC_CLASS_KEYS = {
    'potted plant': 'plant',
    'vase': 'plant',
    'couch': 'couch',
    'sofa': 'couch',
    'dining table': 'table',
    'table': 'table',
    'refrigerator': 'refrigerator',
    'fridge': 'refrigerator',
    'pedestrian': 'pedestrian',
    'person': 'pedestrian',
}

STATIC_CLASS_GEOMETRY_RULES = {
    # semantic class -> (min planar area [m^2], max planar area [m^2], max aspect)
    'plant': (0.04, 0.65, 3.0),
    'couch': (0.45, 3.0, 3.0),
    'table': (0.25, 3.5, 5.0),
    'chair': (0.12, 0.6, 4.0),
    'refrigerator': (0.15, 2.5, 5.0),
}


def normalize_class_name(name):
    name = str(name or 'unknown').strip().lower().replace('_', ' ')
    while '  ' in name:
        name = name.replace('  ', ' ')
    return MEMORY_CLASS_NORMALIZATION.get(name, name)


def semantic_class_key(name):
    name = normalize_class_name(name)
    return SEMANTIC_CLASS_KEYS.get(name, name)


def parse_class_groups(values):
    groups = []
    for item in values or []:
        parts = str(item).replace('|', ',').split(',')
        keys = {semantic_class_key(p) for p in parts if p.strip()}
        keys.discard('unknown')
        if len(keys) >= 2:
            groups.append(keys)
    return groups


def parse_class_priority(value):
    return [normalize_class_name(v) for v in str(value or '').split(',')
            if v.strip()]


def load_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()

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
        _ = int(read_token())
        if magic == b'P5':
            data = np.frombuffer(f.read(w * h), dtype=np.uint8)
        else:
            vals = f.read().split()
            data = np.array([int(v) for v in vals[:w * h]], dtype=np.uint8)
    return data.reshape(h, w)


def occupied_world_points(map_yaml):
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(map_yaml), meta['image'])
    img = load_pgm(pgm_path)
    negate = int(meta.get('negate', 0))
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    if negate:
        occ_prob = img.astype(np.float32) / 255.0
    else:
        occ_prob = (255.0 - img.astype(np.float32)) / 255.0
    occ = occ_prob >= occ_thresh
    ys, xs = np.nonzero(occ)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h, _ = occ.shape
    return np.column_stack((
        ox + (xs + 0.5) * res,
        oy + (h - ys - 0.5) * res,
    ))


def nearest_occ_dist(occ_xy, x, y):
    if occ_xy.size == 0:
        return float('inf')
    d = np.hypot(occ_xy[:, 0] - x, occ_xy[:, 1] - y)
    return float(d.min())


def passes_geometry(class_name, sx, sy):
    rule = STATIC_CLASS_GEOMETRY_RULES.get(semantic_class_key(class_name))
    if rule is None:
        return False
    sx = abs(float(sx))
    sy = abs(float(sy))
    if sx <= 1e-3 or sy <= 1e-3:
        return False
    area = sx * sy
    aspect = max(sx, sy) / max(1e-3, min(sx, sy))
    min_area, max_area, max_aspect = rule
    return min_area <= area <= max_area and aspect <= max_aspect


def combined_existence(pa, pb):
    pa = min(0.999, max(0.001, float(pa)))
    pb = min(0.999, max(0.001, float(pb)))
    return min(0.999, max(pa, pb, 1.0 - (1.0 - pa) * (1.0 - pb)))


def class_priority_rank(class_name, priority):
    name = normalize_class_name(class_name)
    if name in priority:
        return priority.index(name)
    key = semantic_class_key(name)
    for i, item in enumerate(priority):
        if semantic_class_key(item) == key:
            return i
    return len(priority)


def compatible_merge_limit(class_a, class_b, args, groups):
    key_a = semantic_class_key(class_a)
    key_b = semantic_class_key(class_b)
    if key_a == 'unknown' or key_b == 'unknown':
        return 0.0
    if key_a == key_b:
        return args.merge_same_class_dist
    if args.merge_compatible_dist <= 0.0:
        return 0.0
    for group in groups:
        if key_a in group and key_b in group:
            return args.merge_compatible_dist
    return 0.0


def load_rows(con):
    cols = [
        'id', 'label', 'class_name', 'x', 'y', 'z',
        'size_x', 'size_y', 'size_z', 'existence', 'hits', 'last_seen',
    ]
    return con.execute(
        'SELECT %s FROM objects' % ', '.join(cols)).fetchall()


def merge_object_component(con, rows, priority):
    rows = sorted(
        rows, key=lambda r: (int(r['hits']), float(r['existence'])),
        reverse=True)
    base = rows[0]
    total_hits = max(1, sum(max(1, int(r['hits'])) for r in rows))
    weights = [max(1, int(r['hits'])) / float(total_hits) for r in rows]
    class_name, label = choose_merged_component_class(rows, priority)
    existence = 0.001
    for r in rows:
        existence = combined_existence(existence, r['existence'])
    con.execute(
        'UPDATE objects SET x=?, y=?, z=?, size_x=?, size_y=?, size_z=?, '
        'label=?, class_name=?, existence=?, hits=?, last_seen=? WHERE id=?',
        (sum(w * float(r['x']) for w, r in zip(weights, rows)),
         sum(w * float(r['y']) for w, r in zip(weights, rows)),
         sum(w * float(r['z']) for w, r in zip(weights, rows)),
         sum(w * float(r['size_x']) for w, r in zip(weights, rows)),
         sum(w * float(r['size_y']) for w, r in zip(weights, rows)),
         sum(w * float(r['size_z']) for w, r in zip(weights, rows)),
         int(label),
         class_name,
         existence,
         total_hits,
         max(float(r['last_seen']) for r in rows),
         int(base['id'])))
    removed = []
    for r in rows[1:]:
        con.execute('DELETE FROM objects WHERE id=?', (int(r['id']),))
        removed.append(int(r['id']))
    return int(base['id']), removed, class_name


def choose_merged_component_class(rows, priority):
    known = [r for r in rows if r['class_name'] != 'unknown']
    if not known:
        return 'unknown', 0

    def key(o):
        rank = class_priority_rank(o['class_name'], priority)
        return (int(o['hits']), float(o['existence']), -rank)

    chosen = max(known, key=key)
    return chosen['class_name'], chosen['label']


def merge_components(rows, args, groups):
    n = len(rows)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri = find(i)
        rj = find(j)
        if ri != rj:
            parent[rj] = ri

    for i, a in enumerate(rows):
        for j in range(i + 1, n):
            b = rows[j]
            limit = compatible_merge_limit(
                a['class_name'], b['class_name'], args, groups)
            if limit <= 0.0:
                continue
            d = math.hypot(float(a['x']) - float(b['x']),
                           float(a['y']) - float(b['y']))
            if d < limit:
                union(i, j)

    comps = {}
    for i, r in enumerate(rows):
        comps.setdefault(find(i), []).append(r)
    return [c for c in comps.values() if len(c) >= 2]


def merge_compatible_objects(con, args):
    groups = parse_class_groups(args.merge_compatible_group)
    priority = parse_class_priority(args.merge_class_priority)
    merged = []
    while True:
        rows = load_rows(con)
        comps = merge_components(rows, args, groups)
        if not comps:
            return merged
        comps.sort(key=len, reverse=True)
        kept_id, removed_ids, class_name = merge_object_component(
            con, comps[0], priority)
        merged.append((kept_id, removed_ids, class_name))


def filter_db(args):
    if os.path.abspath(args.db) != os.path.abspath(args.out_db):
        shutil.copyfile(args.db, args.out_db)
    occ_xy = occupied_world_points(args.map) if args.map_support_dist >= 0.0 else None
    con = sqlite3.connect(args.out_db)
    con.row_factory = sqlite3.Row
    rows = con.execute(
        'SELECT id, class_name, x, y, size_x, size_y FROM objects').fetchall()
    deleted = []
    for r in rows:
        reasons = []
        if args.static_class_geometry_filter and not passes_geometry(
                r['class_name'], r['size_x'], r['size_y']):
            reasons.append('geometry')
        if occ_xy is not None:
            d = nearest_occ_dist(occ_xy, float(r['x']), float(r['y']))
            if d > args.map_support_dist:
                reasons.append(f'map_support:{d:.3f}m')
        if reasons:
            con.execute('DELETE FROM objects WHERE id=?', (int(r['id']),))
            deleted.append((int(r['id']), str(r['class_name']), ','.join(reasons)))
    merged = merge_compatible_objects(con, args)
    con.commit()
    con.close()
    print(f'filtered {args.db} -> {args.out_db}; deleted={len(deleted)}')
    for oid, cls, reason in deleted:
        print(f'  delete #{oid} {cls}: {reason}')
    print(f'merged={len(merged)}')
    for kept_id, removed_ids, cls in merged:
        removed = ','.join(f'#{oid}' for oid in removed_ids)
        print(f'  merge {removed} -> #{kept_id} as {cls}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', required=True)
    ap.add_argument('--out-db', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--map-support-dist', type=float, default=-1.0)
    ap.add_argument('--static-class-geometry-filter', action='store_true')
    ap.add_argument(
        '--merge-same-class-dist', type=float, default=0.0,
        help='merge same semantic class objects within this distance [m]')
    ap.add_argument(
        '--merge-compatible-dist', type=float, default=0.0,
        help='merge compatible but different semantic classes within this distance [m]')
    ap.add_argument(
        '--merge-compatible-group', action='append', default=[],
        help='comma-separated compatible classes, e.g. chair,couch,dining table')
    ap.add_argument(
        '--merge-class-priority', default='',
        help='comma-separated class priority used when merged evidence ties')
    args = ap.parse_args()
    filter_db(args)


if __name__ == '__main__':
    main()
