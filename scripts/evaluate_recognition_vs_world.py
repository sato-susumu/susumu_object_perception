#!/usr/bin/env python3
"""Compare semantic recognition results with static objects in a Webots world.

The detection input is the SQLite database written by object_memory_node.py.
The ground truth input is a Webots .wbt file. This script parses recognizable
static object nodes, converts their world coordinates into the map frame by
subtracting the robot's initial Webots translation, and matches them against
object memory detections by class label and 2D distance.

Outputs:
  <out-prefix>.md    human-readable report
  <out-prefix>.json  machine-readable report
  <out-prefix>.csv   flat row table
  <out-prefix>.png   map overlay, if --map is provided
"""

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
import os
import re
import sqlite3

import numpy as np
import yaml


# Webots PROTO type -> acceptable object_memory class_name values.
# These are COCO-oriented names because object_classifier_node.py publishes
# COCO fine classes through /perception/object_fine_classes.
SUPPORTED_WORLD_TYPES = {
    'PottedTree': ['potted plant', 'vase'],
    'BunchOfSunFlowers': ['potted plant', 'vase'],
    'Sofa': ['couch', 'sofa'],
    'Armchair': ['chair'],
    'Table': ['dining table', 'table'],
    'Fridge': ['refrigerator', 'fridge'],
    'Pedestrian': ['person', 'pedestrian'],
}

UNSUPPORTED_WORLD_TYPES = {
    'Cabinet': 'default COCO YOLO weights have no cabinet class',
    'CardboardBox': 'default COCO YOLO weights have no cardboard box class',
    'DirectionPanel': 'not evaluated by object_memory static object DB',
    'Door': 'structural element, not an object recognition target',
    'Floor': 'structural element',
    'FloorLight': 'default COCO YOLO weights have no floor light/lamp class',
    'LandscapePainting': 'default COCO YOLO weights have no painting class',
    'Radiator': 'default COCO YOLO weights have no radiator class',
    'RectangleArena': 'structural element',
    'Wall': 'structural element',
    'Window': 'structural element',
}


@dataclass
class ExpectedObject:
    eid: str
    wbt_type: str
    name: str
    world_x: float
    world_y: float
    map_x: float
    map_y: float
    accepted_labels: list


@dataclass
class Detection:
    did: int
    class_name: str
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float
    existence: float
    hits: int
    last_seen: float


def normalized_label(label):
    label = str(label or '').lower().strip()
    label = label.replace('_', ' ').replace('-', ' ')
    return re.sub(r'\s+', ' ', label)


def labels_match(detection_label, accepted_labels):
    det = normalized_label(detection_label)
    return det in {normalized_label(v) for v in accepted_labels}


def read_balanced_body(text, open_brace_index):
    depth = 0
    for i in range(open_brace_index, len(text)):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return text[open_brace_index:i + 1]
    return text[open_brace_index:]


def parse_vec3(body, key):
    m = re.search(
        rf'\b{re.escape(key)}\s+'
        r'([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)',
        body,
    )
    if not m:
        return None
    return tuple(float(m.group(i)) for i in range(1, 4))


def parse_name(body, fallback):
    m = re.search(r'\bname\s+"([^"]+)"', body)
    return m.group(1) if m else fallback


def parse_robot_xy(wbt_path, robot_xy):
    if robot_xy is not None:
        return float(robot_xy[0]), float(robot_xy[1])
    text = open(wbt_path, encoding='utf-8').read()
    m = re.search(
        r'\bTurtleBot\w*\s*\{.*?\btranslation\s+'
        r'([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)',
        text,
        re.DOTALL,
    )
    if not m:
        return 0.0, 0.0
    return float(m.group(1)), float(m.group(2))


def parse_world_objects(wbt_path, robot_xy, extra_targets, ignored_types=None):
    text = open(wbt_path, encoding='utf-8').read()
    rx, ry = parse_robot_xy(wbt_path, robot_xy)
    ignored_types = set(ignored_types or [])
    target_types = dict(SUPPORTED_WORLD_TYPES)
    for typ, labels in extra_targets.items():
        target_types[typ] = labels
    known_types = set(target_types) | set(UNSUPPORTED_WORLD_TYPES) | ignored_types

    expected = []
    skipped = []
    counts = {}
    node_re = re.compile(r'\b([A-Z][A-Za-z0-9_]*)\s*\{')
    for m in node_re.finditer(text):
        typ = m.group(1)
        if typ not in known_types:
            continue
        body = read_balanced_body(text, m.end() - 1)
        tr = parse_vec3(body, 'translation')
        name = parse_name(body, typ)
        if tr is None:
            skipped.append({
                'wbt_type': typ,
                'name': name,
                'reason': 'no translation',
            })
            continue
        if typ in ignored_types:
            skipped.append({
                'wbt_type': typ,
                'name': name,
                'world_x': tr[0],
                'world_y': tr[1],
                'reason': 'ignored by --ignore-type',
            })
            continue
        if typ not in target_types:
            skipped.append({
                'wbt_type': typ,
                'name': name,
                'world_x': tr[0],
                'world_y': tr[1],
                'reason': UNSUPPORTED_WORLD_TYPES.get(typ, 'unsupported'),
            })
            continue
        counts[typ] = counts.get(typ, 0) + 1
        eid = f'{typ}[{counts[typ]}]'
        expected.append(ExpectedObject(
            eid=eid,
            wbt_type=typ,
            name=name,
            world_x=tr[0],
            world_y=tr[1],
            map_x=tr[0] - rx,
            map_y=tr[1] - ry,
            accepted_labels=target_types[typ],
        ))
    return expected, skipped, (rx, ry)


def load_detections(db_path, min_existence, min_hits):
    if not os.path.exists(db_path):
        raise FileNotFoundError(db_path)
    con = sqlite3.connect(db_path)
    con.row_factory = sqlite3.Row
    try:
        rows = con.execute(
            """
            SELECT id, class_name, x, y, z,
                   size_x, size_y, size_z, existence, hits, last_seen
            FROM objects
            WHERE existence >= ? AND hits >= ?
            ORDER BY id
            """,
            (min_existence, min_hits),
        ).fetchall()
    finally:
        con.close()
    return [
        Detection(
            did=int(r['id']),
            class_name=str(r['class_name'] or 'unknown'),
            x=float(r['x']),
            y=float(r['y']),
            z=float(r['z'] or 0.0),
            size_x=float(r['size_x'] or 0.0),
            size_y=float(r['size_y'] or 0.0),
            size_z=float(r['size_z'] or 0.0),
            existence=float(r['existence']),
            hits=int(r['hits']),
            last_seen=float(r['last_seen'] or 0.0),
        )
        for r in rows
    ]


def distance(expected, detection):
    return math.hypot(expected.map_x - detection.x,
                      expected.map_y - detection.y)


def evaluate(expected, detections, match_distance):
    correct_candidates = []
    for ei, exp in enumerate(expected):
        for di, det in enumerate(detections):
            dist = distance(exp, det)
            if dist <= match_distance and labels_match(
                    det.class_name, exp.accepted_labels):
                correct_candidates.append((dist, ei, di))

    matched_expected = set()
    matched_detections = set()
    correct = []
    for dist, ei, di in sorted(correct_candidates):
        if ei in matched_expected or di in matched_detections:
            continue
        matched_expected.add(ei)
        matched_detections.add(di)
        correct.append({
            'expected': asdict(expected[ei]),
            'detection': asdict(detections[di]),
            'distance_m': dist,
        })

    wrong_candidates = []
    for ei, exp in enumerate(expected):
        if ei in matched_expected:
            continue
        for di, det in enumerate(detections):
            if di in matched_detections:
                continue
            dist = distance(exp, det)
            if dist <= match_distance:
                wrong_candidates.append((dist, ei, di))

    wrong_label = []
    for dist, ei, di in sorted(wrong_candidates):
        if ei in matched_expected or di in matched_detections:
            continue
        matched_expected.add(ei)
        matched_detections.add(di)
        wrong_label.append({
            'expected': asdict(expected[ei]),
            'detection': asdict(detections[di]),
            'distance_m': dist,
        })

    missed = [
        asdict(exp) for i, exp in enumerate(expected)
        if i not in matched_expected
    ]
    extra = [
        asdict(det) for i, det in enumerate(detections)
        if i not in matched_detections
    ]

    total_expected = len(expected)
    total_detections = len(detections)
    tp = len(correct)
    class_aware_fp = max(0, total_detections - tp)
    class_aware_fn = max(0, total_expected - tp)
    precision = tp / total_detections if total_detections else 0.0
    recall = tp / total_expected if total_expected else 0.0
    f1 = (2.0 * precision * recall / (precision + recall)
          if (precision + recall) > 0.0 else 0.0)
    return {
        'summary': {
            'expected_count': total_expected,
            'detection_count': total_detections,
            'correct_count': tp,
            'wrong_label_count': len(wrong_label),
            'missed_without_near_detection_count': len(missed),
            'extra_detection_count': len(extra),
            'class_aware_false_positive_count': class_aware_fp,
            'class_aware_false_negative_count': class_aware_fn,
            'precision': precision,
            'recall': recall,
            'f1': f1,
            'match_distance_m': match_distance,
        },
        'correct': correct,
        'wrong_label': wrong_label,
        'missed': missed,
        'extra': extra,
    }


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

        width = int(read_token())
        height = int(read_token())
        int(read_token())
        if magic == b'P5':
            data = np.frombuffer(f.read(width * height), dtype=np.uint8)
        else:
            data = np.array(
                [int(v) for v in f.read().split()[:width * height]],
                dtype=np.uint8,
            )
    return data.reshape(height, width)


def map_to_cell(x, y, meta, width, height):
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    cx = (x - ox) / res - 0.5
    cy = height - 1 - ((y - oy) / res - 0.5)
    return cx, cy


def render_png(map_yaml, out_png, result, scale):
    if not map_yaml:
        return
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(map_yaml), meta['image'])
    img = load_pgm(pgm_path)
    height, width = img.shape

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if scale <= 0.0:
        scale = max(1.0, 560.0 / max(width, height))
    fig_w = max(7.5, (width * scale) / 80.0)
    fig_h = max(7.5, (height * scale) / 80.0)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=140)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper')
    summary = result['summary']
    ax.set_title(
        f"recognition vs world: TP={summary['correct_count']} "
        f"wrong={summary['wrong_label_count']} extra={summary['extra_detection_count']} "
        f"FN={summary['class_aware_false_negative_count']} "
        f"no_det={summary['missed_without_near_detection_count']}",
        fontsize=10,
    )

    def plot_expected(exp, marker, color, text, rank):
        cx, cy = map_to_cell(exp['map_x'], exp['map_y'],
                             meta, width, height)
        ax.plot(cx, cy, marker, color=color, markersize=8,
                markeredgewidth=1.5, zorder=4)
        ax.annotate(
            text,
            (cx, cy),
            xytext=(6 + (rank % 4) * 8, 6 + ((rank // 4) % 3) * 8),
            textcoords='offset points',
            fontsize=7,
            color='black',
            bbox={
                'boxstyle': 'round,pad=0.20',
                'facecolor': 'white',
                'edgecolor': color,
                'alpha': 0.88,
            },
            arrowprops={'arrowstyle': '-', 'color': color, 'lw': 0.8},
            zorder=5,
        )
        return cx, cy

    def plot_detection(det, marker, color, text, rank):
        cx, cy = map_to_cell(det['x'], det['y'], meta, width, height)
        ax.plot(cx, cy, marker, color=color, markersize=7,
                markeredgewidth=1.3, zorder=3)
        ax.annotate(
            text,
            (cx, cy),
            xytext=(6 + (rank % 4) * 8, -12 - ((rank // 4) % 3) * 8),
            textcoords='offset points',
            fontsize=7,
            color='black',
            bbox={
                'boxstyle': 'round,pad=0.20',
                'facecolor': 'white',
                'edgecolor': color,
                'alpha': 0.84,
            },
            arrowprops={'arrowstyle': '-', 'color': color, 'lw': 0.8},
            zorder=5,
        )
        return cx, cy

    rank = 0
    for row in result['correct']:
        exp = row['expected']
        det = row['detection']
        ex, ey = plot_expected(
            exp, 'o', '#2ca02c',
            f"TP {exp['eid']}\n{det['class_name']} #{det['did']}",
            rank,
        )
        dx, dy = map_to_cell(det['x'], det['y'], meta, width, height)
        ax.plot([ex, dx], [ey, dy], color='#2ca02c', lw=1.0, alpha=0.6)
        rank += 1

    for row in result['wrong_label']:
        exp = row['expected']
        det = row['detection']
        ex, ey = plot_expected(
            exp, 'x', '#ff7f0e',
            f"WRONG {exp['eid']}\nGT {exp['wbt_type']} / det {det['class_name']}",
            rank,
        )
        dx, dy = map_to_cell(det['x'], det['y'], meta, width, height)
        ax.plot([ex, dx], [ey, dy], color='#ff7f0e', lw=1.0, alpha=0.7)
        rank += 1

    for exp in result['missed']:
        plot_expected(
            exp, 'x', '#d62728',
            f"MISS {exp['eid']}\n{exp['wbt_type']}",
            rank,
        )
        rank += 1

    for i, det in enumerate(result['extra']):
        plot_detection(
            det, '+', '#d62728',
            f"EXTRA #{det['did']}\n{det['class_name']}",
            i,
        )

    handles = [
        plt.Line2D([0], [0], marker='o', color='#2ca02c',
                   linestyle='', label='correct'),
        plt.Line2D([0], [0], marker='x', color='#ff7f0e',
                   linestyle='', label='wrong label'),
        plt.Line2D([0], [0], marker='x', color='#d62728',
                   linestyle='', label='missed'),
        plt.Line2D([0], [0], marker='+', color='#d62728',
                   linestyle='', label='extra detection'),
    ]
    ax.legend(handles=handles, loc='upper center',
              bbox_to_anchor=(0.5, -0.08), ncol=4, fontsize=8)
    os.makedirs(os.path.dirname(out_png) or '.', exist_ok=True)
    fig.savefig(out_png, bbox_inches='tight')
    plt.close(fig)


def fmt_float(value, digits=3):
    return f'{float(value):.{digits}f}'


def write_markdown(path, report):
    summary = report['summary']
    inputs = report['inputs']
    lines = [
        '# Recognition World Comparison',
        '',
        '## Inputs',
        '',
        f"- world: `{inputs['wbt']}`",
        f"- map: `{inputs.get('map') or ''}`",
        f"- db: `{inputs['db']}`",
        f"- robot_world_xy: {inputs['robot_world_xy']}",
        f"- min_existence: {inputs['min_existence']}",
        f"- min_hits: {inputs['min_hits']}",
        f"- ignored_types: {inputs.get('ignored_types', [])}",
        f"- match_distance_m: {summary['match_distance_m']}",
        '',
        '## Summary',
        '',
        '| metric | value |',
        '|---|---:|',
    ]
    metric_order = [
        'expected_count',
        'detection_count',
        'correct_count',
        'wrong_label_count',
        'missed_without_near_detection_count',
        'extra_detection_count',
        'class_aware_false_positive_count',
        'class_aware_false_negative_count',
        'precision',
        'recall',
        'f1',
    ]
    for key in metric_order:
        value = summary[key]
        if isinstance(value, float):
            value = fmt_float(value)
        lines.append(f'| {key} | {value} |')

    def add_match_table(title, rows, include_expected=True):
        lines.extend(['', f'## {title}', ''])
        if not rows:
            lines.append('None.')
            return
        lines.append(
            '| expected | accepted | detection | det_label | dist_m | exist | hits |'
        )
        lines.append('|---|---|---|---|---:|---:|---:|')
        for row in rows:
            exp = row['expected']
            det = row['detection']
            lines.append(
                f"| {exp['eid']} {exp['wbt_type']} `{exp['name']}` "
                f"({fmt_float(exp['map_x'], 2)}, {fmt_float(exp['map_y'], 2)}) "
                f"| {', '.join(exp['accepted_labels'])} "
                f"| #{det['did']} ({fmt_float(det['x'], 2)}, {fmt_float(det['y'], 2)}) "
                f"| {det['class_name']} "
                f"| {fmt_float(row['distance_m'], 2)} "
                f"| {fmt_float(det['existence'], 2)} "
                f"| {det['hits']} |"
            )

    add_match_table('Correct Matches', report['correct'])
    add_match_table('Wrong Label Near Ground Truth', report['wrong_label'])

    lines.extend(['', '## Missed Ground Truth', ''])
    if report['missed']:
        lines.append('| expected | accepted | map_xy | world_xy |')
        lines.append('|---|---|---:|---:|')
        for exp in report['missed']:
            lines.append(
                f"| {exp['eid']} {exp['wbt_type']} `{exp['name']}` "
                f"| {', '.join(exp['accepted_labels'])} "
                f"| ({fmt_float(exp['map_x'], 2)}, {fmt_float(exp['map_y'], 2)}) "
                f"| ({fmt_float(exp['world_x'], 2)}, {fmt_float(exp['world_y'], 2)}) |"
            )
    else:
        lines.append('None.')

    lines.extend(['', '## Extra Detections', ''])
    if report['extra']:
        lines.append('| detection | label | map_xy | exist | hits |')
        lines.append('|---|---|---:|---:|---:|')
        for det in report['extra']:
            lines.append(
                f"| #{det['did']} | {det['class_name']} "
                f"| ({fmt_float(det['x'], 2)}, {fmt_float(det['y'], 2)}) "
                f"| {fmt_float(det['existence'], 2)} | {det['hits']} |"
            )
    else:
        lines.append('None.')

    lines.extend(['', '## Skipped World Objects', ''])
    if report['skipped_world_objects']:
        lines.append('| type | name | world_xy | reason |')
        lines.append('|---|---|---:|---|')
        for obj in report['skipped_world_objects']:
            xy = ''
            if 'world_x' in obj:
                xy = f"({fmt_float(obj['world_x'], 2)}, {fmt_float(obj['world_y'], 2)})"
            lines.append(
                f"| {obj['wbt_type']} | `{obj.get('name', '')}` | {xy} "
                f"| {obj['reason']} |"
            )
    else:
        lines.append('None.')
    lines.append('')

    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def write_csv(path, report):
    fields = [
        'status', 'expected_id', 'expected_type', 'expected_name',
        'expected_labels', 'expected_x', 'expected_y',
        'detection_id', 'detection_class', 'detection_x', 'detection_y',
        'distance_m', 'existence', 'hits',
    ]
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for status in ('correct', 'wrong_label'):
            for row in report[status]:
                exp = row['expected']
                det = row['detection']
                writer.writerow({
                    'status': status,
                    'expected_id': exp['eid'],
                    'expected_type': exp['wbt_type'],
                    'expected_name': exp['name'],
                    'expected_labels': ';'.join(exp['accepted_labels']),
                    'expected_x': exp['map_x'],
                    'expected_y': exp['map_y'],
                    'detection_id': det['did'],
                    'detection_class': det['class_name'],
                    'detection_x': det['x'],
                    'detection_y': det['y'],
                    'distance_m': row['distance_m'],
                    'existence': det['existence'],
                    'hits': det['hits'],
                })
        for exp in report['missed']:
            writer.writerow({
                'status': 'missed',
                'expected_id': exp['eid'],
                'expected_type': exp['wbt_type'],
                'expected_name': exp['name'],
                'expected_labels': ';'.join(exp['accepted_labels']),
                'expected_x': exp['map_x'],
                'expected_y': exp['map_y'],
            })
        for det in report['extra']:
            writer.writerow({
                'status': 'extra',
                'detection_id': det['did'],
                'detection_class': det['class_name'],
                'detection_x': det['x'],
                'detection_y': det['y'],
                'existence': det['existence'],
                'hits': det['hits'],
            })


def parse_extra_targets(items):
    out = {}
    for item in items:
        if '=' not in item:
            raise ValueError(
                f'--target-type must be Type=label1,label2: {item}')
        typ, labels = item.split('=', 1)
        label_list = [v.strip() for v in labels.split(',') if v.strip()]
        if not typ.strip() or not label_list:
            raise ValueError(
                f'--target-type must include type and labels: {item}')
        out[typ.strip()] = label_list
    return out


def run(args):
    extra_targets = parse_extra_targets(args.target_type)
    ignored_types = set(args.ignore_type)
    expected, skipped, robot_xy = parse_world_objects(
        args.wbt, args.robot, extra_targets, ignored_types)
    detections = load_detections(
        args.db, args.min_existence, args.min_hits)
    ignored_labels = set()
    target_types = dict(SUPPORTED_WORLD_TYPES)
    target_types.update(extra_targets)
    for typ in ignored_types:
        ignored_labels.update(normalized_label(v)
                              for v in target_types.get(typ, []))
    if ignored_labels:
        detections = [
            d for d in detections
            if normalized_label(d.class_name) not in ignored_labels
        ]
    result = evaluate(expected, detections, args.match_distance)
    report = {
        'inputs': {
            'wbt': args.wbt,
            'map': args.map,
            'db': args.db,
            'robot_world_xy': [robot_xy[0], robot_xy[1]],
            'min_existence': args.min_existence,
            'min_hits': args.min_hits,
            'ignored_types': sorted(ignored_types),
        },
        'summary': result['summary'],
        'correct': result['correct'],
        'wrong_label': result['wrong_label'],
        'missed': result['missed'],
        'extra': result['extra'],
        'skipped_world_objects': skipped,
    }

    prefix = args.out_prefix
    os.makedirs(os.path.dirname(prefix) or '.', exist_ok=True)
    json_path = f'{prefix}.json'
    md_path = f'{prefix}.md'
    csv_path = f'{prefix}.csv'
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    write_markdown(md_path, report)
    write_csv(csv_path, report)
    png_path = None
    if args.map:
        png_path = f'{prefix}.png'
        render_png(args.map, png_path, report, args.scale)

    summary = report['summary']
    print(
        f"expected={summary['expected_count']} "
        f"detections={summary['detection_count']} "
        f"correct={summary['correct_count']} "
        f"wrong_label={summary['wrong_label_count']} "
        f"extra={summary['extra_detection_count']} "
        f"precision={summary['precision']:.3f} "
        f"recall={summary['recall']:.3f} "
        f"f1={summary['f1']:.3f}"
    )
    print(f'wrote {md_path}')
    print(f'wrote {json_path}')
    print(f'wrote {csv_path}')
    if png_path:
        print(f'wrote {png_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wbt', required=True, help='Webots world file')
    ap.add_argument('--map', default='', help='saved map yaml for PNG overlay')
    ap.add_argument('--db', required=True, help='object_memory sqlite3 path')
    ap.add_argument('--out-prefix', required=True,
                    help='output path without extension')
    ap.add_argument('--robot', nargs=2, type=float, default=None,
                    help='robot initial Webots x y; default parses TurtleBot')
    ap.add_argument('--min-existence', type=float, default=0.5)
    ap.add_argument('--min-hits', type=int, default=5)
    ap.add_argument('--match-distance', type=float, default=1.0)
    ap.add_argument('--scale', type=float, default=0.0,
                    help='PNG map enlargement factor. 0 chooses automatically')
    ap.add_argument(
        '--target-type',
        action='append',
        default=[],
        help='extend/override expected world objects: Type=label1,label2',
    )
    ap.add_argument(
        '--ignore-type',
        action='append',
        default=[],
        help='ignore this Webots PROTO type and detections with its labels',
    )
    args = ap.parse_args()
    run(args)


if __name__ == '__main__':
    main()
