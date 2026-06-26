#!/usr/bin/env python3
"""Generate map-safe debug waypoints that face selected Webots objects.

This is a diagnostic companion to evaluate_recognition_vs_world.py.  It uses
the saved SLAM map for reachability and Webots world objects only to choose
which missed targets to inspect in a controlled live run.
"""

import argparse
import csv
from dataclasses import asdict
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import yaml
from scipy import ndimage


def _load_eval_module():
    path = Path(__file__).with_name('evaluate_recognition_vs_world.py')
    spec = importlib.util.spec_from_file_location(
        '_susumu_evaluate_recognition_vs_world', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


class Grid:

    def __init__(self, map_yaml):
        self.map_yaml = map_yaml
        with open(map_yaml, encoding='utf-8') as f:
            self.meta = yaml.safe_load(f)
        pgm_path = os.path.join(
            os.path.dirname(map_yaml), self.meta['image'])
        if not os.path.exists(pgm_path):
            raise FileNotFoundError(
                f'map image missing: {pgm_path} referenced by {map_yaml}')
        self.image = load_pgm(pgm_path)
        self.height, self.width = self.image.shape
        self.resolution = float(self.meta['resolution'])
        self.origin_x = float(self.meta['origin'][0])
        self.origin_y = float(self.meta['origin'][1])
        negate = int(self.meta.get('negate', 0))
        occ_thresh = float(self.meta.get('occupied_thresh', 0.65))
        free_thresh = float(self.meta.get('free_thresh', 0.25))
        if negate:
            occ_prob = self.image.astype(np.float32) / 255.0
        else:
            occ_prob = (255.0 - self.image.astype(np.float32)) / 255.0
        self.occupied = occ_prob >= occ_thresh
        self.free = occ_prob <= free_thresh
        self.unknown = ~(self.occupied | self.free)
        self.clearance_cells = ndimage.distance_transform_edt(self.free)
        ys, xs = np.nonzero(self.occupied)
        if len(xs):
            self.occupied_xy = np.column_stack((
                self.origin_x + (xs + 0.5) * self.resolution,
                self.origin_y + (self.height - ys - 0.5) * self.resolution,
            )).astype(np.float32)
        else:
            self.occupied_xy = np.empty((0, 2), dtype=np.float32)

    def world_to_cell(self, x, y):
        cx = int(math.floor((x - self.origin_x) / self.resolution))
        cy_from_bottom = int(math.floor((y - self.origin_y) / self.resolution))
        cy = self.height - 1 - cy_from_bottom
        return cx, cy

    def cell_to_world(self, cx, cy):
        return (
            self.origin_x + (cx + 0.5) * self.resolution,
            self.origin_y + (self.height - cy - 0.5) * self.resolution,
        )

    def inside(self, cx, cy):
        return 0 <= cx < self.width and 0 <= cy < self.height

    def nearest_occupied_distance(self, x, y):
        if self.occupied_xy.size == 0:
            return None
        d = np.hypot(self.occupied_xy[:, 0] - x,
                     self.occupied_xy[:, 1] - y)
        return float(d.min())


def _main_component(grid, clearance_m, start_xy):
    passable = grid.free & (
        grid.clearance_cells >= clearance_m / grid.resolution)
    labels, count = ndimage.label(passable)
    if count == 0:
        raise RuntimeError(
            f'no passable cell at connect_clearance={clearance_m}m')
    start_label = 0
    if start_xy is not None:
        sx, sy = grid.world_to_cell(start_xy[0], start_xy[1])
        if grid.inside(sx, sy):
            start_label = int(labels[sy, sx])
    if start_label > 0:
        label = start_label
        source = 'start'
    else:
        sizes = ndimage.sum(
            np.ones_like(labels, dtype=np.int32),
            labels,
            range(1, count + 1),
        )
        label = int(np.argmax(sizes)) + 1
        source = 'largest'
    return labels == label, source


def _line_is_clear(grid, x0, y0, x1, y1, stop_before_m):
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist <= 1.0e-6:
        return False
    usable = max(0.0, dist - stop_before_m)
    steps = max(2, int(math.ceil(usable / (grid.resolution * 0.5))))
    for i in range(1, steps + 1):
        t = (usable / dist) * (i / steps)
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        cx, cy = grid.world_to_cell(x, y)
        if not grid.inside(cx, cy):
            return False
        if not grid.free[cy, cx]:
            return False
    return True


def _foreground_blockers(view_x, view_y, target, all_targets, args):
    """Find world-truth objects in front of the target from this viewpoint.

    This script already uses world truth to create diagnostic viewpoints.  The
    blocker score is only for choosing a better inspection pose; it is not used
    by normal patrol or final recognition scoring.
    """
    dx = target.map_x - view_x
    dy = target.map_y - view_y
    target_dist = math.hypot(dx, dy)
    if target_dist <= 1.0e-6:
        return []
    ux = dx / target_dist
    uy = dy / target_dist
    max_angle = math.radians(args.foreground_angle_deg)
    blockers = []
    for other in all_targets:
        if other.eid == target.eid:
            continue
        ox = other.map_x - view_x
        oy = other.map_y - view_y
        along = ox * ux + oy * uy
        if along <= 0.0 or along >= target_dist - args.foreground_stop_before:
            continue
        lateral = abs(ox * uy - oy * ux)
        angle = math.atan2(lateral, max(along, 1.0e-6))
        if angle <= max_angle or lateral <= args.foreground_lateral_m:
            blockers.append({
                'eid': other.eid,
                'wbt_type': other.wbt_type,
                'name': other.name,
                'distance_along_m': float(along),
                'lateral_m': float(lateral),
                'angle_deg': float(math.degrees(angle)),
            })
    blockers.sort(key=lambda row: (row['lateral_m'], row['distance_along_m']))
    return blockers


def _candidate_viewpoints(grid, route_mask, target, all_targets, args):
    angle_step = math.radians(args.angle_step_deg)
    radii = np.arange(args.min_radius, args.max_radius + 1.0e-9,
                      args.radius_step)
    candidates = []
    checked = 0
    free_count = 0
    los_count = 0
    for radius in radii:
        steps = max(1, int(round((2.0 * math.pi) / angle_step)))
        for k in range(steps):
            theta = 2.0 * math.pi * k / steps
            x = target.map_x + radius * math.cos(theta)
            y = target.map_y + radius * math.sin(theta)
            checked += 1
            cx, cy = grid.world_to_cell(x, y)
            if not grid.inside(cx, cy):
                continue
            if not route_mask[cy, cx]:
                continue
            clearance = grid.clearance_cells[cy, cx] * grid.resolution
            if clearance < args.clearance:
                continue
            free_count += 1
            los = _line_is_clear(
                grid, x, y, target.map_x, target.map_y,
                args.los_stop_before)
            if los:
                los_count += 1
            if args.require_los and not los:
                continue
            blockers = _foreground_blockers(x, y, target, all_targets, args)
            projected_size_score = 1.0 / max(radius, 1.0e-6)
            yaw = math.atan2(target.map_y - y, target.map_x - x)
            score = (
                (0.0 if los else 100.0)
                + abs(radius - args.standoff)
                - min(clearance, 1.5) * 0.08
                - args.projected_size_weight * projected_size_score
                + args.foreground_penalty * len(blockers)
            )
            candidates.append({
                'x': float(x),
                'y': float(y),
                'yaw': float(yaw),
                'distance_m': float(radius),
                'clearance_m': float(clearance),
                'line_of_sight': bool(los),
                'foreground_blocker_count': len(blockers),
                'foreground_blockers': blockers[:5],
                'projected_size_score': float(projected_size_score),
                'score': float(score),
            })
    candidates.sort(key=lambda row: row['score'])
    return candidates, {
        'sampled_count': checked,
        'free_clearance_candidate_count': free_count,
        'line_of_sight_candidate_count': los_count,
    }


def _parse_extra_target(values):
    out = {}
    for value in values:
        if '=' not in value:
            raise ValueError(
                '--target-type-extra must be Type=label1,label2')
        typ, labels = value.split('=', 1)
        out[typ.strip()] = [
            v.strip() for v in labels.split(',') if v.strip()
        ]
    return out


def _write_yaml(path, map_yaml, selected):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    waypoints = [
        [round(row['viewpoint']['x'], 3),
         round(row['viewpoint']['y'], 3),
         round(row['viewpoint']['yaw'], 4)]
        for row in selected
        if row.get('viewpoint') is not None
    ]
    data = {
        'map': os.path.basename(map_yaml),
        'frame_id': 'map',
        'purpose': 'recognition_debug_viewpoints',
        'waypoints': waypoints,
        'targets': [
            {
                'eid': row['target']['eid'],
                'wbt_type': row['target']['wbt_type'],
                'name': row['target']['name'],
                'map_x': round(row['target']['map_x'], 3),
                'map_y': round(row['target']['map_y'], 3),
                'waypoint_index': row.get('waypoint_index'),
            }
            for row in selected
        ],
    }
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
    return len(waypoints)


def _write_json(path, report):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)


def _write_csv(path, rows):
    fields = [
        'eid', 'wbt_type', 'name', 'target_x', 'target_y',
        'nearest_map_occupied_m', 'has_map_support', 'waypoint_index',
        'view_x', 'view_y', 'yaw_rad', 'yaw_deg', 'view_distance_m',
        'view_clearance_m', 'line_of_sight', 'sampled_count',
        'free_clearance_candidate_count', 'line_of_sight_candidate_count',
        'foreground_blocker_count', 'foreground_blockers', 'score', 'status',
    ]
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            target = row['target']
            vp = row.get('viewpoint')
            writer.writerow({
                'eid': target['eid'],
                'wbt_type': target['wbt_type'],
                'name': target['name'],
                'target_x': f"{target['map_x']:.3f}",
                'target_y': f"{target['map_y']:.3f}",
                'nearest_map_occupied_m': (
                    '' if target['nearest_map_occupied_m'] is None
                    else f"{target['nearest_map_occupied_m']:.3f}"),
                'has_map_support': target['has_map_support'],
                'waypoint_index': row.get('waypoint_index', ''),
                'view_x': '' if vp is None else f"{vp['x']:.3f}",
                'view_y': '' if vp is None else f"{vp['y']:.3f}",
                'yaw_rad': '' if vp is None else f"{vp['yaw']:.4f}",
                'yaw_deg': '' if vp is None else (
                    f"{math.degrees(vp['yaw']):.1f}"),
                'view_distance_m': (
                    '' if vp is None else f"{vp['distance_m']:.3f}"),
                'view_clearance_m': (
                    '' if vp is None else f"{vp['clearance_m']:.3f}"),
                'line_of_sight': '' if vp is None else vp['line_of_sight'],
                'sampled_count': row['diagnostic']['sampled_count'],
                'free_clearance_candidate_count':
                    row['diagnostic']['free_clearance_candidate_count'],
                'line_of_sight_candidate_count':
                    row['diagnostic']['line_of_sight_candidate_count'],
                'foreground_blocker_count': (
                    '' if vp is None else vp.get('foreground_blocker_count', 0)),
                'foreground_blockers': (
                    '' if vp is None else ','.join(
                        b.get('eid', '')
                        for b in vp.get('foreground_blockers', []))),
                'score': '' if vp is None else f"{vp['score']:.3f}",
                'status': row['status'],
            })


def _write_markdown(path, report):
    s = report['summary']
    lines = [
        '# Recognition Debug Viewpoints',
        '',
        '## Summary',
        '',
        f"- target_count: `{s['target_count']}`",
        f"- generated_waypoints: `{s['generated_waypoints']}`",
        f"- targets_with_map_support: `{s['targets_with_map_support']}`",
        f"- selected_with_line_of_sight: `{s['selected_with_line_of_sight']}`",
        f"- no_candidate_count: `{s['no_candidate_count']}`",
        f"- component_source: `{s['component_source']}`",
        '',
        '## Inputs',
        '',
        f"- map: `{report['inputs']['map']}`",
        f"- world: `{report['inputs']['wbt']}`",
        f"- target_types: `{', '.join(report['inputs']['target_types'])}`",
        '',
        '## Viewpoints',
        '',
        '| target | map support | waypoint | view distance | clearance | LOS | foreground | status |',
        '|---|---:|---|---:|---:|---|---:|---|',
    ]
    for row in report['targets']:
        target = row['target']
        vp = row.get('viewpoint')
        if target['nearest_map_occupied_m'] is None:
            support = ''
        else:
            support = (
                f"{target['nearest_map_occupied_m']:.2f}m / "
                f"{target['has_map_support']}")
        if vp is None:
            wp = ''
            dist = ''
            clearance = ''
            los = ''
            foreground = ''
        else:
            wp = (
                f"#{row['waypoint_index']} "
                f"({vp['x']:.2f}, {vp['y']:.2f}, "
                f"{math.degrees(vp['yaw']):.0f}deg)")
            dist = f"{vp['distance_m']:.2f}m"
            clearance = f"{vp['clearance_m']:.2f}m"
            los = str(vp['line_of_sight'])
            foreground = str(vp.get('foreground_blocker_count', 0))
        lines.append(
            f"| `{target['eid']}` | {support} | {wp} | {dist} | "
            f"{clearance} | {los} | {foreground} | {row['status']} |")
    lines.extend([
        '',
        '## Notes',
        '',
        '- These waypoints are for live diagnosis only. They use world truth to select targets, so they are not patrol waypoints for final recognition scoring.',
        '- The line-of-sight check uses the saved SLAM map and stops before the target center so that the target obstacle itself does not block the ray.',
    ])
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True, help='SLAM map YAML')
    parser.add_argument('--wbt', required=True, help='Webots world file')
    parser.add_argument('--out', required=True, help='output waypoint YAML')
    parser.add_argument('--report-prefix', default='',
                        help='prefix for .md/.json/.csv reports')
    parser.add_argument('--target-type', action='append', default=[],
                        help='Webots PROTO type to inspect')
    parser.add_argument('--target-type-extra', action='append', default=[],
                        help='extra target mapping: Type=label1,label2')
    parser.add_argument('--ignore-type', action='append', default=[],
                        help='world type to ignore while parsing')
    parser.add_argument('--robot-xy', type=float, nargs=2, default=None,
                        help='robot initial Webots x y; default parses world')
    parser.add_argument('--start-x', type=float, default=0.0)
    parser.add_argument('--start-y', type=float, default=0.0)
    parser.add_argument('--map-support-dist', type=float, default=0.55)
    parser.add_argument('--connect-clearance', type=float, default=0.30)
    parser.add_argument('--clearance', type=float, default=0.60)
    parser.add_argument('--min-radius', type=float, default=1.0)
    parser.add_argument('--max-radius', type=float, default=2.2)
    parser.add_argument('--radius-step', type=float, default=0.10)
    parser.add_argument('--standoff', type=float, default=1.4)
    parser.add_argument('--angle-step-deg', type=float, default=5.0)
    parser.add_argument('--los-stop-before', type=float, default=0.45)
    parser.add_argument('--require-los', action='store_true',
                        help='drop candidates without map line-of-sight')
    parser.add_argument('--foreground-angle-deg', type=float, default=10.0,
                        help='angular gate for world-truth foreground blockers')
    parser.add_argument('--foreground-lateral-m', type=float, default=0.35,
                        help='lateral gate for world-truth foreground blockers')
    parser.add_argument('--foreground-stop-before', type=float, default=0.35,
                        help='ignore blockers this close to the target center along the ray')
    parser.add_argument('--foreground-penalty', type=float, default=0.8,
                        help='score penalty per world-truth foreground blocker')
    parser.add_argument('--projected-size-weight', type=float, default=0.10,
                        help='score bonus for closer viewpoints, proportional to 1/distance')
    args = parser.parse_args()

    if args.radius_step <= 0.0 or args.angle_step_deg <= 0.0:
        parser.error('--radius-step and --angle-step-deg must be positive')
    if args.max_radius < args.min_radius:
        parser.error('--max-radius must be >= --min-radius')

    eval_mod = _load_eval_module()
    extra = _parse_extra_target(args.target_type_extra)
    expected, skipped, robot_xy = eval_mod.parse_world_objects(
        args.wbt,
        args.robot_xy,
        extra_targets=extra,
        ignored_types=args.ignore_type,
    )
    target_types = args.target_type or sorted(
        {obj.wbt_type for obj in expected})
    target_type_set = set(target_types)
    targets = [obj for obj in expected if obj.wbt_type in target_type_set]
    if not targets:
        raise RuntimeError(
            'no target objects matched --target-type; available: '
            + ', '.join(sorted({obj.wbt_type for obj in expected})))

    grid = Grid(args.map)
    route_mask, component_source = _main_component(
        grid, args.connect_clearance, (args.start_x, args.start_y))

    rows = []
    waypoint_index = 0
    for target in targets:
        nearest_occ = grid.nearest_occupied_distance(
            target.map_x, target.map_y)
        target.nearest_map_occupied_m = nearest_occ
        target.has_map_support = bool(
            nearest_occ is not None and nearest_occ <= args.map_support_dist)
        candidates, diagnostic = _candidate_viewpoints(
            grid, route_mask, target, expected, args)
        viewpoint = candidates[0] if candidates else None
        status = 'selected' if viewpoint is not None else 'no_candidate'
        row = {
            'target': asdict(target),
            'viewpoint': viewpoint,
            'diagnostic': diagnostic,
            'status': status,
            'waypoint_index': None,
        }
        if viewpoint is not None:
            row['waypoint_index'] = waypoint_index
            waypoint_index += 1
        rows.append(row)

    report_prefix = args.report_prefix or os.path.splitext(args.out)[0]
    generated = _write_yaml(args.out, args.map, rows)
    report = {
        'inputs': {
            'map': args.map,
            'wbt': args.wbt,
            'out': args.out,
            'target_types': target_types,
            'robot_xy': robot_xy,
            'skipped_count': len(skipped),
            'require_los': args.require_los,
            'connect_clearance_m': args.connect_clearance,
            'clearance_m': args.clearance,
            'radius_range_m': [args.min_radius, args.max_radius],
            'standoff_m': args.standoff,
            'foreground_angle_deg': args.foreground_angle_deg,
            'foreground_lateral_m': args.foreground_lateral_m,
            'foreground_penalty': args.foreground_penalty,
            'projected_size_weight': args.projected_size_weight,
        },
        'summary': {
            'target_count': len(rows),
            'generated_waypoints': generated,
            'targets_with_map_support': sum(
                1 for row in rows
                if row['target'].get('has_map_support')),
            'selected_with_line_of_sight': sum(
                1 for row in rows
                if row.get('viewpoint') is not None
                and row['viewpoint'].get('line_of_sight')),
            'no_candidate_count': sum(
                1 for row in rows if row.get('viewpoint') is None),
            'component_source': component_source,
        },
        'targets': rows,
    }
    _write_json(report_prefix + '.json', report)
    _write_csv(report_prefix + '.csv', rows)
    _write_markdown(report_prefix + '.md', report)

    print(
        f'generated {generated}/{len(rows)} debug viewpoints -> {args.out}')
    print(
        f"  map_support={report['summary']['targets_with_map_support']} "
        f"los={report['summary']['selected_with_line_of_sight']} "
        f"no_candidate={report['summary']['no_candidate_count']} "
        f"component={component_source}")
    print(f'  reports -> {report_prefix}.md/.json/.csv')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
