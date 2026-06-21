#!/usr/bin/env python3
"""Generate inspection waypoints for low-coverage objects in a WBT/map pair.

This is a bridge between the offline WBT-vs-map evaluator and live sensor
checks. It places the robot on known-free map cells near low-coverage fence
objects and points the robot toward the missed WBT geometry. The resulting YAML
can be fed to waypoint_nav_node.py, which accepts [x, y] and [x, y, yaw].
"""

import argparse
import math
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy import ndimage

from check_map_vs_world import (load_pgm, object_sample_points, parse_wbt,
                                summarize_alignment, worst_objects)


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def find_robot_xy(wbt_path):
    txt = open(wbt_path).read()
    m = re.search(
        r'TurtleBot\w*\s*\{[^}]*?translation\s+([-\d.]+)\s+([-\d.]+)',
        txt)
    return (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)


def yaw_to_quat(yaw):
    return {'z': math.sin(yaw * 0.5), 'w': math.cos(yaw * 0.5)}


def line_cells(x0, y0, x1, y1, steps):
    xs = np.linspace(x0, x1, steps)
    ys = np.linspace(y0, y1, steps)
    return np.round(xs).astype(np.int32), np.round(ys).astype(np.int32)


def line_score(img_display, x0, y0, x1, y1):
    h, w = img_display.shape
    xs, ys = line_cells(x0, y0, x1, y1, 80)
    # Ignore the last few cells because the target itself may be a missing
    # occupied trace that is currently classified as free.
    xs = xs[:-4]
    ys = ys[:-4]
    ok = (0 <= xs) & (xs < w) & (0 <= ys) & (ys < h)
    if not np.all(ok):
        return None
    vals = img_display[ys, xs]
    occupied = int(np.count_nonzero(vals < 50))
    unknown = int(np.count_nonzero((vals >= 50) & (vals < 250)))
    if occupied:
        return None
    return unknown


def reachable_free_mask(meta, img_display, connect_clearance_m):
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    free = img_display >= 250
    clearance = ndimage.distance_transform_edt(free) * res
    passable = free & (clearance >= connect_clearance_m)
    rx = int(round((0.0 - ox) / res))
    ry = int(round((0.0 - oy) / res))
    labels, count = ndimage.label(passable)
    if count == 0:
        return passable
    h, w = passable.shape
    if 0 <= rx < w and 0 <= ry < h and labels[ry, rx] > 0:
        label = labels[ry, rx]
    else:
        yy, xx = np.nonzero(passable)
        if len(xx) == 0:
            return passable
        nearest = np.argmin((xx - rx) ** 2 + (yy - ry) ** 2)
        label = labels[yy[nearest], xx[nearest]]
    return labels == label


def object_target(report, obj, rxw, ryw):
    rows = [
        row for row in report['samples']
        if row['object_index'] == obj['object_index'] and row['inside_map']
    ]
    missed = [
        row for row in rows
        if row['distance_to_occupied_m'] is not None
        and row['distance_to_occupied_m'] > report['summary']['threshold_m']
        and row['cell_class'] in ('free', 'unknown')
    ]
    source = missed or rows
    if source:
        pts = np.asarray([row['world_xy'] for row in source], dtype=np.float64)
        wx, wy = pts.mean(axis=0)
    else:
        pts = np.asarray(object_sample_points(obj), dtype=np.float64)
        wx, wy = pts.mean(axis=0)
    return float(wx - rxw), float(wy - ryw)


def choose_probe_pose(target, meta, img_display, clearance_m, preferred_m,
                      min_m, max_m, reachable=None):
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    h, w = img_display.shape

    tx = (target[0] - ox) / res
    ty = (target[1] - oy) / res
    if not (0 <= tx < w and 0 <= ty < h):
        return None

    free = img_display >= 250
    clearance = ndimage.distance_transform_edt(free) * res
    valid = free & (clearance >= clearance_m)
    if reachable is not None:
        valid &= reachable
    yy, xx = np.nonzero(valid)
    if len(xx) == 0:
        return None

    dist_m = np.hypot((xx - tx) * res, (yy - ty) * res)
    ring = (min_m <= dist_m) & (dist_m <= max_m)
    if not np.any(ring):
        return None

    cand_x = xx[ring]
    cand_y = yy[ring]
    cand_d = dist_m[ring]
    cand_clearance = clearance[cand_y, cand_x]
    score = np.abs(cand_d - preferred_m) - 0.05 * cand_clearance
    order = np.argsort(score)

    best = None
    for idx in order[:1000]:
        x = int(cand_x[idx])
        y = int(cand_y[idx])
        unknown_on_line = line_score(img_display, x, y, tx, ty)
        if unknown_on_line is None:
            continue
        final_score = float(score[idx] + 0.02 * unknown_on_line)
        if best is None or final_score < best[0]:
            best = (final_score, x, y, float(cand_d[idx]),
                    float(cand_clearance[idx]), unknown_on_line)
            if unknown_on_line == 0:
                break
    if best is None:
        return None

    _, x, y, distance_m, clearance_at_pose, unknown_on_line = best
    mx = ox + x * res
    my = oy + y * res
    yaw = math.atan2(target[1] - my, target[0] - mx)
    return {
        'x': float(mx),
        'y': float(my),
        'yaw': float(yaw),
        'distance_to_target_m': distance_m,
        'clearance_m': clearance_at_pose,
        'unknown_cells_on_view_line': int(unknown_on_line),
    }


def draw_debug(out_png, img_display, meta, targets):
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    h, w = img_display.shape
    fig, ax = plt.subplots(figsize=(w / 45, h / 45))
    ax.imshow(img_display, cmap='gray', vmin=0, vmax=255, origin='lower')
    for i, target in enumerate(targets):
        px = (target['pose']['x'] - ox) / res
        py = (target['pose']['y'] - oy) / res
        tx = (target['target'][0] - ox) / res
        ty = (target['target'][1] - oy) / res
        ax.plot(tx, ty, 'rx', ms=8, mew=2)
        ax.arrow(px, py, tx - px, ty - py, color='yellow',
                 length_includes_head=True, head_width=6, linewidth=1.5)
        ax.text(px, py, str(i), color='white', fontsize=8)
    ax.set_xlim(-20, w + 20)
    ax.set_ylim(-20, h + 20)
    ax.set_title('fence probe waypoints: yellow=robot pose to target')
    fig.savefig(out_png, dpi=90, bbox_inches='tight')


def order_targets_nearest(targets, start=(0.0, 0.0)):
    ordered = []
    remaining = list(targets)
    cur = start
    while remaining:
        idx = min(
            range(len(remaining)),
            key=lambda i: math.hypot(
                remaining[i]['pose']['x'] - cur[0],
                remaining[i]['pose']['y'] - cur[1]))
        nxt = remaining.pop(idx)
        ordered.append(nxt)
        cur = (nxt['pose']['x'], nxt['pose']['y'])
    return ordered


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wbt', required=True)
    parser.add_argument('--map', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--out-png', default='')
    parser.add_argument('--kind', default='fence')
    parser.add_argument('--max-targets', type=int, default=6)
    parser.add_argument('--max-coverage', type=float, default=0.55)
    parser.add_argument('--clearance-m', type=float, default=0.35)
    parser.add_argument('--connect-clearance-m', type=float, default=0.25,
                        help='free-space connected component clearance [m]')
    parser.add_argument('--preferred-distance-m', type=float, default=1.5)
    parser.add_argument('--min-distance-m', type=float, default=0.8)
    parser.add_argument('--max-distance-m', type=float, default=3.0)
    parser.add_argument('--order', choices=('coverage', 'nearest'),
                        default='nearest',
                        help='output order for probe waypoints')
    parser.add_argument('--occupied-distance-threshold-m', type=float,
                        default=0.5)
    args = parser.parse_args()

    meta = yaml.safe_load(open(args.map))
    img = load_pgm(os.path.join(os.path.dirname(args.map), meta['image']))
    img_display = img[::-1]
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    rxw, ryw = find_robot_xy(args.wbt)
    reachable = reachable_free_mask(
        meta, img_display, args.connect_clearance_m)

    def world_to_map_px(wx, wy):
        mx = wx - rxw
        my = wy - ryw
        return (mx - ox) / res, (my - oy) / res

    objs = parse_wbt(args.wbt)
    report = summarize_alignment(
        objs, img_display, world_to_map_px, res,
        args.occupied_distance_threshold_m)
    by_index = {i: obj for i, obj in enumerate(objs)}

    targets = []
    for obj in worst_objects(report, args.kind, args.max_targets * 4):
        coverage = obj['coverage_inside']
        if coverage is None or coverage > args.max_coverage:
            continue
        source_obj = by_index[obj['object_index']]
        target = object_target(report, obj, rxw, ryw)
        pose = choose_probe_pose(
            target, meta, img_display, args.clearance_m,
            args.preferred_distance_m, args.min_distance_m,
            args.max_distance_m, reachable)
        if pose is None:
            continue
        if any(math.hypot(pose['x'] - prev['pose']['x'],
                          pose['y'] - prev['pose']['y']) < 0.6
               for prev in targets):
            continue
        targets.append({
            'object_index': obj['object_index'],
            'object_name': obj.get('object_name', ''),
            'type': obj['type'],
            'kind': obj['kind'],
            'coverage_inside': obj['coverage_inside'],
            'cell_counts': obj['cell_counts'],
            'target': [round(target[0], 3), round(target[1], 3)],
            'pose': {
                'x': round(pose['x'], 3),
                'y': round(pose['y'], 3),
                'yaw': round(pose['yaw'], 6),
                'yaw_deg': round(math.degrees(pose['yaw']), 2),
                'orientation': yaw_to_quat(pose['yaw']),
                'distance_to_target_m': round(
                    pose['distance_to_target_m'], 3),
                'clearance_m': round(pose['clearance_m'], 3),
                'unknown_cells_on_view_line': (
                    pose['unknown_cells_on_view_line']),
            },
            'source_size': source_obj.get('size'),
        })
        if len(targets) >= args.max_targets:
            break
    if args.order == 'nearest':
        targets = order_targets_nearest(targets)

    out_data = {
        'map': os.path.basename(args.map),
        'frame_id': 'map',
        'source_wbt': os.path.basename(args.wbt),
        'generated_by': 'generate_fence_probe_waypoints.py',
        'purpose': 'inspect /scan and /lidar/points near low-coverage fence',
        'waypoint_format': '[x, y, yaw_rad]',
        'connect_clearance_m': args.connect_clearance_m,
        'order': args.order,
        'reachable_component_cells': int(np.count_nonzero(reachable)),
        'waypoints': [
            [t['pose']['x'], t['pose']['y'], t['pose']['yaw']]
            for t in targets
        ],
        'targets': targets,
    }
    with open(args.out, 'w') as f:
        yaml.dump(out_data, f, Dumper=NoAliasDumper, sort_keys=False,
                  allow_unicode=False)

    if args.out_png:
        draw_debug(args.out_png, img_display, meta, targets)

    print(f'wrote {args.out} ({len(targets)} probe waypoints)')
    if args.out_png:
        print(f'wrote {args.out_png}')
    for i, target in enumerate(targets):
        pose = target['pose']
        print(
            f'  #{i} obj={target["object_index"]} '
            f'{target["object_name"]} coverage='
            f'{target["coverage_inside"]:.3f} '
            f'pose=({pose["x"]:.2f},{pose["y"]:.2f},'
            f'{pose["yaw_deg"]:.1f}deg) '
            f'target=({target["target"][0]:.2f},{target["target"][1]:.2f}) '
            f'clearance={pose["clearance_m"]:.2f}m')


if __name__ == '__main__':
    main()
