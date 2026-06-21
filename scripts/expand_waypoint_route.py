#!/usr/bin/env python3
"""Expand a waypoint route into map-constrained intermediate goals.

This is an outdoor patrol post-processor.  It keeps the waypoint order produced
by generate_outdoor_waypoints.py, but replaces long pose-to-pose jumps with
intermediate goals sampled along the saved-map geodesic path.  The purpose is
to make the mission executor follow a centerline-like corridor instead of
asking Nav2 to take broad shortcuts between sparse route goals.
"""

import argparse
import csv
import importlib.util
import json
import math
import os
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage


def _load_waypoint_tools():
    path = Path(__file__).with_name('generate_waypoints.py')
    spec = importlib.util.spec_from_file_location('_generate_waypoints', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _cell_to_map(cell, meta, height):
    cx, cy = cell
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    return (
        ox + (cx + 0.5) * res,
        oy + (height - 1 - cy + 0.5) * res,
    )


def _map_to_cell(x, y, meta, height):
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    return (
        int(round((float(x) - ox) / res - 0.5)),
        int(round(height - 1 - ((float(y) - oy) / res - 0.5))),
    )


def _nearest_mask_cell(mask, cell, max_radius_cells):
    x, y = cell
    h, w = mask.shape
    if 0 <= x < w and 0 <= y < h and mask[y, x]:
        return cell, 0
    best = None
    best_d2 = None
    rmax = int(max_radius_cells)
    for r in range(1, rmax + 1):
        y0 = max(0, y - r)
        y1 = min(h - 1, y + r)
        x0 = max(0, x - r)
        x1 = min(w - 1, x + r)
        for yy in range(y0, y1 + 1):
            for xx in (x0, x1):
                if mask[yy, xx]:
                    d2 = (xx - x) ** 2 + (yy - y) ** 2
                    if best is None or d2 < best_d2:
                        best = (xx, yy)
                        best_d2 = d2
        for xx in range(x0 + 1, x1):
            for yy in (y0, y1):
                if mask[yy, xx]:
                    d2 = (xx - x) ** 2 + (yy - y) ** 2
                    if best is None or d2 < best_d2:
                        best = (xx, yy)
                        best_d2 = d2
        if best is not None:
            return best, r
    return cell, None


def _path_cumulative(path, resolution):
    cumulative = [0.0]
    for i in range(1, len(path)):
        dx = abs(path[i][0] - path[i - 1][0])
        dy = abs(path[i][1] - path[i - 1][1])
        cumulative.append(
            cumulative[-1] + (1.41421356 if dx and dy else 1.0) * resolution)
    return cumulative


def _expand_edge(path, resolution, max_segment_length, prefer_mask,
                 prefer_window_m):
    if len(path) <= 2 or max_segment_length <= 0.0:
        return [path[0], path[-1]], [], 0
    cumulative = _path_cumulative(path, resolution)
    total = cumulative[-1]
    expanded = [path[0]]
    segment_lengths = []
    inserted = 0
    last_idx = 0
    window_cells = max(1, int(round(prefer_window_m / resolution)))
    while total - cumulative[last_idx] > max_segment_length:
        target = cumulative[last_idx] + max_segment_length
        target_idx = next(
            i for i in range(last_idx + 1, len(cumulative))
            if cumulative[i] >= target)
        target_idx = min(target_idx, len(path) - 2)
        lo = max(last_idx + 1, target_idx - window_cells)
        hi = min(len(path) - 2, target_idx + window_cells)
        candidates = [
            i for i in range(lo, hi + 1)
            if prefer_mask[path[i][1], path[i][0]]
        ]
        if candidates:
            split_idx = min(
                candidates, key=lambda i: abs(cumulative[i] - target))
        else:
            split_idx = target_idx
        if split_idx <= last_idx:
            split_idx = min(last_idx + 1, len(path) - 2)
        if split_idx <= last_idx:
            break
        cell = path[split_idx]
        if expanded[-1] != cell:
            expanded.append(cell)
            inserted += 1
            segment_lengths.append(cumulative[split_idx] -
                                   cumulative[last_idx])
        last_idx = split_idx
    if expanded[-1] != path[-1]:
        expanded.append(path[-1])
        segment_lengths.append(total - cumulative[last_idx])
    return expanded, segment_lengths, inserted


def _edge_stats(edge_index, start, goal, path, dist_cells, resolution):
    if not path:
        return {
            'edge': edge_index,
            'from_cell': start,
            'to_cell': goal,
            'path_cells': 0,
            'geodesic_m': float('inf'),
            'min_clearance_m': 0.0,
        }
    cumulative = _path_cumulative(path, resolution)
    min_clearance = min(dist_cells[y, x] * resolution for x, y in path)
    return {
        'edge': edge_index,
        'from_cell': start,
        'to_cell': goal,
        'path_cells': len(path),
        'geodesic_m': cumulative[-1],
        'min_clearance_m': min_clearance,
    }


def _write_report(prefix, summary, edge_rows, meta, height):
    out_dir = os.path.dirname(prefix)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    rows = []
    for row in edge_rows:
        out = dict(row)
        sx, sy = out.pop('from_cell')
        gx, gy = out.pop('to_cell')
        smx, smy = _cell_to_map((sx, sy), meta, height)
        gmx, gmy = _cell_to_map((gx, gy), meta, height)
        out.update({
            'from_x': round(smx, 3),
            'from_y': round(smy, 3),
            'to_x': round(gmx, 3),
            'to_y': round(gmy, 3),
            'geodesic_m': round(row['geodesic_m'], 3)
            if math.isfinite(row['geodesic_m']) else 'inf',
            'min_clearance_m': round(row['min_clearance_m'], 3),
        })
        rows.append(out)
    with open(prefix + '.json', 'w') as f:
        json.dump({'summary': summary, 'edges': rows},
                  f, indent=2, ensure_ascii=False)
        f.write('\n')
    keys = [
        'edge', 'from_x', 'from_y', 'to_x', 'to_y', 'path_cells',
        'geodesic_m', 'min_clearance_m', 'inserted',
    ]
    with open(prefix + '.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in keys})
    worst = sorted(
        rows,
        key=lambda r: float(r['geodesic_m'])
        if r['geodesic_m'] != 'inf' else float('inf'),
        reverse=True)
    lines = [
        '# Expanded waypoint route report',
        '',
        f"- input_waypoints: `{summary['input_waypoints']}`",
        f"- output_waypoints: `{summary['output_waypoints']}`",
        f"- inserted_waypoints: `{summary['inserted_waypoints']}`",
        f"- max_segment_length_m: `{summary['max_segment_length_m']}`",
        f"- max_output_segment_m: `{summary['max_output_segment_m']}`",
        '',
        '| edge | from | to | geodesic | min clearance | inserted |',
        '|---:|---|---|---:|---:|---:|',
    ]
    for row in worst[:20]:
        lines.append(
            f"| {row['edge']} | "
            f"({row['from_x']}, {row['from_y']}) | "
            f"({row['to_x']}, {row['to_y']}) | "
            f"{row['geodesic_m']} | {row['min_clearance_m']} | "
            f"{row.get('inserted', 0)} |")
    with open(prefix + '.md', 'w') as f:
        f.write('\n'.join(lines) + '\n')


def _write_png(path, img, route_cells, original_cells, meta, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    h, w = img.shape
    fig, ax = plt.subplots(figsize=(w / 50.0, h / 50.0), dpi=120)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper')
    if route_cells:
        xs = [c[0] for c in route_cells]
        ys = [c[1] for c in route_cells]
        ax.plot(xs, ys, '-', color='orange', linewidth=1.2, alpha=0.85)
        ax.plot(xs, ys, '.', color='royalblue', markersize=2.5)
    if original_cells:
        xs = [c[0] for c in original_cells]
        ys = [c[1] for c in original_cells]
        ax.plot(xs, ys, 'o', color='red', markersize=3.5,
                fillstyle='none', label='original')
    ax.set_title(title, fontsize=9)
    ax.set_xlabel('x [cell]')
    ax.set_ylabel('y [cell]')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True, help='saved map YAML')
    parser.add_argument('--waypoints', required=True, help='input waypoint YAML')
    parser.add_argument('--out', required=True, help='expanded waypoint YAML')
    parser.add_argument('--max-segment-length', type=float, default=1.5)
    parser.add_argument('--connect-clearance', type=float, default=0.35)
    parser.add_argument('--route-clearance', type=float, default=None)
    parser.add_argument('--clearance', type=float, default=0.75)
    parser.add_argument('--limit-radius', type=float, default=14.0)
    parser.add_argument('--limit-center-x', type=float, default=0.0)
    parser.add_argument('--limit-center-y', type=float, default=0.0)
    parser.add_argument('--snap-radius', type=float, default=0.5)
    parser.add_argument('--prefer-place-window', type=float, default=0.5)
    parser.add_argument('--report-prefix', default='')
    parser.add_argument('--no-png', action='store_true')
    args = parser.parse_args()
    if args.route_clearance is None:
        args.route_clearance = args.connect_clearance

    tools = _load_waypoint_tools()
    with open(args.map) as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(args.map), meta['image'])
    img = tools.load_pgm(img_path)
    h, w = img.shape
    res = float(meta['resolution'])
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.25))
    p = (255.0 - img.astype(np.float32)) / 255.0
    free = p <= free_thresh
    occ = p >= occ_thresh
    unknown = ~free & ~occ
    dist_cells = ndimage.distance_transform_edt(~(occ | unknown))

    limit_mask = np.ones_like(free, dtype=bool)
    if args.limit_radius > 0.0:
        grid_y, grid_x = np.indices((h, w))
        map_x = meta['origin'][0] + (grid_x + 0.5) * res
        map_y = meta['origin'][1] + (h - 1 - grid_y + 0.5) * res
        limit_mask = ((map_x - args.limit_center_x) ** 2
                      + (map_y - args.limit_center_y) ** 2
                      <= args.limit_radius ** 2)

    connectable = free & limit_mask & (
        dist_cells >= args.connect_clearance / res)
    route_passable = connectable & (dist_cells >= args.route_clearance / res)
    labels, n_lab = ndimage.label(route_passable)
    if n_lab == 0:
        raise RuntimeError('route_clearance leaves no connected component')
    sizes = ndimage.sum(np.ones_like(labels), labels, range(1, n_lab + 1))
    main_label = int(np.argmax(sizes)) + 1
    passable = labels == main_label
    prefer_mask = passable & (dist_cells >= args.clearance / res)

    with open(args.waypoints) as f:
        src = yaml.safe_load(f)
    raw_waypoints = src.get('waypoints', [])
    original_cells = [
        _map_to_cell(p[0], p[1], meta, h)
        for p in raw_waypoints
    ]
    snap_cells = []
    snap_radius_cells = max(1, int(math.ceil(args.snap_radius / res)))
    snapped = 0
    for cell in original_cells:
        new_cell, radius = _nearest_mask_cell(passable, cell, snap_radius_cells)
        if radius not in (0, None):
            snapped += 1
        snap_cells.append(new_cell)

    expanded = []
    edge_rows = []
    inserted_total = 0
    segment_lengths = []
    for edge_idx, (start, goal) in enumerate(
            zip(snap_cells[:-1], snap_cells[1:])):
        path = tools._shortest_path(passable, start, goal)
        row = _edge_stats(edge_idx, start, goal, path, dist_cells, res)
        if not path:
            cells = [start, goal]
            inserted = 0
            segs = [float('inf')]
        else:
            cells, segs, inserted = _expand_edge(
                path, res, args.max_segment_length,
                prefer_mask, args.prefer_place_window)
        row['inserted'] = inserted
        edge_rows.append(row)
        inserted_total += inserted
        segment_lengths.extend(segs)
        if not expanded:
            expanded.append(cells[0])
        for cell in cells[1:]:
            if expanded[-1] != cell:
                expanded.append(cell)

    out_waypoints = [
        [round(x, 3), round(y, 3)]
        for x, y in (_cell_to_map(cell, meta, h) for cell in expanded)
    ]
    out = {
        'map': os.path.basename(args.map),
        'frame_id': src.get('frame_id', 'map'),
        'source_waypoints': os.path.basename(args.waypoints),
        'route_expansion': {
            'max_segment_length_m': args.max_segment_length,
            'connect_clearance_m': args.connect_clearance,
            'route_clearance_m': args.route_clearance,
            'preferred_clearance_m': args.clearance,
            'snapped_waypoints': snapped,
        },
        'waypoints': out_waypoints,
    }
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'w') as f:
        yaml.safe_dump(out, f, default_flow_style=None, sort_keys=False)

    finite_segments = [s for s in segment_lengths if math.isfinite(s)]
    summary = {
        'map': args.map,
        'input_waypoints': len(raw_waypoints),
        'output_waypoints': len(out_waypoints),
        'inserted_waypoints': inserted_total,
        'snapped_waypoints': snapped,
        'max_segment_length_m': args.max_segment_length,
        'max_output_segment_m': round(max(finite_segments), 3)
        if finite_segments else None,
        'mean_output_segment_m': round(
            sum(finite_segments) / len(finite_segments), 3)
        if finite_segments else None,
    }
    print(f"expanded: {summary['input_waypoints']} -> "
          f"{summary['output_waypoints']} waypoints -> {args.out}")
    print(f"  inserted={inserted_total} snapped={snapped} "
          f"max_segment={summary['max_output_segment_m']}m "
          f"mean_segment={summary['mean_output_segment_m']}m")

    prefix = args.report_prefix
    if prefix:
        _write_report(prefix, summary, edge_rows, meta, h)
    if not args.no_png:
        png_path = os.path.splitext(args.out)[0] + '.png'
        _write_png(
            png_path, img, expanded, original_cells, meta,
            f"{os.path.basename(args.out)} {len(out_waypoints)}pts")
        print(f'  overlay -> {png_path}')


if __name__ == '__main__':
    main()
