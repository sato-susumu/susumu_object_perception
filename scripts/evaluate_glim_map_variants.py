#!/usr/bin/env python3
"""Generate and evaluate GLIM-to-2D map variants from the same cloud.

This is an orchestration script for the outdoor GLIM-first mapping route:
1. Run glim_cloud_to_2d_map.py for each trajectory condition.
2. Run local map quality metrics.
3. Compare the generated map with the Webots world truth for evaluation only.
4. Write JSON/CSV/Markdown summaries so the next improvement cycle has a
   concrete baseline instead of relying on visual inspection.
"""

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import eval_map_quality  # noqa: E402
import glim_cloud_to_2d_map  # noqa: E402


SUMMARY_FIELDS = [
    'label',
    'status',
    'adopted_candidate',
    'trajectory',
    'map_yaml',
    'preview_png',
    'map_report',
    'vs_world_png',
    'vs_world_report',
    'vs_world_csv',
    'error',
    'size_cells',
    'size_m',
    'points_total',
    'points_free_band',
    'points_obstacle_band',
    'raycast_carved_cells',
    'free_cells',
    'occupied_cells',
    'unknown_cells',
    'wall_rate_pct',
    'main_component_rate_pct',
    'component_count',
    'near_ratio_inside',
    'fence_mean_coverage',
    'fence_near_ratio_inside',
    'fence_free_samples',
    'fence_unknown_samples',
    'fence_occupied_samples',
    'obstacle_mean_coverage',
    'building_mean_coverage',
]


def safe_label(label):
    value = re.sub(r'[^A-Za-z0-9_.-]+', '_', label.strip())
    value = value.strip('._-')
    if not value:
        raise ValueError(f'invalid empty label from {label!r}')
    return value


def parse_trajectory_spec(spec):
    if '=' not in spec:
        raise ValueError(
            f'expected LABEL=PATH for --trajectory, got {spec!r}')
    label, path = spec.split('=', 1)
    label = safe_label(label)
    if label == 'none':
        raise ValueError('label "none" is reserved for the no-trajectory variant')
    if not path:
        raise ValueError(f'{spec!r}: trajectory path is empty')
    return label, path


def variant_path(prefix, label, suffix):
    prefix = Path(prefix)
    return prefix.with_name(f'{prefix.name}_{label}{suffix}')


def prefix_yaml_path(prefix):
    path = Path(prefix)
    if path.suffix in ('.yaml', '.yml'):
        return path
    return path.with_suffix('.yaml')


def as_abs(path):
    return os.path.abspath(os.fspath(path)) if path else ''


def histogram_count(report, value, fallback=0):
    hist = report.get('histogram') or {}
    return int(hist.get(value, hist.get(str(value), fallback)))


def percent(value):
    return None if value is None else round(float(value), 6)


def kind_stats(vs_report, kind):
    return (vs_report.get('summary', {})
            .get('by_kind', {})
            .get(kind, {}))


def cell_count(stats, cell_class):
    return int((stats.get('cell_counts') or {}).get(cell_class, 0))


def run_vs_world(args, map_yaml, label):
    out_png = variant_path(args.out_prefix, label, '_vs_world.png')
    out_json = variant_path(args.out_prefix, label, '_vs_world.json')
    out_csv = variant_path(args.out_prefix, label, '_vs_world.csv')
    out_png.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(SCRIPT_DIR / 'check_map_vs_world.py'),
        '--wbt', args.wbt,
        '--map', str(map_yaml),
        '--out', str(out_png),
        '--report', str(out_json),
        '--object-report', str(out_csv),
        '--occupied-distance-threshold-m',
        str(args.occupied_distance_threshold_m),
    ]
    if args.robot is not None:
        cmd += ['--robot', str(args.robot[0]), str(args.robot[1])]
    if args.show_all_world:
        cmd.append('--show-all-world')

    completed = subprocess.run(
        cmd, check=False, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    if completed.returncode != 0:
        raise RuntimeError(
            'check_map_vs_world.py failed for '
            f'{label} (exit {completed.returncode})\n'
            f'stdout:\n{completed.stdout}\n'
            f'stderr:\n{completed.stderr}')

    with open(out_json) as f:
        report = json.load(f)
    return report, out_png, out_json, out_csv


def build_variant(args, label, trajectory):
    map_yaml = variant_path(args.out_prefix, label, '.yaml')
    preview_png = variant_path(args.out_prefix, label, '.png')
    map_report_json = variant_path(args.out_prefix, label, '.json')
    map_args = SimpleNamespace(
        cloud=args.cloud,
        out=str(map_yaml),
        trajectory=trajectory or '',
        resolution=args.resolution,
        margin=args.margin,
        ground_z=args.ground_z,
        ground_percentile=args.ground_percentile,
        free_z_min=args.free_z_min,
        free_z_max=args.free_z_max,
        obstacle_z_min=args.obstacle_z_min,
        obstacle_z_max=args.obstacle_z_max,
        free_dilation=args.free_dilation,
        occupied_dilation=args.occupied_dilation,
        limit_radius=args.limit_radius,
        raycast_max_range=args.raycast_max_range,
        raycast_max_points=args.raycast_max_points,
        preview=str(preview_png),
        report=str(map_report_json),
    )
    map_report = glim_cloud_to_2d_map.build_map(map_args)
    quality = eval_map_quality.evaluate(str(map_yaml), args.connect_clearance)
    vs_report, vs_png, vs_json, vs_csv = run_vs_world(args, map_yaml, label)

    summary = vs_report.get('summary', {})
    fence = kind_stats(vs_report, 'fence')
    obstacle = kind_stats(vs_report, 'obstacle')
    building = kind_stats(vs_report, 'building')

    return {
        'label': label,
        'status': 'ok',
        'adopted_candidate': False,
        'trajectory': as_abs(trajectory),
        'map_yaml': as_abs(map_yaml),
        'preview_png': as_abs(preview_png),
        'map_report': as_abs(map_report_json),
        'vs_world_png': as_abs(vs_png),
        'vs_world_report': as_abs(vs_json),
        'vs_world_csv': as_abs(vs_csv),
        'error': '',
        'size_cells': 'x'.join(str(v) for v in map_report['size_cells']),
        'size_m': 'x'.join(f'{float(v):.3f}' for v in map_report['size_m']),
        'points_total': int(map_report.get('points_total', 0)),
        'points_free_band': int(map_report.get('points_free_band', 0)),
        'points_obstacle_band': int(map_report.get('points_obstacle_band', 0)),
        'raycast_carved_cells': int(map_report.get('raycast_carved_cells', 0)),
        'free_cells': histogram_count(map_report, 254, quality['free']),
        'occupied_cells': histogram_count(map_report, 0, quality['occ']),
        'unknown_cells': histogram_count(map_report, 127, quality['unk']),
        'wall_rate_pct': round(float(quality['wall_rate']), 6),
        'main_component_rate_pct': round(float(quality['main_rate']), 6),
        'component_count': int(quality['n_components']),
        'near_ratio_inside': percent(summary.get('near_ratio_inside')),
        'fence_mean_coverage': percent(fence.get('mean_object_coverage_inside')),
        'fence_near_ratio_inside': percent(fence.get('near_ratio_inside')),
        'fence_free_samples': cell_count(fence, 'free'),
        'fence_unknown_samples': cell_count(fence, 'unknown'),
        'fence_occupied_samples': cell_count(fence, 'occupied'),
        'obstacle_mean_coverage': percent(
            obstacle.get('mean_object_coverage_inside')),
        'building_mean_coverage': percent(
            building.get('mean_object_coverage_inside')),
    }


def error_row(args, label, trajectory, exc):
    return {
        'label': label,
        'status': 'error',
        'adopted_candidate': False,
        'trajectory': as_abs(trajectory),
        'map_yaml': as_abs(variant_path(args.out_prefix, label, '.yaml')),
        'preview_png': as_abs(variant_path(args.out_prefix, label, '.png')),
        'map_report': as_abs(variant_path(args.out_prefix, label, '.json')),
        'vs_world_png': as_abs(variant_path(args.out_prefix, label, '_vs_world.png')),
        'vs_world_report': as_abs(variant_path(args.out_prefix, label, '_vs_world.json')),
        'vs_world_csv': as_abs(variant_path(args.out_prefix, label, '_vs_world.csv')),
        'error': str(exc),
    }


def metric(row, key):
    value = row.get(key)
    if value in (None, ''):
        return None
    return float(value)


def passes_against_baseline(row, baseline, max_loss):
    if row.get('status') != 'ok':
        return False
    for key in ('near_ratio_inside', 'fence_mean_coverage'):
        base = metric(baseline, key)
        value = metric(row, key)
        if base is not None and value is not None and value < base - max_loss:
            return False
    return True


def select_candidate(rows, max_loss):
    ok_rows = [r for r in rows if r.get('status') == 'ok']
    if not ok_rows:
        return None
    baseline = next((r for r in ok_rows if r['label'] == 'none'), ok_rows[0])
    candidates = [
        r for r in ok_rows
        if passes_against_baseline(r, baseline, max_loss)
    ]
    if not candidates:
        return baseline
    return min(
        candidates,
        key=lambda r: (
            int(r.get('unknown_cells') or 0),
            -float(r.get('near_ratio_inside') or 0.0),
            -float(r.get('fence_mean_coverage') or 0.0),
            r['label'],
        ))


def write_summary_json(path, rows, selected, args):
    data = {
        'cloud': as_abs(args.cloud),
        'wbt': as_abs(args.wbt),
        'selected_candidate': None if selected is None else selected['label'],
        'adopted_outputs': getattr(args, '_adopted_outputs', {}),
        'waypoints_output': getattr(args, '_waypoints_output', {}),
        'selection_rule': (
            'lowest unknown_cells among variants whose near_ratio_inside and '
            'fence_mean_coverage do not drop more than max_coverage_loss from '
            'the no-trajectory baseline'),
        'max_coverage_loss': args.max_coverage_loss,
        'rows': rows,
    }
    with open(path, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_summary_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, '') for field in SUMMARY_FIELDS})


def write_summary_md(path, rows, selected, args):
    with open(path, 'w') as f:
        f.write('# GLIM 2D map variant evaluation\n\n')
        f.write(f'- cloud: `{as_abs(args.cloud)}`\n')
        f.write(f'- wbt: `{as_abs(args.wbt)}`\n')
        if selected is None:
            f.write('- selected: none (all variants failed)\n')
        else:
            f.write(f'- selected: `{selected["label"]}`\n')
        adopted = getattr(args, '_adopted_outputs', {})
        if adopted:
            f.write(f'- adopted map: `{adopted.get("map_yaml", "")}`\n')
        waypoints = getattr(args, '_waypoints_output', {})
        if waypoints:
            f.write(f'- waypoints: `{waypoints.get("waypoints_yaml", "")}`\n')
        f.write(
            '- rule: choose the lowest `unknown_cells` variant that keeps '
            '`near_ratio_inside` and `fence_mean_coverage` within '
            f'{args.max_coverage_loss:.3f} of the no-trajectory baseline.\n\n')
        f.write('| label | adopted | status | rays | unknown | near | fence cov | fence free/unk/occ |\n')
        f.write('|---|---:|---|---:|---:|---:|---:|---|\n')
        for row in rows:
            adopted = 'yes' if row.get('adopted_candidate') else ''
            near = row.get('near_ratio_inside')
            fence_cov = row.get('fence_mean_coverage')
            f.write(
                f'| `{row["label"]}` | {adopted} | {row.get("status", "")} | '
                f'{row.get("raycast_carved_cells", "")} | '
                f'{row.get("unknown_cells", "")} | '
                f'{"" if near is None else near} | '
                f'{"" if fence_cov is None else fence_cov} | '
                f'{row.get("fence_free_samples", "")}/'
                f'{row.get("fence_unknown_samples", "")}/'
                f'{row.get("fence_occupied_samples", "")} |\n')
        f.write('\n')
        f.write('Per-variant PNG/JSON/CSV paths are recorded in the summary JSON/CSV.\n')


def print_summary(rows, selected):
    print('label,status,adopted,rays,unknown,near_ratio,fence_coverage')
    for row in rows:
        print(
            f'{row["label"]},{row.get("status", "")},'
            f'{1 if row.get("adopted_candidate") else 0},'
            f'{row.get("raycast_carved_cells", "")},'
            f'{row.get("unknown_cells", "")},'
            f'{row.get("near_ratio_inside", "")},'
            f'{row.get("fence_mean_coverage", "")}')
    if selected is not None:
        print(f'selected candidate: {selected["label"]}')


def copy_if_exists(src, dst):
    if not src:
        return ''
    src_path = Path(src)
    if not src_path.exists():
        return ''
    dst_path = Path(dst)
    dst_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src_path, dst_path)
    return as_abs(dst_path)


def promote_selected_map(selected, adopt_prefix):
    out_yaml = prefix_yaml_path(adopt_prefix)
    out_stem = out_yaml.with_suffix('')
    out_pgm = out_stem.with_suffix('.pgm')
    out_png = out_stem.with_suffix('.png')
    out_report = out_stem.with_suffix('.json')
    out_vs_png = out_stem.with_name(f'{out_stem.name}_vs_world.png')
    out_vs_json = out_stem.with_name(f'{out_stem.name}_vs_world.json')
    out_vs_csv = out_stem.with_name(f'{out_stem.name}_vs_world.csv')

    src_yaml = Path(selected['map_yaml'])
    with open(src_yaml) as f:
        meta = yaml.safe_load(f)
    src_pgm = src_yaml.parent / meta['image']
    if not src_pgm.exists():
        raise FileNotFoundError(f'selected map image not found: {src_pgm}')

    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src_pgm, out_pgm)
    meta['image'] = out_pgm.name
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)

    outputs = {
        'map_yaml': as_abs(out_yaml),
        'map_pgm': as_abs(out_pgm),
        'preview_png': copy_if_exists(selected.get('preview_png'), out_png),
        'map_report': copy_if_exists(selected.get('map_report'), out_report),
        'vs_world_png': copy_if_exists(selected.get('vs_world_png'), out_vs_png),
        'vs_world_report': copy_if_exists(
            selected.get('vs_world_report'), out_vs_json),
        'vs_world_csv': copy_if_exists(selected.get('vs_world_csv'), out_vs_csv),
    }
    return outputs


def run_waypoint_generation(args, map_yaml):
    out = Path(args.waypoints_out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(SCRIPT_DIR / 'generate_outdoor_waypoints.py'),
        '--map', str(map_yaml),
        '--out', str(out),
        '--spacing', str(args.waypoint_spacing),
        '--clearance', str(args.waypoint_clearance),
        '--connect-clearance', str(args.waypoint_connect_clearance),
        '--max-waypoints', str(args.waypoint_max_waypoints),
        '--max-segment-length', str(args.waypoint_max_segment_length),
        '--limit-radius', str(args.waypoint_limit_radius),
        '--limit-center-x', str(args.waypoint_limit_center_x),
        '--limit-center-y', str(args.waypoint_limit_center_y),
        '--object-viewpoints', str(args.waypoint_object_viewpoints),
    ]
    if args.waypoint_route_clearance is not None:
        cmd += ['--route-clearance', str(args.waypoint_route_clearance)]
    if args.waypoint_no_png:
        cmd.append('--no-png')
    completed = subprocess.run(
        cmd, check=False, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    if completed.returncode != 0:
        raise RuntimeError(
            'generate_outdoor_waypoints.py failed '
            f'(exit {completed.returncode})\n'
            f'stdout:\n{completed.stdout}\n'
            f'stderr:\n{completed.stderr}')
    if not out.exists():
        raise RuntimeError(
            'generate_outdoor_waypoints.py completed but did not create '
            f'{out}\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}')
    png = out.with_suffix('.png')
    return {
        'waypoints_yaml': as_abs(out),
        'waypoints_png': as_abs(png) if png.exists() else '',
        'stdout': completed.stdout,
        'stderr': completed.stderr,
    }


def build_trajectory_list(args):
    variants = []
    if not args.no_none:
        variants.append(('none', ''))
    for spec in args.trajectory:
        label, path = parse_trajectory_spec(spec)
        variants.append((label, path))
    labels = [label for label, _ in variants]
    if len(labels) != len(set(labels)):
        raise SystemExit(f'duplicate variant labels: {labels}')
    return variants


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud', required=True, help='Input GLIM map cloud .ply/.pcd')
    ap.add_argument('--wbt', required=True, help='Webots world for evaluation only')
    ap.add_argument('--out-prefix', required=True,
                    help='Output prefix, e.g. experiments/mapping_outdoor/village_square_eval')
    ap.add_argument('--trajectory', action='append', default=[],
                    help='Trajectory variant as LABEL=PATH. Repeatable.')
    ap.add_argument('--no-none', action='store_true',
                    help='Do not include the no-trajectory baseline')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=0.5)
    ap.add_argument('--ground-z', default='auto')
    ap.add_argument('--ground-percentile', type=float, default=5.0)
    ap.add_argument('--free-z-min', type=float, default=-0.12)
    ap.add_argument('--free-z-max', type=float, default=0.12)
    ap.add_argument('--obstacle-z-min', type=float, default=0.18)
    ap.add_argument('--obstacle-z-max', type=float, default=2.4)
    ap.add_argument('--free-dilation', type=float, default=0.08)
    ap.add_argument('--occupied-dilation', type=float, default=0.08)
    ap.add_argument('--limit-radius', type=float, default=0.0)
    ap.add_argument('--raycast-max-range', type=float, default=18.0)
    ap.add_argument('--raycast-max-points', type=int, default=50000)
    ap.add_argument('--connect-clearance', type=float, default=0.3)
    ap.add_argument('--occupied-distance-threshold-m', type=float, default=0.5)
    ap.add_argument('--max-coverage-loss', type=float, default=0.02,
                    help='Allowed drop from no-trajectory baseline for truth coverage')
    ap.add_argument('--adopt-prefix', default='',
                    help='Copy selected variant to this Nav2 map prefix/path')
    ap.add_argument('--waypoints-out', default='',
                    help='Generate outdoor waypoints from the selected/adopted map')
    ap.add_argument('--waypoint-spacing', type=float, default=4.0)
    ap.add_argument('--waypoint-clearance', type=float, default=0.75)
    ap.add_argument('--waypoint-connect-clearance', type=float, default=0.35)
    ap.add_argument('--waypoint-route-clearance', type=float, default=None,
                    help='experimental edge clearance [m]; default uses waypoint connect clearance')
    ap.add_argument('--waypoint-max-waypoints', type=int, default=40)
    ap.add_argument('--waypoint-max-segment-length', type=float, default=4.0)
    ap.add_argument('--waypoint-limit-radius', type=float, default=14.0)
    ap.add_argument('--waypoint-limit-center-x', type=float, default=0.0)
    ap.add_argument('--waypoint-limit-center-y', type=float, default=0.0)
    ap.add_argument('--waypoint-object-viewpoints', type=int, default=0)
    ap.add_argument('--waypoint-no-png', action='store_true')
    ap.add_argument('--robot', nargs=2, type=float, default=None,
                    help='Robot initial world x y for map/world alignment')
    ap.add_argument('--show-all-world', action='store_true')
    args = ap.parse_args()

    variants = build_trajectory_list(args)
    if not variants:
        raise SystemExit('no variants requested')

    rows = []
    for label, trajectory in variants:
        try:
            rows.append(build_variant(args, label, trajectory))
        except (Exception, SystemExit) as exc:
            rows.append(error_row(args, label, trajectory, exc))

    selected = select_candidate(rows, args.max_coverage_loss)
    if selected is not None:
        selected['adopted_candidate'] = True

    args._adopted_outputs = {}
    args._waypoints_output = {}
    if selected is not None and args.adopt_prefix:
        args._adopted_outputs = promote_selected_map(selected, args.adopt_prefix)
    if selected is not None and args.waypoints_out:
        waypoint_map = (
            args._adopted_outputs.get('map_yaml')
            if args._adopted_outputs else selected['map_yaml'])
        args._waypoints_output = run_waypoint_generation(args, waypoint_map)

    prefix = Path(args.out_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    write_summary_json(prefix.with_name(f'{prefix.name}_summary.json'),
                       rows, selected, args)
    write_summary_csv(prefix.with_name(f'{prefix.name}_summary.csv'), rows)
    write_summary_md(prefix.with_name(f'{prefix.name}_summary.md'),
                     rows, selected, args)
    print_summary(rows, selected)

    if selected is None:
        raise SystemExit('all variants failed')


if __name__ == '__main__':
    main()
