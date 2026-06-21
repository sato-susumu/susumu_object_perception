#!/usr/bin/env python3
"""Convert a GLIM point cloud map into a Nav2-compatible 2D occupancy map.

Input is a point cloud exported from GLIM/offline_viewer (PLY) or an equivalent
PCD. Ground-band points become free cells, points above the ground band become
occupied cells. If a GLIM trajectory in TUM format is given, free-space rays are
also carved from trajectory poses toward obstacle points.
"""

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage
from scipy.spatial import cKDTree


PLY_TYPES = {
    'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1',
    'short': '<i2', 'int16': '<i2', 'ushort': '<u2', 'uint16': '<u2',
    'int': '<i4', 'int32': '<i4', 'uint': '<u4', 'uint32': '<u4',
    'float': '<f4', 'float32': '<f4', 'double': '<f8', 'float64': '<f8',
}

PCD_TYPES = {
    ('F', 4): '<f4', ('F', 8): '<f8',
    ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
    ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
}


def read_ply_xyz(path):
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise ValueError(f'{path}: not a PLY file')
        fmt = None
        vertex_count = None
        properties = []
        in_vertex = False
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f'{path}: PLY header has no end_header')
            line = raw.decode('ascii', errors='replace').strip()
            if line.startswith('format '):
                fmt = line.split()[1]
            elif line.startswith('element '):
                parts = line.split()
                in_vertex = parts[1] == 'vertex'
                if in_vertex:
                    vertex_count = int(parts[2])
            elif in_vertex and line.startswith('property '):
                parts = line.split()
                if parts[1] == 'list':
                    raise ValueError(f'{path}: list properties are not supported')
                properties.append((parts[2], parts[1]))
            elif line == 'end_header':
                break

        if fmt is None or vertex_count is None:
            raise ValueError(f'{path}: incomplete PLY header')
        names = [p[0] for p in properties]
        missing = {'x', 'y', 'z'} - set(names)
        if missing:
            raise ValueError(f'{path}: missing PLY properties {sorted(missing)}')

        if fmt == 'ascii':
            data = np.loadtxt(f, max_rows=vertex_count, dtype=np.float64)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            return data[:, [names.index('x'), names.index('y'), names.index('z')]].astype(np.float32)

        if fmt != 'binary_little_endian':
            raise ValueError(f'{path}: unsupported PLY format {fmt}')

        dtype = []
        for name, typ in properties:
            if typ not in PLY_TYPES:
                raise ValueError(f'{path}: unsupported PLY property type {typ}')
            dtype.append((name, PLY_TYPES[typ]))
        arr = np.frombuffer(f.read(np.dtype(dtype).itemsize * vertex_count),
                            dtype=np.dtype(dtype), count=vertex_count)
        return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)


def read_pcd_xyz(path):
    header = {}
    with open(path, 'rb') as f:
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f'{path}: PCD header has no DATA line')
            line = raw.decode('ascii', errors='replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *vals = line.split()
            header[key.upper()] = vals
            if key.upper() == 'DATA':
                break

        fields = header.get('FIELDS') or header.get('COLUMNS')
        if not fields:
            raise ValueError(f'{path}: PCD missing FIELDS')
        sizes = [int(v) for v in header.get('SIZE', [])]
        types = header.get('TYPE', [])
        counts = [int(v) for v in header.get('COUNT', ['1'] * len(fields))]
        points = int((header.get('POINTS') or [0])[0])
        if not points:
            width = int((header.get('WIDTH') or [0])[0])
            height = int((header.get('HEIGHT') or [1])[0])
            points = width * height
        missing = {'x', 'y', 'z'} - set(fields)
        if missing:
            raise ValueError(f'{path}: missing PCD fields {sorted(missing)}')

        if header['DATA'][0].lower() == 'ascii':
            data = np.loadtxt(f, max_rows=points, dtype=np.float64)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            return data[:, [fields.index('x'), fields.index('y'), fields.index('z')]].astype(np.float32)

        if header['DATA'][0].lower() != 'binary':
            raise ValueError(f'{path}: compressed PCD is not supported')

        dtype = []
        for field, size, typ, count in zip(fields, sizes, types, counts):
            np_typ = PCD_TYPES.get((typ, size))
            if np_typ is None:
                raise ValueError(f'{path}: unsupported PCD type {typ}{size}')
            if count == 1:
                dtype.append((field, np_typ))
            else:
                dtype.append((field, np_typ, (count,)))
        arr = np.frombuffer(f.read(np.dtype(dtype).itemsize * points),
                            dtype=np.dtype(dtype), count=points)
        return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)


def read_cloud_xyz(path):
    suffix = Path(path).suffix.lower()
    if suffix == '.ply':
        return read_ply_xyz(path)
    if suffix == '.pcd':
        return read_pcd_xyz(path)
    raise ValueError(f'{path}: expected .ply or .pcd')


def read_tum_trajectory(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = [float(v) for v in line.split()]
            if len(vals) < 4:
                continue
            # GLIM writes TUM rows: t x y z qx qy qz qw.
            rows.append(vals[1:4])
    if not rows:
        raise ValueError(f'{path}: no TUM trajectory poses')
    return np.asarray(rows, dtype=np.float32)


def parse_ground_z(value, points, percentile):
    if value.lower() == 'auto':
        return float(np.percentile(points[:, 2], percentile))
    return float(value)


def compute_bounds(points, margin):
    min_xy = np.min(points[:, :2], axis=0) - margin
    max_xy = np.max(points[:, :2], axis=0) + margin
    return float(min_xy[0]), float(min_xy[1]), float(max_xy[0]), float(max_xy[1])


def world_to_cell(x, y, origin_x, origin_y, resolution, height):
    mx = int(math.floor((x - origin_x) / resolution))
    my_bottom = int(math.floor((y - origin_y) / resolution))
    return mx, height - 1 - my_bottom


def points_to_cells(points, origin_x, origin_y, resolution, width, height):
    mx = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int32)
    my_bottom = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int32)
    my = height - 1 - my_bottom
    valid = (mx >= 0) & (mx < width) & (my >= 0) & (my < height)
    return mx[valid], my[valid]


def disk(radius_cells):
    if radius_cells <= 0:
        return np.ones((1, 1), dtype=bool)
    y, x = np.ogrid[-radius_cells:radius_cells + 1,
                    -radius_cells:radius_cells + 1]
    return (x * x + y * y) <= radius_cells * radius_cells


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def carve_free_rays(free, trajectory, obstacle_points, origin_x, origin_y,
                    resolution, max_range, max_points):
    if trajectory is None or obstacle_points.size == 0:
        return 0
    pts = obstacle_points
    if max_points > 0 and len(pts) > max_points:
        idx = np.linspace(0, len(pts) - 1, max_points).astype(np.int64)
        pts = pts[idx]
    tree = cKDTree(trajectory[:, :2])
    dists, pose_idx = tree.query(pts[:, :2], k=1)
    height, width = free.shape
    carved = 0
    for point, dist, idx in zip(pts, dists, pose_idx):
        if not np.isfinite(dist) or dist > max_range:
            continue
        p0 = trajectory[int(idx)]
        x0, y0 = world_to_cell(p0[0], p0[1], origin_x, origin_y,
                               resolution, height)
        x1, y1 = world_to_cell(point[0], point[1], origin_x, origin_y,
                               resolution, height)
        if not (0 <= x0 < width and 0 <= y0 < height
                and 0 <= x1 < width and 0 <= y1 < height):
            continue
        cells = list(bresenham(x0, y0, x1, y1))
        for x, y in cells[:-1]:
            if 0 <= x < width and 0 <= y < height and not free[y, x]:
                free[y, x] = True
                carved += 1
    return carved


def write_pgm(path, img):
    with open(path, 'wb') as f:
        h, w = img.shape
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(img.astype(np.uint8).tobytes())


def write_preview(path, img):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    h, w = img.shape
    fig, ax = plt.subplots(figsize=(max(6, w / 100), max(6, h / 100)), dpi=120)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper')
    ax.set_axis_off()
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


def build_map(args):
    points = read_cloud_xyz(args.cloud)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    if len(points) == 0:
        raise SystemExit('ERROR: cloud has no finite xyz points')
    if args.limit_radius > 0.0:
        r = np.hypot(points[:, 0], points[:, 1])
        points = points[r <= args.limit_radius]
    ground_z = parse_ground_z(args.ground_z, points, args.ground_percentile)
    rel_z = points[:, 2] - ground_z
    free_points = points[
        (rel_z >= args.free_z_min) & (rel_z <= args.free_z_max)]
    obstacle_points = points[
        (rel_z >= args.obstacle_z_min) & (rel_z <= args.obstacle_z_max)]
    if len(free_points) == 0:
        raise SystemExit('ERROR: no free/ground-band points after z filtering')
    if len(obstacle_points) == 0:
        raise SystemExit('ERROR: no obstacle points after z filtering')

    map_points = np.concatenate([free_points[:, :2], obstacle_points[:, :2]])
    bounds = compute_bounds(map_points, args.margin)
    min_x, min_y, max_x, max_y = bounds
    width = int(math.ceil((max_x - min_x) / args.resolution))
    height = int(math.ceil((max_y - min_y) / args.resolution))

    free = np.zeros((height, width), dtype=bool)
    occ = np.zeros((height, width), dtype=bool)
    mx, my = points_to_cells(free_points, min_x, min_y, args.resolution,
                             width, height)
    free[my, mx] = True
    mx, my = points_to_cells(obstacle_points, min_x, min_y, args.resolution,
                             width, height)
    occ[my, mx] = True

    trajectory = None
    carved = 0
    if args.trajectory:
        trajectory = read_tum_trajectory(args.trajectory)
        carved = carve_free_rays(
            free, trajectory, obstacle_points, min_x, min_y, args.resolution,
            args.raycast_max_range, args.raycast_max_points)

    free_radius = int(math.ceil(args.free_dilation / args.resolution))
    occ_radius = int(math.ceil(args.occupied_dilation / args.resolution))
    if free_radius > 0:
        free = ndimage.binary_dilation(free, structure=disk(free_radius))
    if occ_radius > 0:
        occ = ndimage.binary_dilation(occ, structure=disk(occ_radius))

    img = np.full((height, width), 127, dtype=np.uint8)
    img[free] = 254
    img[occ] = 0

    out_yaml = Path(args.out)
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    out_pgm = out_yaml.with_suffix('.pgm')
    write_pgm(out_pgm, img)
    meta = {
        'image': out_pgm.name,
        'mode': 'trinary',
        'resolution': float(args.resolution),
        'origin': [round(min_x, 6), round(min_y, 6), 0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    if args.preview:
        write_preview(args.preview, img)

    hist = {int(k): int(v) for k, v in zip(*np.unique(img, return_counts=True))}
    report = {
        'cloud': os.path.abspath(args.cloud),
        'trajectory': os.path.abspath(args.trajectory) if args.trajectory else '',
        'map_yaml': os.path.abspath(out_yaml),
        'map_pgm': os.path.abspath(out_pgm),
        'resolution': args.resolution,
        'origin': meta['origin'],
        'size_cells': [width, height],
        'size_m': [round(width * args.resolution, 3),
                   round(height * args.resolution, 3)],
        'ground_z': ground_z,
        'points_total': int(len(points)),
        'points_free_band': int(len(free_points)),
        'points_obstacle_band': int(len(obstacle_points)),
        'raycast_carved_cells': int(carved),
        'histogram': hist,
    }
    if args.report:
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        with open(args.report, 'w') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud', required=True, help='Input GLIM map cloud .ply/.pcd')
    ap.add_argument('--out', required=True, help='Output Nav2 map YAML')
    ap.add_argument('--trajectory', default='',
                    help='Optional GLIM traj_lidar.txt / odom_lidar.txt in TUM format')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=0.5)
    ap.add_argument('--ground-z', default='auto',
                    help='Ground z in cloud frame, or auto')
    ap.add_argument('--ground-percentile', type=float, default=5.0)
    ap.add_argument('--free-z-min', type=float, default=-0.12)
    ap.add_argument('--free-z-max', type=float, default=0.12)
    ap.add_argument('--obstacle-z-min', type=float, default=0.18)
    ap.add_argument('--obstacle-z-max', type=float, default=2.4)
    ap.add_argument('--free-dilation', type=float, default=0.08)
    ap.add_argument('--occupied-dilation', type=float, default=0.08)
    ap.add_argument('--limit-radius', type=float, default=0.0,
                    help='Keep cloud points within this radius from GLIM origin')
    ap.add_argument('--raycast-max-range', type=float, default=18.0)
    ap.add_argument('--raycast-max-points', type=int, default=50000)
    ap.add_argument('--preview', default='', help='Optional preview PNG')
    ap.add_argument('--report', default='', help='Optional JSON report')
    args = ap.parse_args()

    report = build_map(args)
    print(f'generated {args.out} ({report["size_cells"][0]}x{report["size_cells"][1]}, '
          f'res={args.resolution})')
    print(f'  ground_z={report["ground_z"]:.3f} points='
          f'{report["points_total"]} free={report["points_free_band"]} '
          f'occupied={report["points_obstacle_band"]} rays={report["raycast_carved_cells"]}')
    print(f'  hist={report["histogram"]}')


if __name__ == '__main__':
    main()
