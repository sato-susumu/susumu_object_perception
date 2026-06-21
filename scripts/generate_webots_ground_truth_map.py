#!/usr/bin/env python3
"""Generate a ground-truth occupancy grid from a trimmed Webots world.

This is evaluation data, not an operational map-generation path. Outdoor task
maps must be produced by slam_toolbox from sensor data without reading the
world-derived ground truth. Use the generated grid only to inspect or score a
SLAM-produced map after mapping has finished.
"""

import argparse
import math
import os
import re
from pathlib import Path

import numpy as np
import yaml


RECT_PROTO_SIZES = {
    'Bench': (1.55, 0.55),
    'StoneBench': (1.70, 0.60),
    'OldBench': (1.55, 0.55),
    'PublicBin': (0.45, 0.45),
    'TrashBin': (0.45, 0.45),
    'Pergolas': (2.8, 2.8),
}

CIRCLE_PROTO_RADII = {
    'StreetLight': 0.18,
    'TrafficCone': 0.22,
    'Cypress': 0.55,
    'PalmTree': 0.65,
    'PottedTree': 0.55,
}

NODE_TYPES = tuple(sorted(
    {'Floor', 'PicketFence', 'PicketFenceWithDoor', *RECT_PROTO_SIZES,
     *CIRCLE_PROTO_RADII},
    key=len,
    reverse=True,
))


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
                return text[i:j + 1]
        j += 1
    return text[i:j]


def parse_xyz(body, key, default=(0.0, 0.0, 0.0)):
    m = re.search(
        rf'\b{key}\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',
        body)
    if not m:
        return default
    return tuple(float(m.group(i)) for i in range(1, 4))


def parse_size3(body):
    m = re.search(
        r'\bboundingObject\s+Box\s*\{\s*size\s+([-\d.eE+]+)\s+'
        r'([-\d.eE+]+)\s+([-\d.eE+]+)',
        body,
        re.S,
    )
    if not m:
        return None
    return tuple(float(m.group(i)) for i in range(1, 4))


def parse_size2(body):
    m = re.search(r'\bsize\s+([-\d.eE+]+)\s+([-\d.eE+]+)', body)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2))


def parse_yaw(body):
    m = re.search(
        r'\brotation\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+'
        r'([-\d.eE+]+)',
        body,
    )
    if not m:
        return 0.0
    z = float(m.group(3))
    return float(m.group(4)) if abs(z) > 0.5 else 0.0


def parse_name(body):
    m = re.search(r'\bname\s+"([^"]+)"', body)
    return m.group(1) if m else ''


def rotate_xy(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def to_map_xy(world_xy, robot_xy):
    return world_xy[0] - robot_xy[0], world_xy[1] - robot_xy[1]


def local_center_to_map(world_xy, robot_xy, yaw, local_xy):
    lx, ly = rotate_xy(local_xy[0], local_xy[1], yaw)
    return world_xy[0] + lx - robot_xy[0], world_xy[1] + ly - robot_xy[1]


def parse_robot_xy(text):
    m = re.search(
        r'TurtleBot\w*\s*\{[^{}]*?\btranslation\s+([-\d.eE+]+)\s+'
        r'([-\d.eE+]+)\s+([-\d.eE+]+)',
        text,
        re.S,
    )
    if not m:
        return 0.0, 0.0
    return float(m.group(1)), float(m.group(2))


def parse_world(path, min_obstacle_height, min_obstacle_top):
    text = Path(path).read_text()
    robot_xy = parse_robot_xy(text)
    floors = []
    rects = []
    circles = []

    type_re = '|'.join(re.escape(t) for t in NODE_TYPES)
    for m in re.finditer(rf'\b({type_re})\b\s*\{{', text):
        typ = m.group(1)
        body = node_body(text, m.end())
        wx, wy, _ = parse_xyz(body, 'translation')
        yaw = parse_yaw(body)
        center = to_map_xy((wx, wy), robot_xy)

        if typ == 'Floor':
            size = parse_size2(body)
            if size:
                floors.append((center, size, yaw, parse_name(body) or typ))
            continue

        if typ == 'PicketFence':
            seg = re.search(r'\bnumberOfSegments\s+(\d+)', body)
            n = int(seg.group(1)) if seg else 1
            size = (0.04, 1.7 * n)
            c = local_center_to_map((wx, wy), robot_xy, yaw, (0.0, -0.85 * n))
            rects.append((c, size, yaw, typ))
            continue

        if typ == 'PicketFenceWithDoor':
            boxes = [
                ((0.0, 0.8700000000000002), (0.04, 1.75), 0.0),
                ((0.0, 3.689999999999992), (0.04, 1.75), 0.0),
                ((-0.3975640202570147, 2.0529991572942476), (0.04, 1.13), 0.85),
            ]
            for local_center, size, local_yaw in boxes:
                c = local_center_to_map((wx, wy), robot_xy, yaw, local_center)
                rects.append((c, size, yaw + local_yaw, typ))
            continue

        if typ in RECT_PROTO_SIZES:
            rects.append((center, RECT_PROTO_SIZES[typ], yaw, typ))
            continue

        if typ in CIRCLE_PROTO_RADII:
            circles.append((center, CIRCLE_PROTO_RADII[typ], typ))

    for m in re.finditer(r'\bSolid\s*\{', text):
        body = node_body(text, m.end())
        size3 = parse_size3(body)
        if not size3:
            continue
        wx, wy, wz = parse_xyz(body, 'translation')
        sx, sy, sz = size3
        if sz < min_obstacle_height or wz + sz / 2.0 < min_obstacle_top:
            continue
        # Ignore synthetic helper links under the robot. They have no useful
        # static footprint in the world map.
        name = parse_name(body)
        if name in {'imu_link'}:
            continue
        rects.append((to_map_xy((wx, wy), robot_xy), (sx, sy), parse_yaw(body),
                      name or 'Solid'))

    return robot_xy, floors, rects, circles


def rect_bounds(center, size, yaw):
    sx, sy = size
    pts = []
    for lx, ly in [(-sx / 2, -sy / 2), (sx / 2, -sy / 2),
                   (sx / 2, sy / 2), (-sx / 2, sy / 2)]:
        rx, ry = rotate_xy(lx, ly, yaw)
        pts.append((center[0] + rx, center[1] + ry))
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def compute_bounds(floors, rects, circles, margin):
    boxes = [rect_bounds(c, s, y) for c, s, y, _ in floors]
    boxes.extend(rect_bounds(c, s, y) for c, s, y, _ in rects)
    boxes.extend((c[0] - r, c[1] - r, c[0] + r, c[1] + r)
                 for c, r, _ in circles)
    min_x = min(b[0] for b in boxes) - margin
    min_y = min(b[1] for b in boxes) - margin
    max_x = max(b[2] for b in boxes) + margin
    max_y = max(b[3] for b in boxes) + margin
    return min_x, min_y, max_x, max_y


def make_grid(bounds, resolution):
    min_x, min_y, max_x, max_y = bounds
    w = int(math.ceil((max_x - min_x) / resolution))
    h = int(math.ceil((max_y - min_y) / resolution))
    xs = min_x + (np.arange(w, dtype=np.float32) + 0.5) * resolution
    ys_bottom = min_y + (np.arange(h, dtype=np.float32) + 0.5) * resolution
    # Row 0 in the PGM is the top row, so y decreases with row index.
    ys = ys_bottom[::-1]
    xx, yy = np.meshgrid(xs, ys)
    return xx, yy, w, h


def rect_mask(xx, yy, center, size, yaw, pad):
    dx = xx - center[0]
    dy = yy - center[1]
    c = math.cos(yaw)
    s = math.sin(yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return ((np.abs(lx) <= size[0] / 2.0 + pad)
            & (np.abs(ly) <= size[1] / 2.0 + pad))


def circle_mask(xx, yy, center, radius, pad):
    return (xx - center[0]) ** 2 + (yy - center[1]) ** 2 <= (radius + pad) ** 2


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
    fig, ax = plt.subplots(figsize=(max(6, w / 90), max(6, h / 90)), dpi=120)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper')
    ax.set_title(os.path.basename(path))
    ax.set_axis_off()
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wbt', required=True, help='Input Webots .wbt file')
    ap.add_argument('--out', required=True,
                    help='Output ground-truth YAML path. PGM is written next to it.')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=0.5)
    ap.add_argument('--obstacle-padding', type=float, default=0.08)
    ap.add_argument('--min-obstacle-height', type=float, default=0.18)
    ap.add_argument('--min-obstacle-top', type=float, default=0.25)
    ap.add_argument('--preview', default='', help='Optional preview PNG path')
    args = ap.parse_args()

    robot_xy, floors, rects, circles = parse_world(
        args.wbt, args.min_obstacle_height, args.min_obstacle_top)
    if not floors:
        raise SystemExit('ERROR: no Floor nodes found; cannot define free area')

    bounds = compute_bounds(floors, rects, circles, args.margin)
    xx, yy, w, h = make_grid(bounds, args.resolution)
    # Use a middle gray for unknown so both Nav2 map_server and this repo's
    # offline tools keep it distinct from free. Map-saver style 205 is easy to
    # confuse with free when tools apply only occupancy thresholds.
    img = np.full((h, w), 127, dtype=np.uint8)

    free = np.zeros((h, w), dtype=bool)
    for center, size, yaw, _ in floors:
        free |= rect_mask(xx, yy, center, size, yaw, 0.0)
    img[free] = 254

    occupied = np.zeros((h, w), dtype=bool)
    for center, size, yaw, _ in rects:
        occupied |= rect_mask(xx, yy, center, size, yaw, args.obstacle_padding)
    for center, radius, _ in circles:
        occupied |= circle_mask(xx, yy, center, radius, args.obstacle_padding)
    img[occupied] = 0

    out_yaml = Path(args.out)
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    out_pgm = out_yaml.with_suffix('.pgm')
    write_pgm(out_pgm, img)
    meta = {
        'image': out_pgm.name,
        'mode': 'trinary',
        'resolution': float(args.resolution),
        'origin': [round(float(bounds[0]), 6), round(float(bounds[1]), 6), 0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    if args.preview:
        write_preview(args.preview, img)

    vals, counts = np.unique(img, return_counts=True)
    hist = {int(v): int(c) for v, c in zip(vals, counts)}
    print(f'generated {out_yaml} ({w}x{h}, res={args.resolution})')
    print(f'  robot_world=({robot_xy[0]:.3f},{robot_xy[1]:.3f}) '
          f'origin=({bounds[0]:.3f},{bounds[1]:.3f})')
    print(f'  floors={len(floors)} rect_obstacles={len(rects)} '
          f'circle_obstacles={len(circles)} hist={hist}')
    if args.preview:
        print(f'  preview={args.preview}')


if __name__ == '__main__':
    main()
