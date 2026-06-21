#!/usr/bin/env python3
"""Render semantic recognition results on a saved occupancy grid map.

The input database is produced by object_memory_node.py. The output PNG is a
review artifact for the recognition task: it shows the 2D map with object
positions, IDs, class labels, existence probabilities, and hit counts.
"""

import argparse
import os
import sqlite3

import numpy as np
import yaml


def load_pgm(path):
    """Read P5(binary) / P2(ascii) PGM into a uint8 numpy array."""
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


CLASS_NORMALIZATION = {
    'sofa': 'couch',
    'table': 'dining table',
    'fridge': 'refrigerator',
}


def normalize_class_name(name):
    name = str(name or 'unknown').strip().lower().replace('_', ' ')
    while '  ' in name:
        name = name.replace('  ', ' ')
    return CLASS_NORMALIZATION.get(name, name)


def load_objects(db_path, min_existence, min_hits, ignore_classes=None):
    if not os.path.exists(db_path):
        raise FileNotFoundError(db_path)
    ignore_classes = {normalize_class_name(c) for c in (ignore_classes or [])}
    con = sqlite3.connect(db_path)
    con.row_factory = sqlite3.Row
    try:
        rows = con.execute(
            """
            SELECT id, class_name, x, y, z,
                   size_x, size_y, size_z, existence, hits, last_seen
            FROM objects
            WHERE existence >= ? AND hits >= ?
            ORDER BY class_name, id
            """,
            (min_existence, min_hits),
        ).fetchall()
    finally:
        con.close()
    objects = [dict(r) for r in rows]
    if ignore_classes:
        objects = [
            o for o in objects
            if normalize_class_name(o.get('class_name')) not in ignore_classes
        ]
    return objects


def map_to_cell(x, y, meta, width, height):
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    cx = (x - ox) / res - 0.5
    cy = height - 1 - ((y - oy) / res - 0.5)
    return cx, cy


def render(map_yaml, db_path, out_path, min_existence, min_hits, scale,
           ignore_classes=None):
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(map_yaml), meta['image'])
    if not os.path.exists(pgm_path):
        raise FileNotFoundError(
            'map image missing: %s referenced by %s. '
            'Run `ros2 run susumu_object_perception validate_map_assets.py %s` '
            'and regenerate the map image with nav2_map_server map_saver_cli.'
            % (pgm_path, map_yaml, map_yaml))
    img = load_pgm(pgm_path)
    h, w = img.shape
    objects = load_objects(
        db_path, min_existence, min_hits, ignore_classes=ignore_classes)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    # Keep small indoor maps readable by rendering them larger than their raw
    # pixel size. The user can override this for publication-style figures.
    if scale <= 0.0:
        scale = max(1.0, 520.0 / max(w, h))
    fig_w = max(7.0, (w * scale) / 80.0)
    fig_h = max(7.0, (h * scale) / 80.0)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=140)

    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper')
    ax.set_title(
        f'{os.path.basename(out_path)}  {len(objects)} objects  '
        f'min_exist={min_existence:.2f} min_hits={min_hits}',
        fontsize=10,
    )
    ax.set_xlabel('x [cell]')
    ax.set_ylabel('y [cell]')

    palette = [
        '#d62728', '#1f77b4', '#2ca02c', '#ff7f0e',
        '#9467bd', '#17becf', '#8c564b', '#e377c2',
    ]
    class_to_color = {}

    for rank, obj in enumerate(objects):
        cls = str(obj['class_name'] or 'unknown')
        if cls not in class_to_color:
            class_to_color[cls] = palette[len(class_to_color) % len(palette)]
        color = class_to_color[cls]
        cx, cy = map_to_cell(float(obj['x']), float(obj['y']), meta, w, h)

        # Draw an axis-aligned footprint estimate when dimensions are available.
        res = float(meta['resolution'])
        sx = max(0.25, float(obj.get('size_x') or 0.25)) / res
        sy = max(0.25, float(obj.get('size_y') or 0.25)) / res
        rect = Rectangle(
            (cx - sx / 2.0, cy - sy / 2.0),
            sx, sy,
            linewidth=1.2,
            edgecolor=color,
            facecolor=color,
            alpha=0.18,
            zorder=2,
        )
        ax.add_patch(rect)
        ax.plot(cx, cy, 'o', color=color, markersize=5.5, zorder=3)

        label = (
            f"#{obj['id']} {cls}\n"
            f"p={float(obj['existence']):.2f} hits={int(obj['hits'])}"
        )
        # Stagger labels so repeated detections in a compact map remain readable.
        offset = ((rank % 4) * 8 + 6, ((rank // 4) % 3) * 8 + 6)
        ax.annotate(
            label,
            (cx, cy),
            xytext=offset,
            textcoords='offset points',
            fontsize=8,
            color='black',
            bbox={
                'boxstyle': 'round,pad=0.25',
                'facecolor': 'white',
                'edgecolor': color,
                'alpha': 0.88,
            },
            arrowprops={
                'arrowstyle': '-',
                'color': color,
                'lw': 0.8,
                'alpha': 0.8,
            },
            zorder=4,
        )

    if class_to_color:
        handles = [
            plt.Line2D(
                [0], [0], marker='o', linestyle='',
                markerfacecolor=color, markeredgecolor=color,
                label=cls,
            )
            for cls, color in sorted(class_to_color.items())
        ]
        ax.legend(
            handles=handles,
            loc='upper center',
            bbox_to_anchor=(0.5, -0.10),
            ncol=min(4, len(handles)),
            fontsize=8,
            framealpha=0.9,
        )
    else:
        ax.text(
            0.5, -0.10,
            'No objects passed the filter',
            transform=ax.transAxes,
            ha='center',
            va='top',
            fontsize=9,
        )

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(
        f'rendered {len(objects)} objects from {db_path} on {map_yaml} '
        f'-> {out_path}'
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='map yaml path')
    ap.add_argument('--db', required=True, help='object_memory sqlite3 path')
    ap.add_argument('--out', required=True, help='output PNG path')
    ap.add_argument('--min-existence', type=float, default=0.3)
    ap.add_argument('--min-hits', type=int, default=1)
    ap.add_argument(
        '--scale',
        type=float,
        default=0.0,
        help='map enlargement factor. 0 chooses an automatic readable scale',
    )
    ap.add_argument(
        '--ignore-class', action='append', default=[],
        help='class_name to omit from the overlay; can be repeated')
    args = ap.parse_args()
    render(args.map, args.db, args.out,
           args.min_existence, args.min_hits, args.scale,
           ignore_classes=args.ignore_class)


if __name__ == '__main__':
    try:
        main()
    except FileNotFoundError as exc:
        raise SystemExit(str(exc))
