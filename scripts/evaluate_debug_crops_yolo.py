#!/usr/bin/env python3
"""Run controlled YOLO comparisons on object-classifier debug crops."""

import argparse
from collections import Counter
import csv
import json
import math
import os

import cv2
import numpy as np


def load_jsonl(path):
    rows = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def target_object_ids(recorder_json, target):
    if not recorder_json or not target:
        return set()
    with open(recorder_json, encoding='utf-8') as f:
        report = json.load(f)
    for row in report.get('targets', []):
        if row.get('target', {}).get('eid') == target:
            return set(row.get('object_ids', []))
    raise RuntimeError(f'target not found in recorder report: {target}')


def center_window_overlap(x1, y1, x2, y2, w, h, window_frac):
    frac = max(1e-3, min(1.0, float(window_frac)))
    ww = w * frac
    wh = h * frac
    cx = w * 0.5
    cy = h * 0.5
    wx1 = cx - ww * 0.5
    wx2 = cx + ww * 0.5
    wy1 = cy - wh * 0.5
    wy2 = cy + wh * 0.5
    ix1 = max(wx1, x1)
    iy1 = max(wy1, y1)
    ix2 = min(wx2, x2)
    iy2 = min(wy2, y2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    return ((ix2 - ix1) * (iy2 - iy1)) / max(ww * wh, 1e-6)


def mask_center_overlap(poly, w, h, window_frac):
    if poly is None or len(poly) < 3:
        return 0.0
    pts = np.asarray(poly, dtype=np.float32)
    frac = max(1e-3, min(1.0, float(window_frac)))
    ww = w * frac
    wh = h * frac
    cx = w * 0.5
    cy = h * 0.5
    xs = np.linspace(cx - ww * 0.5, cx + ww * 0.5, 5)
    ys = np.linspace(cy - wh * 0.5, cy + wh * 0.5, 5)
    inside = 0
    total = 0
    for yy in ys:
        for xx in xs:
            total += 1
            if cv2.pointPolygonTest(pts, (float(xx), float(yy)), False) >= 0:
                inside += 1
    return inside / float(max(total, 1))


def evaluate_weight(weight, crop_rows, args):
    import torch

    # Match object_classifier_node.py on PyTorch >= 2.6, where torch.load()
    # defaults to weights_only=True and older Ultralytics .pt checkpoints fail.
    # This script is an offline diagnostic and only loads user-specified weights.
    orig_load = torch.load

    def patched_load(*load_args, **load_kwargs):
        load_kwargs.setdefault('weights_only', False)
        return orig_load(*load_args, **load_kwargs)

    torch.load = patched_load
    from ultralytics import YOLO

    try:
        model = YOLO(weight)
    finally:
        torch.load = orig_load
    paths = [row['path'] for row in crop_rows]
    results = model.predict(
        paths,
        conf=args.predict_conf,
        imgsz=args.imgsz,
        batch=max(1, args.batch),
        verbose=False,
    )
    rows = []
    raw_classes = Counter()
    accepted_classes = Counter()
    selected_classes = Counter()
    refrigerator_raw = 0
    refrigerator_accepted = 0
    for crop_row, result in zip(crop_rows, results):
        img = cv2.imread(crop_row['path'])
        if img is None:
            continue
        h, w = img.shape[:2]
        names = result.names
        masks_xy = []
        if getattr(result, 'masks', None) is not None and result.masks is not None:
            masks_xy = list(getattr(result.masks, 'xy', []) or [])
        candidates = []
        accepted = []
        for bi, box in enumerate(result.boxes):
            conf = float(box.conf)
            x1, y1, x2, y2 = [
                float(v) for v in box.xyxy[0].detach().cpu().tolist()]
            name = str(names[int(box.cls)]).lower()
            raw_classes[name] += 1
            if name in ('refrigerator', 'fridge'):
                refrigerator_raw += 1
            area_frac = max(0.0, (x2 - x1) * (y2 - y1)) / float(w * h)
            bx = (x1 + x2) * 0.5
            by = (y1 + y2) * 0.5
            dx = abs(bx - w * 0.5) / max(w * 0.5, 1.0)
            dy = abs(by - h * 0.5) / max(h * 0.5, 1.0)
            contains_center = x1 <= w * 0.5 <= x2 and y1 <= h * 0.5 <= y2
            center_overlap = center_window_overlap(
                x1, y1, x2, y2, w, h, args.center_window_frac)
            mask_overlap = (
                mask_center_overlap(
                    masks_xy[bi], w, h, args.mask_center_window_frac)
                if bi < len(masks_xy) else 0.0)
            reason = 'accepted'
            if area_frac < args.min_box_area_frac:
                reason = 'box_area'
            elif args.center_tolerance_frac >= 0.0:
                near_center = (
                    dx <= args.center_tolerance_frac and
                    dy <= args.center_tolerance_frac)
                if not (contains_center or near_center):
                    reason = 'center_tolerance'
            if reason == 'accepted' and args.min_center_window_overlap > 0.0 and \
                    center_overlap < args.min_center_window_overlap:
                reason = 'center_window_overlap'
            if reason == 'accepted' and args.require_mask_center and \
                    mask_overlap < args.min_mask_center_overlap:
                reason = 'mask_center_overlap'
            if reason == 'accepted' and conf < args.accept_conf:
                reason = 'min_accept_conf'
            cand = {
                'class': name,
                'conf': conf,
                'reason': reason,
                'area_frac': area_frac,
                'center_dx': dx,
                'center_dy': dy,
                'center_overlap': center_overlap,
                'mask_overlap': mask_overlap,
                'bbox_xyxy': [x1, y1, x2, y2],
            }
            candidates.append(cand)
            if reason == 'accepted':
                accepted.append(cand)
                accepted_classes[name] += 1
                if name in ('refrigerator', 'fridge'):
                    refrigerator_accepted += 1
        selected = None
        if accepted:
            selected = max(accepted, key=lambda c: c['conf'])
            selected_classes[selected['class']] += 1
        rows.append({
            'crop': crop_row,
            'candidate_count': len(candidates),
            'accepted_count': len(accepted),
            'selected': selected,
            'candidates': candidates,
        })
    return {
        'weight': weight,
        'crop_count': len(crop_rows),
        'raw_class_counts': dict(raw_classes),
        'accepted_class_counts': dict(accepted_classes),
        'selected_class_counts': dict(selected_classes),
        'refrigerator_raw_count': refrigerator_raw,
        'refrigerator_accepted_count': refrigerator_accepted,
        'rows': rows,
    }


def write_reports(prefix, report):
    out_dir = os.path.dirname(prefix)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(prefix + '.json', 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write('\n')
    with open(prefix + '.csv', 'w', newline='', encoding='utf-8') as f:
        fields = [
            'weight', 'crop_count', 'refrigerator_raw',
            'refrigerator_accepted', 'raw_classes', 'accepted_classes',
            'selected_classes',
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in report['weights']:
            writer.writerow({
                'weight': row['weight'],
                'crop_count': row['crop_count'],
                'refrigerator_raw': row['refrigerator_raw_count'],
                'refrigerator_accepted': row['refrigerator_accepted_count'],
                'raw_classes': row['raw_class_counts'],
                'accepted_classes': row['accepted_class_counts'],
                'selected_classes': row['selected_class_counts'],
            })
    lines = [
        '# Debug Crop YOLO Comparison',
        '',
        '## Inputs',
        '',
        f"- metadata: `{report['inputs']['metadata']}`",
        f"- recorder_json: `{report['inputs'].get('recorder_json', '')}`",
        f"- target: `{report['inputs'].get('target', '')}`",
        f"- crop_count: `{report['summary']['crop_count']}`",
        '',
        '## Summary',
        '',
        '| weight | crops | refrigerator raw | refrigerator accepted | raw classes | accepted classes | selected classes |',
        '|---|---:|---:|---:|---|---|---|',
    ]
    for row in report['weights']:
        lines.append(
            f"| `{row['weight']}` | {row['crop_count']} "
            f"| {row['refrigerator_raw_count']} "
            f"| {row['refrigerator_accepted_count']} "
            f"| {row['raw_class_counts']} "
            f"| {row['accepted_class_counts']} "
            f"| {row['selected_class_counts']} |")
    with open(prefix + '.md', 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--metadata', required=True)
    parser.add_argument('--out-prefix', required=True)
    parser.add_argument('--recorder-json', default='')
    parser.add_argument('--target', default='')
    parser.add_argument('--object-id', action='append', default=[])
    parser.add_argument('--weights', action='append', required=True)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--predict-conf', type=float, default=0.01)
    parser.add_argument('--accept-conf', type=float, default=0.15)
    parser.add_argument('--min-box-area-frac', type=float, default=0.0005)
    parser.add_argument('--center-tolerance-frac', type=float, default=0.45)
    parser.add_argument('--center-window-frac', type=float, default=0.25)
    parser.add_argument('--min-center-window-overlap', type=float, default=0.0)
    parser.add_argument('--require-mask-center', action='store_true')
    parser.add_argument('--mask-center-window-frac', type=float, default=0.25)
    parser.add_argument('--min-mask-center-overlap', type=float, default=0.04)
    args = parser.parse_args()

    crop_rows = load_jsonl(args.metadata)
    ids = set(args.object_id)
    if args.recorder_json and args.target:
        ids |= target_object_ids(args.recorder_json, args.target)
    if ids:
        crop_rows = [row for row in crop_rows if row.get('object_id') in ids]
    crop_rows = [row for row in crop_rows if os.path.exists(row.get('path', ''))]
    if not crop_rows:
        raise RuntimeError('no matching crop rows')

    weights = [evaluate_weight(weight, crop_rows, args)
               for weight in args.weights]
    report = {
        'inputs': {
            'metadata': args.metadata,
            'recorder_json': args.recorder_json,
            'target': args.target,
            'object_ids': sorted(ids),
            'imgsz': args.imgsz,
            'predict_conf': args.predict_conf,
            'accept_conf': args.accept_conf,
            'require_mask_center': args.require_mask_center,
        },
        'summary': {
            'crop_count': len(crop_rows),
            'weights': args.weights,
        },
        'weights': weights,
    }
    write_reports(args.out_prefix, report)
    for row in weights:
        print(
            f"{row['weight']}: crops={row['crop_count']} "
            f"refrigerator_raw={row['refrigerator_raw_count']} "
            f"refrigerator_accepted={row['refrigerator_accepted_count']} "
            f"selected={row['selected_class_counts']}")


if __name__ == '__main__':
    main()
