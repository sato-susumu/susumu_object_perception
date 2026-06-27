#!/usr/bin/env python3
"""traffic light live stats JSON を名前付き summary に変換する。

既存の city_traffic_stats.json は TrafficSignalElement の enum 値をそのまま histogram
キーにしているため、レビュー時に "3" が GREEN だと毎回読み替える必要がある。このツールは
Autoware TrafficSignalElement の既知 enum 名へ展開し、Markdown と正規化 JSON を出す。
"""

import argparse
import json
import os


COLOR_NAMES = {
    0: 'UNKNOWN',
    1: 'RED',
    2: 'AMBER',
    3: 'GREEN',
    4: 'WHITE',
}

SHAPE_NAMES = {
    0: 'UNKNOWN',
    1: 'CIRCLE',
    2: 'LEFT_ARROW',
    3: 'RIGHT_ARROW',
    4: 'UP_ARROW',
    5: 'DOWN_ARROW',
    6: 'UP_LEFT_ARROW',
    7: 'UP_RIGHT_ARROW',
    8: 'DOWN_LEFT_ARROW',
    9: 'DOWN_RIGHT_ARROW',
    10: 'CROSS',
}

STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'SOLID_OFF',
    2: 'SOLID_ON',
    3: 'FLASHING',
}


def _named_hist(hist, names):
    out = {}
    for key, value in sorted(hist.items(), key=lambda kv: int(kv[0])):
        idx = int(key)
        out[names.get(idx, f'ENUM_{idx}')] = int(value)
    return out


def _int_key_hist(hist):
    return {
        str(int(key)): int(value)
        for key, value in sorted(hist.items(), key=lambda kv: int(kv[0]))
    }


def _named_nested_color_hist(hist):
    out = {}
    for signal_id, color_hist in sorted(hist.items(), key=lambda kv: int(kv[0])):
        out[str(int(signal_id))] = _named_hist(color_hist, COLOR_NAMES)
    return out


def summarize(stats, criteria):
    frames = int(stats.get('frames', 0) or 0)
    color_hist = _named_hist(stats.get('color_hist', {}), COLOR_NAMES)
    shape_hist = _named_hist(stats.get('shape_hist', {}), SHAPE_NAMES)
    status_hist = _named_hist(stats.get('status_hist', {}), STATUS_NAMES)
    signal_id_hist = _int_key_hist(stats.get('signal_id_hist', {}) or {})
    signal_id_color_hist = _named_nested_color_hist(
        stats.get('signal_id_color_hist', {}) or {})
    stop_like = color_hist.get('RED', 0) + color_hist.get('AMBER', 0)
    green = color_hist.get('GREEN', 0)
    detections = sum(color_hist.values())
    signal_observations = sum(signal_id_hist.values())
    top_signal_id = None
    top_signal_count = 0
    top_signal_ratio = None
    if signal_id_hist:
        top_signal_id, top_signal_count = max(
            signal_id_hist.items(), key=lambda kv: kv[1])
        top_signal_ratio = top_signal_count / max(signal_observations, 1)
    dominant_color = None
    if color_hist:
        dominant_color = max(color_hist.items(), key=lambda kv: kv[1])[0]
    confidence = stats.get('confidence', {}) or {}
    failures = []
    min_frames = int(criteria.get('min_frames', 1))
    min_detections = int(criteria.get('min_detections', 1))
    min_unique_signals = int(criteria.get('min_unique_signals', 1))
    max_unique_signal_ids = int(criteria.get('max_unique_signal_ids', 0) or 0)
    min_confidence_mean = float(criteria.get('min_confidence_mean', 0.0))
    min_top_signal_ratio = float(criteria.get('min_top_signal_ratio', 0.0))
    confidence_mean = confidence.get('mean')
    if frames < min_frames:
        failures.append(f'frames {frames} < {min_frames}')
    if detections < min_detections:
        failures.append(f'detections {detections} < {min_detections}')
    unique_ids = int(stats.get('unique_signal_ids') or 0)
    if unique_ids < min_unique_signals:
        failures.append(f'unique_signal_ids {unique_ids} < {min_unique_signals}')
    if max_unique_signal_ids > 0 and unique_ids > max_unique_signal_ids:
        failures.append(
            f'unique_signal_ids {unique_ids} > {max_unique_signal_ids}')
    if confidence_mean is not None and float(confidence_mean) < min_confidence_mean:
        failures.append(
            f'confidence.mean {float(confidence_mean):.3f} < {min_confidence_mean:.3f}')
    if min_top_signal_ratio > 0.0:
        if top_signal_ratio is None:
            failures.append('signal_id_hist missing')
        elif top_signal_ratio < min_top_signal_ratio:
            failures.append(
                f'top_signal_ratio {top_signal_ratio:.3f} < {min_top_signal_ratio:.3f}')
    return {
        'schema_version': 3,
        'validation_passed': not failures,
        'failures': failures,
        'summary': {
            'frames': frames,
            'detections': detections,
            'unique_signal_ids': unique_ids,
            'top_signal_id': top_signal_id,
            'top_signal_ratio': top_signal_ratio,
            'dominant_color': dominant_color,
            'confidence_mean': confidence_mean,
        },
        'world': stats.get('world'),
        'backend': stats.get('backend'),
        'omni_views': stats.get('omni_views'),
        'duration_sec': stats.get('duration_sec'),
        'frames': frames,
        'rate_hz': stats.get('rate_hz'),
        'unique_signal_ids': stats.get('unique_signal_ids'),
        'detections': detections,
        'signal_id_hist': signal_id_hist,
        'signal_id_color_hist_named': signal_id_color_hist,
        'top_signal_id': top_signal_id,
        'top_signal_count': top_signal_count,
        'top_signal_ratio': top_signal_ratio,
        'color_hist_named': color_hist,
        'shape_hist_named': shape_hist,
        'status_hist_named': status_hist,
        'dominant_color': dominant_color,
        'stop_like_count': stop_like,
        'green_count': green,
        'stop_like_ratio': stop_like / max(detections, 1),
        'green_ratio': green / max(detections, 1),
        'confidence': confidence,
        'criteria': criteria,
    }


def write_json(path, summary):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_md(path, stats_path, summary):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    conf = summary.get('confidence') or {}
    lines = [
        '# Traffic Light Stats Summary',
        '',
        f"- validation_passed: `{str(summary.get('validation_passed', False)).lower()}`",
        f"- schema_version: `{summary.get('schema_version')}`",
        f'- stats: `{stats_path}`',
        f"- world: `{summary.get('world')}`",
        f"- backend: `{summary.get('backend')}`",
        f"- frames: `{summary.get('frames')}`",
        f"- rate_hz: `{summary.get('rate_hz')}`",
        f"- unique_signal_ids: `{summary.get('unique_signal_ids')}`",
        f"- top_signal_id: `{summary.get('top_signal_id')}`",
        f"- top_signal_ratio: `{summary.get('top_signal_ratio')}`",
        f"- dominant_color: `{summary.get('dominant_color')}`",
        f"- stop_like_ratio: `{summary.get('stop_like_ratio'):.3f}`",
        f"- green_ratio: `{summary.get('green_ratio'):.3f}`",
        f"- confidence_mean: `{conf.get('mean')}`",
        '',
        '## Color Histogram',
        '',
        '| color | count |',
        '|---|---:|',
    ]
    for color, count in summary['color_hist_named'].items():
        lines.append(f'| {color} | {count} |')
    lines.extend([
        '',
        '## Shape Histogram',
        '',
        '| shape | count |',
        '|---|---:|',
    ])
    for shape, count in summary['shape_hist_named'].items():
        lines.append(f'| {shape} | {count} |')
    lines.extend([
        '',
        '## Status Histogram',
        '',
        '| status | count |',
        '|---|---:|',
    ])
    for status, count in summary['status_hist_named'].items():
        lines.append(f'| {status} | {count} |')
    lines.extend([
        '',
        '## Signal ID Histogram',
        '',
        '| signal_id | count | colors |',
        '|---:|---:|---|',
    ])
    for signal_id, count in summary.get('signal_id_hist', {}).items():
        colors = summary.get('signal_id_color_hist_named', {}).get(signal_id, {})
        color_text = ', '.join(f'{name}={value}' for name, value in colors.items())
        lines.append(f'| {signal_id} | {count} | {color_text} |')
    if summary.get('failures'):
        lines.extend(['', '## Failures'])
        lines.extend(f"- {failure}" for failure in summary['failures'])
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stats', required=True, help='traffic stats JSON path')
    ap.add_argument('--json-out', default='', help='normalized summary JSON')
    ap.add_argument('--md-out', default='', help='Markdown summary')
    ap.add_argument('--min-frames', type=int, default=1)
    ap.add_argument('--min-detections', type=int, default=1)
    ap.add_argument('--min-unique-signals', type=int, default=1)
    ap.add_argument('--max-unique-signal-ids', type=int, default=0,
                    help='0 disables the upper bound')
    ap.add_argument('--min-confidence-mean', type=float, default=0.0)
    ap.add_argument('--min-top-signal-ratio', type=float, default=0.0,
                    help='0 disables ID stability validation')
    ap.add_argument('--require-pass', action='store_true',
                    help='return non-zero when validation_passed is false')
    args = ap.parse_args()

    with open(args.stats) as f:
        stats = json.load(f)
    criteria = {
        'min_frames': args.min_frames,
        'min_detections': args.min_detections,
        'min_unique_signals': args.min_unique_signals,
        'max_unique_signal_ids': args.max_unique_signal_ids,
        'min_confidence_mean': args.min_confidence_mean,
        'min_top_signal_ratio': args.min_top_signal_ratio,
    }
    summary = summarize(stats, criteria)
    print(
        f"{summary.get('world')} backend={summary.get('backend')} "
        f"frames={summary.get('frames')} unique_ids={summary.get('unique_signal_ids')} "
        f"top_id={summary.get('top_signal_id')} "
        f"top_ratio={summary.get('top_signal_ratio')} "
        f"colors={summary['color_hist_named']} "
        f"confidence_mean={summary.get('confidence', {}).get('mean')}"
    )
    if args.json_out:
        write_json(args.json_out, summary)
        print(f'JSON: {args.json_out}')
    if args.md_out:
        write_md(args.md_out, args.stats, summary)
        print(f'MD: {args.md_out}')
    if summary['validation_passed']:
        print('validation_passed=true')
        return 0
    print('validation_passed=false')
    for failure in summary['failures']:
        print(f'- {failure}')
    return 2 if args.require_pass else 0


if __name__ == '__main__':
    raise SystemExit(main())
