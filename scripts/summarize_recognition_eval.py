#!/usr/bin/env python3
"""recognition eval JSON 群を横比較 summary にまとめる。

evaluate_recognition_vs_world.py は各条件ごとに詳細な JSON/CSV/MD/PNG を出す。このツールは
複数の eval JSON から summary 指標だけを抜き出し、採用判断用の比較表を JSON/Markdown で
保存する。認識本体には world 真値を使わず、既存の評価成果物だけを後処理する。
"""

import argparse
from collections import Counter
import hashlib
import json
import os


SUMMARY_KEYS = (
    'expected_count',
    'detection_count',
    'correct_count',
    'wrong_label_count',
    'extra_detection_count',
    'missed_without_near_detection_count',
    'class_aware_false_positive_count',
    'class_aware_false_negative_count',
    'expected_with_map_support_count',
    'missed_with_map_support_count',
    'precision',
    'recall',
    'f1',
)


def label_for(path, report):
    base = os.path.basename(path)
    stem = base[:-5] if base.endswith('.json') else base
    if stem.endswith('_recognition_eval_ignore_table_sofa'):
        return 'ignore_table_sofa'
    if stem.endswith('_recognition_eval'):
        return 'all_targets'
    inputs = report.get('inputs', {})
    ignored = inputs.get('ignored_types') or []
    if ignored:
        return 'ignore_' + '_'.join(str(v).lower() for v in ignored)
    return stem


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def _near_detection(row, key):
    near = row.get(key) or {}
    det = near.get('detection') or {}
    return {
        'distance_m': near.get('distance_m'),
        'within_match_distance': near.get('within_match_distance'),
        'label_match': near.get('label_match'),
        'detection_id': det.get('did'),
        'class_name': det.get('class_name'),
    }


def failure_details(report):
    missed = []
    for row in report.get('missed', []):
        missed.append({
            'expected_id': row.get('eid'),
            'expected_name': row.get('name'),
            'expected_type': row.get('wbt_type'),
            'accepted_labels': row.get('accepted_labels', []),
            'map_x': row.get('map_x'),
            'map_y': row.get('map_y'),
            'has_map_support': row.get('has_map_support'),
            'nearest_detection': _near_detection(row, 'nearest_detection'),
            'nearest_label_detection': _near_detection(
                row, 'nearest_label_detection'),
        })

    wrong = []
    for row in report.get('wrong_label', []):
        exp = row.get('expected') or {}
        det = row.get('detection') or {}
        wrong.append({
            'expected_id': exp.get('eid'),
            'expected_name': exp.get('name'),
            'expected_type': exp.get('wbt_type'),
            'accepted_labels': exp.get('accepted_labels', []),
            'detection_id': det.get('did'),
            'detected_class': det.get('class_name'),
            'distance_m': row.get('distance_m'),
        })

    extra = []
    for det in report.get('extra', []):
        extra.append({
            'detection_id': det.get('did'),
            'class_name': det.get('class_name'),
            'x': det.get('x'),
            'y': det.get('y'),
            'existence': det.get('existence'),
            'hits': det.get('hits'),
        })

    return {
        'missed': missed,
        'wrong_label': wrong,
        'extra': extra,
        'missed_type_hist': dict(Counter(
            item['expected_type'] for item in missed if item.get('expected_type'))),
        'wrong_detected_class_hist': dict(Counter(
            item['detected_class'] for item in wrong if item.get('detected_class'))),
        'extra_class_hist': dict(Counter(
            item['class_name'] for item in extra if item.get('class_name'))),
    }


def load_row(path):
    with open(path) as f:
        report = json.load(f)
    summary = report.get('summary', {})
    row = {
        'label': label_for(path, report),
        'path': path,
        'report_sha256': file_sha256(path),
        'failure_details': failure_details(report),
    }
    for key in SUMMARY_KEYS:
        if key in summary:
            row[key] = summary[key]
    return row


def build_report(rows, criteria):
    ranked = sorted(rows, key=lambda r: float(r.get('f1', 0.0)), reverse=True)
    best = ranked[0] if ranked else None
    failures = []
    min_best_f1 = float(criteria.get('min_best_f1', 0.0))
    required_best_label = str(criteria.get('required_best_label') or '')
    if best is None:
        failures.append('no reports')
    else:
        best_f1 = float(best.get('f1', 0.0))
        if best_f1 < min_best_f1:
            failures.append(f'best f1 {best_f1:.3f} < {min_best_f1:.3f}')
        if required_best_label and best.get('label') != required_best_label:
            failures.append(
                f"best label {best.get('label')!r} != {required_best_label!r}")
    return {
        'schema_version': 2,
        'validation_passed': not failures,
        'failures': failures,
        'criteria': criteria,
        'summary': {
            'report_count': len(rows),
            'best_label': None if best is None else best.get('label'),
            'best_f1': None if best is None else best.get('f1'),
            'best_precision': None if best is None else best.get('precision'),
            'best_recall': None if best is None else best.get('recall'),
        },
        'reports': rows,
        'best_by_f1': best,
    }


def write_json(path, report):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_md(path, report):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    rows = report['reports']
    ranked = sorted(rows, key=lambda r: float(r.get('f1', 0.0)), reverse=True)
    lines = [
        '# Recognition Eval Summary',
        '',
        f"- validation_passed: `{str(report['validation_passed']).lower()}`",
        f"- schema_version: `{report['schema_version']}`",
        f"- criteria: `{report['criteria']}`",
    ]
    if ranked:
        lines.append(f"- best_by_f1: `{ranked[0]['label']}` (`{float(ranked[0].get('f1', 0.0)):.3f}`)")
        lines.append('')
    lines.extend([
        '| label | expected | detections | correct | wrong | extra | precision | recall | F1 | FN | missed_map_support |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ])
    for row in rows:
        lines.append(
            f"| {row['label']} | {int(row.get('expected_count', 0))} | "
            f"{int(row.get('detection_count', 0))} | "
            f"{int(row.get('correct_count', 0))} | "
            f"{int(row.get('wrong_label_count', 0))} | "
            f"{int(row.get('extra_detection_count', 0))} | "
            f"{float(row.get('precision', 0.0)):.3f} | "
            f"{float(row.get('recall', 0.0)):.3f} | "
            f"{float(row.get('f1', 0.0)):.3f} | "
            f"{int(row.get('class_aware_false_negative_count', 0))} | "
            f"{int(row.get('missed_with_map_support_count', 0))} |"
        )
    lines.extend(['', '## Failure Details'])
    for row in rows:
        details = row.get('failure_details', {})
        lines.extend([
            '',
            f"### {row['label']}",
            '',
            f"- missed_type_hist: `{details.get('missed_type_hist', {})}`",
            f"- extra_class_hist: `{details.get('extra_class_hist', {})}`",
        ])
        missed = details.get('missed', [])
        if missed:
            lines.extend([
                '',
                '| missed | type | labels | map support | nearest class | nearest dist | nearest label dist |',
                '|---|---|---|---|---|---:|---:|',
            ])
            for item in missed:
                nearest = item.get('nearest_detection') or {}
                nearest_label = item.get('nearest_label_detection') or {}
                nearest_dist = nearest.get('distance_m')
                nearest_label_dist = nearest_label.get('distance_m')
                lines.append(
                    f"| {item.get('expected_name', '')} | "
                    f"{item.get('expected_type', '')} | "
                    f"{','.join(item.get('accepted_labels', []))} | "
                    f"{item.get('has_map_support', '')} | "
                    f"{nearest.get('class_name') or ''} | "
                    f"{'' if nearest_dist is None else f'{nearest_dist:.2f}'} | "
                    f"{'' if nearest_label_dist is None else f'{nearest_label_dist:.2f}'} |"
                )
        wrong = details.get('wrong_label', [])
        if wrong:
            lines.extend([
                '',
                '| wrong expected | type | detected | distance |',
                '|---|---|---|---:|',
            ])
            for item in wrong:
                dist = item.get('distance_m')
                lines.append(
                    f"| {item.get('expected_name', '')} | "
                    f"{item.get('expected_type', '')} | "
                    f"{item.get('detected_class', '')} | "
                    f"{'' if dist is None else f'{dist:.2f}'} |"
                )
        extra = details.get('extra', [])
        if extra:
            lines.extend([
                '',
                '| extra id | class | hits | existence |',
                '|---:|---|---:|---:|',
            ])
            for item in extra:
                existence = item.get('existence')
                lines.append(
                    f"| {item.get('detection_id', '')} | "
                    f"{item.get('class_name', '')} | "
                    f"{item.get('hits', '')} | "
                    f"{'' if existence is None else f'{existence:.3f}'} |"
                )
    if report.get('failures'):
        lines.extend(['', '## Failures'])
        lines.extend(f"- {failure}" for failure in report['failures'])
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('reports', nargs='+', help='*_recognition_eval*.json')
    ap.add_argument('--json-out', default='', help='summary JSON output')
    ap.add_argument('--md-out', default='', help='summary Markdown output')
    ap.add_argument('--min-best-f1', type=float, default=0.0)
    ap.add_argument('--require-best-label', default='')
    ap.add_argument('--require-pass', action='store_true',
                    help='return non-zero when validation_passed is false')
    args = ap.parse_args()

    rows = [load_row(p) for p in args.reports]
    criteria = {
        'min_best_f1': args.min_best_f1,
        'required_best_label': args.require_best_label,
    }
    report = build_report(rows, criteria)
    for row in rows:
        print(
            f"{row['label']}: expected={row.get('expected_count')} "
            f"detections={row.get('detection_count')} "
            f"correct={row.get('correct_count')} "
            f"precision={float(row.get('precision', 0.0)):.3f} "
            f"recall={float(row.get('recall', 0.0)):.3f} "
            f"f1={float(row.get('f1', 0.0)):.3f}"
        )
    if args.json_out:
        write_json(args.json_out, report)
        print(f'JSON: {args.json_out}')
    if args.md_out:
        write_md(args.md_out, report)
        print(f'MD: {args.md_out}')
    if report['validation_passed']:
        print('validation_passed=true')
        return 0
    print('validation_passed=false')
    for failure in report['failures']:
        print(f'- {failure}')
    return 2 if args.require_pass else 0


if __name__ == '__main__':
    raise SystemExit(main())
