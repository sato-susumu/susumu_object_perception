#!/usr/bin/env python3
"""巡回結果レポート(JSON)を地図に重ねて「後から確認できる」可視化を作る。

waypoint_nav_node.py に report_prefix を渡すと、各ウェイポイントの結果が JSON/CSV/Markdown で
残る（reached/missed、reason、Nav2 feedback 由来の nav_recoveries・nav_distance_remaining_m など）。
このツールはその JSON と保存地図を読み、各点を

  reached かつ recovery 少             -> 緑（順調に到達）
  reached だが recovery 多(>=2)        -> 黄（到達したが苦戦）
  missed                              -> 赤（スキップ。理由を併記）

で色分けし、巡回順の経路線と一緒に PNG にする。巡回後に 1 枚で「どこで詰まった/スキップしたか」が
分かる。リアルタイム表示に頼らず、後から客観的にレビューするための成果物。

使い方:
  ros2 run susumu_object_perception visualize_patrol_result.py \
      --map outputs/mapping_indoor/indoor.yaml --report /tmp/indoor_patrol.json \
      --out /tmp/indoor_patrol_result.png
"""

import argparse
from collections import Counter
import json
import os

import yaml
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--report', required=True, help='waypoint_nav の結果 JSON')
    ap.add_argument('--out', default='/tmp/patrol_result.png')
    ap.add_argument('--json-out', default='',
                    help='optional JSON summary output path')
    ap.add_argument('--md-out', default='',
                    help='optional Markdown summary output path')
    ap.add_argument('--recovery-warn', type=int, default=2,
                    help='この回数以上の recovery を「苦戦(黄)」とする')
    ap.add_argument('--max-missed', type=int, default=0,
                    help='validation で許容する missed waypoint 数')
    ap.add_argument('--max-struggled', type=int, default=0,
                    help='validation で許容する struggled waypoint 数')
    ap.add_argument('--require-pass', action='store_true',
                    help='validation_passed=false なら非ゼロ終了')
    args = ap.parse_args()

    meta = yaml.safe_load(open(args.map))
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    pgm = os.path.join(os.path.dirname(args.map), meta['image'])
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape

    report = json.load(open(args.report))
    report_summary = report.get('summary', {}) if isinstance(report, dict) else {}
    results = report.get('results', report if isinstance(report, list) else [])
    # 最後の lap だけを見る（loop 時は最新周回）。
    if results:
        last_lap = max(r.get('lap', 0) for r in results)
        results = [r for r in results if r.get('lap', 0) == last_lap]

    def to_px(mx, my):
        return ((mx - ox) / res - 0.5, h - 1 - ((my - oy) / res - 0.5))

    # render_recognition_overlay.py と同パターンで自動拡大 (小さい地図でも文字を読める
    # サイズに)。 ユーザー指示「認識は地図上に結果表示、 小さければ自動拡大」 を巡回
    # 可視化にも適用 (iter40)。 既存固定スケール w/50, h/50 inch だと 200x100 cell で
    # 4x2 inch しか無く、 ラベル/legend が小さくなる。
    scale = max(1.0, 520.0 / max(w, h))
    fig_w = max(7.0, (w * scale) / 80.0)
    fig_h = max(7.0, (h * scale) / 80.0)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=120)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper')

    xs_line, ys_line = [], []
    n_reached = n_missed = n_struggle = 0
    missed_indices = []
    struggled_indices = []
    for r in results:
        px, py = to_px(r['x'], r['y'])
        xs_line.append(px)
        ys_line.append(py)
        rec = r.get('nav_recoveries', 0) or 0
        if r['result'] == 'missed':
            color = 'red'
            n_missed += 1
            missed_indices.append(r['index'])
        elif rec >= args.recovery_warn:
            color = 'gold'
            n_struggle += 1
            n_reached += 1
            struggled_indices.append(r['index'])
        else:
            color = 'limegreen'
            n_reached += 1
        ax.plot(px, py, 'o', color=color, markersize=7, zorder=3)
        label = str(r['index'])
        if r['result'] == 'missed':
            label += f"✗({r.get('reason', '')})"
        elif rec:
            label += f"(rec{rec})"
        ax.annotate(label, (px, py), color='black', fontsize=5,
                    xytext=(3, 3), textcoords='offset points', zorder=4,
                    bbox=dict(facecolor='white', alpha=0.65,
                              edgecolor='none', pad=0.3))
    ax.plot(xs_line, ys_line, '-', color='orange', linewidth=1.2,
            alpha=0.8, zorder=2)

    ax.plot([], [], 'o', color='limegreen', label='reached')
    ax.plot([], [], 'o', color='gold', label=f'reached but struggled(rec>={args.recovery_warn})')
    ax.plot([], [], 'o', color='red', label='missed (skipped)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fontsize=7)
    ax.set_title(f'patrol result: reached={n_reached} missed={n_missed} '
                 f'struggled={n_struggle} / {len(results)}')
    plt.tight_layout()
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    plt.savefig(args.out, dpi=100, bbox_inches='tight')
    print(f'reached={n_reached} missed={n_missed} struggled={n_struggle} '
          f'/ {len(results)}')
    print(f'saved {args.out}')
    reason_counts = Counter(str(r.get('reason', '')) for r in results)
    recovery_values = [int(r.get('nav_recoveries', 0) or 0) for r in results]
    duration_values = [
        float(r.get('duration_sec'))
        for r in results
        if r.get('duration_sec') is not None
    ]
    criteria = {
        'max_missed': args.max_missed,
        'max_struggled': args.max_struggled,
        'recovery_warn': args.recovery_warn,
    }
    failures = []
    if n_missed > args.max_missed:
        failures.append(f'missed {n_missed} > {args.max_missed}')
    if n_struggle > args.max_struggled:
        failures.append(f'struggled {n_struggle} > {args.max_struggled}')
    summary = {
        'schema_version': 3,
        'validation_passed': not failures,
        'criteria': criteria,
        'failures': failures,
        'map': args.map,
        'report': args.report,
        'out': args.out,
        'lap': max((r.get('lap', 0) for r in results), default=0),
        'total': len(results),
        'reached': n_reached,
        'missed': n_missed,
        'struggled': n_struggle,
        'completion_rate': n_reached / max(len(results), 1),
        'clean_completion_rate': (
            (n_reached - n_struggle) / max(len(results), 1)),
        'recovery_warn': args.recovery_warn,
        'missed_indices': missed_indices,
        'struggled_indices': struggled_indices,
        'reason_counts': dict(reason_counts),
        'max_nav_recoveries': max(recovery_values) if recovery_values else 0,
        'total_nav_recoveries': int(sum(recovery_values)),
        'safe_pose_recovery_count': int(
            report_summary.get('safe_pose_recovery_count') or 0),
        'step_event_count': int(
            report_summary.get('step_event_count') or len(
                report.get('step_events', []) if isinstance(report, dict) else [])),
        'duration_mean_sec': (
            sum(duration_values) / len(duration_values)
            if duration_values else None),
        'duration_max_sec': max(duration_values) if duration_values else None,
    }
    if args.json_out:
        json_dir = os.path.dirname(args.json_out)
        if json_dir:
            os.makedirs(json_dir, exist_ok=True)
        with open(args.json_out, 'w') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
            f.write('\n')
        print(f'saved {args.json_out}')
    if args.md_out:
        md_dir = os.path.dirname(args.md_out)
        if md_dir:
            os.makedirs(md_dir, exist_ok=True)
        lines = [
            '# Patrol Result Summary',
            '',
            f"- validation_passed: `{str(summary['validation_passed']).lower()}`",
            f"- schema_version: `{summary['schema_version']}`",
            f"- report: `{args.report}`",
            f"- criteria: `{criteria}`",
            f"- reached: `{n_reached}/{len(results)}`",
            f"- completion_rate: `{summary['completion_rate']:.3f}`",
            f"- clean_completion_rate: `{summary['clean_completion_rate']:.3f}`",
            f"- missed: `{missed_indices}`",
            f"- struggled: `{struggled_indices}`",
            f"- total_nav_recoveries: `{summary['total_nav_recoveries']}`",
            f"- max_nav_recoveries: `{summary['max_nav_recoveries']}`",
            f"- safe_pose_recovery_count: `{summary['safe_pose_recovery_count']}`",
            f"- step_event_count: `{summary['step_event_count']}`",
            f"- duration_mean_sec: `{summary['duration_mean_sec']}`",
            f"- duration_max_sec: `{summary['duration_max_sec']}`",
            f"- png: `{args.out}`",
            '',
            '| reason | count |',
            '|---|---:|',
        ]
        for reason, count in sorted(reason_counts.items()):
            lines.append(f'| {reason} | {count} |')
        if failures:
            lines.extend(['', '## Failures'])
            lines.extend(f'- {failure}' for failure in failures)
        with open(args.md_out, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print(f'saved {args.md_out}')
    # missed/struggled の点を一覧（後から確認用）。
    for r in results:
        rec = r.get('nav_recoveries', 0) or 0
        if r['result'] == 'missed' or rec >= args.recovery_warn:
            print(f"  #{r['index']} ({r['x']:.1f},{r['y']:.1f}) {r['result']} "
                  f"reason={r.get('reason','')} recoveries={rec} "
                  f"dist_remaining={r.get('nav_distance_remaining_m')}")
    if summary['validation_passed']:
        print('validation_passed=true')
        return 0
    print('validation_passed=false')
    for failure in failures:
        print(f'- {failure}')
    return 2 if args.require_pass else 0


if __name__ == '__main__':
    raise SystemExit(main())
