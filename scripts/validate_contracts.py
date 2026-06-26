#!/usr/bin/env python3
"""outputs/ 配下の contracts ファイル配置の inventory レポート + 欠落検査。

iter110 で導入 (memory `contracts-png-inventory` を実コード化)。
docs/tasks/*.md で「最終成果物」 として明示されるファイル名パターンと
outputs/ 配下の実ファイルを照合し、 タスク別 coverage を報告する。

使い方:
  python3 scripts/validate_contracts.py
  ros2 run susumu_object_perception validate_contracts.py

挙動:
- outputs/ 全サブディレクトリを列挙
- 各サブディレクトリ内のファイル種別 (拡張子・命名パターン) で集計
- AGENTS.md / docs/tasks/README.md で期待されているサブディレクトリと比較
- waypoint_navigation の patrol_result.png は waypoint_generation 配下に置く規約も検査
"""
import os
import sys
from collections import defaultdict


PKG = 'susumu_object_perception'

# AGENTS.md タスク別正本表で示される 8 タスクと、 期待される outputs/ サブディレクトリ
# (None は「サブディレクトリ自体は別タスクと共用」 = waypoint_navigation の case)
TASK_EXPECTED_SUBDIR = {
    'mapping_indoor':        'mapping_indoor',
    'mapping_outdoor':       'mapping_outdoor',
    'waypoint_generation':   'waypoint_generation',
    'waypoint_navigation':   None,
    'recognition_object':    'recognition',
    'recognition_signal':    'traffic_light_recognition',
    'colorized_pointcloud':  'colorized_pointcloud',
    'extrinsic_calibration': 'extrinsic_calibration',
}


def resolve_outputs_dir():
    src_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'outputs')
    if os.path.isdir(src_dir):
        return src_dir
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(PKG), 'outputs')
    except Exception:
        return src_dir


def categorize_file(name):
    """ファイル名からカテゴリを分類 (拡張子と suffix の組合せ)."""
    base = os.path.basename(name).lower()
    if base.endswith('.png'):
        return 'png'
    if base.endswith('.pgm'):
        return 'pgm'
    if base.endswith('.yaml') and '_waypoints' in base:
        return 'waypoints'
    if base.endswith('.yaml'):
        return 'yaml'
    if base.endswith('.json'):
        return 'json'
    if base.endswith('.csv'):
        return 'csv'
    if base.endswith('.md'):
        return 'md'
    if base.endswith('.ply'):
        return 'ply'
    return 'other'


def inventory(outputs_dir):
    """サブディレクトリ別の inventory dict."""
    inv = {}
    if not os.path.isdir(outputs_dir):
        return inv
    for entry in sorted(os.listdir(outputs_dir)):
        sub = os.path.join(outputs_dir, entry)
        if not os.path.isdir(sub):
            continue
        cats = defaultdict(list)
        for f in sorted(os.listdir(sub)):
            full = os.path.join(sub, f)
            if not os.path.isfile(full):
                continue
            cats[categorize_file(f)].append(f)
        inv[entry] = dict(cats)
    return inv


def main():
    outputs_dir = resolve_outputs_dir()
    inv = inventory(outputs_dir)
    print(f'outputs/ scan: {outputs_dir}')
    print(f'subdirs found: {len(inv)}')
    print()

    # サブディレクトリ別表示
    total_png = 0
    for sub, cats in sorted(inv.items()):
        n_png = len(cats.get('png', []))
        total_png += n_png
        kinds = ', '.join(f'{k}={len(v)}' for k, v in sorted(cats.items()))
        print(f'  [{sub}] {kinds}')
    print()
    print(f'Total PNG files across outputs/: {total_png}')
    print()

    # AGENTS タスク表との照合
    print('Task vs subdirectory check (AGENTS task table):')
    issues = []
    found_subdirs = set(inv.keys())
    for task, expected in TASK_EXPECTED_SUBDIR.items():
        if expected is None:
            # waypoint_navigation の場合: waypoint_generation 配下に 4 ファイルセット期待
            # docs/tasks/waypoint_navigation.md「最終成果物」 = patrol_report.{json,csv,md} + patrol_result.png
            wg_files = inv.get('waypoint_generation', {})
            patrol_pngs = [f for f in wg_files.get('png', [])
                           if 'patrol_result' in f.lower()]
            patrol_jsons = [f for f in wg_files.get('json', [])
                            if 'patrol_report' in f.lower()]
            patrol_csvs = [f for f in wg_files.get('csv', [])
                           if 'patrol_report' in f.lower()]
            patrol_mds = [f for f in wg_files.get('md', [])
                          if 'patrol_report' in f.lower()]
            missing_kinds = []
            if not patrol_pngs:
                missing_kinds.append('patrol_result.png')
            if not patrol_jsons:
                missing_kinds.append('patrol_report.json')
            if not patrol_csvs:
                missing_kinds.append('patrol_report.csv')
            if not patrol_mds:
                missing_kinds.append('patrol_report.md')
            if not missing_kinds:
                status = (f'OK ({len(patrol_pngs)} PNG + {len(patrol_jsons)} JSON + '
                          f'{len(patrol_csvs)} CSV + {len(patrol_mds)} MD in waypoint_generation/)')
            else:
                status = f'MISSING ({", ".join(missing_kinds)} expected in waypoint_generation/)'
                issues.append(f'{task}: {status}')
            print(f'  {task}: {status}')
        else:
            if expected in found_subdirs:
                n_files = sum(len(v) for v in inv[expected].values())
                status = f'OK ({n_files} files)'
            else:
                status = f'MISSING (subdir outputs/{expected}/ not found)'
                issues.append(f'{task}: {status}')
            print(f'  {task}: {status}')

    # 期待されない subdir があれば孤立扱い
    expected_subdirs = {v for v in TASK_EXPECTED_SUBDIR.values() if v}
    orphan = found_subdirs - expected_subdirs
    if orphan:
        print()
        print(f'Orphan subdirectories (not in AGENTS task table): {sorted(orphan)}')

    print()
    if issues:
        print(f'Summary: {len(issues)} issues')
        return 1
    else:
        print(f'Summary: all tasks have expected subdirectory')
        return 0


if __name__ == '__main__':
    sys.exit(main())
