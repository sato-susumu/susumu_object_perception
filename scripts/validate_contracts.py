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
import json
import hashlib
import math
from collections import defaultdict
import yaml


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


def read_json(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def read_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def resolve_repo_path(path, outputs_dir):
    if os.path.isabs(path):
        return path
    repo_root = os.path.dirname(outputs_dir)
    return os.path.join(repo_root, path)


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
            patrol_report_jsons = [f for f in wg_files.get('json', [])
                                   if 'patrol_report' in f.lower()]
            patrol_result_jsons = [f for f in wg_files.get('json', [])
                                   if 'patrol_result' in f.lower()]
            patrol_csvs = [f for f in wg_files.get('csv', [])
                           if 'patrol_report' in f.lower()]
            patrol_report_mds = [f for f in wg_files.get('md', [])
                                 if 'patrol_report' in f.lower()]
            patrol_result_mds = [f for f in wg_files.get('md', [])
                                 if 'patrol_result' in f.lower()]
            missing_kinds = []
            if not patrol_pngs:
                missing_kinds.append('patrol_result.png')
            if not patrol_report_jsons:
                missing_kinds.append('patrol_report.json')
            if not patrol_result_jsons:
                missing_kinds.append('patrol_result.json')
            if not patrol_csvs:
                missing_kinds.append('patrol_report.csv')
            if not patrol_report_mds:
                missing_kinds.append('patrol_report.md')
            if not patrol_result_mds:
                missing_kinds.append('patrol_result.md')
            if not missing_kinds:
                status = (
                    f'OK ({len(patrol_pngs)} result PNG + '
                    f'{len(patrol_result_jsons)} result JSON + '
                    f'{len(patrol_result_mds)} result MD + '
                    f'{len(patrol_report_jsons)} report JSON + '
                    f'{len(patrol_csvs)} report CSV + '
                    f'{len(patrol_report_mds)} report MD in waypoint_generation/)'
                )
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

    # タスク固有の契約名検査。サブディレクトリが存在するだけでは、後段が読む
    # 固定名ファイルの欠落を見逃すため、主要 contracts は名前まで確認する。
    print()
    print('Named contract checks:')
    mapping_files = {
        f for files in inv.get('mapping_indoor', {}).values() for f in files
    }
    mapping_missing = []
    for world in ('indoor', 'break_room', 'cafe'):
        for suffix in ('.yaml', '.pgm', '_eval.png', '_eval.json', '_eval.md'):
            name = f'{world}{suffix}'
            if name not in mapping_files:
                mapping_missing.append(name)
        if world in ('indoor', 'break_room'):
            for suffix in ('_vs_world.png', '_vs_world.json', '_vs_world.csv'):
                name = f'{world}{suffix}'
                if name not in mapping_files:
                    mapping_missing.append(name)
    for name in (
        'mapping_indoor_quality_summary.json',
        'mapping_indoor_quality_summary.md',
        'mapping_indoor_assets_summary.json',
        'mapping_indoor_assets_summary.md',
    ):
        if name not in mapping_files:
            mapping_missing.append(name)
    mapping_failed = []
    mapping_dir = os.path.join(outputs_dir, 'mapping_indoor')
    for world in ('indoor', 'break_room', 'cafe'):
        eval_json = os.path.join(mapping_dir, f'{world}_eval.json')
        if not os.path.isfile(eval_json):
            continue
        try:
            data = read_json(eval_json)
        except Exception as e:
            mapping_failed.append(f'{world}_eval.json unreadable: {e}')
            continue
        if data.get('validation_passed') is not True:
            mapping_failed.append(
                f"{world}_eval.json validation_passed="
                f"{data.get('validation_passed')!r}")
        for row in data.get('maps', []):
            map_path = resolve_repo_path(row.get('map', ''), outputs_dir)
            image_path = resolve_repo_path(row.get('image', ''), outputs_dir)
            if not row.get('map_sha256'):
                mapping_failed.append(f'{world}_eval.json missing map_sha256')
            elif os.path.isfile(map_path) and file_sha256(map_path) != row.get('map_sha256'):
                mapping_failed.append(f'{world}_eval.json map_sha256 mismatch')
            if not row.get('image_sha256'):
                mapping_failed.append(f'{world}_eval.json missing image_sha256')
            elif os.path.isfile(image_path) and file_sha256(image_path) != row.get('image_sha256'):
                mapping_failed.append(f'{world}_eval.json image_sha256 mismatch')
    for world in ('indoor', 'break_room'):
        vs_json = os.path.join(mapping_dir, f'{world}_vs_world.json')
        if not os.path.isfile(vs_json):
            continue
        try:
            data = read_json(vs_json)
        except Exception as e:
            mapping_failed.append(f'{world}_vs_world.json unreadable: {e}')
            continue
        if int(data.get('schema_version') or 0) < 3:
            mapping_failed.append(f'{world}_vs_world.json schema_version < 3')
        if data.get('validation_passed') is not True:
            mapping_failed.append(
                f"{world}_vs_world.json validation_passed="
                f"{data.get('validation_passed')!r}")
        if not isinstance(data.get('summary'), dict):
            mapping_failed.append(f'{world}_vs_world.json missing summary')
        hashes = data.get('inputs_hash')
        inputs = data.get('inputs') or {}
        if not isinstance(hashes, dict):
            mapping_failed.append(f'{world}_vs_world.json missing inputs_hash')
        else:
            for input_key, hash_key in (
                ('wbt', 'wbt_sha256'),
                ('map', 'map_sha256'),
                ('map_image', 'map_image_sha256'),
            ):
                input_path = inputs.get(input_key)
                recorded_hash = hashes.get(hash_key)
                if not input_path:
                    mapping_failed.append(
                        f'{world}_vs_world.json missing inputs.{input_key}')
                    continue
                if not recorded_hash:
                    mapping_failed.append(
                        f'{world}_vs_world.json missing inputs_hash.{hash_key}')
                    continue
                path = resolve_repo_path(input_path, outputs_dir)
                if not os.path.isfile(path):
                    mapping_failed.append(
                        f'{world}_vs_world.json inputs.{input_key} missing source file')
                elif recorded_hash != file_sha256(path):
                    mapping_failed.append(
                        f'{world}_vs_world.json inputs_hash.{hash_key} mismatch')
    for name in (
        'mapping_indoor_quality_summary.json',
        'mapping_indoor_assets_summary.json',
    ):
        path = os.path.join(mapping_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            data = read_json(path)
        except Exception as e:
            mapping_failed.append(f'{name} unreadable: {e}')
            continue
        if data.get('validation_passed') is not True:
            mapping_failed.append(
                f"{name} validation_passed={data.get('validation_passed')!r}")
        if not isinstance(data.get('summary'), dict):
            mapping_failed.append(f'{name} missing summary')
        if name == 'mapping_indoor_quality_summary.json':
            for row in data.get('maps', []):
                map_path = resolve_repo_path(row.get('map', ''), outputs_dir)
                image_path = resolve_repo_path(row.get('image', ''), outputs_dir)
                if not row.get('map_sha256'):
                    mapping_failed.append(f'{name} missing map_sha256')
                elif os.path.isfile(map_path) and file_sha256(map_path) != row.get('map_sha256'):
                    mapping_failed.append(f'{name} map_sha256 mismatch')
                if not row.get('image_sha256'):
                    mapping_failed.append(f'{name} missing image_sha256')
                elif os.path.isfile(image_path) and file_sha256(image_path) != row.get('image_sha256'):
                    mapping_failed.append(f'{name} image_sha256 mismatch')
        if name == 'mapping_indoor_assets_summary.json':
            if int(data.get('schema_version') or 0) < 3:
                mapping_failed.append(f'{name} schema_version < 3')
            for row in data.get('results', []):
                if row.get('kind') != 'occupancy_map':
                    continue
                map_path = resolve_repo_path(row.get('map', ''), outputs_dir)
                image_path = resolve_repo_path(
                    row.get('image_path') or row.get('image', ''), outputs_dir)
                if not row.get('map_sha256'):
                    mapping_failed.append(f'{name} missing map_sha256')
                elif os.path.isfile(map_path) and file_sha256(map_path) != row.get('map_sha256'):
                    mapping_failed.append(f'{name} map_sha256 mismatch')
                if not row.get('image_sha256'):
                    mapping_failed.append(f'{name} missing image_sha256')
                elif os.path.isfile(image_path) and file_sha256(image_path) != row.get('image_sha256'):
                    mapping_failed.append(f'{name} image_sha256 mismatch')
    if mapping_missing or mapping_failed:
        details = []
        if mapping_missing:
            details.append(f'missing {", ".join(mapping_missing)}')
        if mapping_failed:
            details.append(f'failed {", ".join(mapping_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'mapping_indoor named contracts: {status}')
    else:
        status = 'OK (maps + passing per-map evals + passing set/assets summaries; passing Webots vs_world)'
    print(f'  mapping_indoor named contracts: {status}')

    mapping_outdoor_files = {
        f for files in inv.get('mapping_outdoor', {}).values() for f in files
    }
    mapping_outdoor_missing = []
    for world in ('village_square_trimmed', 'village_park_trimmed'):
        for suffix in ('_gt.yaml', '_gt.pgm', '_gt_preview.png'):
            name = f'{world}{suffix}'
            if name not in mapping_outdoor_files:
                mapping_outdoor_missing.append(name)
    for name in (
        'mapping_outdoor_assets_summary.json',
        'mapping_outdoor_assets_summary.md',
    ):
        if name not in mapping_outdoor_files:
            mapping_outdoor_missing.append(name)
    mapping_outdoor_failed = []
    mapping_outdoor_dir = os.path.join(outputs_dir, 'mapping_outdoor')
    outdoor_assets = os.path.join(
        mapping_outdoor_dir, 'mapping_outdoor_assets_summary.json')
    if os.path.isfile(outdoor_assets):
        try:
            data = read_json(outdoor_assets)
        except Exception as e:
            mapping_outdoor_failed.append(
                f'mapping_outdoor_assets_summary.json unreadable: {e}')
        else:
            if data.get('validation_passed') is not True:
                mapping_outdoor_failed.append(
                    'mapping_outdoor_assets_summary.json '
                    f"validation_passed={data.get('validation_passed')!r}")
            if not isinstance(data.get('summary'), dict):
                mapping_outdoor_failed.append(
                    'mapping_outdoor_assets_summary.json missing summary')
            if int(data.get('schema_version') or 0) < 3:
                mapping_outdoor_failed.append(
                    'mapping_outdoor_assets_summary.json schema_version < 3')
            for row in data.get('results', []):
                if row.get('kind') != 'occupancy_map':
                    continue
                map_path = resolve_repo_path(row.get('map', ''), outputs_dir)
                image_path = resolve_repo_path(
                    row.get('image_path') or row.get('image', ''), outputs_dir)
                if not row.get('map_sha256'):
                    mapping_outdoor_failed.append(
                        'mapping_outdoor_assets_summary.json missing map_sha256')
                elif os.path.isfile(map_path) and file_sha256(map_path) != row.get('map_sha256'):
                    mapping_outdoor_failed.append(
                        'mapping_outdoor_assets_summary.json map_sha256 mismatch')
                if not row.get('image_sha256'):
                    mapping_outdoor_failed.append(
                        'mapping_outdoor_assets_summary.json missing image_sha256')
                elif os.path.isfile(image_path) and file_sha256(image_path) != row.get('image_sha256'):
                    mapping_outdoor_failed.append(
                        'mapping_outdoor_assets_summary.json image_sha256 mismatch')
    if mapping_outdoor_missing or mapping_outdoor_failed:
        details = []
        if mapping_outdoor_missing:
            details.append(f'missing {", ".join(mapping_outdoor_missing)}')
        if mapping_outdoor_failed:
            details.append(f'failed {", ".join(mapping_outdoor_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'mapping_outdoor named contracts: {status}')
    else:
        status = 'OK (evaluation gt maps + previews + passing asset summary)'
    print(f'  mapping_outdoor named contracts: {status}')

    traffic_files = {
        f for files in inv.get('traffic_light_recognition', {}).values() for f in files
    }
    traffic_required = (
        'city_traffic_annotated.png',
        'city_traffic_stats.json',
        'city_traffic_stats_summary.json',
        'city_traffic_stats.md',
    )
    traffic_missing = [f for f in traffic_required if f not in traffic_files]
    traffic_failed = []
    traffic_dir = os.path.join(outputs_dir, 'traffic_light_recognition')
    traffic_summary = os.path.join(
        traffic_dir, 'city_traffic_stats_summary.json')
    if os.path.isfile(traffic_summary):
        try:
            data = read_json(traffic_summary)
        except Exception as e:
            traffic_failed.append(f'city_traffic_stats_summary.json unreadable: {e}')
        else:
            if data.get('validation_passed') is not True:
                traffic_failed.append(
                    'city_traffic_stats_summary.json '
                    f"validation_passed={data.get('validation_passed')!r}")
            if not isinstance(data.get('summary'), dict):
                traffic_failed.append(
                    'city_traffic_stats_summary.json missing summary')
            if int(data.get('schema_version') or 0) < 3:
                traffic_failed.append(
                    'city_traffic_stats_summary.json schema_version < 3')
            if not data.get('signal_id_hist'):
                traffic_failed.append(
                    'city_traffic_stats_summary.json missing signal_id_hist')
            if data.get('summary', {}).get('top_signal_id') is None:
                traffic_failed.append(
                    'city_traffic_stats_summary.json missing top_signal_id')
    if traffic_missing or traffic_failed:
        details = []
        if traffic_missing:
            details.append(f'missing {", ".join(traffic_missing)}')
        if traffic_failed:
            details.append(f'failed {", ".join(traffic_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'recognition_signal named contracts: {status}')
    else:
        status = 'OK (annotated PNG + raw stats + passing named stats summary)'
    print(f'  recognition_signal named contracts: {status}')

    recognition_files = {
        f for files in inv.get('recognition', {}).values() for f in files
    }
    recognition_required = (
        'indoor_recognition_overlay.png',
        'indoor_recognition_eval.md',
        'indoor_recognition_eval.json',
        'indoor_recognition_eval.csv',
        'indoor_recognition_eval.png',
        'indoor_recognition_eval_ignore_table_sofa.md',
        'indoor_recognition_eval_ignore_table_sofa.json',
        'indoor_recognition_eval_ignore_table_sofa.csv',
        'indoor_recognition_eval_ignore_table_sofa.png',
        'indoor_recognition_eval_summary.json',
        'indoor_recognition_eval_summary.md',
    )
    recognition_missing = [
        f for f in recognition_required if f not in recognition_files
    ]
    recognition_failed = []
    recognition_dir = os.path.join(outputs_dir, 'recognition')
    recognition_summary = os.path.join(
        recognition_dir, 'indoor_recognition_eval_summary.json')
    if os.path.isfile(recognition_summary):
        try:
            data = read_json(recognition_summary)
        except Exception as e:
            recognition_failed.append(
                f'indoor_recognition_eval_summary.json unreadable: {e}')
        else:
            if int(data.get('schema_version') or 0) < 2:
                recognition_failed.append(
                    'indoor_recognition_eval_summary.json schema_version < 2')
            if data.get('validation_passed') is not True:
                recognition_failed.append(
                    'indoor_recognition_eval_summary.json '
                    f"validation_passed={data.get('validation_passed')!r}")
            if not isinstance(data.get('summary'), dict):
                recognition_failed.append(
                    'indoor_recognition_eval_summary.json missing summary')
            best = data.get('best_by_f1') or {}
            if best.get('label') != 'ignore_table_sofa':
                recognition_failed.append(
                    'indoor_recognition_eval_summary.json '
                    f"best label={best.get('label')!r}")
            if float((best.get('f1') if best else 0.0) or 0.0) < 0.70:
                recognition_failed.append(
                    'indoor_recognition_eval_summary.json best f1 < 0.70')
            for row in data.get('reports', []):
                path = resolve_repo_path(row.get('path', ''), outputs_dir)
                expected_hash = row.get('report_sha256')
                if not expected_hash:
                    recognition_failed.append(
                        'indoor_recognition_eval_summary.json '
                        f"missing report_sha256 for {row.get('label')!r}")
                elif os.path.isfile(path) and file_sha256(path) != expected_hash:
                    recognition_failed.append(
                        'indoor_recognition_eval_summary.json '
                        f"report_sha256 mismatch for {row.get('label')!r}")
    if recognition_missing or recognition_failed:
        details = []
        if recognition_missing:
            details.append(f'missing {", ".join(recognition_missing)}')
        if recognition_failed:
            details.append(f'failed {", ".join(recognition_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'recognition_object named contracts: {status}')
    else:
        status = 'OK (overlay + evals + passing comparison summary)'
    print(f'  recognition_object named contracts: {status}')

    waypoint_files = {
        f for files in inv.get('waypoint_generation', {}).values() for f in files
    }
    waypoint_required = []
    for stem in (
        'indoor_waypoints',
        'break_room_waypoints',
        'cafe_waypoints',
    ):
        waypoint_required.extend((
            f'{stem}.yaml',
            f'{stem}.png',
            f'{stem}_check.json',
            f'{stem}_check.md',
        ))
    waypoint_missing = [f for f in waypoint_required if f not in waypoint_files]
    waypoint_failed = []
    waypoint_dir = os.path.join(outputs_dir, 'waypoint_generation')
    for stem in (
        'indoor_waypoints',
        'break_room_waypoints',
        'cafe_waypoints',
    ):
        waypoint_yaml = os.path.join(waypoint_dir, f'{stem}.yaml')
        if os.path.isfile(waypoint_yaml):
            try:
                yaml_data = read_yaml(waypoint_yaml)
            except Exception as e:
                waypoint_failed.append(f'{stem}.yaml unreadable: {e}')
            else:
                if int((yaml_data or {}).get('schema_version') or 0) < 2:
                    waypoint_failed.append(f'{stem}.yaml schema_version < 2')
                provenance = (yaml_data or {}).get('provenance')
                if not isinstance(provenance, dict):
                    waypoint_failed.append(f'{stem}.yaml missing provenance')
                else:
                    for input_key, hash_key in (
                        ('map', 'map_sha256'),
                        ('map_image', 'map_image_sha256'),
                    ):
                        input_path = provenance.get(input_key)
                        recorded_hash = provenance.get(hash_key)
                        if not input_path:
                            waypoint_failed.append(
                                f'{stem}.yaml missing provenance.{input_key}')
                            continue
                        if not recorded_hash:
                            waypoint_failed.append(
                                f'{stem}.yaml missing provenance.{hash_key}')
                            continue
                        path = resolve_repo_path(input_path, outputs_dir)
                        if not os.path.isfile(path):
                            waypoint_failed.append(
                                f'{stem}.yaml provenance.{input_key} missing source file')
                        elif recorded_hash != file_sha256(path):
                            waypoint_failed.append(
                                f'{stem}.yaml provenance.{hash_key} mismatch')
                    if not isinstance(provenance.get('parameters'), dict):
                        waypoint_failed.append(
                            f'{stem}.yaml missing provenance.parameters')
                    if not isinstance(provenance.get('hazard_files'), list):
                        waypoint_failed.append(
                            f'{stem}.yaml missing provenance.hazard_files')
        check_json = os.path.join(waypoint_dir, f'{stem}_check.json')
        if not os.path.isfile(check_json):
            continue
        try:
            data = read_json(check_json)
        except Exception as e:
            waypoint_failed.append(f'{stem}_check.json unreadable: {e}')
            continue
        if data.get('validation_passed') is not True:
            waypoint_failed.append(
                f"{stem}_check.json validation_passed="
                f"{data.get('validation_passed')!r}")
        if int(data.get('schema_version') or 0) < 3:
            waypoint_failed.append(f'{stem}_check.json schema_version < 3')
        if not isinstance(data.get('summary'), dict):
            waypoint_failed.append(f'{stem}_check.json missing summary')
        hashes = data.get('inputs_hash')
        inputs = data.get('inputs') or {}
        if not isinstance(hashes, dict):
            waypoint_failed.append(f'{stem}_check.json missing inputs_hash')
            continue
        for input_key, hash_key in (
            ('map', 'map_sha256'),
            ('map_image', 'map_image_sha256'),
            ('waypoints', 'waypoints_sha256'),
        ):
            input_path = inputs.get(input_key)
            recorded_hash = hashes.get(hash_key)
            if not input_path:
                waypoint_failed.append(
                    f'{stem}_check.json missing inputs.{input_key}')
                continue
            if not recorded_hash:
                waypoint_failed.append(
                    f'{stem}_check.json missing inputs_hash.{hash_key}')
                continue
            path = resolve_repo_path(input_path, outputs_dir)
            if not os.path.isfile(path):
                waypoint_failed.append(
                    f'{stem}_check.json inputs.{input_key} missing source file')
            elif recorded_hash != file_sha256(path):
                waypoint_failed.append(
                    f'{stem}_check.json inputs_hash.{hash_key} mismatch')
    if waypoint_missing or waypoint_failed:
        details = []
        if waypoint_missing:
            details.append(f'missing {", ".join(waypoint_missing)}')
        if waypoint_failed:
            details.append(f'failed {", ".join(waypoint_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'waypoint_generation named contracts: {status}')
    else:
        status = 'OK (indoor/break_room/cafe waypoint YAML/PNG + passing quality checks)'
    print(f'  waypoint_generation named contracts: {status}')

    patrol_missing = []
    patrol_failed = []
    for world in ('indoor', 'break_room'):
        for suffix in (
            '_patrol_report.json',
            '_patrol_report.csv',
            '_patrol_report.md',
            '_patrol_result.png',
            '_patrol_result.json',
            '_patrol_result.md',
        ):
            name = f'{world}{suffix}'
            if name not in waypoint_files:
                patrol_missing.append(name)
        result_json = os.path.join(waypoint_dir, f'{world}_patrol_result.json')
        if os.path.isfile(result_json):
            try:
                data = read_json(result_json)
            except Exception as e:
                patrol_failed.append(f'{world}_patrol_result.json unreadable: {e}')
                continue
            if data.get('validation_passed') is not True:
                patrol_failed.append(
                    f"{world}_patrol_result.json validation_passed="
                    f"{data.get('validation_passed')!r}"
                )
            if int(data.get('schema_version') or 0) < 3:
                patrol_failed.append(
                    f'{world}_patrol_result.json schema_version < 3')
            if not isinstance(data.get('criteria'), dict):
                patrol_failed.append(
                    f'{world}_patrol_result.json missing criteria')
            if not isinstance(data.get('failures'), list):
                patrol_failed.append(
                    f'{world}_patrol_result.json missing failures')
            if 'safe_pose_recovery_count' not in data:
                patrol_failed.append(
                    f'{world}_patrol_result.json missing safe_pose_recovery_count')
            if 'step_event_count' not in data:
                patrol_failed.append(
                    f'{world}_patrol_result.json missing step_event_count')
    if patrol_missing or patrol_failed:
        details = []
        if patrol_missing:
            details.append(f'missing {", ".join(patrol_missing)}')
        if patrol_failed:
            details.append(f'failed {", ".join(patrol_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'waypoint_navigation named contracts: {status}')
    else:
        status = 'OK (indoor/break_room patrol reports + validation_passed result summaries)'
    print(f'  waypoint_navigation named contracts: {status}')

    colorized_files = {
        f for files in inv.get('colorized_pointcloud', {}).values() for f in files
    }
    colorized_stems = (
        'colorized_pointcloud_indoor_apriltag_calib_final',
        'colorized_pointcloud_indoor_goal_run_final',
        'colorized_pointcloud_breakroom_apriltag_calib_final',
    )
    colorized_missing = []
    for stem in colorized_stems:
        for suffix in ('.ply', '_check.png', '_check.json', '_check.md'):
            name = f'{stem}{suffix}'
            if name not in colorized_files:
                colorized_missing.append(name)
    for name in (
        'colorized_pointcloud_quality_summary.json',
        'colorized_pointcloud_quality_summary.md',
    ):
        if name not in colorized_files:
            colorized_missing.append(name)
    colorized_failed = []
    colorized_dir = os.path.join(outputs_dir, 'colorized_pointcloud')
    for stem in colorized_stems:
        check_json = os.path.join(colorized_dir, f'{stem}_check.json')
        ply_path = os.path.join(colorized_dir, f'{stem}.ply')
        if not os.path.isfile(check_json):
            continue
        try:
            data = read_json(check_json)
        except Exception as e:
            colorized_failed.append(f'{stem}_check.json unreadable: {e}')
            continue
        if int(data.get('schema_version') or 0) < 4:
            colorized_failed.append(f'{stem}_check.json schema_version < 4')
        if data.get('validation_passed') is not True:
            colorized_failed.append(
                f"{stem}_check.json validation_passed="
                f"{data.get('validation_passed')!r}")
        if not isinstance(data.get('summary'), dict):
            colorized_failed.append(f'{stem}_check.json missing summary')
        if 'header_vertices' not in data:
            colorized_failed.append(f'{stem}_check.json missing header_vertices')
        if not isinstance(data.get('properties'), list):
            colorized_failed.append(f'{stem}_check.json missing properties')
        recorded_ply = data.get('ply')
        recorded_hash = data.get('ply_sha256')
        if not recorded_ply:
            colorized_failed.append(f'{stem}_check.json missing ply')
        elif os.path.basename(recorded_ply) != f'{stem}.ply':
            colorized_failed.append(
                f'{stem}_check.json ply points to {recorded_ply!r}')
        if not recorded_hash:
            colorized_failed.append(f'{stem}_check.json missing ply_sha256')
        elif os.path.isfile(ply_path) and recorded_hash != file_sha256(ply_path):
            colorized_failed.append(f'{stem}_check.json ply_sha256 mismatch')
    quality_json = os.path.join(
        colorized_dir, 'colorized_pointcloud_quality_summary.json')
    if os.path.isfile(quality_json):
        try:
            data = read_json(quality_json)
        except Exception as e:
            colorized_failed.append(
                f'colorized_pointcloud_quality_summary.json unreadable: {e}')
        else:
            if data.get('validation_passed') is not True:
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json '
                    f"validation_passed={data.get('validation_passed')!r}")
            if not isinstance(data.get('summary'), dict):
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json missing summary')
            if int(data.get('schema_version') or 0) < 5:
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json schema_version < 5')
            summary = data.get('summary') or {}
            if summary.get('duplicate_input_name_count') not in (0, None):
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json duplicate input names')
            if summary.get('duplicate_file_hash_count') not in (0, None):
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json duplicate file hashes')
            if data.get('duplicate_input_names') not in ([], None):
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json duplicate_input_names not empty')
            if data.get('duplicate_file_hashes') not in ([], None):
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json duplicate_file_hashes not empty')
            items = data.get('items')
            if not isinstance(items, list):
                colorized_failed.append(
                    'colorized_pointcloud_quality_summary.json missing items')
            else:
                seen_ply = set()
                seen_hashes = {}
                for item in items:
                    if not isinstance(item, dict) or item.get('kind') != 'ply':
                        continue
                    name = item.get('name')
                    recorded_hash = item.get('file_sha256')
                    if not name:
                        colorized_failed.append(
                            'colorized_pointcloud_quality_summary.json item missing name')
                        continue
                    if item.get('property_types_valid') is not True:
                        colorized_failed.append(
                            f'colorized_pointcloud_quality_summary.json {name} invalid property types')
                    if not isinstance(item.get('property_types'), dict):
                        colorized_failed.append(
                            f'colorized_pointcloud_quality_summary.json {name} missing property_types')
                    if not recorded_hash:
                        colorized_failed.append(
                            f'colorized_pointcloud_quality_summary.json {name} missing file_sha256')
                        continue
                    path = resolve_repo_path(name, outputs_dir)
                    basename = os.path.basename(path)
                    seen_ply.add(basename)
                    seen_hashes.setdefault(recorded_hash, []).append(basename)
                    if not os.path.isfile(path):
                        colorized_failed.append(
                            f'colorized_pointcloud_quality_summary.json {name} missing source PLY')
                    elif recorded_hash != file_sha256(path):
                        colorized_failed.append(
                            f'colorized_pointcloud_quality_summary.json {name} file_sha256 mismatch')
                for stem in colorized_stems:
                    if f'{stem}.ply' not in seen_ply:
                        colorized_failed.append(
                            'colorized_pointcloud_quality_summary.json '
                            f'missing item for {stem}.ply')
                for digest, basenames in seen_hashes.items():
                    if digest and len(basenames) > 1:
                        colorized_failed.append(
                            'colorized_pointcloud_quality_summary.json '
                            f'duplicate file_sha256 for {sorted(basenames)}')
    if colorized_missing or colorized_failed:
        details = []
        if colorized_missing:
            details.append(f'missing {", ".join(colorized_missing)}')
        if colorized_failed:
            details.append(f'failed {", ".join(colorized_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'colorized_pointcloud named contracts: {status}')
    else:
        status = 'OK (3 final PLY files + passing per-PLY checks + passing quality summary)'
    print(f'  colorized_pointcloud named contracts: {status}')

    extrinsic_files = {
        f for files in inv.get('extrinsic_calibration', {}).values() for f in files
    }
    extrinsic_required = (
        'calib.json',
        'calib_summary.png',
        'calib_summary.json',
        'calib_summary.md',
    )
    extrinsic_missing = [
        f for f in extrinsic_required if f not in extrinsic_files
    ]
    extrinsic_failed = []
    extrinsic_dir = os.path.join(outputs_dir, 'extrinsic_calibration')
    calib_summary = os.path.join(extrinsic_dir, 'calib_summary.json')
    if os.path.isfile(calib_summary):
        try:
            data = read_json(calib_summary)
        except Exception as e:
            extrinsic_failed.append(f'calib_summary.json unreadable: {e}')
        else:
            if int(data.get('schema_version') or 0) < 3:
                extrinsic_failed.append('calib_summary.json schema_version < 3')
            if data.get('validation_passed') is not True:
                extrinsic_failed.append(
                    'calib_summary.json '
                    f"validation_passed={data.get('validation_passed')!r}")
            summary = data.get('summary')
            criteria = data.get('criteria')
            metrics = data.get('metrics')
            if not isinstance(summary, dict):
                extrinsic_failed.append('calib_summary.json missing summary')
                summary = {}
            if not isinstance(criteria, dict):
                extrinsic_failed.append('calib_summary.json missing criteria')
                criteria = {}
            if not isinstance(metrics, dict):
                extrinsic_failed.append('calib_summary.json missing metrics')
                metrics = {}
            if 'max_quaternion_norm_error' not in criteria:
                extrinsic_failed.append(
                    'calib_summary.json missing max_quaternion_norm_error')
            if metrics.get('transform_length') != 7:
                extrinsic_failed.append(
                    'calib_summary.json T_lidar_camera length is not 7')
            if metrics.get('transform_finite') is not True:
                extrinsic_failed.append(
                    'calib_summary.json T_lidar_camera is not finite')
            duplicate_tag_ids = metrics.get('duplicate_tag_ids')
            if duplicate_tag_ids not in ([], None):
                extrinsic_failed.append(
                    'calib_summary.json duplicate used tag IDs present')
            unique_tag_count = metrics.get(
                'unique_tag_count', summary.get('unique_tag_count'))
            min_used_tags = criteria.get('min_used_tags')
            if isinstance(min_used_tags, int):
                if not isinstance(unique_tag_count, int):
                    extrinsic_failed.append(
                        'calib_summary.json unique_tag_count missing')
                elif unique_tag_count < min_used_tags:
                    extrinsic_failed.append(
                        'calib_summary.json unique_tag_count below minimum')
            q_norm = metrics.get('quaternion_norm', summary.get('quaternion_norm'))
            q_norm_max_err = criteria.get('max_quaternion_norm_error')
            if not isinstance(q_norm, (int, float)) or not math.isfinite(q_norm):
                extrinsic_failed.append(
                    'calib_summary.json quaternion_norm missing/non-finite')
            elif isinstance(q_norm_max_err, (int, float)):
                if abs(float(q_norm) - 1.0) > float(q_norm_max_err):
                    extrinsic_failed.append(
                        'calib_summary.json quaternion norm outside tolerance')
            expected_hash = (data.get('inputs') or {}).get('calib_sha256')
            calib_json = os.path.join(extrinsic_dir, 'calib.json')
            if not expected_hash:
                extrinsic_failed.append('calib_summary.json missing calib_sha256')
            elif os.path.isfile(calib_json):
                actual_hash = file_sha256(calib_json)
                if actual_hash != expected_hash:
                    extrinsic_failed.append(
                        'calib_summary.json calib_sha256 mismatch')
    if extrinsic_missing or extrinsic_failed:
        details = []
        if extrinsic_missing:
            details.append(f'missing {", ".join(extrinsic_missing)}')
        if extrinsic_failed:
            details.append(f'failed {", ".join(extrinsic_failed)}')
        status = f'FAILED ({"; ".join(details)})'
        issues.append(f'extrinsic_calibration named contracts: {status}')
    else:
        status = 'OK (calib JSON + passing PNG/JSON/MD summary)'
    print(f'  extrinsic_calibration named contracts: {status}')

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
