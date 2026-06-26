#!/usr/bin/env python3
"""susumu_object_perception の全 launch ファイルを --show-args で parse 検証。

iter99 で導入。 個別 smoke (実起動) は重いので CI には載せられないが、 launch
ファイルの構文と DeclareLaunchArgument の宣言は --show-args だけで検証できる。
このスクリプトは全 .launch.py を一斉に parse して、 syntax/import エラーを早期
発見するためのもの。

使い方:
  python3 scripts/validate_launches.py

挙動:
- src/susumu_object_perception/launch/*.launch.py を列挙
- 各 launch に対し `ros2 launch <pkg> <launch> --show-args` を実行
- exit code != 0 を fail としてカウントし最終 summary
- description 内の "Error" / "FATAL" 等は偽陽性を避けるため出力スキャンしない
"""
import os
import subprocess
import sys


PKG = 'susumu_object_perception'


def find_launches(launch_dir):
    launches = []
    if not os.path.isdir(launch_dir):
        print(f'launch dir not found: {launch_dir}', file=sys.stderr)
        return launches
    for f in sorted(os.listdir(launch_dir)):
        if f.endswith('.launch.py'):
            launches.append(f)
    return launches


def resolve_launch_dir():
    # 優先 1: スクリプト位置から source ツリーを推定 (scripts/ の親が pkg root)
    src_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch')
    if os.path.isdir(src_dir):
        return src_dir
    # 優先 2: ament_index で installed package share/<pkg>/launch を解決
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(PKG), 'launch')
    except Exception:
        return src_dir


def main():
    launch_dir = resolve_launch_dir()
    launches = find_launches(launch_dir)
    print(f'Found {len(launches)} launch files in {launch_dir}')
    print()

    passed = 0
    failed = []
    for launch in launches:
        cmd = ['ros2', 'launch', PKG, launch, '--show-args']
        try:
            result = subprocess.run(
                cmd, capture_output=True, timeout=30, text=True)
            if result.returncode == 0:
                passed += 1
                status = 'PASS'
            else:
                failed.append((launch, result.returncode, result.stderr[-200:]))
                status = 'FAIL'
        except subprocess.TimeoutExpired:
            failed.append((launch, -1, 'timeout 30s'))
            status = 'TIMEOUT'
        print(f'  [{status}] {launch}')

    print()
    print(f'Summary: {passed}/{len(launches)} passed, {len(failed)} failed')
    if failed:
        print()
        print('Failures:')
        for launch, rc, msg in failed:
            print(f'  {launch}: rc={rc}')
            print(f'    stderr (tail): {msg}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
