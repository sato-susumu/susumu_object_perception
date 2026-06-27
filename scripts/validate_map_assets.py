#!/usr/bin/env python3
"""Validate occupancy-grid YAML files and their referenced image assets.

This catches the common case where `outputs/mapping_*/<name>.yaml` is present but the
referenced `.pgm` image is missing or is a broken symlink. PGM/PNG artifacts are
ignored by git in this repository, so text-only checks can otherwise pass while
map support, overlay rendering, waypoint generation, or Nav2 map loading later
fail with a low-level FileNotFoundError.
"""

import argparse
import hashlib
import json
from pathlib import Path

import yaml


def _default_maps():
    return sorted(
        list(Path('outputs/mapping_indoor').glob('*.yaml'))
        + list(Path('outputs/mapping_outdoor').glob('*.yaml'))
    )


def _load_yaml(path):
    with path.open() as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        data = {}
    return data


def _resolve_image_path(map_yaml, image_value):
    image_path = Path(str(image_value))
    if image_path.is_absolute():
        return image_path
    return map_yaml.parent / image_path


def file_sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def validate_one(map_yaml):
    map_yaml = Path(map_yaml)
    result = {
        'map': str(map_yaml),
        'map_sha256': '',
        'kind': 'not_occupancy_map',
        'image': '',
        'image_path': '',
        'image_sha256': '',
        'ok': True,
        'reason': '',
    }
    if not map_yaml.exists():
        result.update({
            'kind': 'missing_yaml',
            'ok': False,
            'reason': 'yaml_missing',
        })
        return result

    result['map_sha256'] = file_sha256(map_yaml)
    try:
        meta = _load_yaml(map_yaml)
    except Exception as exc:
        result.update({
            'kind': 'invalid_yaml',
            'ok': False,
            'reason': 'yaml_parse_failed: %s' % exc,
        })
        return result

    if 'image' not in meta:
        return result

    image_path = _resolve_image_path(map_yaml, meta['image'])
    result.update({
        'kind': 'occupancy_map',
        'image': str(meta['image']),
        'image_path': str(image_path),
    })

    if image_path.exists():
        result['reason'] = 'ok'
        result['image_sha256'] = file_sha256(image_path)
        return result

    result['ok'] = False
    if image_path.is_symlink():
        result['reason'] = 'broken_symlink'
    else:
        result['reason'] = 'image_missing'
    return result


def print_table(results, only_bad=False):
    rows = [r for r in results if not only_bad or not r['ok']]
    if not rows:
        print('all checked map assets are OK')
        return
    print('%-54s %-7s %s' % ('map', 'status', 'image'))
    print('-' * 90)
    for r in rows:
        status = 'OK' if r['ok'] else 'NG'
        detail = r['reason'] or r['kind']
        if r['kind'] == 'not_occupancy_map':
            detail = 'skip:no_image_key'
        image = r['image_path'] or r['image']
        print('%-54s %-7s %s (%s)' % (r['map'], status, image, detail))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        'maps',
        nargs='*',
        help='occupancy-grid YAML files. Defaults to outputs/mapping_*/*.yaml',
    )
    ap.add_argument(
        '--only-bad',
        action='store_true',
        help='print only missing/invalid assets',
    )
    ap.add_argument(
        '--json',
        action='store_true',
        help='print machine-readable JSON',
    )
    ap.add_argument('--json-out', default='',
                    help='optional machine-readable summary path')
    ap.add_argument('--md-out', default='',
                    help='optional Markdown summary path')
    args = ap.parse_args()

    map_paths = [Path(p) for p in args.maps] if args.maps else _default_maps()
    if not map_paths:
        message = 'no map YAML files found. Pass map paths, or run from the package root.'
        if args.json:
            print(json.dumps({'ok': False, 'error': message, 'results': []},
                             indent=2, ensure_ascii=False))
        else:
            print(message)
        raise SystemExit(2)

    results = [validate_one(p) for p in map_paths]
    bad = [r for r in results if not r['ok']]
    ok = not bad

    if args.json:
        print(json.dumps({'ok': ok, 'results': results},
                         indent=2, ensure_ascii=False))
    else:
        print_table(results, only_bad=args.only_bad)
        if bad:
            print()
            print('Fix: regenerate the missing map image with Nav2 map_saver_cli,')
            print('for example: ros2 run nav2_map_server map_saver_cli -f outputs/mapping_indoor/<map_name>')
    if args.json_out:
        write_json(args.json_out, results, ok)
        print(f'JSON: {args.json_out}')
    if args.md_out:
        write_markdown(args.md_out, results, ok)
        print(f'MD: {args.md_out}')

    raise SystemExit(1 if bad else 0)


def write_json(path, results, ok):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    passed = sum(1 for r in results if r.get('ok') is True)
    with out.open('w') as f:
        json.dump({
            'schema_version': 3,
            'ok': ok,
            'validation_passed': ok,
            'summary': {
                'checked': len(results),
                'passed': passed,
                'failed': len(results) - passed,
            },
            'results': results,
        }, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_markdown(path, results, ok):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        '# Map Asset Summary',
        '',
        f"- ok: `{str(ok).lower()}`",
        f"- validation_passed: `{str(ok).lower()}`",
        f"- checked: `{len(results)}`",
        '',
        '| map | status | image | map sha256 | image sha256 | reason |',
        '|---|---|---|---|---|---|',
    ]
    for r in results:
        status = 'OK' if r['ok'] else 'NG'
        image = r['image_path'] or r['image']
        map_digest = (r.get('map_sha256') or '')[:12]
        image_digest = (r.get('image_sha256') or '')[:12]
        lines.append(
            f"| `{r['map']}` | {status} | `{image}` | "
            f"`{map_digest}` | `{image_digest}` | {r['reason'] or r['kind']} |")
    out.write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
