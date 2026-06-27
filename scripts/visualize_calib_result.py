#!/usr/bin/env python3
"""extrinsic_calibration の結果を可視化する PNG を出力する。

`outputs/extrinsic_calibration/calib.json` を読み、 期待 (真値) との差分を
bar chart で表示し、 RMS / 使用 tag 数 / quaternion の roll/pitch/yaw を
テーブル形式で重ねる。 ユーザーが「キャリブが妥当か」 を一目で判断できる
ようにする。

使い方:
  python3 visualize_calib_result.py \\
      --calib outputs/extrinsic_calibration/calib.json \\
      --out outputs/extrinsic_calibration/calib_summary.png

期待値 (真値) は --ref-translation で渡せる (既定 0,0,0.55 = lidar_link → omni_camera_link)。
"""

import argparse
import hashlib
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def quat_to_rpy(qx, qy, qz, qw):
    """quaternion -> (roll, pitch, yaw) in degrees."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rotation_angle_deg(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return float('inf')
    qw_n = max(-1.0, min(1.0, abs(qw / norm)))
    return math.degrees(2.0 * math.acos(qw_n))


def is_finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write_json(path, report):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write('\n')


def write_markdown(path, report):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    m = report['metrics']
    c = report['criteria']
    lines = [
        '# Extrinsic Calibration Summary',
        '',
        f"- validation_passed: `{str(report['validation_passed']).lower()}`",
        f"- schema_version: `{report['schema_version']}`",
        f"- method: `{report['meta']['method']}`",
        f"- camera_model: `{report['meta']['camera_model']}`",
        f"- calib_sha256: `{report['inputs']['calib_sha256']}`",
        f"- used_tag_count: `{m['used_tag_count']}`",
        f"- unique_tag_count: `{m['unique_tag_count']}`",
        f"- correspondence_rms_mm: `{m['correspondence_rms_m'] * 1000.0:.2f}`",
        f"- rotation_angle_deg: `{m['rotation_angle_deg']:.3f}`",
        f"- quaternion_norm: `{m['quaternion_norm']:.6f}`",
        f"- translation_error_mm: `{m['translation_error_m'] * 1000.0:.1f}`",
        f"- transform_finite: `{str(m['transform_finite']).lower()}`",
        '',
        '| metric | value | criterion |',
        '|---|---:|---:|',
        f"| transform length | {m['transform_length']} | = 7 |",
        f"| finite transform | {str(m['transform_finite']).lower()} | true |",
        f"| unique tags | {m['unique_tag_count']} | >= {c['min_used_tags']} |",
        f"| RMS [mm] | {m['correspondence_rms_m'] * 1000.0:.2f} | <= {c['max_rms_m'] * 1000.0:.1f} |",
        f"| quaternion norm error | {abs(m['quaternion_norm'] - 1.0):.2e} | <= {c['max_quaternion_norm_error']:.1e} |",
        f"| rotation [deg] | {m['rotation_angle_deg']:.3f} | <= {c['max_rotation_deg']:.1f} |",
        f"| translation error [mm] | {m['translation_error_m'] * 1000.0:.1f} | <= {c['max_translation_error_m'] * 1000.0:.1f} |",
    ]
    if report['failures']:
        lines.extend(['', '## Failures'])
        lines.extend(f'- {failure}' for failure in report['failures'])
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', required=True,
                    help='calib.json (apriltag_extrinsic_calib_node 出力)')
    ap.add_argument('--out', required=True, help='output PNG path')
    ap.add_argument('--ref-translation', nargs=3, type=float,
                    default=[0.0, 0.0, 0.55],
                    help='真値 translation [x y z] (m)')
    ap.add_argument('--json-out', default='',
                    help='optional machine-readable summary path')
    ap.add_argument('--md-out', default='',
                    help='optional Markdown summary path')
    ap.add_argument('--min-used-tags', type=int, default=4)
    ap.add_argument('--max-rms-m', type=float, default=0.010)
    ap.add_argument('--max-rotation-deg', type=float, default=1.0)
    ap.add_argument('--max-translation-error-m', type=float, default=0.030)
    ap.add_argument('--max-quaternion-norm-error', type=float, default=1.0e-3)
    ap.add_argument('--require-pass', action='store_true',
                    help='validation_passed=false なら非ゼロ終了')
    args = ap.parse_args()

    calib_sha256 = file_sha256(args.calib)
    with open(args.calib) as f:
        data = json.load(f)
    t_lc = data.get('results', {}).get('T_lidar_camera', [])
    transform_length = len(t_lc) if isinstance(t_lc, list) else 0
    transform_finite = (
        isinstance(t_lc, list)
        and transform_length == 7
        and all(is_finite_number(v) for v in t_lc)
    )
    if transform_length != 7 or not transform_finite:
        print('invalid T_lidar_camera')
        print(f'  transform length: {transform_length} (expected 7)')
        print(f'  transform_finite: {transform_finite}')
        return 2 if args.require_pass else 0
    t_lc = [float(v) for v in t_lc]
    tx, ty, tz, qx, qy, qz, qw = t_lc
    quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    ref_x, ref_y, ref_z = args.ref_translation
    diff_x = tx - ref_x
    diff_y = ty - ref_y
    diff_z = tz - ref_z
    transl_err = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

    roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
    rot_angle = rotation_angle_deg(qx, qy, qz, qw)
    rms = data.get('apriltag_calib', {}).get('correspondence_rms_m', None)
    used_tags = data.get('apriltag_calib', {}).get('used_tag_ids', [])
    tag_counts = {}
    for tag in used_tags:
        tag_counts[tag] = tag_counts.get(tag, 0) + 1
    unique_tag_ids = sorted(tag_counts)
    duplicate_tag_ids = sorted(
        tag for tag, count in tag_counts.items() if count > 1)
    tag_size = data.get('apriltag_calib', {}).get('tag_size_m', None)
    method = data.get('meta', {}).get('method', 'unknown')
    cam_model = data.get('meta', {}).get('camera_model', 'unknown')

    fig, axes = plt.subplots(1, 2, figsize=(14, 6),
                             gridspec_kw={'width_ratios': [3, 4]})

    # 左: translation 比較 bar chart (estimated vs ref)
    ax = axes[0]
    axes_labels = ['x', 'y', 'z']
    ref_vals = [ref_x, ref_y, ref_z]
    est_vals = [tx, ty, tz]
    x = np.arange(len(axes_labels))
    width = 0.35
    bars_ref = ax.bar(x - width/2, ref_vals, width, label='reference (truth)',
                      color='#1f77b4', alpha=0.8)
    bars_est = ax.bar(x + width/2, est_vals, width, label='estimated',
                      color='#d62728', alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(axes_labels)
    ax.set_ylabel('translation [m]')
    ax.set_title('T_lidar_camera translation\n'
                 f'translation error = {transl_err*1000:.1f} mm')
    ax.legend()
    ax.grid(True, axis='y', alpha=0.3)
    # 各 bar 上に値を表記
    for bars in [bars_ref, bars_est]:
        for b in bars:
            h = b.get_height()
            ax.text(b.get_x() + b.get_width()/2., h,
                    f'{h:.3f}', ha='center', va='bottom', fontsize=9)

    # 右: テーブル形式の数値サマリー
    ax = axes[1]
    ax.axis('off')
    summary_lines = [
        ['method', method],
        ['camera_model', cam_model],
        ['used tags', str(used_tags)],
        ['unique tags', str(unique_tag_ids)],
        ['tag size [m]', f'{tag_size:.4f}' if tag_size else 'N/A'],
        [
            'translation [m]',
            f'x={tx:+.4f}  y={ty:+.4f}  z={tz:+.4f}',
        ],
        [
            'translation diff [mm]',
            f'dx={diff_x*1000:+.1f}  dy={diff_y*1000:+.1f}  dz={diff_z*1000:+.1f}',
        ],
        [
            'translation error [mm]',
            f'{transl_err*1000:.1f}  (vs ref {args.ref_translation})',
        ],
        [
            'rotation (RPY) [deg]',
            f'roll={roll:+.3f}  pitch={pitch:+.3f}  yaw={yaw:+.3f}',
        ],
        ['quaternion (xyzw)',
         f'{qx:+.4f}, {qy:+.4f}, {qz:+.4f}, {qw:+.4f}'],
        ['quaternion norm', f'{quaternion_norm:.6f}'],
        ['correspondence RMS [mm]',
         f'{rms*1000:.2f}' if rms is not None else 'N/A'],
    ]
    cell_text = [[k, v] for k, v in summary_lines]
    tbl = ax.table(cellText=cell_text, colLabels=['field', 'value'],
                   cellLoc='left', colLoc='left', loc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1.0, 1.4)
    # field 列を太字に
    for i in range(len(cell_text)):
        tbl[(i + 1, 0)].set_text_props(weight='bold')
    ax.set_title('calibration summary')

    fig.suptitle('extrinsic calibration result', fontsize=13)
    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    fig.savefig(args.out, dpi=120, bbox_inches='tight')
    plt.close(fig)

    failures = []
    if transform_length != 7:
        failures.append(f'T_lidar_camera length {transform_length} != 7')
    if not transform_finite:
        failures.append('T_lidar_camera contains non-finite values')
    if len(unique_tag_ids) < args.min_used_tags:
        failures.append(
            f'unique used tags {len(unique_tag_ids)} < {args.min_used_tags}')
    if duplicate_tag_ids:
        failures.append(f'duplicate used tag IDs: {duplicate_tag_ids}')
    if rms is None:
        failures.append('correspondence RMS missing')
    elif rms > args.max_rms_m:
        failures.append(
            f'correspondence RMS {rms:.6f} > {args.max_rms_m:.6f}')
    quaternion_norm_error = abs(quaternion_norm - 1.0)
    if quaternion_norm_error > args.max_quaternion_norm_error:
        failures.append(
            f'quaternion norm error {quaternion_norm_error:.6e} > '
            f'{args.max_quaternion_norm_error:.6e}')
    if rot_angle > args.max_rotation_deg:
        failures.append(
            f'rotation angle {rot_angle:.3f} > {args.max_rotation_deg:.3f}')
    if transl_err > args.max_translation_error_m:
        failures.append(
            f'translation error {transl_err:.6f} > '
            f'{args.max_translation_error_m:.6f}')
    report = {
        'schema_version': 3,
        'validation_passed': not failures,
        'failures': failures,
        'summary': {
            'used_tag_count': len(used_tags),
            'unique_tag_count': len(unique_tag_ids),
            'correspondence_rms_m': rms,
            'rotation_angle_deg': rot_angle,
            'quaternion_norm': quaternion_norm,
            'transform_finite': transform_finite,
            'translation_error_m': transl_err,
        },
        'inputs': {
            'calib': args.calib,
            'calib_sha256': calib_sha256,
            'ref_translation': args.ref_translation,
        },
        'criteria': {
            'min_used_tags': args.min_used_tags,
            'max_rms_m': args.max_rms_m,
            'max_rotation_deg': args.max_rotation_deg,
            'max_translation_error_m': args.max_translation_error_m,
            'max_quaternion_norm_error': args.max_quaternion_norm_error,
        },
        'meta': {
            'method': method,
            'camera_model': cam_model,
        },
        'metrics': {
            'translation_m': [tx, ty, tz],
            'translation_diff_m': [diff_x, diff_y, diff_z],
            'translation_error_m': transl_err,
            'rotation_rpy_deg': [roll, pitch, yaw],
            'rotation_angle_deg': rot_angle,
            'quaternion_xyzw': [qx, qy, qz, qw],
            'quaternion_norm': quaternion_norm,
            'transform_length': transform_length,
            'transform_finite': transform_finite,
            'correspondence_rms_m': rms,
            'used_tag_ids': used_tags,
            'used_tag_count': len(used_tags),
            'unique_tag_ids': unique_tag_ids,
            'unique_tag_count': len(unique_tag_ids),
            'duplicate_tag_ids': duplicate_tag_ids,
            'tag_size_m': tag_size,
            'png': args.out,
        },
    }
    if args.json_out:
        write_json(args.json_out, report)
        print(f'saved JSON: {args.json_out}')
    if args.md_out:
        write_markdown(args.md_out, report)
        print(f'saved MD: {args.md_out}')
    print(f'saved PNG: {args.out}')
    print(f'  translation error: {transl_err*1000:.1f} mm')
    print(
        f'  RPY (deg): roll={roll:+.3f} pitch={pitch:+.3f} '
        f'yaw={yaw:+.3f}; angle={rot_angle:.3f}')
    if rms is not None:
        print(f'  correspondence RMS: {rms*1000:.2f} mm')
    if not failures:
        print('  validation_passed=true')
        return 0
    print('  validation_passed=false')
    for failure in failures:
        print(f'  - {failure}')
    return 2 if args.require_pass else 0


if __name__ == '__main__':
    raise SystemExit(main())
