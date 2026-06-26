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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', required=True,
                    help='calib.json (apriltag_extrinsic_calib_node 出力)')
    ap.add_argument('--out', required=True, help='output PNG path')
    ap.add_argument('--ref-translation', nargs=3, type=float,
                    default=[0.0, 0.0, 0.55],
                    help='真値 translation [x y z] (m)')
    args = ap.parse_args()

    with open(args.calib) as f:
        data = json.load(f)
    t_lc = data['results']['T_lidar_camera']
    tx, ty, tz, qx, qy, qz, qw = t_lc
    ref_x, ref_y, ref_z = args.ref_translation
    diff_x = tx - ref_x
    diff_y = ty - ref_y
    diff_z = tz - ref_z
    transl_err = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

    roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
    rms = data.get('apriltag_calib', {}).get('correspondence_rms_m', None)
    used_tags = data.get('apriltag_calib', {}).get('used_tag_ids', [])
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
    print(f'saved PNG: {args.out}')
    print(f'  translation error: {transl_err*1000:.1f} mm')
    print(f'  RPY (deg): roll={roll:+.3f} pitch={pitch:+.3f} yaw={yaw:+.3f}')
    if rms is not None:
        print(f'  correspondence RMS: {rms*1000:.2f} mm')


if __name__ == '__main__':
    main()
