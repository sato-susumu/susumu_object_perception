#!/usr/bin/env python3
"""Webots のキャリブ板に貼る AprilTag(36h11) テクスチャ PNG を生成する。

全天球カメラ + LiDAR の外部キャリブ（docs/omni_lidar_camera.md「AprilTag 既知ターゲット方式」）で
使うタグ画像を作る。`apriltag_ros` には依存せず、OpenCV の `cv2.aruco`（AprilTag 36h11 辞書）で
生成する。タグの周囲には検出に必要な白マージン（quiet zone）を付け、Webots の `ImageTexture` に
貼れる正方形 PNG にする。

出力:
  webots_worlds/apriltag_textures/tag36h11_<id>.png

Webots 側では、板 Shape の appearance を
  PBRAppearance { baseColorMap ImageTexture { url ["apriltag_textures/tag36h11_0.png"] } }
にしてタグ面を作る。タグの「物理エッジ長」は板 Shape の寸法で決まる（PNG 内のタグ占有率と
合わせて calib ノードの tag_size と一致させること）。
"""

import argparse
import os

import cv2
import numpy as np


def make_tag_image(dictionary, tag_id, tag_px, margin_ratio):
    """タグ本体 tag_px[px] + 周囲白マージンの正方形 BGR 画像を返す。"""
    tag = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_px)
    margin = int(round(tag_px * margin_ratio))
    side = tag_px + 2 * margin
    canvas = np.full((side, side), 255, dtype=np.uint8)
    canvas[margin:margin + tag_px, margin:margin + tag_px] = tag
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR), margin, side


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ids', default='0,1,2,3',
                    help='生成するタグ ID（カンマ区切り）。既定は 4 方位パネル用 0..3')
    ap.add_argument('--tag-px', type=int, default=600,
                    help='タグ本体の画素数（マージン除く）')
    ap.add_argument('--margin-ratio', type=float, default=0.25,
                    help='タグ本体に対する白マージンの比（検出の quiet zone）')
    ap.add_argument('--out-dir', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'webots_worlds', 'apriltag_textures'))
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)

    ids = [int(s) for s in args.ids.split(',') if s.strip() != '']
    for tag_id in ids:
        img, margin, side = make_tag_image(
            dictionary, tag_id, args.tag_px, args.margin_ratio)
        path = os.path.join(args.out_dir, f'tag36h11_{tag_id}.png')
        cv2.imwrite(path, img)
        tag_frac = args.tag_px / float(side)
        print(f'wrote {path} ({side}x{side}px, tag fraction={tag_frac:.4f} '
              f'-> 物理板 1 辺 L のとき tag_size = L*{tag_frac:.4f})')


if __name__ == '__main__':
    main()
