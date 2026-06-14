#!/usr/bin/env python3
"""cafe.pgm から机（テーブル）の占有ピクセルを除去するユーティリティ。

perception の 2D 地図照合（object_tracker_node の wall_margin）は「地図上の壁
近傍に居るトラックを壁際ゴースト（緑ボックス）として消す」。だが SLAM 時に机も
薄く占有セルとして焼き込まれており、margin を広げると机検出まで巻き込んでしまう。
机は残したい（人も机も残し、壁際ゴーストだけ消す）ので、地図照合用の地図から机を
消しておく。机は /velodyne_points -> voxel_layer で動的に障害物化するため、static
から消えても Nav2 の衝突回避は効く。

机の位置は cafe world の worldfile の 5 卓（agents 検証で使った真値と同一）。
各卓中心から半径 CLEAR_RADIUS_M を free(254) に塗る。外周壁からは十分離れている
ので壁は無傷（実測: 占有 3102->3069px、机周辺 33px のみ除去）。

使い方:  python3 maps/clear_tables.py
  cafe.pgm.bak（机ありの元地図）から机を消して cafe.pgm を再生成する。
  cafe.pgm.bak が無ければ現 cafe.pgm をバックアップしてから処理する。
"""

import os
import shutil
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PGM = os.path.join(HERE, 'cafe.pgm')
BAK = os.path.join(HERE, 'cafe.pgm.bak')

# cafe.yaml と一致させる。
RES = 0.05
ORIGIN_X = -5.41
ORIGIN_Y = -12.2
# px<OCC_PX_THRESH を占有とみなす（occupied_thresh 0.65 -> 0.35*255 = 89）。
OCC_PX_THRESH = 89
CLEAR_RADIUS_M = 0.5

# cafe world の 5 卓中心 [m]（map フレーム）。
TABLES = [(0.5, -1.6), (2.4, -5.5), (-1.5, -5.5), (2.4, -9.0), (-1.5, -9.0)]


def read_pgm(path):
    hdr = []
    with open(path, 'rb') as f:
        hdr.append(f.readline())               # P5
        line = f.readline()
        while line.startswith(b'#'):           # comment
            hdr.append(line)
            line = f.readline()
        hdr.append(line)                       # "W H"
        w, h = map(int, line.split())
        hdr.append(f.readline())               # maxval
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w).copy()
    return hdr, w, h, data


def main():
    # 元地図（机あり）を確保。初回は現 cafe.pgm をバックアップする。
    if not os.path.exists(BAK):
        shutil.copy(PGM, BAK)
        print('backup created: ' + BAK)

    hdr, w, h, data = read_pgm(BAK)
    r = int(round(CLEAR_RADIUS_M / RES))
    before = int((data < OCC_PX_THRESH).sum())
    cleared = 0
    for mx, my in TABLES:
        cx = int((mx - ORIGIN_X) / RES)
        cy = int((my - ORIGIN_Y) / RES)
        py = h - 1 - cy                        # PGM は行 0 が地図上端（y 反転）
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                yy, xx = py + dy, cx + dx
                if 0 <= yy < h and 0 <= xx < w and data[yy, xx] < OCC_PX_THRESH:
                    data[yy, xx] = 254
                    cleared += 1
    after = int((data < OCC_PX_THRESH).sum())

    with open(PGM, 'wb') as f:
        for line in hdr:
            f.write(line)
        f.write(data.tobytes())
    print('wrote %s: occupied px %d -> %d (%d px around %d tables cleared)'
          % (PGM, before, after, cleared, len(TABLES)))


if __name__ == '__main__':
    main()
