#!/usr/bin/env python3
"""出来上がった地図(PGM/YAML)を Webots world(wbt)の真の構造と照合する（検証用）。

地図内部の整合（連結成分・壁率）だけでは「地図が実 world と合っているか」は分からない。
このスクリプトは wbt の Floor / Wall / 障害物(PottedTree, SimpleBuilding 等)の
translation・size をパースし、地図 PNG に **真の壁(赤線)・床範囲(青枠)・障害物(緑)** を
重ねて出力する。地図の occ(黒)が赤線に沿い、free(白)が床範囲に収まっていれば world と一致。
ズレ・歪み・星形（実構造を成さない）なら不合格。

使い方:
  python3 check_map_vs_world.py --wbt webots_worlds/indoor.wbt --map maps/indoor.yaml \
    --out /tmp/indoor_check.png

注意: Webots は y-up でなく z-up、地図(map)は SLAM 起点が原点。wbt の world 座標と
map 座標には「ロボット初期位置ぶんのオフセット」がある。ロボット初期 translation を
--robot で渡すと map 原点(=ロボット初期位置)に合わせて wbt 座標を平行移動して重ねる。
"""

import argparse
import re

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def load_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()

        def rt():
            tok = b''
            while True:
                c = f.read(1)
                if c in (b' ', b'\t', b'\n', b'\r'):
                    if tok:
                        return tok
                elif c == b'#':
                    f.readline()
                else:
                    tok += c
        w = int(rt())
        h = int(rt())
        int(rt())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8)
    return data.reshape(h, w)


def parse_wbt(path):
    """wbt から Floor / Wall / 障害物の translation, size, rotation を雑にパースする。

    Webots の node は `Type { ... translation x y z ... size sx sy sz ... }` 形式。
    ネストを完全には追わず、Floor/Wall/PottedTree/SimpleBuilding ノードごとに
    直後の translation/size/rotation を拾う（このプロジェクトの world は単純な配置なので可）。
    """
    text = open(path).read()
    objs = []
    # ノード開始位置を全部見つける。
    for m in re.finditer(r'\b(Floor|Wall|PottedTree|SimpleBuilding|RectangleArena)\b\s*\{', text):
        typ = m.group(1)
        # このノードの { から対応する } までを雑に取る（ネスト数えで）。
        i = m.end() - 1
        depth = 0
        j = i
        while j < len(text):
            if text[j] == '{':
                depth += 1
            elif text[j] == '}':
                depth -= 1
                if depth == 0:
                    break
            j += 1
        body = text[i:j]
        tr = re.search(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', body)
        sz = re.search(r'\bsize\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?', body)
        rot = re.search(r'rotation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', body)
        t = (float(tr.group(1)), float(tr.group(2))) if tr else (0.0, 0.0)
        s = None
        if sz:
            s = (float(sz.group(1)), float(sz.group(2)))
        yaw = float(rot.group(4)) if (rot and abs(float(rot.group(3))) > 0.5) else 0.0
        objs.append((typ, t, s, yaw))
    return objs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wbt', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    # ロボット初期位置(world座標)。map 原点(=起点)に合わせるため wbt 座標から引く。
    ap.add_argument('--robot', nargs=2, type=float, default=None,
                    help='ロボット初期 translation x y（省略時 wbt から TurtleBot を探す）')
    args = ap.parse_args()

    meta = yaml.safe_load(open(args.map))
    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    import os
    img = load_pgm(os.path.join(os.path.dirname(args.map), meta['image']))
    h, w = img.shape

    objs = parse_wbt(args.wbt)
    # ロボット初期位置（world→map のオフセット）。
    if args.robot:
        rxw, ryw = args.robot
    else:
        txt = open(args.wbt).read()
        m = re.search(r'TurtleBot\w*\s*\{[^}]*?translation\s+([-\d.]+)\s+([-\d.]+)', txt)
        rxw, ryw = (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)
    print(f'robot world pos = ({rxw}, {ryw}) を map 原点に合わせる')

    def world_to_map_px(wx, wy):
        # world → map座標(ロボット初期位置を原点に) → ピクセル
        mx = wx - rxw
        my = wy - ryw
        px = (mx - ox) / res
        py = (my - oy) / res  # PGM は上下反転だが imshow origin=lower で合わせる
        return px, py

    fig, ax = plt.subplots(figsize=(w / 40, h / 40))
    # PGM(map_server規約)は img[0] が画像最上行＝map y 最大。world点の py は origin からの
    # 上方向距離 (my-oy)/res なので、PGM をそのまま origin='lower' で出すと上下が反転する
    # （break_room で 180度ズレて見えた原因）。img を上下反転して y 向きを world と一致させる。
    ax.imshow(img[::-1], cmap='gray', vmin=0, vmax=255, origin='lower')
    for (typ, (tx, ty), s, yaw) in objs:
        px, py = world_to_map_px(tx, ty)
        if typ in ('Floor', 'RectangleArena') and s:
            wpx, hpx = s[0] / res, s[1] / res
            # Floor の rotation(z軸 90deg)を反映。size はローカル座標なので、90度回転
            # した床は world 上で幅高さが入れ替わる（break_room は Floor が 90度回転して
            # おり、これを無視すると照合が 90度ズレて見えた）。
            if abs(abs(yaw) - 1.5708) < 0.3:
                wpx, hpx = hpx, wpx
            ax.add_patch(Rectangle((px - wpx / 2, py - hpx / 2), wpx, hpx,
                         fill=False, edgecolor='blue', lw=2, label='floor'))
        elif typ == 'Wall' and s:
            wpx, hpx = s[0] / res, s[1] / res
            # yaw 90deg なら幅高さ入替（簡易）。
            if abs(abs(yaw) - 1.5708) < 0.3:
                wpx, hpx = hpx, wpx
            ax.add_patch(Rectangle((px - wpx / 2, py - hpx / 2), wpx, hpx,
                         fill=False, edgecolor='red', lw=2))
        else:  # 障害物
            ax.plot(px, py, 'g+', ms=12, mew=2)
    ax.set_title(f'{os.path.basename(args.map)} vs {os.path.basename(args.wbt)}\n'
                 'red=wbt Wall, blue=Floor, green=obstacle. 地図のocc(黒)が赤線に沿えば一致')
    fig.savefig(args.out, dpi=80, bbox_inches='tight')
    print(f'saved {args.out}')
    # 寸法サマリ
    floors = [o for o in objs if o[0] in ('Floor', 'RectangleArena') and o[2]]
    walls = [o for o in objs if o[0] == 'Wall']
    print(f'wbt: Floor {len(floors)}個 Wall {len(walls)}個 障害物 '
          f'{len(objs)-len(floors)-len(walls)}個')
    for typ, t, s, yaw in floors:
        print(f'  Floor at world{t} size {s} (map原点基準 {t[0]-rxw:.1f},{t[1]-ryw:.1f})')
    print(f'地図実寸 {w*res:.1f}x{h*res:.1f}m')


if __name__ == '__main__':
    main()
