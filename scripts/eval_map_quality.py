#!/usr/bin/env python3
"""保存済み地図の品質をゴール条件で自動評価する（検証用・オフライン）。

AGENTS.md「地図作成タスクのゴール条件」の A 幾何的正しさ / B 完全性 を、保存済み
PGM/YAML から測れる範囲で定量化する。目視に頼らず「どの world がどこでダメか」を
客観的に出すためのもの。

評価指標:
  - 連結成分: free 空間を connect_clearance で通行可能領域にし、最大連結成分が
    free 全体の何 % か（B⑥ 分断されていないか。低いほど斜めノイズ等で分断＝悪い）。
  - 孤立小片の数: 連結成分の個数（多いほどノイズで細切れ＝悪い）。
  - 壁率: occ / (free+occ)（壁がちゃんと captured されているか）。
  - unknown 比率: 地図領域に占める unknown（B⑤ カバー。外周の unknown は除く参考値）。
  - 寸法: 地図の実寸（A④ 実環境と照合する材料）。

使い方:
  python3 eval_map_quality.py maps/indoor.yaml [maps/break_room.yaml ...]
  python3 eval_map_quality.py --connect-clearance 0.3 maps/*.yaml
"""

import argparse
import os

import numpy as np
import yaml
from scipy import ndimage


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


def evaluate(map_yaml, connect_clearance):
    meta = yaml.safe_load(open(map_yaml))
    res = float(meta['resolution'])
    pgm = os.path.join(os.path.dirname(map_yaml), meta['image'])
    img = load_pgm(pgm)
    h, w = img.shape
    # map_server trinary: unknown(205) は p=0.196 で free_thresh(0.25)未満になり free に
    # 誤分類される。この誤分類があると未探索だらけの地図でも「unknown 0」と誤報告し、不良地図を
    # 見逃す。map_server 規約どおり「205 は unknown」を厳守するため free は「明確に白(>=250)」に限定。
    p = (255.0 - img.astype(np.float32)) / 255.0
    occ = p >= float(meta.get('occupied_thresh', 0.65))
    free = (img >= 250) & ~occ
    unknown = ~free & ~occ

    n_free = int(free.sum())
    n_occ = int(occ.sum())
    n_unk = int(unknown.sum())
    wall_rate = 100.0 * n_occ / max(n_free + n_occ, 1)

    # 連結成分: connect_clearance で通行可能領域を作り最大成分の割合を見る。
    blocked = occ | unknown
    dist = ndimage.distance_transform_edt(~blocked)
    conn = free & (dist >= connect_clearance / res)
    lab, nlab = ndimage.label(conn)
    if nlab > 0:
        sizes = ndimage.sum(np.ones_like(lab), lab, range(1, nlab + 1))
        # 意味のある大きさ（>=50セル=0.125m2）の成分だけを「部屋/領域」とみなす。
        # 微小片（壁際の clearance 未満の薄い帯やノイズ）は分断と数えない。
        big_sizes = sizes[sizes >= 50]
        big = int(len(big_sizes))
        # 最大成分が「意味ある連結領域の合計」に占める割合（壁際の細片は分母から除外）。
        # これが高い＝主要な通行可能空間が 1 つに繋がっている。
        main_rate = (100.0 * float(max(sizes)) / max(float(big_sizes.sum()), 1.0)
                     if big else 0.0)
    else:
        main_rate = 0.0
        big = 0

    return dict(
        world=os.path.splitext(os.path.basename(map_yaml))[0],
        wh=f'{w}x{h}', size_m=f'{w*res:.1f}x{h*res:.1f}',
        free=n_free, occ=n_occ, unk=n_unk,
        wall_rate=wall_rate, main_rate=main_rate, n_components=big)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('maps', nargs='+', help='地図 yaml のパス（複数可）')
    ap.add_argument('--connect-clearance', type=float, default=0.3)
    args = ap.parse_args()

    print(f'{"world":12s} {"寸法[m]":10s} {"壁率%":>6s} '
          f'{"最大連結成分%":>12s} {"連結片数":>8s} {"unknown":>8s}  判定')
    print('-' * 78)
    for mp in args.maps:
        try:
            r = evaluate(mp, args.connect_clearance)
        except Exception as e:
            print(f'{mp}: 評価失敗 {e}')
            continue
        # unknown 比率 = 地図全体（free+occ+unknown）に占める unknown の割合。屋内は閉じた
        # 空間なので探索しきれば unknown は小さくなる。大きい＝未探索だらけの不良地図。
        total = r['free'] + r['occ'] + r['unk']
        unk_ratio = r['unk'] / total if total else 0.0
        # 判定: まず unknown が多すぎる地図を NG にする（未探索＝不良）。次に連結領域が
        # 1 個（=主要空間が分断されていない）かつ最大成分が大半(>=90%)なら OK。
        if unk_ratio >= 0.30:
            verdict = f'未探索多い(unknown {unk_ratio*100:.0f}%)'
        elif r['n_components'] <= 1:
            verdict = 'OK' if unk_ratio < 0.10 else f'OK(unknown {unk_ratio*100:.0f}%)'
        elif r['main_rate'] >= 90:
            verdict = 'OK(微小片あり)'
        else:
            verdict = f'分断あり(主要領域{r["n_components"]}個)'
        print(f'{r["world"]:12s} {r["size_m"]:10s} {r["wall_rate"]:6.1f} '
              f'{r["main_rate"]:12.0f} {r["n_components"]:8d} {r["unk"]:8d}  '
              f'{verdict}')
    print('-' * 78)
    print('最大連結成分%: free のうち最大の通行可能領域が占める割合（高いほど良い、'
          '低い=斜めノイズ等で分断）。連結片数: 意味ある大きさの連結成分数（少ないほど良い）。')


if __name__ == '__main__':
    main()
