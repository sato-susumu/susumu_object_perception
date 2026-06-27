#!/usr/bin/env python3
"""render_waypoint_frames.py

既存の <world>_waypoints.yaml (map座標) と <world>.pgm/.yaml (地図) から、
ウェイポイントが 1 点ずつ巡回順に追加されていく PNG フレーム群を生成する。
ライブ実行不要 (artifact から再現可能なので後日も再生成しやすい)。

出力: <out-dir>/0000.png, 0001.png, ... (連番)
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont


def load_pgm(path: Path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        # コメント・トークン読み飛ばし
        def read_tokens(n):
            toks = []
            buf = b''
            while len(toks) < n:
                c = f.read(1)
                if not c:
                    break
                if c == b'#':
                    f.readline()
                    continue
                if c in (b' ', b'\t', b'\n', b'\r'):
                    if buf:
                        toks.append(buf.decode())
                        buf = b''
                else:
                    buf += c
            if buf and len(toks) < n:
                toks.append(buf.decode())
            return toks
        w, h, maxv = (int(t) for t in read_tokens(3))
        if magic == b'P5':
            data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
        elif magic == b'P2':
            toks = []
            for line in f:
                toks.extend(line.split())
            data = np.array([int(t) for t in toks[:w * h]],
                            dtype=np.uint8).reshape(h, w)
        else:
            raise ValueError(f'unsupported PGM magic: {magic!r}')
    return data, w, h, maxv


def pgm_to_rgb(data: np.ndarray) -> np.ndarray:
    """PGM (255=free, 0=occ, 205=unknown 付近) を RGB 化."""
    h, w = data.shape
    rgb = np.full((h, w, 3), 205, dtype=np.uint8)
    free = data >= 250
    occ = data <= 50
    rgb[free] = (254, 254, 254)
    rgb[occ] = (0, 0, 0)
    return rgb


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--waypoints-yaml', required=True)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--target-height', type=int, default=480)
    parser.add_argument('--hold-final', type=int, default=4,
                        help='最後のフレームを何枚複製するか (一時停止演出)')
    args = parser.parse_args()

    map_yaml_path = Path(args.map_yaml).resolve()
    with open(map_yaml_path) as f:
        mp = yaml.safe_load(f)
    pgm_path = (map_yaml_path.parent / mp['image']).resolve()
    res = float(mp['resolution'])
    origin = mp.get('origin', [0.0, 0.0, 0.0])
    ox, oy = float(origin[0]), float(origin[1])

    pgm, W, H, _ = load_pgm(pgm_path)
    rgb = pgm_to_rgb(pgm)

    # crop to known
    known = (pgm <= 50) | (pgm >= 250)
    ys, xs = np.where(known)
    pad = 10
    if len(ys) == 0:
        y0, y1, x0, x1 = 0, H, 0, W
    else:
        y0 = max(0, ys.min() - pad)
        y1 = min(H, ys.max() + pad + 1)
        x0 = max(0, xs.min() - pad)
        x1 = min(W, xs.max() + pad + 1)
    rgb_crop = rgb[y0:y1, x0:x1]

    with open(args.waypoints_yaml) as f:
        wpy = yaml.safe_load(f)
    points = []
    for wp in wpy.get('waypoints', []):
        if isinstance(wp, (list, tuple)):
            x, y = float(wp[0]), float(wp[1])
        else:
            x = float(wp.get('x', wp.get('pose', {}).get('x', 0.0)))
            y = float(wp.get('y', wp.get('pose', {}).get('y', 0.0)))
        col = int((x - ox) / res) - x0
        # PGM の y は地図の Y up 座標を Y down にしたもの。slam_toolbox 等の
        # map_saver 出力は「画像 row 0 が y=origin_y + height*res」の表記。
        row = (H - 1 - int((y - oy) / res)) - y0
        points.append((col, row))

    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    base_img = Image.fromarray(rgb_crop)
    target_h = args.target_height
    if base_img.height != target_h:
        ratio = target_h / base_img.height
        new_w = max(1, int(base_img.width * ratio))
        base_img = base_img.resize((new_w, target_h), Image.NEAREST)
        scale = ratio
    else:
        scale = 1.0

    n = len(points)
    n_frames = max(n, 1)
    scaled_points = [(int(c * scale), int(r * scale)) for c, r in points]

    for i in range(n_frames):
        img = base_img.copy()
        draw = ImageDraw.Draw(img)
        # 巡回経路 (累積)
        if i >= 1:
            draw.line([scaled_points[j] for j in range(i + 1)],
                      fill=(50, 130, 240), width=2)
        # 既存点
        for j in range(i + 1):
            c, r = scaled_points[j]
            rr = 4 if j == i else 3
            color = (240, 50, 50) if j == i else (50, 130, 240)
            draw.ellipse([c - rr, r - rr, c + rr, r + rr],
                         fill=color, outline=(0, 0, 0))
        # 旧バッジは GIF 側で重ねるため、画像内には描かない
        img.save(out_dir / f'{i:04d}.png')

    # 最後の状態を hold_final 枚複製 (静止)
    if n_frames > 0 and args.hold_final > 0:
        last_path = out_dir / f'{n_frames - 1:04d}.png'
        last = Image.open(last_path)
        for k in range(args.hold_final):
            last.save(out_dir / f'{n_frames + k:04d}.png')

    print(f'wrote {n_frames + max(0, args.hold_final)} waypoint frames '
          f'to {out_dir}')


if __name__ == '__main__':
    main()
