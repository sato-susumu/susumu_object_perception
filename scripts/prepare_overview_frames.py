#!/usr/bin/env python3
"""prepare_overview_frames.py

生キャプチャされたフレーム (experiments/.../frames/<phase>/*.png) を入力に取り、
- 縦長 (height > width) なら 90° 回転 (ラベルも読める向きで一緒に回す)
- 左上にタスク名バッジを描画
- 旧版の名残として上端に黒帯バッジが焼き込まれた古いフレームの帯も保守的に除去
した中間フレームを別フォルダ (experiments/.../frames_labeled/<phase>/) に出力する。

GIF 合成 (render_overview_gif.py) はこの labeled フォルダのみを読む。生フレームを
破壊しないので、ラベル設計だけやり直したいときも capture をやり直さずに済む。

使い方:
  python3 scripts/prepare_overview_frames.py \\
      --frames-root experiments/overview_capture/2026-06-27_overview/frames \\
      --out-root   experiments/overview_capture/2026-06-27_overview/frames_labeled
"""
import argparse
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


PHASE_TITLES = {
    'mapping': '1. SLAM Mapping',
    'waypoints': '2. Waypoint Generation',
    'recognition': '3. Waypoint Patrol + Object Recognition',
}


def strip_legacy_band(im: Image.Image) -> Image.Image:
    """旧版で焼き込まれた半透明黒バッジ (上端帯) を保守的に上から固定除去."""
    w, h = im.size
    band_h = max(32, h // 16)
    if h > band_h + 50:
        im = im.crop((0, band_h, w, h))
    return im


def rotate_if_portrait(im: Image.Image) -> Image.Image:
    """縦長 (h>w) なら 90° 反時計回りに回して横長にする."""
    w, h = im.size
    if h > w:
        return im.rotate(90, expand=True)
    return im


def transform_point(px: int, py: int, orig_w: int, orig_h: int,
                    band_h: int, rotate: str):
    """生フレーム上の (px,py) を strip + 回転後の画像座標に変換.

    strip: 上端 band_h を除去 (= y を band_h 引く)。
    rotate=ccw (auto で縦長のとき): 90° 反時計回り (PIL.Image.rotate(90)).
    rotate=cw: 90° 時計回り.
    rotate=none: そのまま.

    PIL の Image.rotate(angle) は反時計回り。
    rotate(90, expand=True): 新しい画像サイズは (orig_h, orig_w)。
    変換: (x, y) -> (y, orig_w - 1 - x)。
    rotate(-90, expand=True): (x, y) -> (orig_h - 1 - y, x)。
    """
    y2 = py - band_h
    h2 = orig_h - band_h
    w2 = orig_w
    if y2 < 0 or y2 >= h2:
        return None
    if rotate == 'none':
        return (px, y2)
    if rotate == 'ccw':
        # rotate(90): (x,y) -> (y, w-1-x), new size (h2, w2)
        return (y2, w2 - 1 - px)
    if rotate == 'cw':
        # rotate(-90): (x,y) -> (h-1-y, x), new size (h2, w2)
        return (h2 - 1 - y2, px)
    return (px, y2)


def overlay_task_badge(img: Image.Image, title: str) -> Image.Image:
    """画面左上に大きなタスク名バッジを描画.

    フォントサイズはバッジが画像幅の 70% を超えないよう自動で縮小する.
    """
    out = img.convert('RGB').copy()
    draw = ImageDraw.Draw(out, 'RGBA')
    pad_x, pad_y = 10, 5
    margin = 10
    max_box_w = int(out.width * 0.55)
    font_size = max(14, out.height // 28)
    font = None
    for _ in range(20):
        try:
            font = ImageFont.truetype(
                '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
                font_size)
        except OSError:
            font = ImageFont.load_default()
            break
        bbox = draw.textbbox((0, 0), title, font=font)
        text_w = bbox[2] - bbox[0]
        if text_w + pad_x * 2 <= max_box_w or font_size <= 11:
            break
        font_size = max(11, font_size - 1)
    bbox = draw.textbbox((0, 0), title, font=font)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]
    box_w = text_w + pad_x * 2
    box_h = text_h + pad_y * 2
    x0, y0 = margin, margin
    x1, y1 = x0 + box_w, y0 + box_h
    draw.rectangle([x0, y0, x1, y1],
                   fill=(20, 20, 20, 230),
                   outline=(255, 215, 0, 255), width=2)
    draw.text((x0 + pad_x, y0 + pad_y - bbox[1]), title,
              fill=(255, 215, 0, 255), font=font)
    return out


def overlay_object_labels(img: Image.Image, objs: list):
    """物体ラベル文字を画像座標に対してまっすぐ描画.

    objs: [{'px': int, 'py': int, 'label': str, 'color': [r,g,b]}, ...]
    """
    if not objs:
        return img
    try:
        font = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 14)
    except OSError:
        font = ImageFont.load_default()
    draw = ImageDraw.Draw(img, 'RGBA')
    for o in objs:
        lbl = o.get('label', '')
        if not lbl:
            continue
        cx, cy = int(o['px']), int(o['py'])
        if not (0 <= cx < img.width and 0 <= cy < img.height):
            continue
        tx, ty = cx + 9, cy - 9
        bbox = draw.textbbox((tx, ty), lbl, font=font)
        # 画像右端を超える場合は左側に書く
        if bbox[2] > img.width - 4:
            tx = cx - 9 - (bbox[2] - bbox[0])
            bbox = draw.textbbox((tx, ty), lbl, font=font)
        # 画像上端を超える場合は下側に書く
        if bbox[1] < 2:
            ty = cy + 9
            bbox = draw.textbbox((tx, ty), lbl, font=font)
        draw.rectangle([bbox[0] - 2, bbox[1] - 1,
                        bbox[2] + 2, bbox[3] + 1],
                       fill=(255, 255, 255, 230))
        draw.text((tx, ty), lbl, fill=(20, 20, 20, 255), font=font)
    return img


def process_phase(src_dir: Path, dst_dir: Path, title: str, force_rotate: str):
    src_dir = src_dir.resolve()
    dst_dir = dst_dir.resolve()
    dst_dir.mkdir(parents=True, exist_ok=True)
    files = sorted(src_dir.glob('*.png'))
    if not files:
        print(f'WARN: no PNGs in {src_dir}', file=sys.stderr)
        return 0
    written = 0
    for fp in files:
        im_orig = Image.open(fp).convert('RGB')
        orig_w, orig_h = im_orig.size
        # strip 帯高さを判定 (strip_legacy_band と同じロジック)
        band_h = max(32, orig_h // 16)
        if orig_h <= band_h + 50:
            band_h = 0
        im = strip_legacy_band(im_orig)
        # 回転判定
        applied = 'none'
        if force_rotate == 'auto':
            if im.height > im.width:
                applied = 'ccw'
                im = im.rotate(90, expand=True)
        elif force_rotate == 'cw':
            applied = 'cw'
            im = im.rotate(-90, expand=True)
        elif force_rotate == 'ccw':
            applied = 'ccw'
            im = im.rotate(90, expand=True)
        # 物体ラベル: 隣の json があれば座標変換して書く
        meta_path = fp.with_suffix('.json')
        if meta_path.exists():
            try:
                with open(meta_path) as f:
                    meta = json.load(f)
                transformed = []
                for o in meta.get('objects', []):
                    p = transform_point(int(o['px']), int(o['py']),
                                        orig_w, orig_h, band_h, applied)
                    if p is None:
                        continue
                    transformed.append({
                        'px': p[0], 'py': p[1],
                        'label': o.get('label', ''),
                        'color': o.get('color', [0, 0, 0]),
                    })
                im = overlay_object_labels(im, transformed)
            except (OSError, json.JSONDecodeError) as e:
                print(f'WARN: failed to load {meta_path}: {e}',
                      file=sys.stderr)
        im = overlay_task_badge(im, title)
        im.save(dst_dir / fp.name)
        written += 1
    return written


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames-root', required=True,
                        help='生フレームの親 (frames/{mapping,waypoints,recognition})')
    parser.add_argument('--out-root', required=True,
                        help='ラベル付き出力の親 (frames_labeled/...)')
    parser.add_argument('--rotate', choices=['auto', 'none', 'cw', 'ccw'],
                        default='auto',
                        help='縦長フレームを横長に回転する向き。'
                             'auto=h>w なら 90° (=ccw)、'
                             'none=回転しない、cw=時計回り、ccw=反時計回り')
    args = parser.parse_args()

    src_root = Path(args.frames_root).resolve()
    dst_root = Path(args.out_root).resolve()
    total = 0
    for phase, title in PHASE_TITLES.items():
        src = src_root / phase
        dst = dst_root / phase
        if not src.is_dir():
            print(f'SKIP phase {phase}: missing {src}', file=sys.stderr)
            continue
        n = process_phase(src, dst, title, args.rotate)
        print(f'{phase}: {n} frames -> {dst}')
        total += n
    print(f'total: {total}')


if __name__ == '__main__':
    main()
