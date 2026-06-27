#!/usr/bin/env python3
"""render_overview_gif.py

ラベル付き加工版フレーム (frames_labeled/{mapping,waypoints,recognition}/) を
1 つの GIF に連結する。ラベル書き込み・回転・帯除去は前段の
prepare_overview_frames.py で済んでいる前提。

サイズ制約 (--max-mb) を超える場合は fps と総枚数を自動で削減して再生成する。
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image


PHASE_ORDER = ['mapping', 'waypoints', 'recognition']


def gather_frames(frames_root: Path) -> list:
    items = []
    for phase in PHASE_ORDER:
        d = frames_root / phase
        if not d.is_dir():
            print(f'WARN: phase dir missing: {d}', file=sys.stderr)
            continue
        files = sorted(d.glob('*.png'))
        if not files:
            print(f'WARN: no frames in {d}', file=sys.stderr)
            continue
        items.append((phase, files))
    return items


def resize_contain(img: Image.Image, box_w: int, box_h: int) -> Image.Image:
    ratio = min(box_w / img.width, box_h / img.height)
    new_w = max(1, int(img.width * ratio))
    new_h = max(1, int(img.height * ratio))
    return img.resize((new_w, new_h), Image.LANCZOS)


def pad_to_canvas(img: Image.Image, canvas_w: int, canvas_h: int,
                  bg=(20, 20, 20)) -> Image.Image:
    bg_img = Image.new('RGB', (canvas_w, canvas_h), bg)
    x = (canvas_w - img.width) // 2
    y = (canvas_h - img.height) // 2
    bg_img.paste(img, (x, y))
    return bg_img


def downsample(files: list, max_n: int) -> list:
    if len(files) <= max_n:
        return files
    if max_n <= 2:
        return [files[0], files[-1]][:max_n]
    step = (len(files) - 1) / (max_n - 1)
    idxs = [round(i * step) for i in range(max_n)]
    idxs = sorted(set(idxs))
    return [files[i] for i in idxs]


def render_gif(frames_root: Path, out_path: Path, target_w: int,
               fps: float, max_per_phase: int,
               hold_last_frames: int) -> Path:
    items = gather_frames(frames_root)
    if not items:
        print('ERROR: no frames found', file=sys.stderr)
        sys.exit(1)

    # 共通キャンバス: 横長 target_w 幅 + 縦は target_w * 0.65 で固定 (README に収まる比率)
    canvas_w = target_w
    canvas_h = int(target_w * 0.65)

    with tempfile.TemporaryDirectory() as tmp:
        tmp_dir = Path(tmp)
        out_idx = 0
        for phase, files in items:
            picked = downsample(files, max_per_phase)
            for fp in picked:
                im = Image.open(fp).convert('RGB')
                im = resize_contain(im, canvas_w, canvas_h)
                im = pad_to_canvas(im, canvas_w, canvas_h)
                im.save(tmp_dir / f'{out_idx:05d}.png')
                out_idx += 1
            if hold_last_frames > 0 and picked:
                im = Image.open(picked[-1]).convert('RGB')
                im = resize_contain(im, canvas_w, canvas_h)
                im = pad_to_canvas(im, canvas_w, canvas_h)
                for _ in range(hold_last_frames):
                    im.save(tmp_dir / f'{out_idx:05d}.png')
                    out_idx += 1

        palette = tmp_dir / 'palette.png'
        cmd_pal = [
            'ffmpeg', '-y', '-framerate', str(fps),
            '-i', str(tmp_dir / '%05d.png'),
            '-vf', f'fps={fps},palettegen=stats_mode=diff',
            str(palette),
        ]
        subprocess.run(cmd_pal, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        cmd_gif = [
            'ffmpeg', '-y', '-framerate', str(fps),
            '-i', str(tmp_dir / '%05d.png'),
            '-i', str(palette),
            '-lavfi',
            f'fps={fps} [x]; [x][1:v] paletteuse=dither=bayer:bayer_scale=5'
            ':diff_mode=rectangle',
            str(out_path),
        ]
        subprocess.run(cmd_gif, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    return out_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames-root', required=True,
                        help='ラベル付き加工フォルダ (frames_labeled)')
    parser.add_argument('--out', required=True)
    parser.add_argument('--target-width', type=int, default=1000)
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--max-per-phase', type=int, default=40)
    parser.add_argument('--hold-last-frames', type=int, default=8)
    parser.add_argument('--max-mb', type=float, default=10.0)
    args = parser.parse_args()

    frames_root = Path(args.frames_root).resolve()
    out_path = Path(args.out).resolve()

    fps = args.fps
    max_per_phase = args.max_per_phase
    target_w = args.target_width
    for attempt in range(6):
        render_gif(frames_root, out_path, target_w, fps, max_per_phase,
                   args.hold_last_frames)
        size_mb = out_path.stat().st_size / (1024 * 1024)
        print(f'attempt={attempt} size={size_mb:.2f}MB '
              f'fps={fps} max_per_phase={max_per_phase} '
              f'width={target_w}')
        if size_mb <= args.max_mb:
            print(f'OK: {out_path} ({size_mb:.2f}MB)')
            return
        if max_per_phase > 20:
            max_per_phase = int(max_per_phase * 0.75)
        elif fps > 6:
            fps = max(6, fps * 0.8)
        elif target_w > 700:
            target_w = max(700, int(target_w * 0.9))
        else:
            print(f'WARN: cannot shrink further, accepting {size_mb:.2f}MB')
            return
    print(f'WARN: ran out of attempts, final {out_path}')


if __name__ == '__main__':
    main()
