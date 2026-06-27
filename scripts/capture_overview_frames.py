#!/usr/bin/env python3
"""capture_overview_frames.py

ROS2 を購読しながら /map または object_memory DB を周期サンプリングし、
GIF 用の連番 PNG フレームを experiments/overview_capture/<日付>_<world>/frames/<phase>/ に保存する。

phase=mapping  : /map (OccupancyGrid) を購読、変化が一定以上あれば PNG 保存
phase=recognize: /map を背景に object_memory DB の物体を載せた PNG を周期保存

後段の render_overview_gif.py がこの PNG 群を読んで 1 つの GIF に合成する。
"""

import argparse
import json
import math
import os
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry


def occupancy_to_rgb(grid: np.ndarray) -> np.ndarray:
    """OccupancyGrid (-1=unknown, 0=free, 100=occ) を RGB 配列に変換 (height, width, 3)."""
    h, w = grid.shape
    rgb = np.full((h, w, 3), 205, dtype=np.uint8)  # unknown=灰
    free_mask = (grid >= 0) & (grid < 50)
    occ_mask = grid >= 50
    rgb[free_mask] = (254, 254, 254)
    rgb[occ_mask] = (0, 0, 0)
    return rgb


def crop_to_known(rgb: np.ndarray, grid: np.ndarray, pad: int = 6) -> tuple:
    """既知セル (free or occ) を含む最小矩形にクロップ。返り値 (rgb_cropped, (y0,x0))."""
    known = grid >= 0
    if not known.any():
        return rgb, (0, 0)
    ys, xs = np.where(known)
    y0, y1 = max(0, ys.min() - pad), min(rgb.shape[0], ys.max() + pad + 1)
    x0, x1 = max(0, xs.min() - pad), min(rgb.shape[1], xs.max() + pad + 1)
    return rgb[y0:y1, x0:x1], (y0, x0)


class MappingCapture(Node):
    def __init__(self, out_dir: Path, period_sec: float, min_change_cells: int,
                 max_frames: int):
        super().__init__('overview_mapping_capture')
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.period_sec = period_sec
        self.min_change_cells = min_change_cells
        self.max_frames = max_frames
        self.frame_index = 0
        self.last_known_count = -1
        self.last_save_t = 0.0
        self.last_grid = None
        self.last_map_info = None  # (origin_x, origin_y, res, height)
        self.robot_xy = None  # (x, y, yaw) in map frame
        # latched map (TRANSIENT_LOCAL) も取れるよう RELIABLE + TRANSIENT_LOCAL
        qos_map = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos_map)
        qos_odom = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_odom)
        self.get_logger().info(
            f'mapping capture started: out={out_dir} period={period_sec}s')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.robot_xy = (p.position.x, p.position.y, yaw)

    def on_map(self, msg: OccupancyGrid):
        now = time.time()
        h, w = msg.info.height, msg.info.width
        if h == 0 or w == 0:
            return
        grid_yup = np.array(msg.data, dtype=np.int16).reshape(h, w)
        # OccupancyGrid は左下原点 (Y up)。画像は左上原点 (Y down)。上下反転して画像へ。
        grid = np.flipud(grid_yup)
        known_count = int((grid >= 0).sum())
        change = known_count - self.last_known_count
        if (now - self.last_save_t) < self.period_sec and change < self.min_change_cells:
            return
        if known_count == self.last_known_count and self.frame_index > 0:
            return
        rgb = occupancy_to_rgb(grid)
        cropped, (y0, x0) = crop_to_known(rgb, grid, pad=10)
        if cropped.size == 0:
            return
        img = Image.fromarray(cropped)

        # 地図情報を保存 (ロボット位置変換用)
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution
        H = h  # OccupancyGrid 高さ (=画像高 = grid_img の行数)

        # crop 後のロボット位置 (PGM ピクセル) を算出
        robot_crop_px = None
        robot_yaw = None
        if self.robot_xy is not None and res > 0:
            rx, ry, ryaw = self.robot_xy
            col_pgm = int((rx - ox) / res)
            row_pgm = H - 1 - int((ry - oy) / res)
            col = col_pgm - x0
            row = row_pgm - y0
            if 0 <= col < img.width and 0 <= row < img.height:
                robot_crop_px = (col, row)
                robot_yaw = ryaw

        # 高さ固定 (480px) で書き出し、最後のリサイズは GIF 側
        target_h = 480
        if img.height != target_h:
            ratio = target_h / img.height
            img = img.resize((max(1, int(img.width * ratio)), target_h),
                             Image.NEAREST)
            if robot_crop_px is not None:
                robot_crop_px = (int(robot_crop_px[0] * ratio),
                                 int(robot_crop_px[1] * ratio))

        # リサイズ後の最終画像上に三角を描く (size は最終解像度基準なので
        # 地図のスケール変動の影響を受けず、 recognition と同じ見栄えになる)
        robot_px = None
        if robot_crop_px is not None and robot_yaw is not None:
            col, row = robot_crop_px
            draw = ImageDraw.Draw(img, 'RGBA')
            size = 10
            ia = -robot_yaw
            pts = []
            for ang_off in (0.0, 2.5, -2.5):
                ang = ia + ang_off
                pts.append((col + size * math.cos(ang),
                            row + size * math.sin(ang)))
            draw.polygon(pts, fill=(20, 20, 220, 255),
                         outline=(255, 255, 255))
            robot_px = (int(col), int(row))

        # 旧バッジは GIF 側で重ねるため、画像内には描かない
        out_path = self.out_dir / f'{self.frame_index:04d}.png'
        img.save(out_path)
        # ロボット位置の JSON メタ (prepare 側で回転変換しても向きを維持できるよう yaw も保存)
        meta = {
            'image_size': [img.width, img.height],
            'robot': None,
        }
        if robot_px is not None and self.robot_xy is not None:
            meta['robot'] = {
                'px': robot_px[0],
                'py': robot_px[1],
                'yaw': float(self.robot_xy[2]),
            }
        meta_path = self.out_dir / f'{self.frame_index:04d}.json'
        with open(meta_path, 'w') as f:
            json.dump(meta, f)

        self.frame_index += 1
        self.last_known_count = known_count
        self.last_save_t = now
        self.last_grid = grid
        if self.frame_index % 5 == 0:
            self.get_logger().info(f'saved {self.frame_index} mapping frames')
        if self.frame_index >= self.max_frames:
            self.get_logger().info('max_frames reached, shutting down')
            rclpy.shutdown()


def load_static_map(map_yaml_path: Path):
    """outputs/mapping_indoor/<world>.yaml + .pgm を読んで OccupancyGrid 風の dict を返す."""
    with open(map_yaml_path) as f:
        mp = yaml.safe_load(f)
    pgm_path = (map_yaml_path.parent / mp['image']).resolve()
    res = float(mp['resolution'])
    origin = mp.get('origin', [0.0, 0.0, 0.0])
    ox, oy = float(origin[0]), float(origin[1])
    with open(pgm_path, 'rb') as f:
        magic = f.readline().strip()

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
        w, h, _ = (int(t) for t in read_tokens(3))
        if magic == b'P5':
            data = np.frombuffer(f.read(w * h),
                                 dtype=np.uint8).reshape(h, w)
        elif magic == b'P2':
            toks = []
            for line in f:
                toks.extend(line.split())
            data = np.array([int(t) for t in toks[:w * h]],
                            dtype=np.uint8).reshape(h, w)
        else:
            raise ValueError(f'unsupported PGM magic: {magic!r}')
    # PGM (Y-down 画像) → OccupancyGrid 風 (Y-up): 0=occ,100=occ; 254=free,0=free; 205=unknown
    # render しやすいよう 「Y-down 画像のまま 0/100/-1」 を作る
    img_grid = np.full((h, w), -1, dtype=np.int16)
    img_grid[data >= 250] = 0
    img_grid[data <= 50] = 100
    return {
        'image_grid': img_grid,  # Y-down 画像座標 (PNG と同じ向き)
        'origin': (ox, oy),
        'resolution': res,
        'height': h,
        'width': w,
    }


class RecognitionCapture(Node):
    """object_memory SQLite DB + /odom + waypoints YAML を周期 poll、
    静的地図の上に「全 WP・巡回経路・現在ロボット位置・認識物体 (ラベル付き)」を描画して保存."""

    PALETTE = [
        (231, 76, 60), (52, 152, 219), (46, 204, 113),
        (241, 196, 15), (155, 89, 182), (230, 126, 34),
        (26, 188, 156), (236, 112, 99), (52, 73, 94),
        (243, 156, 18),
    ]

    def __init__(self, out_dir: Path, db_path: Path,
                 period_sec: float, max_frames: int,
                 map_yaml: Path, waypoints_yaml: Path):
        super().__init__('overview_recognition_capture')
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.db_path = db_path
        self.period_sec = period_sec
        self.max_frames = max_frames
        self.frame_index = 0
        self.robot_xy = None  # (x, y, yaw) in map frame
        self.class_colors = {}

        # 静的地図 (frontier 完了後の保存マップ) を使う
        self.map_data = load_static_map(map_yaml)
        ig = self.map_data['image_grid']
        rgb = occupancy_to_rgb(ig)
        cropped, (self.y0, self.x0) = crop_to_known(rgb, ig, pad=10)
        self.bg_img_raw = Image.fromarray(cropped)
        self.target_h = 480
        if self.bg_img_raw.height != self.target_h:
            ratio = self.target_h / self.bg_img_raw.height
            self.bg_img = self.bg_img_raw.resize(
                (max(1, int(self.bg_img_raw.width * ratio)),
                 self.target_h), Image.NEAREST)
            self.scale = ratio
        else:
            self.bg_img = self.bg_img_raw.copy()
            self.scale = 1.0

        # waypoints (map 座標) を画像座標に変換しておく
        self.waypoints_xy = []
        self.waypoints_px = []
        try:
            with open(waypoints_yaml) as f:
                wpy = yaml.safe_load(f)
            for wp in wpy.get('waypoints', []) or []:
                if isinstance(wp, (list, tuple)):
                    x, y = float(wp[0]), float(wp[1])
                else:
                    x = float(wp.get('x', wp.get('pose', {}).get('x', 0.0)))
                    y = float(wp.get('y', wp.get('pose', {}).get('y', 0.0)))
                self.waypoints_xy.append((x, y))
                self.waypoints_px.append(self.map_xy_to_image(x, y))
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().warn(f'failed to load waypoints: {e}')

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos)
        self.timer = self.create_timer(period_sec, self.on_timer)
        self.get_logger().info(
            f'recognition capture started: out={out_dir} db={db_path} '
            f'waypoints={len(self.waypoints_px)} pts')

    def map_xy_to_image(self, x: float, y: float):
        """map 座標 (x,y) を最終出力画像のピクセル座標へ変換."""
        ox, oy = self.map_data['origin']
        res = self.map_data['resolution']
        H = self.map_data['height']
        col_pgm = int((x - ox) / res)
        # PGM (Y-down 画像) の row。slam_toolbox の origin は左下基準。
        row_pgm = H - 1 - int((y - oy) / res)
        col_crop = col_pgm - self.x0
        row_crop = row_pgm - self.y0
        return (int(col_crop * self.scale), int(row_crop * self.scale))

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        # yaw のみ抽出
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.robot_xy = (p.position.x, p.position.y, yaw)

    def fetch_objects(self):
        if not self.db_path.exists():
            return []
        try:
            con = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True,
                                  timeout=0.5)
            cur = con.cursor()
            cur.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            )
            tables = [r[0] for r in cur.fetchall()]
            target = None
            for t in ('objects', 'object_memory', 'objects_v2'):
                if t in tables:
                    target = t
                    break
            if target is None:
                con.close()
                return []
            cur.execute(f'PRAGMA table_info({target})')
            cols = [r[1] for r in cur.fetchall()]
            x_col = next((c for c in ('x', 'pos_x', 'px') if c in cols), None)
            y_col = next((c for c in ('y', 'pos_y', 'py') if c in cols), None)
            cls_col = next((c for c in ('fine_class', 'class_name', 'label')
                            if c in cols), None)
            if not x_col or not y_col:
                con.close()
                return []
            sel = f'{x_col}, {y_col}'
            if cls_col:
                sel += f', {cls_col}'
            cur.execute(f'SELECT {sel} FROM {target}')
            rows = cur.fetchall()
            con.close()
            out = []
            for r in rows:
                x, y = float(r[0]), float(r[1])
                lbl = str(r[2]) if cls_col and len(r) > 2 and r[2] else ''
                out.append((x, y, lbl))
            return out
        except sqlite3.Error:
            return []

    def get_color(self, label: str):
        if label not in self.class_colors:
            idx = len(self.class_colors) % len(self.PALETTE)
            self.class_colors[label] = self.PALETTE[idx]
        return self.class_colors[label]

    def on_timer(self):
        img = self.bg_img.copy()
        draw = ImageDraw.Draw(img, 'RGBA')

        # WP 巡回経路 (薄い青線)
        if len(self.waypoints_px) >= 2:
            draw.line(self.waypoints_px, fill=(80, 140, 220, 180), width=2)
        for i, (cx, cy) in enumerate(self.waypoints_px):
            draw.ellipse([cx - 4, cy - 4, cx + 4, cy + 4],
                         fill=(80, 140, 220), outline=(0, 0, 0))

        # 認識物体 (クラス色の丸のみ描画。クラス名テキストは prepare 側で後付け)
        objs = self.fetch_objects()
        obj_meta = []  # 後段に渡すラベル付きメタデータ
        for (x, y, lbl) in objs:
            cx, cy = self.map_xy_to_image(x, y)
            if not (0 <= cx < img.width and 0 <= cy < img.height):
                continue
            col = self.get_color(lbl or 'unknown')
            r = 6
            draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                         fill=col + (255,), outline=(0, 0, 0))
            obj_meta.append({
                'px': int(cx),
                'py': int(cy),
                'label': lbl or '',
                'color': list(col),
            })

        # ロボット現在位置 (黒縁の三角)
        if self.robot_xy is not None:
            rx, ry, ryaw = self.robot_xy
            cx, cy = self.map_xy_to_image(rx, ry)
            size = 10
            # 三角形の3頂点 (進行方向に尖る)
            # 画像座標は Y-down で yaw 正方向は世界の +Y を向くと画像では「上=row 減」
            # 世界 yaw を画像 yaw に変換: 画像での角度 = -yaw (rad)
            ia = -ryaw
            pts = []
            for ang_off in (0.0, 2.5, -2.5):
                ang = ia + ang_off
                pts.append((cx + size * math.cos(ang),
                            cy + size * math.sin(ang)))
            draw.polygon(pts, fill=(20, 20, 220, 255),
                         outline=(255, 255, 255))

        # 凡例ボックスは描かない (回転後に縦書きになって読みづらいため)。
        # 物体個別のクラス名は引き続き各点に併記する。

        # タスク名ラベル + 物体クラス名テキストは GIF 化前段
        # (prepare_overview_frames.py) で回転後に書き込む。
        # メタデータ (物体位置・ラベル) を同名 JSON に併出する。
        out_path = self.out_dir / f'{self.frame_index:04d}.png'
        img.save(out_path)
        meta_path = self.out_dir / f'{self.frame_index:04d}.json'
        with open(meta_path, 'w') as f:
            json.dump({
                'image_size': [img.width, img.height],
                'objects': obj_meta,
            }, f)
        self.frame_index += 1
        if self.frame_index % 5 == 0:
            self.get_logger().info(f'saved {self.frame_index} recog frames '
                                   f'(objs={len(objs)})')
        if self.frame_index >= self.max_frames:
            self.get_logger().info('max_frames reached, shutting down')
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', required=True,
                        choices=['mapping', 'recognize'])
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--period-sec', type=float, default=2.0)
    parser.add_argument('--min-change-cells', type=int, default=200,
                        help='mapping: 前回比でこの量以上既知セルが増えれば直ぐ保存')
    parser.add_argument('--max-frames', type=int, default=120)
    parser.add_argument('--db-path', type=str,
                        default=os.path.expanduser(
                            '~/.ros/object_memory.sqlite3'))
    parser.add_argument('--map-yaml', type=str, default='',
                        help='recognize phase で背景にする静的地図 YAML')
    parser.add_argument('--waypoints-yaml', type=str, default='',
                        help='recognize phase で重ねる WP YAML')
    args = parser.parse_args()

    out_dir = Path(args.out_dir).resolve()
    rclpy.init()
    if args.phase == 'mapping':
        node = MappingCapture(out_dir, args.period_sec, args.min_change_cells,
                              args.max_frames)
    else:
        if not args.map_yaml or not args.waypoints_yaml:
            print('--map-yaml と --waypoints-yaml は recognize で必須',
                  file=sys.stderr)
            sys.exit(2)
        node = RecognitionCapture(out_dir, Path(args.db_path),
                                  args.period_sec, args.max_frames,
                                  Path(args.map_yaml),
                                  Path(args.waypoints_yaml))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
