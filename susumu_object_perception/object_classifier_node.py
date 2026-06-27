#!/usr/bin/env python3
# LiDAR 検出物体の画像分類ノード。perception が LiDAR で見つけた動的/静的オブジェクト
# (TrackedObjects) について、その方向の全天球画像クロップを YOLOv8(COCO) で分類し、
# 「それが何か」（person/car/bicycle ...）を判定する。
#
# LiDAR は「そこに物体がある・大きさ・速度」は分かるが「何か」は分からない（自作 tracker の
# classification は移動=PEDESTRIAN/静止=UNKNOWN の 2D 推定のみ）。そこで物体方向の画像を
# 切り出して画像認識し、クラスを確定する。Autoware の detection→classification の発想を
# LiDAR×カメラの late fusion で実現する。
#
# 入力 : /perception/tracked_objects        (autoware_perception_msgs/TrackedObjects)
#        /omni_camera/image_raw/image_color (sensor_msgs/Image, 全天球 正距円筒)
#        TF: camera_frame <- objects.frame
# 出力 : /perception/tracked_objects_classified (TrackedObjects, classification を画像認識で上書き)
#        /perception/object_classes/markers       (visualization_msgs/MarkerArray, 3D クラス名)
#        /perception/object_classes/image_annotated(sensor_msgs/Image, クロップ+クラス名, デバッグ)
#
# 分類器が初期化できない場合、勝手に無分類で素通しせず [FATAL] で終了する（信号検出ノードと
# 同じ方針＝自動フォールバック禁止）。

import math
import json
import os
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time as RclTime
from rclpy.duration import Duration as RclDuration

from sensor_msgs.msg import Image
from autoware_perception_msgs.msg import (TrackedObjects, ObjectClassification)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformException, TransformListener

import cv2

from susumu_object_perception.omni_projection import (
    equirect_uv, euler_xyz_to_matrix, perspective_directions, quat_to_matrix)


# COCO クラス名 → Autoware ObjectClassification.label。挙げていないものは UNKNOWN だが、
# 元の COCO クラス名は marker / 注釈に残す（「何か」は名前で分かる）。
COCO_TO_AUTOWARE = {
    'person': ObjectClassification.PEDESTRIAN,
    'bicycle': ObjectClassification.BICYCLE,
    'car': ObjectClassification.CAR,
    'motorcycle': ObjectClassification.MOTORCYCLE,
    'bus': ObjectClassification.BUS,
    'truck': ObjectClassification.TRUCK,
    'cat': ObjectClassification.ANIMAL,
    'dog': ObjectClassification.ANIMAL,
    'bird': ObjectClassification.ANIMAL,
    'horse': ObjectClassification.ANIMAL,
    'sheep': ObjectClassification.ANIMAL,
    'cow': ObjectClassification.ANIMAL,
}


CLASS_STABILITY_KEYS = {
    'vase': 'plant',
    'potted plant': 'plant',
    'sofa': 'couch',
    'couch': 'couch',
    'table': 'table',
    'dining table': 'table',
    'fridge': 'refrigerator',
    'refrigerator': 'refrigerator',
}

PLANT_COLOR_CLASSES = {'potted plant', 'vase', 'umbrella'}


def class_stability_key(name):
    name = str(name or '').strip().lower().replace('_', ' ')
    while '  ' in name:
        name = name.replace('  ', ' ')
    return CLASS_STABILITY_KEYS.get(name, name)


class YoloClassifier:
    """YOLOv8(ultralytics) でクロップ画像を物体検出し、代表クラスを返す。

    初期化（ultralytics/torch import・重み読込）に失敗したら RuntimeError を投げる。
    ノード側はそれを [FATAL] にして終了する（無分類での素通しはしない）。
    """

    def __init__(self, weights, conf, center_tolerance_frac,
                 min_box_area_frac, center_window_frac,
                 min_center_window_overlap, require_mask_center,
                 mask_center_window_frac, min_mask_center_overlap,
                 plant_color_min_frac, accept_conf=0.0,
                 debug_diagnostics=False):
        self.conf = conf
        self.accept_conf = accept_conf
        self.center_tolerance_frac = center_tolerance_frac
        self.min_box_area_frac = min_box_area_frac
        self.center_window_frac = center_window_frac
        self.min_center_window_overlap = min_center_window_overlap
        self.require_mask_center = require_mask_center
        self.mask_center_window_frac = mask_center_window_frac
        self.min_mask_center_overlap = min_mask_center_overlap
        self.plant_color_min_frac = plant_color_min_frac
        self.debug_diagnostics = bool(debug_diagnostics)
        self.last_debug = []
        self.last_candidates = []
        try:
            import torch
            _orig = torch.load

            def _patched(*a, **k):
                k.setdefault('weights_only', False)
                return _orig(*a, **k)
            torch.load = _patched
            from ultralytics import YOLO
            self.model = YOLO(weights)
        except Exception as e:
            raise RuntimeError(
                'ultralytics/torch 初期化または重み読込に失敗: %s' % e)

    def classify(self, bgr):
        """クロップ BGR → (coco_class_name, confidence) または (None, 0.0)。

        クロップ内で最も信頼度の高い検出のクラスを採る。何も検出されなければ None。
        """
        return self.classify_many([bgr])[0]

    def classify_many(self, bgrs):
        """複数クロップを batch 推論し、各クロップの分類結果を返す。"""
        if not bgrs:
            return []
        kwargs = {'conf': self.conf, 'batch': len(bgrs), 'verbose': False}
        if getattr(self, 'imgsz', 0) > 0:
            kwargs['imgsz'] = int(self.imgsz)
        res = self.model.predict(bgrs, **kwargs)
        outputs = []
        self.last_debug = []
        self.last_candidates = []
        for r, bgr in zip(res, bgrs):
            name, conf, debug, candidates = self._classify_result(r, bgr)
            outputs.append((name, conf))
            self.last_debug.append(debug)
            self.last_candidates.append(candidates)
        return outputs

    def _classify_result(self, result, bgr):
        """Ultralytics Result 1件を既存ゲートで (name, conf) に変換する。"""
        best_name, best_conf = None, 0.0
        h, w = bgr.shape[:2]
        crop_cx = w * 0.5
        crop_cy = h * 0.5
        names = result.names
        masks_xy = []
        if getattr(result, 'masks', None) is not None and result.masks is not None:
            masks_xy = list(getattr(result.masks, 'xy', []) or [])
        debug = []
        accepted_candidates = []
        for bi, b in enumerate(result.boxes):
            c = float(b.conf)
            xyxy = b.xyxy[0].detach().cpu().tolist()
            x1, y1, x2, y2 = [float(v) for v in xyxy]
            area_frac = max(0.0, (x2 - x1) * (y2 - y1)) / float(w * h)
            name = str(names[int(b.cls)]).lower()
            bx = (x1 + x2) * 0.5
            by = (y1 + y2) * 0.5
            dx = abs(bx - crop_cx) / max(crop_cx, 1.0)
            dy = abs(by - crop_cy) / max(crop_cy, 1.0)
            contains_center = (
                x1 <= crop_cx <= x2 and y1 <= crop_cy <= y2)
            center_overlap = self._center_window_overlap(
                x1, y1, x2, y2, w, h, self.center_window_frac)
            mask_overlap = None
            if self.require_mask_center or self.debug_diagnostics:
                mask_overlap = (
                    self._mask_center_overlap(
                        masks_xy[bi], w, h, self.mask_center_window_frac)
                    if bi < len(masks_xy) else 0.0)
            plant_color = None
            if name in PLANT_COLOR_CLASSES and (
                    self.plant_color_min_frac > 0.0 or self.debug_diagnostics):
                plant_color = self._plant_color_fraction(bgr, x1, y1, x2, y2)
            reject_reason = ''
            if area_frac < self.min_box_area_frac:
                reject_reason = 'box_area'
                if self.debug_diagnostics:
                    debug.append(self._candidate_debug(
                        name, c, area_frac, dx, dy, contains_center,
                        center_overlap, mask_overlap, plant_color,
                        reject_reason, (x1, y1, x2, y2)))
                continue
            if self.center_tolerance_frac >= 0.0:
                near_center = (
                    dx <= self.center_tolerance_frac and
                    dy <= self.center_tolerance_frac)
                if not (contains_center or near_center):
                    reject_reason = 'center_tolerance'
                    if self.debug_diagnostics:
                        debug.append(self._candidate_debug(
                            name, c, area_frac, dx, dy, contains_center,
                            center_overlap, mask_overlap, plant_color,
                            reject_reason, (x1, y1, x2, y2)))
                    continue
            if self.min_center_window_overlap > 0.0 and \
                    center_overlap < self.min_center_window_overlap:
                reject_reason = 'center_window_overlap'
                if self.debug_diagnostics:
                    debug.append(self._candidate_debug(
                        name, c, area_frac, dx, dy, contains_center,
                        center_overlap, mask_overlap, plant_color,
                        reject_reason, (x1, y1, x2, y2)))
                continue
            if self.require_mask_center:
                if mask_overlap is None or mask_overlap < self.min_mask_center_overlap:
                    reject_reason = 'mask_center_overlap'
                    if self.debug_diagnostics:
                        debug.append(self._candidate_debug(
                            name, c, area_frac, dx, dy, contains_center,
                            center_overlap, mask_overlap, plant_color,
                            reject_reason, (x1, y1, x2, y2)))
                    continue
            if self.plant_color_min_frac > 0.0 and \
                    name in PLANT_COLOR_CLASSES and \
                    plant_color is not None and \
                    plant_color < self.plant_color_min_frac:
                reject_reason = 'plant_color'
                if self.debug_diagnostics:
                    debug.append(self._candidate_debug(
                        name, c, area_frac, dx, dy, contains_center,
                        center_overlap, mask_overlap, plant_color,
                        reject_reason, (x1, y1, x2, y2)))
                continue
            if c < self.accept_conf:
                reject_reason = 'min_accept_conf'
                if self.debug_diagnostics:
                    debug.append(self._candidate_debug(
                        name, c, area_frac, dx, dy, contains_center,
                        center_overlap, mask_overlap, plant_color,
                        reject_reason, (x1, y1, x2, y2)))
                continue
            cand = self._candidate_debug(
                name, c, area_frac, dx, dy, contains_center,
                center_overlap, mask_overlap, plant_color, 'accepted',
                (x1, y1, x2, y2))
            accepted_candidates.append(cand)
            if self.debug_diagnostics:
                debug.append(cand)
            if c > best_conf:
                best_conf = c
                best_name = name
        return best_name, best_conf, debug, accepted_candidates

    @staticmethod
    def _candidate_debug(name, conf, area_frac, dx, dy, contains_center,
                         center_overlap, mask_overlap, plant_color, reason,
                         bbox_xyxy=None):
        row = {
            'name': name,
            'conf': float(conf),
            'area_frac': float(area_frac),
            'center_dx': float(dx),
            'center_dy': float(dy),
            'contains_center': bool(contains_center),
            'center_overlap': float(center_overlap),
            'mask_overlap': (
                None if mask_overlap is None else float(mask_overlap)),
            'plant_color': (
                None if plant_color is None else float(plant_color)),
            'reason': reason,
        }
        if bbox_xyxy is not None:
            row['bbox_xyxy'] = [float(v) for v in bbox_xyxy]
        return row

    @staticmethod
    def _center_window_overlap(x1, y1, x2, y2, w, h, window_frac):
        frac = max(1e-3, min(1.0, float(window_frac)))
        ww = w * frac
        wh = h * frac
        cx = w * 0.5
        cy = h * 0.5
        wx1 = cx - ww * 0.5
        wx2 = cx + ww * 0.5
        wy1 = cy - wh * 0.5
        wy2 = cy + wh * 0.5
        ix1 = max(wx1, x1)
        iy1 = max(wy1, y1)
        ix2 = min(wx2, x2)
        iy2 = min(wy2, y2)
        if ix2 <= ix1 or iy2 <= iy1:
            return 0.0
        return ((ix2 - ix1) * (iy2 - iy1)) / max(ww * wh, 1e-6)

    @staticmethod
    def _mask_center_overlap(poly, w, h, window_frac):
        if poly is None or len(poly) < 3:
            return 0.0
        pts = np.asarray(poly, dtype=np.float32)
        frac = max(1e-3, min(1.0, float(window_frac)))
        ww = w * frac
        wh = h * frac
        cx = w * 0.5
        cy = h * 0.5
        xs = np.linspace(cx - ww * 0.5, cx + ww * 0.5, 5)
        ys = np.linspace(cy - wh * 0.5, cy + wh * 0.5, 5)
        inside = 0
        total = 0
        for yy in ys:
            for xx in xs:
                total += 1
                if cv2.pointPolygonTest(pts, (float(xx), float(yy)), False) >= 0:
                    inside += 1
        return inside / float(max(total, 1))

    @staticmethod
    def _plant_color_fraction(bgr, x1, y1, x2, y2):
        h, w = bgr.shape[:2]
        ix1 = max(0, min(w - 1, int(math.floor(x1))))
        iy1 = max(0, min(h - 1, int(math.floor(y1))))
        ix2 = max(ix1 + 1, min(w, int(math.ceil(x2))))
        iy2 = max(iy1 + 1, min(h, int(math.ceil(y2))))
        roi = bgr[iy1:iy2, ix1:ix2]
        if roi.size == 0:
            return 0.0
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        vivid = (sat >= 45) & (val >= 45)
        green = (hue >= 35) & (hue <= 90)
        yellow = (hue >= 18) & (hue <= 35)
        plant = vivid & (green | yellow)
        return float(np.count_nonzero(plant)) / float(max(plant.size, 1))


class ObjectClassifierNode(Node):
    def __init__(self):
        super().__init__('object_classifier')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.input_image = self.declare_parameter(
            'input_image', '/omni_camera/image_raw/image_color').value
        self.input_objects = self.declare_parameter(
            'input_objects', '/perception/tracked_objects').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'omni_camera_link').value
        self.crop_w = int(self.declare_parameter('crop_width', 320).value)
        self.crop_h = int(self.declare_parameter('crop_height', 320).value)
        self.crop_fov = math.radians(
            float(self.declare_parameter('crop_fov_deg', 55.0).value))
        # Empty `crop_fovs_deg` keeps the legacy single-crop behavior. Multi-FOV
        # crops are an experiment hook for recognition tuning and are off by
        # default because confidence-only selection can pick background objects.
        self.crop_fovs = self._parse_crop_fovs(
            self.declare_parameter('crop_fovs_deg', '').value,
            self.crop_fov)
        # Diagnostic-only crop center offsets. Empty values keep the legacy
        # center-on-track crop. Offsets are angular shifts in the perspective
        # crop frame and are useful for checking if the 3D track center is
        # slightly biased toward a foreground/background object.
        self.crop_yaw_offsets = self._parse_angle_offsets(
            self.declare_parameter('crop_yaw_offsets_deg', '').value)
        self.crop_pitch_offsets = self._parse_angle_offsets(
            self.declare_parameter('crop_pitch_offsets_deg', '').value)
        # Diagnostic-only shape-height crop centers. Empty values keep the
        # legacy pose-position crop. For object poses that sit on the floor,
        # e.g. Fridge with z=0 and shape.z=1.7, fractions such as 0.5 and 0.75
        # center the crop on the actual object body instead of the floor point.
        self.crop_shape_height_fracs = self._parse_fraction_offsets(
            self.declare_parameter(
                'crop_shape_center_height_fracs', '').value)
        # Diagnostic-only shape-aware crops. When set, the 3D bbox corners are
        # projected to the omni camera direction sphere and a tight perspective
        # crop FOV is chosen from the angular extent plus this margin.
        self.crop_shape_bbox_margins = self._parse_margin_offsets(
            self.declare_parameter(
                'crop_shape_bbox_margins_deg', '').value)
        self.max_objects = int(self.declare_parameter('max_objects', 8).value)
        self.max_inferences_per_cycle = int(self.declare_parameter(
            'max_inferences_per_cycle', 4).value)
        self.projection_model = self.declare_parameter(
            'projection_model', 'webots_cylindrical').value
        rpy = [float(v) for v in self.declare_parameter(
            'calibration_rpy_deg', [0.0, 0.0, 0.0]).value]
        self.calibration_rot = euler_xyz_to_matrix(
            math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2]))
        weights = self.declare_parameter('yolo.weights', 'yolov8n.pt').value
        conf = float(self.declare_parameter('yolo.conf', 0.25).value)
        self.yolo_imgsz = int(self.declare_parameter('yolo.imgsz', 640).value)
        # LiDAR の対象方向をクロップ中心に置いているので、中心から大きく外れた
        # YOLO bbox は背景物体である可能性が高い。-1 で無効化。
        center_tol = float(self.declare_parameter(
            'center_tolerance_frac', 0.45).value)
        min_box_area = float(self.declare_parameter(
            'min_box_area_frac', 0.0005).value)
        # LiDAR 対象方向はクロップ中心に置く。bbox の中心が近いだけでなく、
        # 中心の小ROIと bbox が一定以上重なる場合だけ採ると、背景物体の誤採用を
        # 抑えやすい。min_center_window_overlap=0 で無効。
        center_window_frac = float(self.declare_parameter(
            'center_window_frac', 0.25).value)
        min_center_overlap = float(self.declare_parameter(
            'min_center_window_overlap', 0.0).value)
        # segmentation weight を使う場合の追加ゲート。mask が中心ROIを覆うかで
        # bbox だけの粗い重なりを補う。通常の detect weight では False のまま使う。
        require_mask_center = bool(self.declare_parameter(
            'require_mask_center', False).value)
        mask_center_window_frac = float(self.declare_parameter(
            'mask_center_window_frac', 0.18).value)
        min_mask_center_overlap = float(self.declare_parameter(
            'min_mask_center_overlap', 0.08).value)
        # 植物系ラベルだけに使う色整合性ゲート。Webots indoor の植物は緑/花色を
        # 持つため、家具・箱・壁片の potted plant 誤認を抑える。0 で無効。
        plant_color_min_frac = float(self.declare_parameter(
            'plant_color_min_frac', 0.0).value)
        # 信頼度がこれ未満の分類は採用せず、元の classification を保つ。
        self.min_accept_conf = float(
            self.declare_parameter('min_accept_conf', 0.3).value)
        # 【クラス別 confidence threshold】COCO の似たクラス間 (refrigerator vs cabinet vs
        # dining table 等) で誤分類が起きやすい。クラスごとに採用閾値を上げ下げできる。
        # フォーマット: "class1=0.10,class2=0.30,..." (COCO 元クラス名で指定)。
        # 既定空。 採用例 (FP 多い dining table を厳しく / FN 多い refrigerator を緩めに):
        #   min_accept_conf_overrides:='refrigerator=0.10,fridge=0.10,dining table=0.30,table=0.30'
        # 参考: ultralytics discussion #5983「class-specific confidence threshold」
        self._min_accept_conf_overrides_raw = str(self.declare_parameter(
            'min_accept_conf_overrides', '').value or '')
        self.min_accept_conf_overrides = self._parse_class_conf_overrides(
            self._min_accept_conf_overrides_raw)
        if self.min_accept_conf_overrides:
            self.get_logger().info(
                f'class-specific min_accept_conf: '
                f'{self.min_accept_conf_overrides}')
        # 複数 crop FOV を使う実験時の代表選択。単一 FOV の既定挙動には影響しない。
        # 同じ class 系統が複数 FOV で出る候補を優先し、中心 ROI / mask の整合も加点する。
        self.multi_fov_agreement_bonus = float(self.declare_parameter(
            'multi_fov_agreement_bonus', 0.18).value)
        self.multi_fov_center_overlap_weight = float(self.declare_parameter(
            'multi_fov_center_overlap_weight', 0.10).value)
        self.multi_fov_mask_overlap_weight = float(self.declare_parameter(
            'multi_fov_mask_overlap_weight', 0.12).value)

        # === 間引き ===
        # (1) 処理レート上限 [Hz]。tracked_objects は ~10Hz で来るが、物体の種類は急に
        #     変わらないので分類は低頻度でよい。0 以下で無効。1回の処理周期で
        #     max_inferences_per_cycle 個まで batch 推論する。
        self.max_rate_hz = float(self.declare_parameter('max_rate_hz', 2.0).value)
        self._min_interval = (1.0 / self.max_rate_hz) if self.max_rate_hz > 0 else 0.0
        self._last_proc_t = None
        # (2) トラック ID キャッシュ。分類結果は保持する。reclassify_interval_sec
        #     を正値にすると active なトラックも定期再確認するが、屋内巡回では
        #     一時的な YOLO miss で正解記憶の hits が伸びにくくなったため既定は無効。
        self.cache_ttl = float(self.declare_parameter('cache_ttl_sec', 10.0).value)
        self.reclassify_interval = float(
            self.declare_parameter('reclassify_interval_sec', 0.0).value)
        self.min_consistent_hits = max(1, int(
            self.declare_parameter('min_consistent_hits', 1).value))
        self.max_class_misses = max(1, int(
            self.declare_parameter('max_class_misses', 1).value))
        self.publish_unknown_fine_class_clears = bool(
            self.declare_parameter(
                'publish_unknown_fine_class_clears', False).value)
        self.publish_debug_diagnostics = bool(
            self.declare_parameter(
                'publish_debug_diagnostics', False).value)
        # Debug-only crop capture. Empty directory disables it. Raw crops are
        # saved with JSONL metadata so the same live crops can be re-run through
        # different YOLO weights offline.
        self.debug_crop_dir = os.path.expanduser(str(
            self.declare_parameter('debug_crop_dir', '').value)).strip()
        self.debug_crop_min_interval = float(self.declare_parameter(
            'debug_crop_min_interval_sec', 1.0).value)
        self.debug_crop_max_per_track = int(self.declare_parameter(
            'debug_crop_max_per_track', 3).value)
        self.debug_crop_write_rejected = bool(self.declare_parameter(
            'debug_crop_write_rejected', True).value)
        self._debug_crop_counts = {}
        self._debug_crop_last_t = {}
        self._debug_crop_seq = 0
        self._debug_crop_meta_path = ''
        if self.debug_crop_dir:
            os.makedirs(self.debug_crop_dir, exist_ok=True)
            self._debug_crop_meta_path = os.path.join(
                self.debug_crop_dir, 'metadata.jsonl')
        # uuid(bytes) -> dict(label, coco_name, conf, last_seen_t, last_infer_t,
        #                     candidate_key, candidate_hits, misses)
        self._class_cache = {}

        # 【時刻同期】移動中は tracked_objects の時刻と画像の時刻がズレると、
        # ロボットが動いた分だけ「画像上で物体方向と思った場所」が物理的にズレて
        # クロップ中心が別物体・壁・空白を指し、YOLO の誤分類が空中ゴーストの
        # tracked_objects と紐付いて「何もない場所で何かを検出」する現象になる。
        # 物体時刻に最も近い画像と TF を選び直し、ロボット移動の影響を相殺する。
        # （colorized_pointcloud_node の image_buffer + image_sync_max_dt と同じ作法）
        # 既定 0.5s: Webots cylindrical camera は実機計測で ~0.25s 程度の publish 遅延
        # を持ち、 0.2s では毎フレーム同期不能で分類スキップが多発する。 0.5s ならば
        # 同期失敗を避けつつ、 「latest 画像 + 最新 TF」の素の lookup よりも常に
        # 画像時刻に近い TF を引ける分、 crop 方向の精度は改善する。
        self.image_buffer_len = int(self.declare_parameter(
            'image_buffer_len', 30).value)
        self.image_sync_max_dt = float(self.declare_parameter(
            'image_sync_max_dt', 0.5).value)
        self.image_buffer = deque(maxlen=max(1, self.image_buffer_len))

        # 分類器初期化。失敗したら無分類で素通しせず FATAL で終了（自動フォールバック禁止）。
        try:
            self.classifier = YoloClassifier(
                weights, conf, center_tol, min_box_area,
                center_window_frac, min_center_overlap,
                require_mask_center, mask_center_window_frac,
                min_mask_center_overlap, plant_color_min_frac,
                self.min_accept_conf,
                self.publish_debug_diagnostics)
            self.classifier.imgsz = self.yolo_imgsz
        except RuntimeError as e:
            self.get_logger().fatal(
                'YOLO 分類器の初期化に失敗した: %s\n'
                '無分類での素通しはしない。ultralytics/torch と yolo.weights を用意すること。'
                % e)
            raise

        self.latest_image = None
        self.pub_objects = self.create_publisher(
            TrackedObjects, '/perception/tracked_objects_classified', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/object_classes/markers', 10)
        self.pub_annot = self.create_publisher(
            Image, '/perception/object_classes/image_annotated',
            qos_profile_sensor_data)
        # COCO 細クラス名の副チャネル。Autoware ObjectClassification.label は enum で
        # chair/couch/diningtable 等の什器を表現できない（UNKNOWN に丸まる）。物体メモリが
        # 什器を区別して記憶できるよう、object_id(UUID hex)→COCO名 を DiagnosticArray で
        # 出す（独自 .msg は作らない方針。name=UUID hex, message=COCO名）。
        self.pub_fine = self.create_publisher(
            DiagnosticArray, '/perception/object_fine_classes', 10)
        self.pub_debug = None
        if self.publish_debug_diagnostics:
            self.pub_debug = self.create_publisher(
                DiagnosticArray, '/perception/object_classifier/debug', 10)

        self.create_subscription(
            Image, self.input_image, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            TrackedObjects, self.input_objects, self.on_objects, 10)

        self.get_logger().info(
            'object_classifier started. objects=%s image=%s '
            'crop_fovs_deg=%s crop_yaw_offsets_deg=%s '
            'crop_pitch_offsets_deg=%s crop_shape_height_fracs=%s '
            'crop_shape_bbox_margins_deg=%s '
            'yolo_imgsz=%d '
            '-> /perception/tracked_objects_classified, '
            '/perception/object_classes/markers' % (
                self.input_objects, self.input_image,
                ','.join('%.1f' % math.degrees(v) for v in self.crop_fovs),
                ','.join(
                    '%.1f' % math.degrees(v)
                    for v in self.crop_yaw_offsets),
                ','.join(
                    '%.1f' % math.degrees(v)
                    for v in self.crop_pitch_offsets),
                ','.join(
                    '%.2f' % float(v)
                    for v in self.crop_shape_height_fracs),
                ','.join(
                    '%.1f' % math.degrees(v)
                    for v in self.crop_shape_bbox_margins),
                self.yolo_imgsz))

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning('omni image decode 失敗: %s' % exc)
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.image_buffer.append((t, img, msg.header.stamp))
        # 互換: 古いコードパスから参照される latest_image も維持する。
        self.latest_image = (img, msg.header.stamp)

    def _image_at(self, target_stamp):
        """target_stamp に最も近い画像を返す。image_sync_max_dt を超えるなら (None, dt)。

        target_stamp は builtin_interfaces/Time。バッファに何も無いときは
        (None, None) を返す。ロボット移動中は画像 latency 分だけ object 時刻と
        画像時刻が乖離する。物体方向の crop 中心がズレないよう、物体時刻に近い
        画像を選び直す。
        """
        if not self.image_buffer:
            return None, None, None
        ct = target_stamp.sec + target_stamp.nanosec * 1e-9
        best = None
        for entry in self.image_buffer:
            dt = abs(entry[0] - ct)
            if best is None or dt < best[0]:
                best = (dt, entry)
        if best[0] <= self.image_sync_max_dt:
            _, (_, img, stamp) = best
            return img, best[0], stamp
        return None, best[0], None

    def _transform_point(self, point, source_frame, lookup_stamp=None):
        return self._transform_xyz(
            (float(point.x), float(point.y), float(point.z)),
            source_frame, lookup_stamp)

    def _transform_xyz(self, xyz, source_frame, lookup_stamp=None):
        """lookup_stamp の時点での camera_frame <- source_frame 変換を使う。

        lookup_stamp=None は legacy の latest（rclpy.time.Time()）。これは画像と
        物体の時刻が一致している前提を満たさず、移動中に crop 中心がズレる。
        移動物体・移動ロボット環境では tracked_objects の stamp を渡すこと。
        """
        try:
            if lookup_stamp is None:
                tf = self.tf_buffer.lookup_transform(
                    self.camera_frame, source_frame, rclpy.time.Time())
            else:
                # stamp 指定で取れないときは latest にフォールバック（TF が
                # 過去の bake をすぐ捨てるパス用の救済。logger は throttle で抑制）。
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.camera_frame, source_frame,
                        RclTime.from_msg(lookup_stamp),
                        RclDuration(seconds=0.05))
                except TransformException:
                    tf = self.tf_buffer.lookup_transform(
                        self.camera_frame, source_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warning(
                'no transform %s <- %s: %s'
                % (self.camera_frame, source_frame, exc))
            return None
        rot = quat_to_matrix(tf.transform.rotation)
        trans = np.array([tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z], dtype=np.float32)
        p = np.array(xyz, dtype=np.float32)
        return self.calibration_rot @ (rot @ p + trans)

    @staticmethod
    def _parse_class_conf_overrides(value):
        """Parse "class1=0.10,class2=0.30" into {class: float}.

        Class names are COCO original names (e.g. 'refrigerator', 'dining table').
        Whitespace around names is stripped. Invalid pairs are skipped silently.
        """
        result = {}
        if value is None or value == '':
            return result
        if isinstance(value, dict):
            return {str(k).strip(): float(v) for k, v in value.items()}
        text = str(value)
        for token in text.split(','):
            token = token.strip()
            if not token or '=' not in token:
                continue
            name, _, num = token.partition('=')
            name = name.strip()
            try:
                f = float(num.strip())
            except (TypeError, ValueError):
                continue
            if name:
                result[name] = f
        return result

    def _accept_conf_for(self, coco_name):
        """Return per-class min_accept_conf (falls back to the global default).

        Looks up the COCO original class name. If not found, returns the global
        ``self.min_accept_conf``.
        """
        if not self.min_accept_conf_overrides or not coco_name:
            return self.min_accept_conf
        return self.min_accept_conf_overrides.get(
            coco_name, self.min_accept_conf)

    @staticmethod
    def _parse_crop_fovs(value, fallback_rad):
        """Parse optional comma-separated crop FOV degrees.

        Empty means the legacy single `crop_fov_deg` value. Values are clamped
        to a practical range to avoid accidental 0/180deg projections.
        """
        if value is None or value == '':
            return [fallback_rad]
        if isinstance(value, (list, tuple)):
            parts = value
        else:
            parts = str(value).replace(';', ',').split(',')
        fovs = []
        for part in parts:
            try:
                deg = float(part)
            except (TypeError, ValueError):
                continue
            deg = min(140.0, max(10.0, deg))
            rad = math.radians(deg)
            if all(abs(rad - existing) > math.radians(0.5)
                   for existing in fovs):
                fovs.append(rad)
        return fovs or [fallback_rad]

    @staticmethod
    def _parse_angle_offsets(value):
        """Parse optional comma-separated angular offsets in degrees."""
        if value is None or value == '':
            return [0.0]
        if isinstance(value, (list, tuple)):
            parts = value
        else:
            parts = str(value).replace(';', ',').split(',')
        offsets = []
        for part in parts:
            try:
                deg = float(part)
            except (TypeError, ValueError):
                continue
            deg = min(45.0, max(-45.0, deg))
            rad = math.radians(deg)
            if all(abs(rad - existing) > math.radians(0.1)
                   for existing in offsets):
                offsets.append(rad)
        return offsets or [0.0]

    @staticmethod
    def _parse_fraction_offsets(value):
        """Parse optional comma-separated normalized height fractions."""
        if value is None or value == '':
            return [0.0]
        if isinstance(value, (list, tuple)):
            parts = value
        else:
            parts = str(value).replace(';', ',').split(',')
        offsets = []
        for part in parts:
            try:
                frac = float(part)
            except (TypeError, ValueError):
                continue
            frac = min(1.5, max(-0.5, frac))
            if all(abs(frac - existing) > 0.01 for existing in offsets):
                offsets.append(frac)
        return offsets or [0.0]

    @staticmethod
    def _parse_margin_offsets(value):
        """Parse optional comma-separated positive angular margins in degrees."""
        if value is None or value == '':
            return []
        if isinstance(value, (list, tuple)):
            parts = value
        else:
            parts = str(value).replace(';', ',').split(',')
        margins = []
        for part in parts:
            try:
                deg = float(part)
            except (TypeError, ValueError):
                continue
            deg = min(45.0, max(0.0, deg))
            rad = math.radians(deg)
            if all(abs(rad - existing) > math.radians(0.1)
                   for existing in margins):
                margins.append(rad)
        return margins

    @staticmethod
    def _direction_basis(direction):
        direction = np.asarray(direction, dtype=np.float32)
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return None
        forward = direction / norm
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        if abs(float(np.dot(forward, world_up))) > 0.95:
            world_up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        right = np.cross(forward, world_up)
        right /= max(np.linalg.norm(right), 1e-6)
        up = np.cross(right, forward)
        up /= max(np.linalg.norm(up), 1e-6)
        return forward, right, up

    @staticmethod
    def _offset_direction(direction, yaw_offset=0.0, pitch_offset=0.0):
        basis = ObjectClassifierNode._direction_basis(direction)
        if basis is None:
            return direction
        forward, right, up = basis
        shifted = (forward +
                   right * math.tan(float(yaw_offset)) +
                   up * math.tan(float(pitch_offset)))
        shifted /= max(np.linalg.norm(shifted), 1e-6)
        return shifted.astype(np.float32)

    def _perspective_crop(self, pano, direction, crop_fov=None,
                          yaw_offset=0.0, pitch_offset=0.0):
        direction = self._offset_direction(
            direction, yaw_offset, pitch_offset)
        dirs = perspective_directions(
            direction, self.crop_w, self.crop_h, crop_fov or self.crop_fov)
        if dirs is None:
            return None

        h, w = pano.shape[:2]
        map_x, map_y, _ = equirect_uv(
            dirs, w, h, self.projection_model)
        return cv2.remap(pano, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_WRAP)

    def _crop_center_specs(self, obj, source_frame, lookup_stamp=None):
        pose = obj.kinematics.pose_with_covariance.pose
        p = pose.position
        try:
            shape_z = float(obj.shape.dimensions.z)
        except (AttributeError, TypeError, ValueError):
            shape_z = 0.0
        if shape_z < 0.05:
            shape_z = 0.0
        specs = []
        for height_frac in self.crop_shape_height_fracs:
            cx = float(p.x)
            cy = float(p.y)
            cz = float(p.z) + shape_z * float(height_frac)
            p_cam = self._transform_xyz((cx, cy, cz), source_frame, lookup_stamp)
            if p_cam is not None:
                specs.append((p_cam, float(height_frac), (cx, cy, cz)))
        return specs

    @staticmethod
    def _shape_dimensions(obj):
        try:
            d = obj.shape.dimensions
            return float(d.x), float(d.y), float(d.z)
        except (AttributeError, TypeError, ValueError):
            return 0.0, 0.0, 0.0

    def _bbox_corners_source(self, obj):
        """Return floor-origin bbox corners and center in the object's frame.

        The in-repo tracker publishes z=0 for tracked objects and carries the
        vertical extent in shape.dimensions.z, so the bbox uses z=[0, dz].
        """
        dx, dy, dz = self._shape_dimensions(obj)
        if dx < 0.05 or dy < 0.05 or dz < 0.05:
            return [], None
        pose = obj.kinematics.pose_with_covariance.pose
        base = np.array([
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ], dtype=np.float32)
        rot = quat_to_matrix(pose.orientation)
        corners = []
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (0.0, 1.0):
                    local = np.array([
                        sx * dx,
                        sy * dy,
                        sz * dz,
                    ], dtype=np.float32)
                    corners.append(base + rot @ local)
        center = base + rot @ np.array([0.0, 0.0, dz * 0.5],
                                       dtype=np.float32)
        return corners, center

    def _shape_bbox_crop_specs(self, obj, source_frame, lookup_stamp=None):
        if not self.crop_shape_bbox_margins:
            return []
        corners_src, center_src = self._bbox_corners_source(obj)
        if not corners_src or center_src is None:
            return []
        center_cam = self._transform_xyz(center_src, source_frame, lookup_stamp)
        if center_cam is None or np.linalg.norm(center_cam) < 1e-6:
            return []
        basis = self._direction_basis(center_cam)
        if basis is None:
            return []
        forward, right, up = basis
        corners_cam = []
        for corner in corners_src:
            p_cam = self._transform_xyz(corner, source_frame, lookup_stamp)
            if p_cam is not None and np.linalg.norm(p_cam) > 1e-6:
                corners_cam.append(p_cam)
        if len(corners_cam) < 2:
            return []

        yaws = []
        pitches = []
        for p_cam in corners_cam:
            vec = p_cam / max(np.linalg.norm(p_cam), 1e-6)
            depth = float(np.dot(vec, forward))
            if depth <= 0.05:
                continue
            yaws.append(math.atan2(float(np.dot(vec, right)), depth))
            pitches.append(math.atan2(float(np.dot(vec, up)), depth))
        if len(yaws) < 2 or len(pitches) < 2:
            return []

        yaw_min, yaw_max = min(yaws), max(yaws)
        pitch_min, pitch_max = min(pitches), max(pitches)
        yaw_c = (yaw_min + yaw_max) * 0.5
        pitch_c = (pitch_min + pitch_max) * 0.5
        shifted = (forward +
                   right * math.tan(yaw_c) +
                   up * math.tan(pitch_c))
        shifted /= max(np.linalg.norm(shifted), 1e-6)
        half_extent = max(
            abs(yaw_min - yaw_c),
            abs(yaw_max - yaw_c),
            abs(pitch_min - pitch_c),
            abs(pitch_max - pitch_c))
        if not math.isfinite(half_extent) or half_extent <= 0.0:
            return []

        specs = []
        center_dist = float(np.linalg.norm(center_cam))
        for margin in self.crop_shape_bbox_margins:
            fov = 2.0 * (half_extent + float(margin))
            fov = min(math.radians(140.0), max(math.radians(10.0), fov))
            specs.append((
                shifted * center_dist,
                0.5,
                tuple(float(v) for v in center_src),
                'bbox',
                math.degrees(float(margin)),
                math.degrees(float(fov)),
                fov))
        return specs

    def on_objects(self, msg: TrackedObjects):
        if not msg.objects:
            self.pub_objects.publish(msg)
            return
        # 物体時刻に最も近い画像を選ぶ。ロボット移動中は最新画像 + 最新 TF を
        # 物体時刻に当てると crop 中心が画像内でロボット移動分ズレ、別物体・壁・
        # 空白を YOLO に食わせる空中ゴースト原因になる。同時刻ペアが取れないなら
        # 分類はスキップして物体情報だけ素通しする。
        sync_image, sync_dt, sync_stamp = self._image_at(msg.header.stamp)
        if sync_image is None and self.latest_image is None:
            # 画像が無い間は素通し（分類できないが物体情報は失わない）。
            self.pub_objects.publish(msg)
            return
        if sync_image is None:
            # 同期不能だが画像はある。互換のため latest を使うが分類はせず素通し
            # （古い画像で誤分類するより、未分類のまま下流に渡す方が安全）。
            self.get_logger().warning(
                'object_classifier: 物体時刻に近い画像が無い '
                f'(closest dt={sync_dt}, threshold={self.image_sync_max_dt}s)',
                throttle_duration_sec=2.0)
            self.pub_objects.publish(msg)
            return
        pano = sync_image
        # TF lookup は「画像時刻」に揃える。crop は画像上で行うので、画像時刻時点の
        # ロボット位置から見た物体方向を計算したい。 物体スタンプ (= 最新時刻) で
        # TF を引くと、 ロボットが画像撮影後に移動した分だけ crop 中心が物理的に
        # ズレ、 別物体・壁・空白を YOLO に渡してしまう。 物体の world 座標は時間と
        # ともにあまり動かない (歩行者でも 0.25s で 30cm 程度) ため、 物体位置は
        # 物体スタンプのもの、 TF だけ画像時刻に揃えるのが crop 精度として正しい。
        tf_lookup_stamp = sync_stamp if sync_stamp is not None else msg.header.stamp

        # 間引き(1): 処理レート上限。前回の YOLO 推論から _min_interval 秒経つまで
        # 新規推論せず、キャッシュ済みの分類だけ当てて素通しする。
        now = time.monotonic()

        # 古いキャッシュエントリ(cache_ttl 秒見ていないトラック)を掃除。
        for k in [k for k, v in self._class_cache.items()
                  if now - v.get('last_seen_t', 0.0) > self.cache_ttl]:
            del self._class_cache[k]

        inferred_crops = {}
        if self._can_run_inference(now):
            pending = []
            limit = self.max_inferences_per_cycle
            if limit <= 0:
                limit = self.max_objects
            for i, obj in enumerate(msg.objects[:self.max_objects]):
                tid = bytes(obj.object_id.uuid)
                cached = self._class_cache.get(tid)
                if not self._needs_inference(cached, now):
                    continue
                pose = obj.kinematics.pose_with_covariance.pose
                crops = []
                for p_cam, crop_height_frac, crop_center_xyz in \
                        self._crop_center_specs(
                            obj, msg.header.frame_id, tf_lookup_stamp):
                    for crop_fov in self.crop_fovs:
                        for crop_yaw in self.crop_yaw_offsets:
                            for crop_pitch in self.crop_pitch_offsets:
                                crop = self._perspective_crop(
                                    pano, p_cam, crop_fov,
                                    crop_yaw, crop_pitch)
                                if crop is not None and crop.size > 0:
                                    crops.append((
                                        crop, crop_fov,
                                        crop_yaw, crop_pitch,
                                        crop_height_frac, crop_center_xyz,
                                        'center', None, None))
                for p_cam, crop_height_frac, crop_center_xyz, crop_mode, \
                        bbox_margin_deg, bbox_projected_fov_deg, crop_fov in \
                        self._shape_bbox_crop_specs(
                            obj, msg.header.frame_id, tf_lookup_stamp):
                    for crop_yaw in self.crop_yaw_offsets:
                        for crop_pitch in self.crop_pitch_offsets:
                            crop = self._perspective_crop(
                                pano, p_cam, crop_fov,
                                crop_yaw, crop_pitch)
                            if crop is not None and crop.size > 0:
                                crops.append((
                                    crop, crop_fov,
                                    crop_yaw, crop_pitch,
                                    crop_height_frac, crop_center_xyz,
                                    crop_mode, bbox_margin_deg,
                                    bbox_projected_fov_deg))
                if not crops:
                    self._update_class_cache(tid, None, 0.0, now)
                    continue
                for crop, crop_fov, crop_yaw, crop_pitch, \
                        crop_height_frac, crop_center_xyz, crop_mode, \
                        bbox_margin_deg, bbox_projected_fov_deg in crops:
                    p = pose.position
                    pending.append((
                        tid, crop, crop_fov, crop_yaw, crop_pitch,
                        crop_height_frac,
                        i, msg.header.frame_id,
                        float(p.x), float(p.y), float(p.z),
                        float(crop_center_xyz[0]),
                        float(crop_center_xyz[1]),
                        float(crop_center_xyz[2]),
                        crop_mode,
                        bbox_margin_deg,
                        bbox_projected_fov_deg))
                if len({item[0] for item in pending}) >= limit:
                    break

            if pending:
                results = self.classifier.classify_many(
                    [item[1] for item in pending])
                candidates_by_crop = getattr(
                    self.classifier, 'last_candidates', [])
                entries_by_tid = {}
                for idx, (pending_item, result) in enumerate(
                        zip(pending, results)):
                    tid, crop, crop_fov = pending_item[:3]
                    crop_yaw, crop_pitch = pending_item[3:5]
                    crop_height_frac = pending_item[5]
                    crop_mode = pending_item[14] if len(pending_item) > 14 else 'center'
                    bbox_margin_deg = pending_item[15] if len(pending_item) > 15 else None
                    bbox_projected_fov_deg = (
                        pending_item[16] if len(pending_item) > 16 else None)
                    detected_name, detected_conf = result
                    crop_candidates = (
                        candidates_by_crop[idx]
                        if idx < len(candidates_by_crop) else [])
                    entries_by_tid.setdefault(tid, []).append({
                        'name': detected_name,
                        'conf': float(detected_conf or 0.0),
                        'crop': crop,
                        'crop_fov': crop_fov,
                        'crop_yaw_offset': crop_yaw,
                        'crop_pitch_offset': crop_pitch,
                        'crop_shape_height_frac': crop_height_frac,
                        'crop_mode': crop_mode,
                        'crop_shape_bbox_margin_deg': bbox_margin_deg,
                        'crop_shape_bbox_projected_fov_deg':
                            bbox_projected_fov_deg,
                        'candidates': crop_candidates,
                    })
                best_by_tid = {
                    tid: self._select_multifov_detection(entries)
                    for tid, entries in entries_by_tid.items()
                }
                for tid, selected in best_by_tid.items():
                    if selected is None:
                        continue
                    detected_name, detected_conf, crop, _crop_fov = selected
                    self._update_class_cache(tid, detected_name,
                                             detected_conf, now)
                    inferred_crops[tid] = crop
                if self.pub_debug is not None:
                    self._publish_debug_diagnostics(
                        msg.header, pending, results,
                        getattr(self.classifier, 'last_debug', []))
                self._save_debug_crops(
                    msg.header, pending, results,
                    getattr(self.classifier, 'last_debug', []),
                    candidates_by_crop, now)
                self._last_proc_t = now

        out = TrackedObjects()
        out.header = msg.header
        markers = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        annot_tiles = []
        mid = 0
        # object_id(UUID hex) -> COCO 細クラス名（什器含む）。副チャネルで配信する。
        # 値が None のものは object_memory 側の古い対応を消すための明示的 clear。
        fine_classes = {}

        for i, obj in enumerate(msg.objects):
            new_obj = obj  # 既存物体をそのまま使い classification だけ差し替える
            if i < self.max_objects:
                pose = obj.kinematics.pose_with_covariance.pose
                tid = bytes(obj.object_id.uuid)
                cached = self._class_cache.get(tid)

                coco_name, conf = (None, 0.0)
                if cached is not None:
                    # キャッシュ命中: 再分類せず last_seen だけ更新。
                    cached['last_seen_t'] = now
                    if cached.get('coco_name') is not None:
                        coco_name, conf = cached.get('coco_name'), cached.get('conf', 0.0)

                if coco_name is not None and conf >= self._accept_conf_for(coco_name):
                    label = COCO_TO_AUTOWARE.get(
                        coco_name, ObjectClassification.UNKNOWN)
                    cls = ObjectClassification()
                    cls.label = label
                    cls.probability = float(conf)
                    new_obj.classification = [cls]
                    text = '%s %.2f' % (coco_name, conf)
                    color = (0.1, 0.9, 0.2)
                    # label が UNKNOWN に丸まる什器(chair 等)でも COCO 名は副チャネルへ。
                    fine_classes[bytes(obj.object_id.uuid).hex()] = (coco_name, conf)
                else:
                    text = 'unknown'
                    color = (0.6, 0.6, 0.6)
                    if self.publish_unknown_fine_class_clears:
                        fine_classes[bytes(obj.object_id.uuid).hex()] = (None, 0.0)

                # 3D クラス名マーカー。
                m = Marker()
                m.header = msg.header
                m.ns = 'object_class'
                m.id = mid
                mid += 1
                m.type = Marker.TEXT_VIEW_FACING
                m.action = Marker.ADD
                m.pose = pose
                m.pose.position.z = pose.position.z + 1.0
                m.scale.z = 0.4
                m.color.r, m.color.g, m.color.b = color
                m.color.a = 1.0
                m.text = text
                markers.markers.append(m)

                # 注釈タイル（クロップ + クラス名）。
                crop = inferred_crops.get(tid)
                if crop is not None and crop.size > 0:
                    tile = crop.copy()
                    cv2.putText(tile, text, (6, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 2, cv2.LINE_AA)
                    annot_tiles.append(tile)

            out.objects.append(new_obj)

        self.pub_objects.publish(out)
        self.pub_markers.publish(markers)
        self._publish_fine_classes(msg.header, fine_classes)
        if annot_tiles:
            mosaic = np.hstack(annot_tiles)
            am = self.bridge.cv2_to_imgmsg(mosaic, encoding='bgr8')
            am.header = msg.header
            self.pub_annot.publish(am)

    def _can_run_inference(self, now):
        if self._min_interval <= 0.0 or self._last_proc_t is None:
            return True
        return (now - self._last_proc_t) >= self._min_interval

    def _needs_inference(self, cached, now):
        if cached is None:
            return True
        if cached.get('coco_name') is None:
            return True
        if self.reclassify_interval <= 0.0:
            return False
        return (now - cached.get('last_infer_t', 0.0)) >= self.reclassify_interval

    def _select_multifov_detection(self, entries):
        """Pick one classification from one or more FOV crops for the same track.

        Single-FOV runs keep the legacy confidence-only choice. Multi-FOV runs
        use accepted YOLO candidates from all crops and prefer classes that are
        repeated across FOVs and well aligned with the crop-center ROI/mask.
        """
        if not entries:
            return None
        if len(entries) <= 1:
            e = max(entries, key=lambda item: float(item.get('conf') or 0.0))
            return (e.get('name'), float(e.get('conf') or 0.0),
                    e.get('crop'), e.get('crop_fov'))

        grouped = {}
        for e in entries:
            crop_fov = e.get('crop_fov')
            crop_yaw = float(e.get('crop_yaw_offset') or 0.0)
            crop_pitch = float(e.get('crop_pitch_offset') or 0.0)
            crop_height_frac = float(e.get('crop_shape_height_frac') or 0.0)
            crop_mode = str(e.get('crop_mode') or 'center')
            bbox_margin = e.get('crop_shape_bbox_margin_deg')
            bbox_projected_fov = e.get('crop_shape_bbox_projected_fov_deg')
            fov_key = '%.1f/%+.1f/%+.1f/h%.2f/%s' % (
                math.degrees(float(crop_fov or 0.0)),
                math.degrees(crop_yaw),
                math.degrees(crop_pitch),
                crop_height_frac,
                crop_mode)
            if bbox_margin is not None:
                fov_key += '/bboxm%.1f' % float(bbox_margin)
            for cand in e.get('candidates') or []:
                if cand.get('reason') != 'accepted':
                    continue
                name = str(cand.get('name') or '').strip().lower()
                conf = float(cand.get('conf') or 0.0)
                if not name or conf < self._accept_conf_for(name):
                    continue
                center_overlap = float(cand.get('center_overlap') or 0.0)
                mask_overlap = cand.get('mask_overlap')
                mask_overlap = 0.0 if mask_overlap is None else float(mask_overlap)
                scored = dict(cand)
                scored['_crop'] = e.get('crop')
                scored['_crop_fov'] = crop_fov
                scored['_crop_yaw_offset'] = crop_yaw
                scored['_crop_pitch_offset'] = crop_pitch
                scored['_crop_shape_height_frac'] = crop_height_frac
                scored['_crop_mode'] = crop_mode
                scored['_crop_shape_bbox_margin_deg'] = bbox_margin
                scored['_crop_shape_bbox_projected_fov_deg'] = \
                    bbox_projected_fov
                scored['_fov_key'] = fov_key
                scored['_score'] = (
                    conf +
                    self.multi_fov_center_overlap_weight * center_overlap +
                    self.multi_fov_mask_overlap_weight * mask_overlap)
                grouped.setdefault(class_stability_key(name), []).append(scored)

        if grouped:
            best_tuple = None
            for _key, cands in grouped.items():
                support = len({c['_fov_key'] for c in cands})
                best_cand = max(
                    cands,
                    key=lambda c: (float(c.get('_score') or 0.0),
                                   float(c.get('conf') or 0.0)))
                total_score = (
                    float(best_cand.get('_score') or 0.0) +
                    self.multi_fov_agreement_bonus * max(0, support - 1))
                rank = (
                    total_score,
                    support,
                    float(best_cand.get('conf') or 0.0),
                )
                if best_tuple is None or rank > best_tuple[0]:
                    best_tuple = (rank, best_cand)
            cand = best_tuple[1]
            return (cand.get('name'), float(cand.get('conf') or 0.0),
                    cand.get('_crop'), cand.get('_crop_fov'))

        # Fallback for unusual model/results objects where accepted candidate
        # metadata is unavailable.
        e = max(entries, key=lambda item: float(item.get('conf') or 0.0))
        return (e.get('name'), float(e.get('conf') or 0.0),
                e.get('crop'), e.get('crop_fov'))

    def _update_class_cache(self, tid, detected_name, detected_conf, now):
        cached = self._class_cache.get(tid, {})
        if detected_name is None or detected_conf < self._accept_conf_for(detected_name):
            misses = int(cached.get('misses', 0)) + 1
            if cached.get('coco_name') is not None and \
                    misses < self.max_class_misses:
                cached.update({
                    'last_seen_t': now,
                    'last_infer_t': now,
                    'misses': misses,
                })
                self._class_cache[tid] = cached
                return cached.get('coco_name'), float(cached.get('conf', 0.0))
            self._class_cache[tid] = {
                'label': ObjectClassification.UNKNOWN,
                'coco_name': None,
                'conf': 0.0,
                'last_seen_t': now,
                'last_infer_t': now,
                'candidate_key': None,
                'candidate_hits': 0,
                'misses': misses,
            }
            return None, 0.0

        key = class_stability_key(detected_name)
        if cached.get('candidate_key') == key:
            hits = int(cached.get('candidate_hits', 0)) + 1
        else:
            hits = 1
        stable = hits >= self.min_consistent_hits
        label = COCO_TO_AUTOWARE.get(
            detected_name, ObjectClassification.UNKNOWN)
        self._class_cache[tid] = {
            'label': label if stable else ObjectClassification.UNKNOWN,
            'coco_name': detected_name if stable else None,
            'conf': float(detected_conf) if stable else 0.0,
            'last_seen_t': now,
            'last_infer_t': now,
            'candidate_key': key,
            'candidate_hits': hits,
            'misses': 0,
        }
        if stable:
            return detected_name, float(detected_conf)
        return None, 0.0

    def _publish_fine_classes(self, header, fine_classes):
        """object_id(UUID hex) -> COCO 細クラス名 を DiagnosticArray で配信する。

        1 status が 1 物体に対応する: name=UUID hex, message=COCO名,
        values=[{key:'conf', value:'0.87'}]。物体メモリがこれを引いて什器を区別する。
        """
        da = DiagnosticArray()
        da.header = header
        for uuid_hex, (coco_name, conf) in fine_classes.items():
            st = DiagnosticStatus()
            st.name = uuid_hex
            if coco_name is None:
                st.level = DiagnosticStatus.WARN
                st.message = ''
            else:
                st.level = DiagnosticStatus.OK
                st.message = coco_name
            st.values = [KeyValue(key='conf', value='%.3f' % conf)]
            da.status.append(st)
        self.pub_fine.publish(da)

    def _save_debug_crops(self, header, pending, results, debug_infos,
                          accepted_infos, now):
        if not self.debug_crop_dir or not pending:
            return
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        for idx, (pending_item, result) in enumerate(zip(pending, results)):
            tid, crop, crop_fov = pending_item[:3]
            crop_yaw = pending_item[3] if len(pending_item) > 3 else 0.0
            crop_pitch = pending_item[4] if len(pending_item) > 4 else 0.0
            crop_height_frac = pending_item[5] if len(pending_item) > 5 else 0.0
            obj_index = pending_item[6] if len(pending_item) > 6 else None
            source_frame = pending_item[7] if len(pending_item) > 7 else ''
            obj_xyz = pending_item[8:11] if len(pending_item) > 10 else (None, None, None)
            crop_center_xyz = (
                pending_item[11:14]
                if len(pending_item) > 13 else (None, None, None))
            crop_mode = pending_item[14] if len(pending_item) > 14 else 'center'
            bbox_margin_deg = (
                pending_item[15] if len(pending_item) > 15 else None)
            bbox_projected_fov_deg = (
                pending_item[16] if len(pending_item) > 16 else None)
            tid_hex = bytes(tid).hex()
            fov_deg = math.degrees(float(crop_fov))
            yaw_deg = math.degrees(float(crop_yaw))
            pitch_deg = math.degrees(float(crop_pitch))
            interval_key = '%s/%s/fov%.1f/yaw%+.1f/pitch%+.1f/h%.2f' % (
                tid_hex, str(crop_mode or 'center'),
                fov_deg, yaw_deg, pitch_deg,
                float(crop_height_frac))
            if bbox_margin_deg is not None:
                interval_key += '/bboxm%.1f' % float(bbox_margin_deg)
            count = int(self._debug_crop_counts.get(tid_hex, 0))
            if self.debug_crop_max_per_track >= 0 and \
                    count >= self.debug_crop_max_per_track:
                continue
            last = self._debug_crop_last_t.get(interval_key)
            if last is not None and self.debug_crop_min_interval > 0.0 and \
                    (now - last) < self.debug_crop_min_interval:
                continue
            candidates = debug_infos[idx] if idx < len(debug_infos) else []
            accepted = accepted_infos[idx] if idx < len(accepted_infos) else []
            if not self.debug_crop_write_rejected and not accepted:
                continue
            selected_name, selected_conf = result
            self._debug_crop_seq += 1
            mode_suffix = ''
            if str(crop_mode or 'center') != 'center':
                mode_suffix = f'_{str(crop_mode)}'
                if bbox_margin_deg is not None:
                    mode_suffix += f'm{float(bbox_margin_deg):.1f}'
            basename = (
                f'{self._debug_crop_seq:06d}_'
                f'{tid_hex[:8]}_fov{fov_deg:.1f}'
                f'_yaw{yaw_deg:+.1f}_pitch{pitch_deg:+.1f}'
                f'_h{float(crop_height_frac):.2f}'
                f'{mode_suffix}.png')
            path = os.path.join(self.debug_crop_dir, basename)
            if not cv2.imwrite(path, crop):
                self.get_logger().warning(
                    f'failed to save debug crop: {path}',
                    throttle_duration_sec=2.0)
                continue
            row = {
                'stamp': stamp,
                'object_id': tid_hex,
                'object_index': obj_index,
                'source_frame': source_frame,
                'object_xyz': [
                    None if v is None else float(v) for v in obj_xyz],
                'crop_fov_deg': fov_deg,
                'crop_yaw_offset_deg': yaw_deg,
                'crop_pitch_offset_deg': pitch_deg,
                'crop_shape_height_frac': float(crop_height_frac),
                'crop_mode': str(crop_mode or 'center'),
                'crop_shape_bbox_margin_deg': (
                    None if bbox_margin_deg is None
                    else float(bbox_margin_deg)),
                'crop_shape_bbox_projected_fov_deg': (
                    None if bbox_projected_fov_deg is None
                    else float(bbox_projected_fov_deg)),
                'crop_center_xyz': [
                    None if v is None else float(v)
                    for v in crop_center_xyz],
                'selected': selected_name,
                'selected_conf': float(selected_conf or 0.0),
                'image': basename,
                'path': path,
                'candidates': candidates,
                'accepted_candidates': accepted,
            }
            with open(self._debug_crop_meta_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(row, ensure_ascii=False) + '\n')
            self._debug_crop_counts[tid_hex] = count + 1
            self._debug_crop_last_t[interval_key] = now

    def _publish_debug_diagnostics(self, header, pending, results, debug_infos):
        """分類候補の採否理由を標準 DiagnosticArray で配信する。

        recognition 改善時だけ `publish_debug_diagnostics:=True` で有効にする。
        独自 msg を増やさず、ゲートに落ちた候補の reason/metric を ros2 topic echo
        や bag で追えるようにする。
        """
        da = DiagnosticArray()
        da.header = header
        for obj_i, (pending_item, (selected_name, selected_conf)) in \
                enumerate(zip(pending, results)):
            tid, _crop, crop_fov = pending_item[:3]
            crop_yaw = pending_item[3] if len(pending_item) > 3 else 0.0
            crop_pitch = pending_item[4] if len(pending_item) > 4 else 0.0
            crop_height_frac = pending_item[5] if len(pending_item) > 5 else 0.0
            crop_mode = pending_item[14] if len(pending_item) > 14 else 'center'
            bbox_margin_deg = (
                pending_item[15] if len(pending_item) > 15 else None)
            bbox_projected_fov_deg = (
                pending_item[16] if len(pending_item) > 16 else None)
            tid_hex = bytes(tid).hex()
            fov_deg = math.degrees(float(crop_fov))
            yaw_deg = math.degrees(float(crop_yaw))
            pitch_deg = math.degrees(float(crop_pitch))
            candidates = debug_infos[obj_i] if obj_i < len(debug_infos) else []
            crop_name = 'fov%.1f_yaw%+.1f_pitch%+.1f_h%.2f' % (
                fov_deg, yaw_deg, pitch_deg, float(crop_height_frac))
            if str(crop_mode or 'center') != 'center':
                crop_name += '_%s' % str(crop_mode)
                if bbox_margin_deg is not None:
                    crop_name += 'm%.1f' % float(bbox_margin_deg)
            extra_values = [
                KeyValue(key='crop_mode',
                         value=str(crop_mode or 'center')),
            ]
            if bbox_margin_deg is not None:
                extra_values.append(KeyValue(
                    key='crop_shape_bbox_margin_deg',
                    value='%.1f' % float(bbox_margin_deg)))
            if bbox_projected_fov_deg is not None:
                extra_values.append(KeyValue(
                    key='crop_shape_bbox_projected_fov_deg',
                    value='%.1f' % float(bbox_projected_fov_deg)))
            if not candidates:
                st = DiagnosticStatus()
                st.level = DiagnosticStatus.WARN
                st.name = '%s/%s' % (tid_hex, crop_name)
                st.message = 'no_yolo_detection'
                st.values = [
                    KeyValue(key='crop_fov_deg', value='%.1f' % fov_deg),
                    KeyValue(key='crop_yaw_offset_deg',
                             value='%.1f' % yaw_deg),
                    KeyValue(key='crop_pitch_offset_deg',
                             value='%.1f' % pitch_deg),
                    KeyValue(key='crop_shape_height_frac',
                             value='%.2f' % float(crop_height_frac)),
                    KeyValue(key='selected', value=str(selected_name or '')),
                    KeyValue(key='selected_conf',
                             value='%.3f' % float(selected_conf or 0.0)),
                ] + extra_values
                da.status.append(st)
                continue
            for cand_i, cand in enumerate(candidates):
                reason = str(cand.get('reason', ''))
                st = DiagnosticStatus()
                st.level = (DiagnosticStatus.OK
                            if reason == 'accepted'
                            else DiagnosticStatus.WARN)
                st.name = '%s/%s/%d' % (tid_hex, crop_name, cand_i)
                st.message = reason
                values = [
                    KeyValue(key='crop_fov_deg', value='%.1f' % fov_deg),
                    KeyValue(key='crop_yaw_offset_deg',
                             value='%.1f' % yaw_deg),
                    KeyValue(key='crop_pitch_offset_deg',
                             value='%.1f' % pitch_deg),
                    KeyValue(key='crop_shape_height_frac',
                             value='%.2f' % float(crop_height_frac)),
                    KeyValue(key='crop_mode',
                             value=str(crop_mode or 'center')),
                    KeyValue(key='class', value=str(cand.get('name', ''))),
                    KeyValue(key='conf',
                             value='%.3f' % float(cand.get('conf', 0.0))),
                    KeyValue(key='area_frac',
                             value='%.5f' % float(cand.get('area_frac', 0.0))),
                    KeyValue(key='center_dx',
                             value='%.3f' % float(cand.get('center_dx', 0.0))),
                    KeyValue(key='center_dy',
                             value='%.3f' % float(cand.get('center_dy', 0.0))),
                    KeyValue(key='contains_center',
                             value=str(bool(cand.get('contains_center', False)))),
                    KeyValue(key='center_overlap',
                             value='%.3f' % float(cand.get('center_overlap', 0.0))),
                ]
                values.extend(extra_values[1:])
                if cand.get('mask_overlap') is not None:
                    values.append(KeyValue(
                        key='mask_overlap',
                        value='%.3f' % float(cand.get('mask_overlap'))))
                if cand.get('plant_color') is not None:
                    values.append(KeyValue(
                        key='plant_color',
                        value='%.3f' % float(cand.get('plant_color'))))
                if cand.get('bbox_xyxy') is not None:
                    values.append(KeyValue(
                        key='bbox_xyxy',
                        value=','.join('%.1f' % float(v)
                                       for v in cand.get('bbox_xyxy'))))
                if selected_name is not None:
                    values.append(KeyValue(key='selected',
                                           value=str(selected_name)))
                    values.append(KeyValue(
                        key='selected_conf',
                        value='%.3f' % float(selected_conf or 0.0)))
                st.values = values
                da.status.append(st)
        self.pub_debug.publish(da)


def main():
    rclpy.init()
    node = ObjectClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
