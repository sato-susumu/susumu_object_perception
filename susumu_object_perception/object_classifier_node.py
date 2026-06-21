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
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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
    """YOLOv8(ultralytics) でクロップ画像を物体検出し、最大信頼度のクラスを返す。

    初期化（ultralytics/torch import・重み読込）に失敗したら RuntimeError を投げる。
    ノード側はそれを [FATAL] にして終了する（無分類での素通しはしない）。
    """

    def __init__(self, weights, conf, center_tolerance_frac,
                 min_box_area_frac, center_window_frac,
                 min_center_window_overlap, require_mask_center,
                 mask_center_window_frac, min_mask_center_overlap,
                 plant_color_min_frac, debug_diagnostics=False):
        self.conf = conf
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
        for r, bgr in zip(res, bgrs):
            name, conf, debug = self._classify_result(r, bgr)
            outputs.append((name, conf))
            self.last_debug.append(debug)
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
                        reject_reason))
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
                            reject_reason))
                    continue
            if self.min_center_window_overlap > 0.0 and \
                    center_overlap < self.min_center_window_overlap:
                reject_reason = 'center_window_overlap'
                if self.debug_diagnostics:
                    debug.append(self._candidate_debug(
                        name, c, area_frac, dx, dy, contains_center,
                        center_overlap, mask_overlap, plant_color,
                        reject_reason))
                continue
            if self.require_mask_center:
                if mask_overlap is None or mask_overlap < self.min_mask_center_overlap:
                    reject_reason = 'mask_center_overlap'
                    if self.debug_diagnostics:
                        debug.append(self._candidate_debug(
                            name, c, area_frac, dx, dy, contains_center,
                            center_overlap, mask_overlap, plant_color,
                            reject_reason))
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
                        reject_reason))
                continue
            if self.debug_diagnostics:
                debug.append(self._candidate_debug(
                    name, c, area_frac, dx, dy, contains_center,
                    center_overlap, mask_overlap, plant_color, 'accepted'))
            if c > best_conf:
                best_conf = c
                best_name = name
        return best_name, best_conf, debug

    @staticmethod
    def _candidate_debug(name, conf, area_frac, dx, dy, contains_center,
                         center_overlap, mask_overlap, plant_color, reason):
        return {
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
        # uuid(bytes) -> dict(label, coco_name, conf, last_seen_t, last_infer_t,
        #                     candidate_key, candidate_hits, misses)
        self._class_cache = {}

        # 分類器初期化。失敗したら無分類で素通しせず FATAL で終了（自動フォールバック禁止）。
        try:
            self.classifier = YoloClassifier(
                weights, conf, center_tol, min_box_area,
                center_window_frac, min_center_overlap,
                require_mask_center, mask_center_window_frac,
                min_mask_center_overlap, plant_color_min_frac,
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
            'crop_fovs_deg=%s yolo_imgsz=%d '
            '-> /perception/tracked_objects_classified, '
            '/perception/object_classes/markers' % (
                self.input_objects, self.input_image,
                ','.join('%.1f' % math.degrees(v) for v in self.crop_fovs),
                self.yolo_imgsz))

    def on_image(self, msg):
        try:
            self.latest_image = (
                self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'),
                msg.header.stamp)
        except Exception as exc:
            self.get_logger().warning('omni image decode 失敗: %s' % exc)

    def _transform_point(self, point, source_frame):
        try:
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
        p = np.array([point.x, point.y, point.z], dtype=np.float32)
        return self.calibration_rot @ (rot @ p + trans)

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

    def _perspective_crop(self, pano, direction, crop_fov=None):
        dirs = perspective_directions(
            direction, self.crop_w, self.crop_h, crop_fov or self.crop_fov)
        if dirs is None:
            return None

        h, w = pano.shape[:2]
        map_x, map_y, _ = equirect_uv(
            dirs, w, h, self.projection_model)
        return cv2.remap(pano, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_WRAP)

    def on_objects(self, msg: TrackedObjects):
        if self.latest_image is None or not msg.objects:
            # 画像が無い間は素通し（分類できないが物体情報は失わない）。
            self.pub_objects.publish(msg)
            return
        pano, _ = self.latest_image

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
                p_cam = self._transform_point(pose.position, msg.header.frame_id)
                crops = []
                if p_cam is not None:
                    for crop_fov in self.crop_fovs:
                        crop = self._perspective_crop(pano, p_cam, crop_fov)
                        if crop is not None and crop.size > 0:
                            crops.append((crop, crop_fov))
                if not crops:
                    self._update_class_cache(tid, None, 0.0, now)
                    continue
                for crop, crop_fov in crops:
                    pending.append((tid, crop, crop_fov))
                if len({item[0] for item in pending}) >= limit:
                    break

            if pending:
                results = self.classifier.classify_many(
                    [crop for _, crop, _ in pending])
                best_by_tid = {}
                for (tid, crop, crop_fov), (detected_name, detected_conf) in \
                        zip(pending, results):
                    current = best_by_tid.get(tid)
                    if current is None or detected_conf > current[1]:
                        best_by_tid[tid] = (
                            detected_name, detected_conf, crop, crop_fov)
                for tid, (detected_name, detected_conf, crop, _crop_fov) in \
                        best_by_tid.items():
                    self._update_class_cache(tid, detected_name,
                                             detected_conf, now)
                    inferred_crops[tid] = crop
                if self.pub_debug is not None:
                    self._publish_debug_diagnostics(
                        msg.header, pending, results,
                        getattr(self.classifier, 'last_debug', []))
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

                if coco_name is not None and conf >= self.min_accept_conf:
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

    def _update_class_cache(self, tid, detected_name, detected_conf, now):
        cached = self._class_cache.get(tid, {})
        if detected_name is None or detected_conf < self.min_accept_conf:
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

    def _publish_debug_diagnostics(self, header, pending, results, debug_infos):
        """分類候補の採否理由を標準 DiagnosticArray で配信する。

        recognition 改善時だけ `publish_debug_diagnostics:=True` で有効にする。
        独自 msg を増やさず、ゲートに落ちた候補の reason/metric を ros2 topic echo
        や bag で追えるようにする。
        """
        da = DiagnosticArray()
        da.header = header
        for obj_i, ((tid, _crop, crop_fov), (selected_name, selected_conf)) in \
                enumerate(zip(pending, results)):
            tid_hex = bytes(tid).hex()
            fov_deg = math.degrees(float(crop_fov))
            candidates = debug_infos[obj_i] if obj_i < len(debug_infos) else []
            if not candidates:
                st = DiagnosticStatus()
                st.level = DiagnosticStatus.WARN
                st.name = '%s/fov%.1f' % (tid_hex, fov_deg)
                st.message = 'no_yolo_detection'
                st.values = [
                    KeyValue(key='crop_fov_deg', value='%.1f' % fov_deg),
                    KeyValue(key='selected', value=str(selected_name or '')),
                    KeyValue(key='selected_conf',
                             value='%.3f' % float(selected_conf or 0.0)),
                ]
                da.status.append(st)
                continue
            for cand_i, cand in enumerate(candidates):
                reason = str(cand.get('reason', ''))
                st = DiagnosticStatus()
                st.level = (DiagnosticStatus.OK
                            if reason == 'accepted'
                            else DiagnosticStatus.WARN)
                st.name = '%s/fov%.1f/%d' % (tid_hex, fov_deg, cand_i)
                st.message = reason
                values = [
                    KeyValue(key='crop_fov_deg', value='%.1f' % fov_deg),
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
                if cand.get('mask_overlap') is not None:
                    values.append(KeyValue(
                        key='mask_overlap',
                        value='%.3f' % float(cand.get('mask_overlap'))))
                if cand.get('plant_color') is not None:
                    values.append(KeyValue(
                        key='plant_color',
                        value='%.3f' % float(cand.get('plant_color'))))
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
