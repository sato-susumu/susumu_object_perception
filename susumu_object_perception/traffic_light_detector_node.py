#!/usr/bin/env python3
# 交通信号認識ノード。カメラ画像から信号灯を検出し赤/黄/青を分類して
# autoware_perception_msgs/TrafficSignalArray と vision_msgs/Detection2DArray を出す。
# 設計は docs/traffic_light_recognition.md を参照。
#
# 検出バックエンドは method パラメータで切替（Autoware の classifier_type 思想）:
#   method:=classic … HSV 色マスク + 円形度で検出・色分類（OpenCV のみ、学習不要）
#   method:=yolo    … YOLOv8(ultralytics) で検出＝色分類（重みが要る。初期化失敗時は
#                     classic に勝手に落とさず [FATAL] で終了する＝自動フォールバック禁止）
#
# 安全原則: 誤った GREEN(GO) が最も危険なので、確信が無い検出は UNKNOWN 扱いにする
# （precision 優先）。
#
# 入力 : 全天球カメラ（既定）/omni_camera/image_raw/image_color、または
#        通常カメラ /camera/image_raw（パラメータ input_image）
# 出力 : /perception/traffic_signals  (autoware_perception_msgs/TrafficSignalArray)
#        /perception/traffic_light/rois (vision_msgs/Detection2DArray, 可視化/デバッグ用)
#
# 全天球カメラ前提（omni_mode:=true、既定）:
#   正距円筒（Webots cylindrical）画像を全周 N 分割の透視投影ビューに展開し、各ビューで
#   既存の検出バックエンド（classic/yolo）を実行する。信号灯は透視投影でないと円形度・
#   YOLO・灯位置判定が歪むため、検出は必ず透視ビュー上で行う。各検出には方位(yaw_deg)を
#   付与し、ROI には所属ビュー番号・方位を載せる。
#
# ※ カメラトピックはシミュレータで異なる（Gazebo=/camera/image_raw,
#    Webots 通常カメラ=/camera/image_raw/image_color,
#    Webots 全天球=/omni_camera/image_raw/image_color）。input_image で吸収する。

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from autoware_perception_msgs.msg import (TrafficSignalArray, TrafficSignal,
                                          TrafficSignalElement)
from vision_msgs.msg import Detection2DArray, Detection2D, \
    ObjectHypothesisWithPose, BoundingBox2D

import cv2
from cv_bridge import CvBridge

from susumu_object_perception.omni_projection import (
    equirect_uv, perspective_directions)


def equirect_to_perspective(pano, yaw, pitch, fov, out_w, out_h,
                            projection_model='webots_cylindrical'):
    """正距円筒パノラマから、方位 yaw・仰角 pitch を中心とする透視投影ビューを切り出す。

    object_image_crop_node._perspective_crop と同じ投影式（Webots cylindrical shader 互換）。
    yaw/pitch/fov は rad。borderMode=WRAP で左右の継ぎ目をまたいでサンプルできる。
    """
    # 中心方向ベクトル（カメラ座標: x=前方, y=左, z=上）。
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    forward = np.array([cp * cy, cp * sy, sp], dtype=np.float32)
    dirs = perspective_directions(forward, out_w, out_h, fov)

    h, w = pano.shape[:2]
    map_x, map_y, _ = equirect_uv(dirs, w, h, projection_model)
    return cv2.remap(pano, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_WRAP)


# 色名 → TrafficSignalElement.color 定数。
COLOR_TO_ELEMENT = {
    'red': TrafficSignalElement.RED,
    'amber': TrafficSignalElement.AMBER,
    'green': TrafficSignalElement.GREEN,
}


class ClassicDetector:
    """HSV 色マスク + 円形度で信号灯を検出・色分類する古典 CV バックエンド。

    シミュレータの発光信号や、はっきり点灯した実信号に有効。学習不要・依存は OpenCV のみ。
    各色について HSV 範囲でマスク → 輪郭抽出 → 面積/円形度/アスペクト比で信号灯候補を絞る。
    """

    def __init__(self, node: Node):
        d = node.declare_parameter
        # HSV しきい値（OpenCV の H は 0..179）。赤は色相が 0 跨ぎなので 2 レンジ。
        self.red1_lo = tuple(d('classic.red1_lo', [0, 100, 100]).value)
        self.red1_hi = tuple(d('classic.red1_hi', [10, 255, 255]).value)
        self.red2_lo = tuple(d('classic.red2_lo', [160, 100, 100]).value)
        self.red2_hi = tuple(d('classic.red2_hi', [179, 255, 255]).value)
        self.amber_lo = tuple(d('classic.amber_lo', [15, 100, 100]).value)
        self.amber_hi = tuple(d('classic.amber_hi', [35, 255, 255]).value)
        self.green_lo = tuple(d('classic.green_lo', [40, 80, 80]).value)
        self.green_hi = tuple(d('classic.green_hi', [90, 255, 255]).value)
        # 候補絞り込み。
        self.min_area = int(d('classic.min_area', 20).value)        # 最小画素面積
        self.min_circularity = float(d('classic.min_circularity', 0.55).value)
        self.min_confidence = float(d('classic.min_confidence', 0.5).value)

    def _color_masks(self, hsv):
        red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array(self.red1_lo), np.array(self.red1_hi)),
            cv2.inRange(hsv, np.array(self.red2_lo), np.array(self.red2_hi)))
        amber = cv2.inRange(hsv, np.array(self.amber_lo), np.array(self.amber_hi))
        green = cv2.inRange(hsv, np.array(self.green_lo), np.array(self.green_hi))
        return {'red': red, 'amber': amber, 'green': green}

    def detect(self, bgr):
        """BGR 画像 → 検出リスト [{color, confidence, bbox(x,y,w,h)}]。"""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        out = []
        for color, mask in self._color_masks(hsv).items():
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_area:
                    continue
                perim = cv2.arcLength(c, True)
                if perim <= 0:
                    continue
                # 円形度 4πA/P^2（1.0 が真円）。信号灯は円形なので円形度で絞る。
                circularity = 4.0 * np.pi * area / (perim * perim)
                if circularity < self.min_circularity:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                # confidence は円形度と充填率（輪郭面積/外接矩形面積）の積で近似。
                fill = area / float(w * h) if w * h > 0 else 0.0
                conf = float(np.clip(circularity * fill, 0.0, 1.0))
                if conf < self.min_confidence:
                    continue
                out.append({'color': color, 'confidence': conf,
                            'bbox': (x, y, w, h)})
        return out


class YoloDetector:
    """YOLOv8(ultralytics) バックエンド。Autoware の detector→classifier 2 段に倣い、
    YOLO は信号の「検出（bbox）」を担い、色は bbox 内を ClassicDetector の HSV 判定で決める。

    既定の重みは COCO 学習済み YOLOv8n（class 9 = 'traffic light'）。専用学習(BSTLD等)の
    重みを yolo.weights で与えれば差し替え可能。重みのクラス名に赤/黄/青が含まれる場合は
    それを優先採用し、汎用 'traffic light' クラスのみのときは HSV で色を補う。

    ultralytics/重み読込に失敗したら RuntimeError を投げる。ノード側はそれを [FATAL] にして
    終了する（classic への自動フォールバックはしない）。
    """

    # YOLO クラス名 → 色名（専用重み用）。汎用 coco の 'traffic light' は色を持たないので
    # ここに該当せず、bbox 内 HSV で色を決める。
    CLASS_TO_COLOR = {'red': 'red', 'yellow': 'amber', 'amber': 'amber',
                      'green': 'green'}

    def __init__(self, node: Node, classic: 'ClassicDetector'):
        d = node.declare_parameter
        self.weights = d('yolo.weights', 'yolov8n.pt').value  # 既定 = coco YOLOv8n
        self.conf = float(d('yolo.conf', 0.3).value)
        self.classic = classic  # bbox 内の色判定に流用
        # 灯位置判定: 信号機 bbox 内の点灯位置で色を確証（発光色が機種で異なる問題を吸収）。
        self.position_aware = bool(d('position_aware', True).value)
        # 並び方向: vertical=上赤/下青, horizontal=左赤/右青。
        self.lamp_layout = d('lamp_layout', 'vertical').value
        try:
            import torch
            # torch 2.6+ は torch.load の weights_only 既定が True で、ultralytics 旧版の
            # チェックポイント読込が失敗する。公式/自前の信頼できる重みのみ扱う前提で False に。
            _orig = torch.load

            def _patched(*a, **k):
                k.setdefault('weights_only', False)
                return _orig(*a, **k)
            torch.load = _patched
            from ultralytics import YOLO
            # 重み読込もここで行い、ファイル不在などの失敗も同じ RuntimeError 経路に乗せる
            # （ノード側が fatal を出して終了し、classic に勝手に落ちないため）。
            self.model = YOLO(self.weights)
        except Exception as e:
            raise RuntimeError(
                'ultralytics/torch 初期化または重み読込に失敗: %s' % e)

    def _classify_color(self, bgr, x, y, w, h):
        """信号機 bbox 内を色相 + 点灯位置で判定。発光色が機種で異なっても位置で確証する。

        手順: HSV マスクの和集合で「点灯塊」を取り、その重心の bbox 内相対位置から
        位置由来色（縦型: 上=red/下=green/中=amber、横型: 左=red/右=green/中=amber）を出す。
        色相由来色（最大占有色）と突き合わせ、食い違えば安全側（GREEN を疑う）に倒す。
        """
        import cv2 as _cv2
        import numpy as _np
        crop = bgr[max(0, y):y + h, max(0, x):x + w]
        if crop.size == 0:
            return None, 0.0
        hsv = _cv2.cvtColor(crop, _cv2.COLOR_BGR2HSV)
        masks = self.classic._color_masks(hsv)

        # 色相由来: 最大占有色。
        hue_color, hue_ratio = None, 0.0
        for color, mask in masks.items():
            ratio = float((mask > 0).sum()) / float(mask.size)
            if ratio > hue_ratio:
                hue_color, hue_ratio = color, ratio
        if hue_color is None or hue_ratio < 0.02:
            return None, 0.0

        if not self.position_aware:
            return hue_color, hue_ratio

        # 位置由来: 全色マスクの和集合（=点灯塊）の重心位置で判定。
        lit = masks['red'] | masks['amber'] | masks['green']
        ys, xs = _np.where(lit > 0)
        if len(xs) < 5:
            return hue_color, hue_ratio
        ch, cw = lit.shape[0], lit.shape[1]
        if self.lamp_layout == 'horizontal':
            frac = float(_np.mean(xs)) / max(1, cw)   # 0=左(赤) .. 1=右(青)
        else:
            frac = float(_np.mean(ys)) / max(1, ch)   # 0=上(赤) .. 1=下(青)
        if frac < 0.38:
            pos_color = 'red'
        elif frac > 0.62:
            pos_color = 'green'
        else:
            pos_color = 'amber'

        # 突き合わせ: 一致ならそれ。食い違いは安全側（GREEN を信用しすぎない）。
        if hue_color == pos_color:
            return pos_color, hue_ratio
        # 位置が red/amber を示すのに色相が green と言う等 → 安全側で位置を優先。
        # （誤って GREEN=GO にしないことを最優先＝precision 優先）
        if 'green' in (hue_color, pos_color) and pos_color != 'green':
            return pos_color, hue_ratio * 0.8
        # それ以外の不一致は位置を採用（位置は発光色に依存せず堅牢）。
        return pos_color, hue_ratio * 0.9

    def _result_to_dets(self, r, bgr):
        """1 画像の YOLO 結果(r)を検出 dict のリストに変換する。色は bbox 内 HSV で決める。"""
        out = []
        names = r.names
        for b in r.boxes:
            cls = str(names[int(b.cls)]).lower()
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            w, h = x2 - x1, y2 - y1
            conf = float(b.conf)
            # 重みが色クラスを持つならそれを採用。
            color = self.CLASS_TO_COLOR.get(cls)
            if color is None:
                # 汎用 'traffic light' 等 → bbox 内 HSV で色を決める。
                if 'traffic' not in cls and 'light' not in cls:
                    continue
                color, ratio = self._classify_color(bgr, x1, y1, w, h)
                if color is None or ratio < 0.02:
                    continue  # 点灯色が拾えない（消灯/不明）→ 出さない（precision 優先）
            out.append({'color': color, 'confidence': conf,
                        'bbox': (x1, y1, w, h)})
        return out

    def detect(self, bgr):
        res = self.model.predict(bgr, conf=self.conf, verbose=False)
        out = []
        for r in res:
            out.extend(self._result_to_dets(r, bgr))
        return out

    def detect_batch(self, images):
        """複数画像をまとめて 1 回の predict で推論し、画像ごとの検出 dict リストを返す。

        全天球の全周 N ビューを 1 枚ずつ推論するより、ultralytics のバッチ推論（リスト渡し）の
        ほうが呼び出し/前処理オーバーヘッドが減り速い（GPU では並列化でさらに有利）。各ビューは
        個別画像のままなので、8 ビューを 1 枚に結合する方式と違い解像度（＝小物体の検出力）は
        落ちない。戻り値は images と同じ並びの「検出 dict リストのリスト」。
        """
        if not images:
            return []
        res = self.model.predict(images, conf=self.conf, verbose=False)
        return [self._result_to_dets(r, img) for r, img in zip(res, images)]


class TrafficLightDetectorNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()

        # 全天球モード（既定 ON）。全周を透視投影 N ビューに分けて検出する。
        self.omni_mode = bool(self.declare_parameter('omni_mode', True).value)
        default_input = ('/omni_camera/image_raw/image_color'
                         if self.omni_mode else '/camera/image_raw')
        input_image = self.declare_parameter('input_image', default_input).value
        self.method = self.declare_parameter('method', 'classic').value

        # 全天球の透視展開パラメータ。
        self.num_views = int(self.declare_parameter('omni.num_views', 8).value)
        self.view_fov = math.radians(
            float(self.declare_parameter('omni.view_fov_deg', 75.0).value))
        # 信号灯は通常やや上方にあるので、ビュー中心を少し上向きに振れる。
        self.view_pitch = math.radians(
            float(self.declare_parameter('omni.view_pitch_deg', 0.0).value))
        self.view_w = int(self.declare_parameter('omni.view_width', 640).value)
        self.view_h = int(self.declare_parameter('omni.view_height', 480).value)
        self.projection_model = self.declare_parameter(
            'omni.projection_model', 'webots_cylindrical').value

        # 処理レート上限 [Hz]。画像は 15Hz 等で来るが、信号は急変しないので間引く。
        # 全周 N ビューぶん YOLO/HSV を回すと CPU では重いため、前回処理から
        # 1/max_rate_hz 秒経つまで新しい画像をスキップする。0 以下で間引き無効。
        self.max_rate_hz = float(self.declare_parameter('max_rate_hz', 3.0).value)
        self._min_interval = (1.0 / self.max_rate_hz) if self.max_rate_hz > 0 else 0.0
        self._last_proc_t = None

        # 重複統合のマージ角 [deg]。視野の重なる隣接ビューに同じ信号が重複検出されるので、
        # 方向ベクトルの角度差がこの値未満で同色なら 1 つに統合する（全天球で 1 箇所の信号 →
        # 認識結果 1 つ）。ビュー間隔(360/N)やビュー画角に応じて調整。0 以下で統合無効。
        self.merge_angle = math.radians(
            float(self.declare_parameter('merge_angle_deg', 20.0).value))

        # classic は常に生成（yolo の bbox 内色判定にも流用するため）。
        classic = ClassicDetector(self)

        # バックエンド選択。method:=yolo を指定したのに初期化に失敗した場合、勝手に classic へ
        # フォールバックしない（信号機の物体検出が色起点検出にすり替わると挙動が変わり、
        # 気付かず精度が落ちるため）。明確にエラーを出してノードを終了させる。
        if self.method == 'yolo':
            try:
                self.backend = YoloDetector(self, classic)
                self.get_logger().info('backend=yolo (検出=YOLO, 色=bbox内HSV)')
            except RuntimeError as e:
                self.get_logger().fatal(
                    'method:=yolo を指定したが yolo の初期化に失敗した: %s\n'
                    'classic への自動フォールバックはしない。ultralytics/torch と '
                    'yolo.weights を用意するか、明示的に method:=classic で起動すること。'
                    % e)
                raise
        elif self.method == 'classic':
            self.backend = classic
            self.get_logger().info('backend=classic')
        else:
            self.get_logger().fatal(
                "未知の method='%s'（classic か yolo を指定）" % self.method)
            raise ValueError('unknown method: %s' % self.method)

        self.pub_signals = self.create_publisher(
            TrafficSignalArray, '/perception/traffic_signals', 10)
        self.pub_rois = self.create_publisher(
            Detection2DArray, '/perception/traffic_light/rois', 10)
        self.create_subscription(
            Image, input_image, self.on_image, qos_profile_sensor_data)

        mode = ('omni(%d views, fov=%.0fdeg)'
                % (self.num_views, math.degrees(self.view_fov))
                if self.omni_mode else 'pinhole')
        self.get_logger().info(
            'traffic_light_detector started. input=%s method=%s mode=%s '
            '-> /perception/traffic_signals, /perception/traffic_light/rois'
            % (input_image, self.method, mode))

    def _view_pixel_to_dir(self, view_yaw, bbox):
        """透視ビュー内の bbox 中心画素を、ロボット座標系の方向ベクトルに逆変換する。

        equirect_to_perspective と同じ視錐台（forward/right/up, tan_half）で、
        bbox 中心の正規化画素 (px,py)∈[-1,1] から方向を復元する。これにより検出の
        正確な方位(yaw)・仰角(pitch)が分かり、LiDAR で距離を引いて 3D 位置を出せる。
        """
        x, y, w, h = bbox
        # bbox 中心の正規化座標（画像中心=0、右/下が +。equirect_to_perspective の xx,yy 系）。
        cx = (x + w / 2.0) / self.view_w * 2.0 - 1.0
        cy = (y + h / 2.0) / self.view_h * 2.0 - 1.0

        cyaw, syaw = math.cos(view_yaw), math.sin(view_yaw)
        cp, sp = math.cos(self.view_pitch), math.sin(self.view_pitch)
        forward = np.array([cp * cyaw, cp * syaw, sp], dtype=np.float64)
        world_up = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(forward, world_up))) > 0.95:
            world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, world_up)
        right /= max(np.linalg.norm(right), 1e-6)
        up = np.cross(right, forward)
        up /= max(np.linalg.norm(up), 1e-6)

        tan_half = math.tan(self.view_fov / 2.0)
        aspect = self.view_w / float(self.view_h)
        d = (forward + right * (cx * tan_half * aspect) + up * (-cy * tan_half))
        d /= max(np.linalg.norm(d), 1e-6)
        yaw = math.atan2(d[1], d[0])
        pitch = math.asin(max(-1.0, min(1.0, d[2])))
        return yaw, pitch, d

    def _detect_omni(self, pano):
        """全天球パノラマを全周 N 分割の透視ビューに展開し、各ビューで検出する。

        返り値は検出 dict のリスト。各 dict に元の検出項目（color/confidence/bbox）に加え
        view（ビュー番号）、yaw_deg（ビュー中心方位[deg]）、および bbox 中心から復元した
        正確な方向 dir_yaw_deg / dir_pitch_deg（ロボット座標系）を付ける。後者は LiDAR で
        距離を引いて信号の 3D 位置を出すのに使う。bbox はビュー内座標。
        """
        # 全ビューを先に生成。
        yaws = [2.0 * math.pi * v / self.num_views for v in range(self.num_views)]
        views = [equirect_to_perspective(
            pano, yaw, self.view_pitch, self.view_fov,
            self.view_w, self.view_h, self.projection_model) for yaw in yaws]

        # 検出。yolo はバッチ推論（1 回の predict で全ビュー）で速い。8 ビューを 1 枚に結合する
        # 方式は入力リサイズで解像度が落ち小物体を取りこぼすため採らず、個別ビューのまま
        # バッチに渡す（検出力を保ったまま呼び出し/前処理オーバーヘッドだけ削減）。classic(HSV)は
        # バッチの恩恵が無いので個別処理。
        if hasattr(self.backend, 'detect_batch'):
            per_view = self.backend.detect_batch(views)
        else:
            per_view = [self.backend.detect(view) for view in views]

        out = []
        for v, (yaw, dets) in enumerate(zip(yaws, per_view)):
            for det in dets:
                det = dict(det)
                det['view'] = v
                det['yaw_deg'] = int(round(math.degrees(yaw))) % 360
                d_yaw, d_pitch, d_vec = self._view_pixel_to_dir(yaw, det['bbox'])
                det['dir_yaw_deg'] = math.degrees(d_yaw)
                det['dir_pitch_deg'] = math.degrees(d_pitch)
                det['dir'] = (float(d_vec[0]), float(d_vec[1]), float(d_vec[2]))
                out.append(det)
        # 視野の重なる隣接ビューに同じ信号が重複検出されるので、方向で 1 つに統合する。
        return self._merge_detections(out)

    def _merge_detections(self, dets):
        """全周ビューに重複して出た同一信号を 1 つに統合する。

        全天球を N ビューに分けると視野が重なり、同じ信号機が複数ビューに出る。各検出は方向
        ベクトル `dir`（ロボット座標）を持つので、方向が近く(角度差 < merge_angle)・色が同じ
        検出を 1 グループとみなし、最高信頼度のものを代表に残す（方位ベースの NMS 相当）。
        これで「全天球で 1 箇所の信号 → 認識結果 1 つ」になる。
        """
        if not dets or self.merge_angle <= 0.0:
            return dets
        cos_th = math.cos(self.merge_angle)
        used = [False] * len(dets)
        merged = []
        # 信頼度の高い順に代表を選び、近い方向・同色をまとめる。
        order = sorted(range(len(dets)), key=lambda i: -dets[i]['confidence'])
        for i in order:
            if used[i]:
                continue
            used[i] = True
            rep = dets[i]
            di = np.array(rep['dir'], dtype=float)
            for j in order:
                if used[j] or dets[j]['color'] != rep['color']:
                    continue
                dj = np.array(dets[j]['dir'], dtype=float)
                if float(np.dot(di, dj)) > cos_th:  # 方向が近い＝同一信号
                    used[j] = True
            merged.append(rep)
        return merged

    def on_image(self, msg: Image):
        # 処理レート上限による間引き。前回処理から _min_interval 秒経っていなければスキップ。
        # 実時間(monotonic)で測る（CPU 負荷の実時間が問題なので sim time ではなく実時間）。
        if self._min_interval > 0.0:
            now = time.monotonic()
            if self._last_proc_t is not None and \
                    (now - self._last_proc_t) < self._min_interval:
                return
            self._last_proc_t = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge 変換失敗: %s' % e)
            return

        if self.omni_mode:
            dets = self._detect_omni(bgr)
        else:
            dets = self.backend.detect(bgr)

        sig_arr = TrafficSignalArray()
        sig_arr.stamp = msg.header.stamp
        roi_arr = Detection2DArray()
        roi_arr.header = msg.header

        for i, det in enumerate(dets):
            color = COLOR_TO_ELEMENT.get(det['color'],
                                         TrafficSignalElement.UNKNOWN)
            # 信号状態（TrafficSignal）。地図無しなので id は検出インデックス。
            elem = TrafficSignalElement()
            elem.color = color
            elem.shape = TrafficSignalElement.CIRCLE  # 古典は円のみ判定
            elem.status = TrafficSignalElement.SOLID_ON
            elem.confidence = float(det['confidence'])
            sig = TrafficSignal()
            # 地図無しなので id は識別子。全天球モードでは方位[deg]を入れ、
            # 「どの方向の信号か」を id から読めるようにする（通常モードは検出 index）。
            sig.traffic_signal_id = det.get('yaw_deg', i) if self.omni_mode else i
            sig.elements = [elem]
            sig_arr.signals.append(sig)

            # ROI（可視化/デバッグ用）。bbox は所属ビュー内の座標。
            x, y, w, h = det['bbox']
            d2 = Detection2D()
            d2.header = msg.header
            d2.bbox = BoundingBox2D()
            d2.bbox.center.position.x = float(x + w / 2.0)
            d2.bbox.center.position.y = float(y + h / 2.0)
            d2.bbox.size_x = float(w)
            d2.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            # 全天球モードでは class_id に方位とビュー番号を載せる（可視化が読む）。
            if self.omni_mode:
                hyp.hypothesis.class_id = '%s@%ddeg(v%d)' % (
                    det['color'], det.get('yaw_deg', 0), det.get('view', 0))
                # LiDAR 連携用に bbox 中心から復元した方向単位ベクトル（ロボット座標）を
                # pose.position に載せる。LiDAR 連携ノードがこの方向で距離を引き 3D 位置にする。
                if 'dir' in det:
                    hyp.pose.pose.position.x = det['dir'][0]
                    hyp.pose.pose.position.y = det['dir'][1]
                    hyp.pose.pose.position.z = det['dir'][2]
            else:
                hyp.hypothesis.class_id = det['color']
            hyp.hypothesis.score = float(det['confidence'])
            d2.results = [hyp]
            roi_arr.detections.append(d2)

        self.pub_signals.publish(sig_arr)
        self.pub_rois.publish(roi_arr)


def main():
    rclpy.init()
    node = TrafficLightDetectorNode()
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
