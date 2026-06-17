#!/usr/bin/env python3
# 交通信号認識ノード。カメラ画像から信号灯を検出し赤/黄/青を分類して
# autoware_perception_msgs/TrafficSignalArray と vision_msgs/Detection2DArray を出す。
# 設計は docs/traffic_light_recognition.md を参照。
#
# 検出バックエンドは method パラメータで切替（Autoware の classifier_type 思想）:
#   method:=classic … HSV 色マスク + 円形度で検出・色分類（OpenCV のみ、学習不要）
#   method:=yolo    … YOLOv8(ultralytics) で検出＝色分類（重みが要る。未導入なら起動時に警告）
#
# 安全原則: 誤った GREEN(GO) が最も危険なので、確信が無い検出は UNKNOWN 扱いにする
# （precision 優先）。
#
# 入力 : /camera/image_raw            (sensor_msgs/Image, パラメータ input_image)
# 出力 : /perception/traffic_signals  (autoware_perception_msgs/TrafficSignalArray)
#        /perception/traffic_light/rois (vision_msgs/Detection2DArray, 可視化/デバッグ用)
#
# ※ カメラトピックはシミュレータで異なる（Gazebo=/camera/image_raw,
#    Webots=/camera/image_raw/image_color）。input_image パラメータで吸収する。

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

    ultralytics/重み読込に失敗したら RuntimeError を投げ、ノード側で classic にフォールバックする。
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
        except Exception as e:
            raise RuntimeError('ultralytics/torch 初期化失敗: %s' % e)
        self.model = YOLO(self.weights)

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

    def detect(self, bgr):
        out = []
        res = self.model.predict(bgr, conf=self.conf, verbose=False)
        for r in res:
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


class TrafficLightDetectorNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()

        input_image = self.declare_parameter('input_image',
                                             '/camera/image_raw').value
        self.method = self.declare_parameter('method', 'classic').value

        # classic は常に生成（yolo の bbox 内色判定にも流用するため）。
        classic = ClassicDetector(self)

        # バックエンド選択。yolo 失敗時は classic にフォールバック（起動を止めない）。
        self.backend = None
        if self.method == 'yolo':
            try:
                self.backend = YoloDetector(self, classic)
                self.get_logger().info('backend=yolo (検出=YOLO, 色=bbox内HSV)')
            except RuntimeError as e:
                self.get_logger().warn(
                    'yolo 初期化失敗 → classic にフォールバック: %s' % e)
                self.method = 'classic'
        if self.backend is None:
            self.backend = classic
            self.get_logger().info('backend=classic')

        self.pub_signals = self.create_publisher(
            TrafficSignalArray, '/perception/traffic_signals', 10)
        self.pub_rois = self.create_publisher(
            Detection2DArray, '/perception/traffic_light/rois', 10)
        self.create_subscription(
            Image, input_image, self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            'traffic_light_detector started. input=%s method=%s '
            '-> /perception/traffic_signals, /perception/traffic_light/rois'
            % (input_image, self.method))

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge 変換失敗: %s' % e)
            return

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
            sig.traffic_signal_id = i
            sig.elements = [elem]
            sig_arr.signals.append(sig)

            # ROI（可視化/デバッグ用）。
            x, y, w, h = det['bbox']
            d2 = Detection2D()
            d2.header = msg.header
            d2.bbox = BoundingBox2D()
            d2.bbox.center.position.x = float(x + w / 2.0)
            d2.bbox.center.position.y = float(y + h / 2.0)
            d2.bbox.size_x = float(w)
            d2.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
