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
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformException, TransformListener

import cv2

from susumu_object_perception.colorized_pointcloud_node import (
    WEBOTS_CYLINDRICAL_ROT, euler_xyz_to_matrix, quat_to_matrix)


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


class YoloClassifier:
    """YOLOv8(ultralytics) でクロップ画像を物体検出し、最大信頼度のクラスを返す。

    初期化（ultralytics/torch import・重み読込）に失敗したら RuntimeError を投げる。
    ノード側はそれを [FATAL] にして終了する（無分類での素通しはしない）。
    """

    def __init__(self, weights, conf):
        self.conf = conf
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
        res = self.model.predict(bgr, conf=self.conf, verbose=False)
        best_name, best_conf = None, 0.0
        for r in res:
            names = r.names
            for b in r.boxes:
                c = float(b.conf)
                if c > best_conf:
                    best_conf = c
                    best_name = str(names[int(b.cls)]).lower()
        return best_name, best_conf


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
        self.max_objects = int(self.declare_parameter('max_objects', 8).value)
        self.projection_model = self.declare_parameter(
            'projection_model', 'webots_cylindrical').value
        rpy = [float(v) for v in self.declare_parameter(
            'calibration_rpy_deg', [0.0, 0.0, 0.0]).value]
        self.calibration_rot = euler_xyz_to_matrix(
            math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2]))
        weights = self.declare_parameter('yolo.weights', 'yolov8n.pt').value
        conf = float(self.declare_parameter('yolo.conf', 0.25).value)
        # 信頼度がこれ未満の分類は採用せず、元の classification を保つ。
        self.min_accept_conf = float(
            self.declare_parameter('min_accept_conf', 0.3).value)

        # === 間引き ===
        # (1) 処理レート上限 [Hz]。tracked_objects は ~10Hz で来るが、物体の種類は急に
        #     変わらないので分類は低頻度でよい。0 以下で無効。
        self.max_rate_hz = float(self.declare_parameter('max_rate_hz', 2.0).value)
        self._min_interval = (1.0 / self.max_rate_hz) if self.max_rate_hz > 0 else 0.0
        self._last_proc_t = None
        # (2) トラック ID キャッシュ。一度分類できたトラックは再分類しない（最も効く間引き）。
        #     再分類するのは未分類(unknown のまま)のトラックと新規トラックだけ。キャッシュは
        #     cache_ttl 秒で失効（物体が入れ替わって ID が再利用された場合に備える）。古い
        #     エントリは消えたトラックぶんを掃除する。
        self.cache_ttl = float(self.declare_parameter('cache_ttl_sec', 10.0).value)
        # uuid(bytes) -> (label, coco_name, conf, last_seen_t)
        self._class_cache = {}

        # 分類器初期化。失敗したら無分類で素通しせず FATAL で終了（自動フォールバック禁止）。
        try:
            self.classifier = YoloClassifier(weights, conf)
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

        self.create_subscription(
            Image, self.input_image, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            TrackedObjects, self.input_objects, self.on_objects, 10)

        self.get_logger().info(
            'object_classifier started. objects=%s image=%s '
            '-> /perception/tracked_objects_classified, '
            '/perception/object_classes/markers' % (
                self.input_objects, self.input_image))

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

    def _perspective_crop(self, pano, direction):
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

        xs = np.linspace(-1.0, 1.0, self.crop_w, dtype=np.float32)
        ys = np.linspace(-1.0, 1.0, self.crop_h, dtype=np.float32)
        xx, yy = np.meshgrid(xs, ys)
        tan_half = math.tan(self.crop_fov / 2.0)
        aspect = self.crop_w / float(self.crop_h)
        dirs = (forward.reshape(1, 1, 3) +
                right.reshape(1, 1, 3) * (xx[..., None] * tan_half * aspect) +
                up.reshape(1, 1, 3) * (-yy[..., None] * tan_half))
        dirs /= np.maximum(np.linalg.norm(dirs, axis=2, keepdims=True), 1e-6)

        h, w = pano.shape[:2]
        if self.projection_model == 'webots_cylindrical':
            dp = dirs @ WEBOTS_CYLINDRICAL_ROT.T
            yaw = np.arctan2(dp[:, :, 1], dp[:, :, 0])
            z_unit = np.clip(dp[:, :, 2], -1.0, 1.0)
            v_angle = np.arccos(z_unit) - math.pi / 2.0
            map_x = ((0.5 - yaw / (2.0 * math.pi)) * w % w).astype(np.float32)
            map_y = ((0.5 + v_angle / math.pi) * h).astype(np.float32)
        else:
            yaw = np.arctan2(-dirs[:, :, 1], dirs[:, :, 0])
            pitch = np.arcsin(np.clip(dirs[:, :, 2], -1.0, 1.0))
            map_x = (((yaw + math.pi) / (2.0 * math.pi) * w) % w).astype(np.float32)
            map_y = ((math.pi / 2.0 + pitch) / math.pi * h).astype(np.float32)
        return cv2.remap(pano, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_WRAP)

    def on_objects(self, msg: TrackedObjects):
        if self.latest_image is None or not msg.objects:
            # 画像が無い間は素通し（分類できないが物体情報は失わない）。
            self.pub_objects.publish(msg)
            return
        pano, _ = self.latest_image

        # 間引き(1): 処理レート上限。前回の YOLO 推論から _min_interval 秒経っていなければ、
        # 新規推論はせず「キャッシュ済みの分類だけ当てて素通し」する（物体情報は失わない）。
        now = time.monotonic()
        run_inference = True
        if self._min_interval > 0.0 and self._last_proc_t is not None and \
                (now - self._last_proc_t) < self._min_interval:
            run_inference = False

        # 古いキャッシュエントリ(cache_ttl 秒見ていないトラック)を掃除。
        for k in [k for k, v in self._class_cache.items()
                  if now - v[3] > self.cache_ttl]:
            del self._class_cache[k]

        out = TrackedObjects()
        out.header = msg.header
        markers = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        annot_tiles = []
        mid = 0

        for i, obj in enumerate(msg.objects):
            new_obj = obj  # 既存物体をそのまま使い classification だけ差し替える
            if i < self.max_objects:
                pose = obj.kinematics.pose_with_covariance.pose
                tid = bytes(obj.object_id.uuid)
                cached = self._class_cache.get(tid)

                coco_name, conf = (None, 0.0)
                crop = None
                # 間引き(2): 既に分類できているトラックは再分類しない（最も効く）。
                # 未分類(キャッシュ無し or unknown)のトラックだけ、かつレート上限内のときだけ推論。
                need_infer = (cached is None or cached[0] == ObjectClassification.UNKNOWN)
                if need_infer and run_inference:
                    p_cam = self._transform_point(pose.position, msg.header.frame_id)
                    crop = self._perspective_crop(pano, p_cam) \
                        if p_cam is not None else None
                    if crop is not None and crop.size > 0:
                        coco_name, conf = self.classifier.classify(crop)
                        self._last_proc_t = now  # 実際に推論したら時刻更新
                        if coco_name is not None and conf >= self.min_accept_conf:
                            lbl = COCO_TO_AUTOWARE.get(
                                coco_name, ObjectClassification.UNKNOWN)
                            self._class_cache[tid] = (lbl, coco_name, conf, now)
                        else:
                            self._class_cache[tid] = (
                                ObjectClassification.UNKNOWN, None, 0.0, now)
                elif cached is not None:
                    # キャッシュ命中: 再分類せず last_seen だけ更新。
                    self._class_cache[tid] = (cached[0], cached[1], cached[2], now)
                    if cached[0] != ObjectClassification.UNKNOWN:
                        coco_name, conf = cached[1], cached[2]

                if coco_name is not None and conf >= self.min_accept_conf:
                    label = COCO_TO_AUTOWARE.get(
                        coco_name, ObjectClassification.UNKNOWN)
                    cls = ObjectClassification()
                    cls.label = label
                    cls.probability = float(conf)
                    new_obj.classification = [cls]
                    text = '%s %.2f' % (coco_name, conf)
                    color = (0.1, 0.9, 0.2)
                else:
                    text = 'unknown'
                    color = (0.6, 0.6, 0.6)

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
                if crop is not None and crop.size > 0:
                    tile = crop.copy()
                    cv2.putText(tile, text, (6, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 2, cv2.LINE_AA)
                    annot_tiles.append(tile)

            out.objects.append(new_obj)

        self.pub_objects.publish(out)
        self.pub_markers.publish(markers)
        if annot_tiles:
            mosaic = np.hstack(annot_tiles)
            am = self.bridge.cv2_to_imgmsg(mosaic, encoding='bgr8')
            am.header = msg.header
            self.pub_annot.publish(am)


def main():
    rclpy.init()
    node = ObjectClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
