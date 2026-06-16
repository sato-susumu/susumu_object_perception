#!/usr/bin/env python3
"""perception パイプラインの DetectedObjects / TrackedObjects を RViz 用 MarkerArray に
変換する自作可視化ノード。

Autoware 純正の autoware_perception_rviz_plugin もあるが、表示方法・色を自由に
作り込みたいので、標準の visualization_msgs/MarkerArray で軽量に可視化する。

  /perception/detected_objects_in_map (DetectedObjects) → 青の枠（その瞬間の検出クラスタ）
  /perception/tracked_objects         (TrackedObjects)  → 移動=赤 / 静止=緑 の枠
                                                   + ラベル/速度テキスト + 速度ベクトル矢印
  /perception/predicted_objects       (PredictedObjects)→ 黄の予測パス（LINE_STRIP）

色の意味:
  青      : 検出（まだ追跡 ID なし）
  マゼンタ : 追跡中かつ PEDESTRIAN（2D 地図の free space で移動 → 歩行者と推定）
  赤      : 追跡中かつ移動物体だが未分類（UNKNOWN）
  緑      : 追跡中だが静止（壁・什器）

テキスト（`#<追跡ID>  <速度>[km/h]`）は、spencer_tracking_rviz_plugin /
leg_tracker など Nav2 系の人追跡プラグインの作法に合わせる:
  - 背景パネルは付けない（RViz の TEXT_VIEW_FACING は背景なしが標準）
  - 字高は控えめ（leg_tracker 0.1 / spencer は font_scale 係数）。本実装は 0.22
  - 文字色は白一色にせず、ボックスと同系統の暗めの識別色（移動=暗赤 / 静止=暗緑）。
    白い地図上でも読めるよう、ボックスより暗くする
  - ラベル名（UNKNOWN 等）は出さず、追跡 ID と速度のみ
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from autoware_perception_msgs.msg import (
    DetectedObjects, TrackedObjects, PredictedObjects, ObjectClassification)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


PEDESTRIAN_LABEL = ObjectClassification.PEDESTRIAN  # 2D 地図で推定した歩行者の色分け用


def color(r, g, b, a=0.8):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def uuid_to_int(uuid_bytes):
    """UUID 先頭 4 バイトを int に戻す（トラッカーが ID をそこに格納している）。"""
    return int.from_bytes(bytes(uuid_bytes[0:4]), 'little')


class PerceptionMarkerNode(Node):

    def __init__(self):
        super().__init__('perception_marker')

        self.declare_parameter('detected_topic', '/perception/detected_objects_in_map')
        self.declare_parameter('tracked_topic', '/perception/tracked_objects')
        self.declare_parameter('predicted_topic', '/perception/predicted_objects')
        self.declare_parameter('marker_topic', '/perception/markers')
        # lifetime はマーカーが自動消滅するまでの秒数。入力(tracked)の publish 間隔が
        # 高負荷で 1Hz 程度(0.7〜1.0s)まで落ちることがあり、lifetime がそれより短いと
        # 次のマーカーが来る前に寿命切れして点滅・消失する。余裕を持って 1.0s にする。
        # （消えた物体は tracker が落とすと markers から外れ、lifetime 切れで消える）
        self.declare_parameter('marker_lifetime_sec', 1.0)

        self.lifetime = float(self.get_parameter('marker_lifetime_sec').value)

        # 各入力の最新メッセージだけ保持し、タイマーで「全 ns を含む完全な MarkerArray」を
        # まとめて publish する。検出/追跡/予測のコールバックが別々に部分的な MarkerArray を
        # publish すると、tracked_id を含まない更新が混ざってラベル/ボックスが点滅・消失する
        # （RViz の表示タイミングと lifetime の相互作用）。一括 publish で必ず全マーカーを
        # 揃えて出すことで点滅を根治する。
        self._latest_detected = None
        self._latest_tracked = None
        self._latest_predicted = None

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, qos)
        self.create_subscription(
            DetectedObjects, self.get_parameter('detected_topic').value,
            self.on_detected, qos)
        self.create_subscription(
            TrackedObjects, self.get_parameter('tracked_topic').value,
            self.on_tracked, qos)
        self.create_subscription(
            PredictedObjects, self.get_parameter('predicted_topic').value,
            self.on_predicted, qos)

        # 15Hz で全マーカーを一括 publish（入力レートより速く、lifetime より十分密）
        self.create_timer(1.0 / 15.0, self.publish_all)

        self.get_logger().info('perception_marker started.')

    def _lifetime_msg(self):
        d = rclpy.duration.Duration(seconds=self.lifetime).to_msg()
        return d

    def _box(self, ns, mid, header, pose, dims, col):
        m = Marker()
        m.header = header
        m.ns = ns
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pose
        # dimensions が 0 だと RViz に出ないので最小値を入れる。
        m.scale.x = max(0.05, dims.x)
        m.scale.y = max(0.05, dims.y)
        m.scale.z = max(0.05, dims.z)
        m.color = col
        m.lifetime = self._lifetime_msg()
        return m

    def on_detected(self, msg: DetectedObjects):
        self._latest_detected = msg

    def on_tracked(self, msg: TrackedObjects):
        self._latest_tracked = msg

    def on_predicted(self, msg: PredictedObjects):
        self._latest_predicted = msg

    def publish_all(self):
        """保持している最新の検出/追跡/予測を、全 ns を含む 1 つの MarkerArray にして
        まとめて publish する（点滅防止のため必ず全マーカーを揃えて出す）。"""
        arr = MarkerArray()
        if self._latest_detected is not None:
            self._build_detected(self._latest_detected, arr)
        if self._latest_tracked is not None:
            self._build_tracked(self._latest_tracked, arr)
        if self._latest_predicted is not None:
            self._build_predicted(self._latest_predicted, arr)
        if arr.markers:
            self.pub.publish(arr)

    def _build_detected(self, msg: DetectedObjects, arr: MarkerArray):
        for i, obj in enumerate(msg.objects):
            pose = obj.kinematics.pose_with_covariance.pose
            arr.markers.append(self._box(
                'detected', i, msg.header, pose, obj.shape.dimensions,
                color(0.1, 0.4, 1.0, 0.4)))  # 青・半透明

    def _build_tracked(self, msg: TrackedObjects, arr: MarkerArray):
        for obj in msg.objects:
            tid = uuid_to_int(obj.object_id.uuid)
            pose = obj.kinematics.pose_with_covariance.pose
            moving = not obj.kinematics.is_stationary
            label = obj.classification[0].label if obj.classification else 0
            # 色は classification ベース（2D 地図で推定した PEDESTRIAN を区別）。
            # テキストはボックスと同系統だが暗めにして白い地図上でも読めるようにする。
            # 「色＝識別色」は spencer/leg_tracker など Nav2 系プラグインの定番作法。
            if label == PEDESTRIAN_LABEL:
                box_col = color(1.0, 0.2, 0.9, 0.6)   # マゼンタ（人）
                text_col = color(0.7, 0.0, 0.6, 1.0)
            elif moving:
                box_col = color(1.0, 0.2, 0.2, 0.6)   # 赤（移動・未分類）
                text_col = color(0.75, 0.0, 0.0, 1.0)
            else:
                box_col = color(0.2, 0.9, 0.2, 0.5)   # 緑（静止・什器）
                text_col = color(0.0, 0.5, 0.0, 1.0)

            arr.markers.append(self._box(
                'tracked', tid, msg.header, pose, obj.shape.dimensions, box_col))

            # トラッキング ID / 速度テキスト（物体の上に表示）。
            # 文字列は `#<ID>  <速度>[km/h]`（ラベル名は表示しない）。サイズ・色は
            # spencer / leg_tracker の作法に合わせ、控えめな字高＋トラック識別色
            # （背景パネルは付けない）。
            vx = obj.kinematics.twist_with_covariance.twist.linear.x
            vy = obj.kinematics.twist_with_covariance.twist.linear.y
            speed_kmph = math.hypot(vx, vy) * 3.6
            txt = Marker()
            txt.header = msg.header
            txt.ns = 'tracked_id'
            txt.id = tid
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = pose.position.x
            txt.pose.position.y = pose.position.y
            txt.pose.position.z = pose.position.z + max(0.5, obj.shape.dimensions.z) + 0.25
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.22  # 字高[m]。leg_tracker(0.1)より読みやすく、過大にしない
            txt.color = text_col
            txt.text = f'#{tid}  {speed_kmph:.0f}[km/h]'
            txt.lifetime = self._lifetime_msg()
            arr.markers.append(txt)

            # 速度ベクトル矢印（移動物体のみ）
            if moving and math.hypot(vx, vy) > 1e-2:
                arrow = Marker()
                arrow.header = msg.header
                arrow.ns = 'tracked_vel'
                arrow.id = tid
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                start = Point(x=pose.position.x, y=pose.position.y, z=0.1)
                end = Point(x=pose.position.x + vx, y=pose.position.y + vy, z=0.1)
                arrow.points = [start, end]
                arrow.scale.x = 0.05  # 軸径
                arrow.scale.y = 0.12  # 矢じり径
                arrow.scale.z = 0.0
                arrow.color = color(1.0, 1.0, 0.0, 0.9)  # 黄
                arrow.lifetime = self._lifetime_msg()
                arr.markers.append(arrow)

    def _build_predicted(self, msg: PredictedObjects, arr: MarkerArray):
        """予測パス（PredictedObjects.predicted_paths）を LINE_STRIP で表示。"""
        for obj in msg.objects:
            tid = uuid_to_int(obj.object_id.uuid)
            for j, pred in enumerate(obj.kinematics.predicted_paths):
                if len(pred.path) < 2:
                    continue
                line = Marker()
                line.header = msg.header
                line.ns = 'predicted_path'
                line.id = tid * 8 + j  # トラック毎に複数パス（マルチモーダル）を区別
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.04  # 線幅[m]
                # confidence が高いほど不透明な黄。予測なので半透明寄り。
                a = max(0.25, min(0.9, pred.confidence))
                line.color = color(1.0, 0.8, 0.0, a)  # 黄〜オレンジ
                line.pose.orientation.w = 1.0
                for pose in pred.path:
                    line.points.append(Point(
                        x=pose.position.x, y=pose.position.y, z=0.05))
                line.lifetime = self._lifetime_msg()
                arr.markers.append(line)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionMarkerNode()
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
