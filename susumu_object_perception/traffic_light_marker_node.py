#!/usr/bin/env python3
# 交通信号認識の可視化ノード。traffic_light_detector_node の検出結果（画像内 bbox）を
# カメラ画像に重畳した注釈画像を出す。信号は地図無しでは 3D 位置を持たない（画像内 ROI のみ）
# ため、LiDAR perception の MarkerArray とは別に、画像注釈で可視化するのが素直
# （RViz の Image Display で見られる）。
#
# 入力 : 全天球 /omni_camera/image_raw/image_color（既定）or 通常 /camera/image_raw
#          （パラメータ input_image）                 sensor_msgs/Image
#        /perception/traffic_light/rois                vision_msgs/Detection2DArray
# 出力 : /perception/traffic_light/image_annotated      sensor_msgs/Image
#        （bbox + 色ラベル + 信頼度を重畳）
#
# 全天球モード（omni_mode:=true、既定）では detector の bbox は「透視ビュー内座標」なので
# 全天球画像にそのまま重ねると位置がずれる。class_id に載った方位(yaw_deg)から全天球画像上の
# おおよその x 位置（縦帯）を描き、色ラベルと方位を重畳する（正確な逆投影はせず軽量・実用優先）。
#
# 検出器(detector)と本ノードは別プロセスなので、最新の画像と最新の rois を保持して
# 画像更新ごとに描画する（厳密な時刻同期はしない＝軽量・実用優先）。

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

import cv2
from cv_bridge import CvBridge


# 色名 → 描画色（BGR）。
DRAW_COLOR = {
    'red': (0, 0, 255),
    'amber': (0, 200, 255),
    'green': (0, 255, 0),
}


def parse_class_id(class_id):
    """class_id を (color, yaw_deg or None) に分解する。
    全天球: 'red@90deg(v2)' → ('red', 90)。通常: 'red' → ('red', None)。
    """
    if '@' in class_id:
        color = class_id.split('@', 1)[0]
        yaw = None
        try:
            yaw = int(class_id.split('@', 1)[1].split('deg', 1)[0])
        except (ValueError, IndexError):
            yaw = None
        return color, yaw
    return class_id, None


class TrafficLightMarkerNode(Node):
    def __init__(self):
        super().__init__('traffic_light_marker')
        self.bridge = CvBridge()
        self.omni_mode = bool(self.declare_parameter('omni_mode', True).value)
        default_input = ('/omni_camera/image_raw/image_color'
                         if self.omni_mode else '/camera/image_raw')
        input_image = self.declare_parameter('input_image', default_input).value

        self.latest_rois = None
        self.pub = self.create_publisher(
            Image, '/perception/traffic_light/image_annotated', 10)
        self.create_subscription(
            Detection2DArray, '/perception/traffic_light/rois',
            self.on_rois, 10)
        self.create_subscription(
            Image, input_image, self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            'traffic_light_marker started. input=%s mode=%s '
            'rois=/perception/traffic_light/rois '
            '-> /perception/traffic_light/image_annotated'
            % (input_image, 'omni' if self.omni_mode else 'pinhole'))

    def on_rois(self, msg: Detection2DArray):
        self.latest_rois = msg

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge 変換失敗: %s' % e)
            return

        rois = self.latest_rois
        img_h, img_w = bgr.shape[:2]
        if rois is not None:
            for det in rois.detections:
                label = 'tl'
                score = 0.0
                if det.results:
                    label = det.results[0].hypothesis.class_id
                    score = det.results[0].hypothesis.score
                color, yaw_deg = parse_class_id(label)
                col = DRAW_COLOR.get(color, (255, 255, 255))

                if self.omni_mode and yaw_deg is not None:
                    # 方位 yaw_deg を全天球画像上の x（縦帯）に概略マッピングする。
                    # colorize と同じ Webots cylindrical: x = (0.5 - yaw/2π) * width。
                    yaw = math.radians(yaw_deg)
                    cx = int(((0.5 - yaw / (2.0 * math.pi)) % 1.0) * img_w)
                    cv2.line(bgr, (cx, 0), (cx, img_h), col, 2, cv2.LINE_AA)
                    cv2.putText(
                        bgr, '%s %ddeg %.2f' % (color, yaw_deg, score),
                        (max(0, cx - 60), 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
                else:
                    # 通常モード（または方位不明）: 従来どおり bbox を重畳。
                    cx = det.bbox.center.position.x
                    cy = det.bbox.center.position.y
                    w = det.bbox.size_x
                    h = det.bbox.size_y
                    x1 = int(cx - w / 2.0)
                    y1 = int(cy - h / 2.0)
                    x2 = int(cx + w / 2.0)
                    y2 = int(cy + h / 2.0)
                    cv2.rectangle(bgr, (x1, y1), (x2, y2), col, 2)
                    cv2.putText(bgr, '%s %.2f' % (color, score),
                                (x1, max(0, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)

        out = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TrafficLightMarkerNode()
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
