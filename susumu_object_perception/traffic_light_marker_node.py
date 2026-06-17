#!/usr/bin/env python3
# 交通信号認識の可視化ノード。traffic_light_detector_node の検出結果（画像内 bbox）を
# カメラ画像に重畳した注釈画像を出す。信号は地図無しでは 3D 位置を持たない（画像内 ROI のみ）
# ため、LiDAR perception の MarkerArray とは別に、画像注釈で可視化するのが素直
# （RViz の Image Display で見られる）。
#
# 入力 : /camera/image_raw（パラメータ input_image）   sensor_msgs/Image
#        /perception/traffic_light/rois                vision_msgs/Detection2DArray
# 出力 : /perception/traffic_light/image_annotated      sensor_msgs/Image
#        （bbox + 色ラベル + 信頼度を重畳）
#
# 検出器(detector)と本ノードは別プロセスなので、最新の画像と最新の rois を保持して
# 画像更新ごとに描画する（厳密な時刻同期はしない＝軽量・実用優先）。

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


class TrafficLightMarkerNode(Node):
    def __init__(self):
        super().__init__('traffic_light_marker')
        self.bridge = CvBridge()
        input_image = self.declare_parameter('input_image',
                                             '/camera/image_raw').value

        self.latest_rois = None
        self.pub = self.create_publisher(
            Image, '/perception/traffic_light/image_annotated', 10)
        self.create_subscription(
            Detection2DArray, '/perception/traffic_light/rois',
            self.on_rois, 10)
        self.create_subscription(
            Image, input_image, self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            'traffic_light_marker started. input=%s, rois=/perception/traffic_light/rois'
            ' -> /perception/traffic_light/image_annotated' % input_image)

    def on_rois(self, msg: Detection2DArray):
        self.latest_rois = msg

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('cv_bridge 変換失敗: %s' % e)
            return

        rois = self.latest_rois
        if rois is not None:
            for det in rois.detections:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                w = det.bbox.size_x
                h = det.bbox.size_y
                x1 = int(cx - w / 2.0)
                y1 = int(cy - h / 2.0)
                x2 = int(cx + w / 2.0)
                y2 = int(cy + h / 2.0)
                label = 'tl'
                score = 0.0
                if det.results:
                    label = det.results[0].hypothesis.class_id
                    score = det.results[0].hypothesis.score
                col = DRAW_COLOR.get(label, (255, 255, 255))
                cv2.rectangle(bgr, (x1, y1), (x2, y2), col, 2)
                cv2.putText(bgr, '%s %.2f' % (label, score), (x1, max(0, y1 - 6)),
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
