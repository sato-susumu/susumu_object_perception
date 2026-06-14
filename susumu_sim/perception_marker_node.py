#!/usr/bin/env python3
"""perception パイプラインの DetectedObjects / TrackedObjects を RViz 用 MarkerArray に
変換する自作可視化ノード。

Autoware には autoware_perception_rviz_plugin という専用プラグインがあるが、RViz の
設定が重いのと、検出（瞬間）と追跡（ID・速度付き）を色分けして直感的に見せたいので、
標準の visualization_msgs/MarkerArray で軽量に可視化する。

  /perception/detected_objects (DetectedObjects) → 青の枠（その瞬間の検出クラスタ）
  /perception/tracked_objects  (TrackedObjects)  → 移動=赤 / 静止=緑 の枠
                                                   + ID テキスト + 速度ベクトル矢印

色の意味:
  青  : 検出（まだ追跡 ID なし）
  赤  : 追跡中かつ移動物体（人など）
  緑  : 追跡中だが静止（壁・什器）
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def color(r, g, b, a=0.8):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def uuid_to_int(uuid_bytes):
    """UUID 先頭 4 バイトを int に戻す（トラッカーが ID をそこに格納している）。"""
    return int.from_bytes(bytes(uuid_bytes[0:4]), 'little')


class PerceptionMarkerNode(Node):

    def __init__(self):
        super().__init__('perception_marker')

        self.declare_parameter('detected_topic', '/perception/detected_objects')
        self.declare_parameter('tracked_topic', '/perception/tracked_objects')
        self.declare_parameter('marker_topic', '/perception/markers')
        self.declare_parameter('marker_lifetime_sec', 0.3)

        self.lifetime = float(self.get_parameter('marker_lifetime_sec').value)

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, qos)
        self.create_subscription(
            DetectedObjects, self.get_parameter('detected_topic').value,
            self.on_detected, qos)
        self.create_subscription(
            TrackedObjects, self.get_parameter('tracked_topic').value,
            self.on_tracked, qos)

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
        arr = MarkerArray()
        for i, obj in enumerate(msg.objects):
            pose = obj.kinematics.pose_with_covariance.pose
            arr.markers.append(self._box(
                'detected', i, msg.header, pose, obj.shape.dimensions,
                color(0.1, 0.4, 1.0, 0.4)))  # 青・半透明
        self.pub.publish(arr)

    def on_tracked(self, msg: TrackedObjects):
        arr = MarkerArray()
        for obj in msg.objects:
            tid = uuid_to_int(obj.object_id.uuid)
            pose = obj.kinematics.pose_with_covariance.pose
            moving = not obj.kinematics.is_stationary
            col = color(1.0, 0.1, 0.1, 0.6) if moving else color(0.1, 0.9, 0.1, 0.5)

            arr.markers.append(self._box(
                'tracked', tid, msg.header, pose, obj.shape.dimensions, col))

            # ID テキスト（物体の上に表示）
            txt = Marker()
            txt.header = msg.header
            txt.ns = 'tracked_id'
            txt.id = tid
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = pose.position.x
            txt.pose.position.y = pose.position.y
            txt.pose.position.z = pose.position.z + max(0.5, obj.shape.dimensions.z) + 0.2
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.3
            txt.color = color(1.0, 1.0, 1.0, 1.0)
            vx = obj.kinematics.twist_with_covariance.twist.linear.x
            vy = obj.kinematics.twist_with_covariance.twist.linear.y
            speed = math.hypot(vx, vy)
            txt.text = f'#{tid} {speed:.2f}m/s'
            txt.lifetime = self._lifetime_msg()
            arr.markers.append(txt)

            # 速度ベクトル矢印（移動物体のみ）
            if moving and speed > 1e-2:
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

        self.pub.publish(arr)


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
