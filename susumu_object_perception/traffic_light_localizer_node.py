#!/usr/bin/env python3
# 信号の 3D 位置推定ノード。traffic_light_detector_node が出す検出（方向ベクトル付き ROI）と
# 3D LiDAR 点群を組み合わせ、各信号の 3D 位置（ロボット座標）を推定する。
#
# 全天球カメラの検出は「方位・仰角（方向ベクトル）」しか持たず、その方向のどの距離に信号が
# あるかは画像だけでは決まらない。そこで LiDAR 点群から、検出方向に近く・信号機の高さ帯に
# ある点を集め、最近傍の距離クラスタの重心を信号の 3D 位置とする。
#
# 入力 : /perception/traffic_light/rois  (vision_msgs/Detection2DArray)
#          results[0].pose.pose.position = 検出方向の単位ベクトル（ロボット/カメラ座標）
#          results[0].hypothesis.class_id = 'color@yawdeg(vN)'
#        /lidar/points または /lidar/points/point_cloud (sensor_msgs/PointCloud2)
# 出力 : /perception/traffic_light/poses   (geometry_msgs/PoseArray, frame=lidar フレーム)
#        /perception/traffic_light/markers (visualization_msgs/MarkerArray, RViz 表示)
#
# 注意: 検出方向はカメラ（omni_camera_link）基準、点群は lidar_link 基準で、両者は同じロボット
# 前方(+X)・上(+Z)向きの軸だが原点（取り付け高さ）が違う。方向は単位ベクトルなので向きの差は
# 小さく、ここでは方向ベクトルをそのまま lidar フレームの方向として扱う（取り付けは同軸・
# 平行）。厳密化が要るなら camera->lidar の回転を TF で適用する。

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2


# 色名 → マーカー色(RGBA)。
MARKER_RGBA = {
    'red': (1.0, 0.0, 0.0, 0.9),
    'amber': (1.0, 0.75, 0.0, 0.9),
    'green': (0.0, 1.0, 0.0, 0.9),
}


def parse_color(class_id):
    """'red@90deg(v2)' → 'red'。'red' → 'red'。"""
    return class_id.split('@', 1)[0] if '@' in class_id else class_id


class TrafficLightLocalizerNode(Node):
    def __init__(self):
        super().__init__('traffic_light_localizer')

        points_topic = self.declare_parameter(
            'points_topic', '/lidar/points').value
        # 検出方向に対する許容角（円錐の半頂角）[deg]。
        self.angle_tol = math.radians(
            float(self.declare_parameter('angle_tol_deg', 8.0).value))
        # 検出方向の仰角(pitch)も使うか。false なら方位(yaw)だけで絞り、縦は高さ帯フィルタに任せる。
        # 全天球カメラが信号を急角度で見上げる配置では pitch が灯火の真方向とずれるため既定 false。
        self.use_direction_pitch = bool(
            self.declare_parameter('use_direction_pitch', False).value)
        # 信号機構造物の高さ帯 [m]（lidar フレーム z）。地面・低い物体を除外する。
        self.min_height = float(self.declare_parameter('min_height', 0.7).value)
        self.max_height = float(self.declare_parameter('max_height', 6.0).value)
        # 距離クラスタの厚み [m]。最近傍点から この範囲内を 1 つの信号塊とみなす。
        self.cluster_depth = float(
            self.declare_parameter('cluster_depth', 1.0).value)
        # 有効とみなす最小点数（少なすぎる方向は雑音として捨てる）。
        self.min_points = int(self.declare_parameter('min_points', 4).value)
        # 距離レンジ [m]。
        self.min_range = float(self.declare_parameter('min_range', 1.0).value)
        self.max_range = float(self.declare_parameter('max_range', 40.0).value)

        self.latest_cloud = None
        self.cloud_frame = 'lidar_link'

        self.pub_poses = self.create_publisher(
            PoseArray, '/perception/traffic_light/poses', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/traffic_light/markers', 10)
        self.create_subscription(
            PointCloud2, points_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, '/perception/traffic_light/rois',
            self.on_rois, 10)

        self.get_logger().info(
            'traffic_light_localizer started. points=%s rois=/perception/traffic_light/rois'
            ' -> /perception/traffic_light/poses, /perception/traffic_light/markers'
            % points_topic)

    def on_cloud(self, msg: PointCloud2):
        pts = [(float(p[0]), float(p[1]), float(p[2]))
               for p in pc2.read_points(
                   msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        self.latest_cloud = np.array(pts, dtype=float) if pts else None
        self.cloud_frame = msg.header.frame_id or 'lidar_link'

    def _localize(self, direction):
        """方向単位ベクトルに対し、その方向に近く高さ帯にある最近傍クラスタの重心を返す。

        見つからなければ None。返り値は (x,y,z) (lidar フレーム)。
        """
        pts = self.latest_cloud
        if pts is None or len(pts) == 0:
            return None
        d = np.array(direction, dtype=float)
        nd = np.linalg.norm(d)
        if nd < 1e-6:
            return None
        d = d / nd

        rng = np.linalg.norm(pts, axis=1)
        valid = (rng > self.min_range) & (rng < self.max_range) & \
                (pts[:, 2] > self.min_height) & (pts[:, 2] < self.max_height)
        cand = pts[valid]
        crng = rng[valid]
        if len(cand) == 0:
            return None

        # 検出方向に対する角度フィルタ。方位(yaw)は信頼できるが、全天球カメラが信号を急角度で
        # 見上げる配置だと仰角(pitch)は灯火の真の方向とずれることがある。use_direction_pitch
        # が false のときは方位成分だけで円錐を張り、高さ帯フィルタで縦を絞る。
        if self.use_direction_pitch:
            unit = cand / np.maximum(crng[:, None], 1e-6)
            cos = unit @ d
            within = cos > math.cos(self.angle_tol)
        else:
            d_az = math.atan2(d[1], d[0])
            cand_az = np.arctan2(cand[:, 1], cand[:, 0])
            daz = np.abs(np.arctan2(np.sin(cand_az - d_az),
                                    np.cos(cand_az - d_az)))
            within = daz < self.angle_tol
        sel = cand[within]
        srng = crng[within]
        if len(sel) < self.min_points:
            return None

        # 最近傍の距離まとまり（最近点から cluster_depth 以内）の重心を信号位置にする。
        # 信号機は背景（建物・壁）より手前にあるのが普通なので、最近傍まとまりを採る。
        # 背景の方が点が多くても引っ張られないよう、最頻ビンではなく最近傍を使う。
        near = sel[srng < srng.min() + self.cluster_depth]
        if len(near) < self.min_points:
            return None
        return tuple(near.mean(axis=0))

    def on_rois(self, msg: Detection2DArray):
        if self.latest_cloud is None:
            return

        poses = PoseArray()
        poses.header.stamp = msg.header.stamp
        poses.header.frame_id = self.cloud_frame
        markers = MarkerArray()

        # 既存マーカーを消す DELETEALL を先頭に。
        clear = Marker()
        clear.header.frame_id = self.cloud_frame
        clear.header.stamp = msg.header.stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        mid = 0
        for det in msg.detections:
            if not det.results:
                continue
            res = det.results[0]
            color = parse_color(res.hypothesis.class_id)
            p = res.pose.pose.position
            direction = (p.x, p.y, p.z)
            if abs(p.x) + abs(p.y) + abs(p.z) < 1e-6:
                continue  # 方向ベクトル無し（通常モード等）→ 3D 化しない

            pos = self._localize(direction)
            if pos is None:
                continue

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            pose.orientation.w = 1.0
            poses.poses.append(pose)

            # 球マーカー（色＝信号色）。
            m = Marker()
            m.header.frame_id = self.cloud_frame
            m.header.stamp = msg.header.stamp
            m.ns = 'traffic_light'
            m.id = mid
            mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = m.scale.y = m.scale.z = 0.4
            r, g, b, a = MARKER_RGBA.get(color, (1.0, 1.0, 1.0, 0.9))
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
            markers.markers.append(m)

            # テキスト（色 + 距離）。
            t = Marker()
            t.header.frame_id = self.cloud_frame
            t.header.stamp = msg.header.stamp
            t.ns = 'traffic_light_text'
            t.id = mid
            mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose = Pose()
            t.pose.position.x = pos[0]
            t.pose.position.y = pos[1]
            t.pose.position.z = pos[2] + 0.4
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            dist = math.hypot(pos[0], pos[1])
            t.text = '%s %.1fm' % (color, dist)
            markers.markers.append(t)

        self.pub_poses.publish(poses)
        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = TrafficLightLocalizerNode()
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
