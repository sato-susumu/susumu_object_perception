#!/usr/bin/env python3
"""追跡結果を使って検出（クラスタ）の過分割を統合する自作ノード（detection_by_tracker）。

Autoware の `autoware_detection_by_tracker` の **過分割統合（Cluster Merger）部の 2D 版**。
euclidean クラスタリングは 1 人を複数の小クラスタに割ってしまう（over-segmentation）こと
があり、検出が分裂・点滅する。Autoware は **1 フレーム前の tracker の位置・サイズを参照**
して、同一トラックに対応する複数の検出を 1 つに統合し、検出を安定化する。本実装はその
過分割統合のみを踏襲する（under-segmentation の IoU 反復分割は未実装＝第2段）。

循環構造（Autoware と同じ）:
  検出(shaped) ─┐
                ├→ detection_by_tracker → 統合検出 → map_roi → tracker ─┐
  tracker出力 ──┘←──────────────────────────────────────────────────┘
tracker の最新出力を購読し、次フレームの検出統合に使う。tracker 未起動（初回）や
TF 不在のときは検出を素通しする（パイプラインを止めない設計）。

入出力:
  in : /perception/detected_objects_shaped (DetectedObjects, frame=velodyne_link)
       /perception/tracked_objects          (TrackedObjects, frame=odom)
  out: /perception/detected_objects_merged  (DetectedObjects, frame=velodyne_link)

アルゴリズム（Cluster Merger 踏襲）:
  1. tracker の各トラックを検出フレーム(velodyne_link)へ TF 変換し、位置と OBB 半径を得る。
  2. 各検出を最近傍トラック（中心間距離が assign_radius + トラック半径以内）に割り当てる。
  3. 同一トラックに 2 個以上の検出が割り当たったら 1 つに統合。統合後の shape は
     **統合領域の no_ground 点群を L字フィットで再推定**する（包含 BBox にすると離れた
     検出を覆って巨大化するため。Autoware 本家も点群を shape_estimation で再フィットし、
     tracker サイズを参照する）。点群不足/失敗時は tracker サイズにフォールバック。
  4. どのトラックにも属さない検出はそのまま通す（新規物体を消さない）。
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
from tf2_ros import TransformException

import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from autoware_perception_msgs.msg import (
    DetectedObjects, DetectedObject, TrackedObjects, Shape)
from geometry_msgs.msg import Quaternion

# 統合後の shape は包含 BBox ではなく、点群を L字フィットで再推定する（Autoware
# 本家と同じ）。shape_estimation_node の L字フィット実装を再利用する。
from susumu_object_perception.shape_estimation_node import fit_l_shape


def yaw_of(q: Quaternion):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class DetectionByTrackerNode(Node):

    def __init__(self):
        super().__init__('detection_by_tracker')

        self.declare_parameter('input_objects', '/perception/detected_objects_shaped')
        self.declare_parameter('input_tracks', '/perception/tracked_objects')
        self.declare_parameter('output_objects', '/perception/detected_objects_merged')
        # 検出をトラックに割り当てる距離 [m]（トラックの OBB 半径に加算）
        self.declare_parameter('assign_radius', 0.4)

        self.declare_parameter('input_cloud', '/perception/no_ground/pointcloud')
        # 統合検出の中心から点を集める半径 [m]（再フィット用）
        self.declare_parameter('cluster_radius', 0.6)
        self.declare_parameter('min_points', 6)

        self.assign_radius = float(self.get_parameter('assign_radius').value)
        self.cluster_radius = float(self.get_parameter('cluster_radius').value)
        self.min_points = int(self.get_parameter('min_points').value)

        self._latest_tracks = None  # 最新 TrackedObjects（前フレーム相当）
        self._cloud_xy = None       # 最新 no_ground 点群 (N,2)（統合再フィット用）

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10)
        cloud_qos = QoSProfile(depth=1)
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(
            DetectedObjects, self.get_parameter('output_objects').value, qos)
        self.create_subscription(
            TrackedObjects, self.get_parameter('input_tracks').value,
            self.on_tracks, qos)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_cloud').value,
            self.on_cloud, cloud_qos)
        self.create_subscription(
            DetectedObjects, self.get_parameter('input_objects').value,
            self.on_objects, qos)

        self.get_logger().info(
            'detection_by_tracker started (cluster merger, 2D).')

    def on_tracks(self, msg: TrackedObjects):
        self._latest_tracks = msg

    def on_cloud(self, msg: PointCloud2):
        rec = pc2.read_points(msg, field_names=('x', 'y'), skip_nans=True)
        if rec.size == 0:
            self._cloud_xy = None
            return
        xy = np.empty((rec.shape[0], 2), dtype=np.float64)
        xy[:, 0] = rec['x']
        xy[:, 1] = rec['y']
        self._cloud_xy = xy

    def _tracks_in_frame(self, target_frame):
        """最新トラックを target_frame(検出フレーム) に変換した [(x,y,radius,yaw)] を返す。

        TF 不在やトラック無しなら空リスト（→ 統合せず素通し）。
        """
        tracks = self._latest_tracks
        if tracks is None or not tracks.objects:
            return []
        src = tracks.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, src, rclpy.time.Time())
        except TransformException:
            return []
        t = tf.transform.translation
        q = tf.transform.rotation
        tyaw = yaw_of(q)
        c, s = math.cos(tyaw), math.sin(tyaw)

        out = []
        for o in tracks.objects:
            p = o.kinematics.pose_with_covariance.pose.position
            x = t.x + c * p.x - s * p.y
            y = t.y + s * p.x + c * p.y
            # OBB の外接円半径（対角の半分）
            d = o.shape.dimensions
            radius = 0.5 * math.hypot(max(0.05, d.x), max(0.05, d.y))
            yaw = tyaw + yaw_of(o.kinematics.pose_with_covariance.pose.orientation)
            out.append((x, y, radius, yaw))
        return out

    def on_objects(self, msg: DetectedObjects):
        tracks = self._tracks_in_frame(msg.header.frame_id)

        out = DetectedObjects()
        out.header = msg.header

        if not tracks:
            # トラック情報なし → そのまま通す（初回/TF不在）
            out.objects = list(msg.objects)
            self.pub.publish(out)
            return

        # 各検出を最近傍トラックに割り当て（範囲内のみ）。-1 = 未割当。
        assign = []
        for obj in msg.objects:
            p = obj.kinematics.pose_with_covariance.pose.position
            best, best_d2 = -1, None
            for ti, (tx, ty, tr, tyaw) in enumerate(tracks):
                d2 = (p.x - tx) ** 2 + (p.y - ty) ** 2
                reach = (self.assign_radius + tr) ** 2
                if d2 <= reach and (best_d2 is None or d2 < best_d2):
                    best, best_d2 = ti, d2
            assign.append(best)

        # トラックごとに割り当たった検出を集める
        groups = {}
        for di, ti in enumerate(assign):
            if ti < 0:
                # 未割当はそのまま出す（新規物体を消さない）
                out.objects.append(msg.objects[di])
            else:
                groups.setdefault(ti, []).append(di)

        # 各グループを統合
        for ti, dis in groups.items():
            if len(dis) == 1:
                # 1 対 1 はそのまま（過分割ではない）
                out.objects.append(msg.objects[dis[0]])
                continue
            out.objects.append(self._merge(msg.objects, dis, tracks[ti]))

        self.pub.publish(out)

    def _merge(self, objects, dis, track):
        """同一トラックに属する複数検出を 1 つに統合する（Cluster Merger）。

        Autoware 本家と同じく、**統合後の shape は包含 BBox ではなく、統合領域の点群を
        L字フィットで再推定**する（包含 BBox だと離れた検出を覆って巨大化するため）。
        点群が足りない/フィット失敗時は tracker のサイズにフォールバックする
        （Autoware が tracker サイズを参照情報にするのと同趣旨。巨大化しない）。
        """
        tx, ty, tr, tyaw = track

        # 統合する検出群の重心
        cxs = [objects[di].kinematics.pose_with_covariance.pose.position.x for di in dis]
        cys = [objects[di].kinematics.pose_with_covariance.pose.position.y for di in dis]
        cx, cy = float(np.mean(cxs)), float(np.mean(cys))
        zmax = max(0.05, max(
            objects[di].shape.dimensions.z for di in dis))
        z0 = objects[dis[0]].kinematics.pose_with_covariance.pose.position.z

        # 重心周辺の no_ground 点を集めて L字フィットで再推定（Autoware 流）
        fitted = None
        cloud = self._cloud_xy
        if cloud is not None:
            r2 = self.cluster_radius * self.cluster_radius
            d2 = (cloud[:, 0] - cx) ** 2 + (cloud[:, 1] - cy) ** 2
            near = cloud[d2 <= r2]
            if near.shape[0] >= self.min_points:
                fitted = fit_l_shape(near)

        m = DetectedObject()
        m.existence_probability = objects[dis[0]].existence_probability
        m.classification = objects[dis[0]].classification

        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        if fitted is not None:
            fcx, fcy, dim_x, dim_y, fyaw = fitted
            m.kinematics.pose_with_covariance.pose.position.x = float(fcx)
            m.kinematics.pose_with_covariance.pose.position.y = float(fcy)
            m.kinematics.pose_with_covariance.pose.orientation = yaw_to_quat(fyaw)
            shape.dimensions.x = float(dim_x)
            shape.dimensions.y = float(dim_y)
        else:
            # フォールバック: tracker サイズ（巨大化を避ける）
            m.kinematics.pose_with_covariance.pose.position.x = cx
            m.kinematics.pose_with_covariance.pose.position.y = cy
            m.kinematics.pose_with_covariance.pose.orientation = yaw_to_quat(tyaw)
            # tracker 半径から OBB 寸法を復元（外接円→正方近似）。tr は対角半分。
            side = max(0.1, tr * math.sqrt(2.0))
            shape.dimensions.x = side
            shape.dimensions.y = side
        shape.dimensions.z = zmax
        m.kinematics.pose_with_covariance.pose.position.z = z0
        m.shape = shape
        return m


def main(args=None):
    rclpy.init(args=args)
    node = DetectionByTrackerNode()
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
