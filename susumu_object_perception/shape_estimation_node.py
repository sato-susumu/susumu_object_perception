#!/usr/bin/env python3
"""検出クラスタに OBB（有向境界ボックス）の形状・向きを推定する自作ノード。

Autoware の `autoware_shape_estimation`（universe）は型インターフェースが
`tier4_perception_msgs::DetectedObjectsWithFeature`（旧世代）前提で、apt 版の検出
パイプライン（`autoware_perception_msgs/DetectedObjects` を出す新世代）とは世代が
合わず、そのままでは繋がらない。そこで **L字フィットのアルゴリズムだけを Autoware
公式ソース（`autoware_shape_estimation/lib/model/bounding_box.cpp`）から踏襲**し、
入出力は標準型 + autoware_perception_msgs で自作した。

アルゴリズムは Zhang et al., "Efficient L-Shape Fitting for Vehicle Detection
Using Laser Scanners"(IV 2017) の Closeness Criterion 法。Autoware の実装と同じ:
  - 角度 θ を 1°刻みで grid search（`optimize`）
  - 各 θ で点群を直交2軸 e1,e2 に射影し closeness criterion を計算
    （`calcClosenessCriterion`、d_min=0.1^2 / d_max=0.4^2 クランプ）
  - q 最大の θ* から4辺の直線を立て、交点で中心・寸法・yaw を出す（`fitLShape`）

入力:
  /perception/detected_objects        (DetectedObjects)  … euclidean_cluster の検出（位置のみ）
  /perception/no_ground/pointcloud    (PointCloud2)      … 地面除去点群（クラスタ点の供給源）
出力:
  /perception/detected_objects_shaped (DetectedObjects)  … shape(OBB) を埋めた検出

各検出について、no_ground 点群から検出中心の半径 `cluster_radius` 内の点（XY）を集めて
L字フィットする。点が少なすぎる検出は既定 BBox（人サイズ）でフォールバックする。
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from autoware_perception_msgs.msg import DetectedObjects, Shape
from geometry_msgs.msg import Quaternion


# Autoware bounding_box.cpp の定数（closeness criterion のクランプ閾値）。
_D_MIN = 0.1 * 0.1
_D_MAX = 0.4 * 0.4
_ANGLE_RES = math.pi / 180.0  # 1°刻み grid search
_EPSILON = 0.001


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def calc_closeness_criterion(c1, c2):
    """Autoware calcClosenessCriterion の移植。c1,c2 は射影座標の np 配列。"""
    min_c1, max_c1 = c1.min(), c1.max()
    min_c2, max_c2 = c2.min(), c2.max()
    # 各点の「最寄り辺までの距離」の二乗
    d1 = np.minimum(max_c1 - c1, c1 - min_c1) ** 2
    d2 = np.minimum(max_c2 - c2, c2 - min_c2) ** 2
    d = np.minimum(d1, d2)
    # d_max 超の点は無視、d_min でクランプして 1/d を足す
    mask = d <= _D_MAX
    if not np.any(mask):
        return 0.0
    dd = np.maximum(d[mask], _D_MIN)
    return float(np.sum(1.0 / dd))


def fit_l_shape(xy):
    """Autoware fitLShape の移植。xy は (N,2) の点群。

    戻り値: (cx, cy, dim_x, dim_y, yaw) または None。
    """
    if xy.shape[0] < 2:
        return None

    x = xy[:, 0]
    y = xy[:, 1]

    # --- optimize(): θ を grid search し closeness 最大の θ* を得る ---
    best_q = -1.0
    theta_star = 0.0
    theta = 0.0
    # 0〜90°で十分（矩形は 90°周期）。Autoware は ref_yaw が無ければ全周だが
    # 屋内・無参照なので [0, π/2) を 1°刻みで探索する。
    while theta <= math.pi / 2.0 + _EPSILON:
        ct, st = math.cos(theta), math.sin(theta)
        c1 = x * ct + y * st          # e1=(cosθ, sinθ) への射影
        c2 = -x * st + y * ct         # e2=(-sinθ, cosθ) への射影
        q = calc_closeness_criterion(c1, c2)
        if q > best_q:
            best_q = q
            theta_star = theta
        theta += _ANGLE_RES

    # --- fitLShape(): θ* から4辺を立て交点で中心・寸法・yaw ---
    ct, st = math.cos(theta_star), math.sin(theta_star)
    c1 = x * ct + y * st
    c2 = -x * st + y * ct
    min_c1, max_c1 = c1.min(), c1.max()
    min_c2, max_c2 = c2.min(), c2.max()

    # 4辺の直線 a*x + b*y = c（Autoware と同じ係数）
    a1, b1, cc1 = ct, st, min_c1
    a2, b2, cc2 = -st, ct, min_c2
    a3, b3, cc3 = ct, st, max_c1
    a4, b4, cc4 = -st, ct, max_c2

    def intersect(a_i, b_i, c_i, a_j, b_j, c_j):
        denom = a_i * b_j - a_j * b_i
        if abs(denom) < 1e-9:
            return None
        px = (b_j * c_i - b_i * c_j) / denom
        py = (a_i * c_j - a_j * c_i) / denom
        return px, py

    p1 = intersect(a1, b1, cc1, a2, b2, cc2)
    p2 = intersect(a3, b3, cc3, a4, b4, cc4)
    if p1 is None or p2 is None:
        return None

    cx = (p1[0] + p2[0]) * 0.5
    cy = (p1[1] + p2[1]) * 0.5
    # 寸法は射影レンジ（e1 方向 = θ* 軸）
    dim_x = max(max_c1 - min_c1, _EPSILON)
    dim_y = max(max_c2 - min_c2, _EPSILON)
    yaw = theta_star
    return cx, cy, dim_x, dim_y, yaw


class ShapeEstimationNode(Node):

    def __init__(self):
        super().__init__('shape_estimation')

        self.declare_parameter('input_objects', '/perception/detected_objects')
        self.declare_parameter('input_cloud', '/perception/no_ground/pointcloud')
        self.declare_parameter('output_objects', '/perception/detected_objects_shaped')
        # 検出中心から点を集める半径 [m]（人クラスタを覆う程度）
        self.declare_parameter('cluster_radius', 0.6)
        # L字フィットに必要な最小点数。これ未満は既定 BBox でフォールバック
        self.declare_parameter('min_points', 6)
        # フォールバック既定 BBox（人サイズ）[m]
        self.declare_parameter('default_size_xy', 0.6)
        self.declare_parameter('default_size_z', 1.7)

        self.cluster_radius = float(self.get_parameter('cluster_radius').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.default_xy = float(self.get_parameter('default_size_xy').value)
        self.default_z = float(self.get_parameter('default_size_z').value)

        # 点群は最新フレームだけ保持（Best Effort）
        cloud_qos = QoSProfile(depth=1)
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._cloud_xy = None  # 最新 no_ground 点群の (N,2)

        self.pub = self.create_publisher(
            DetectedObjects, self.get_parameter('output_objects').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_cloud').value,
            self.on_cloud, cloud_qos)
        self.create_subscription(
            DetectedObjects, self.get_parameter('input_objects').value,
            self.on_objects, 10)

        self.get_logger().info('shape_estimation started (Autoware L-shape fitting).')

    def on_cloud(self, msg: PointCloud2):
        # read_points_numpy は全フィールドの datatype が揃っている前提なので、
        # x,y,z(float32) と intensity/rgb 等が混在する点群では使えない。
        # read_points で構造化配列を得て x,y だけ float の (N,2) に変換する。
        rec = pc2.read_points(msg, field_names=('x', 'y'), skip_nans=True)
        if rec.size == 0:
            self._cloud_xy = None
            return
        xy = np.empty((rec.shape[0], 2), dtype=np.float64)
        xy[:, 0] = rec['x']
        xy[:, 1] = rec['y']
        self._cloud_xy = xy

    def on_objects(self, msg: DetectedObjects):
        out = DetectedObjects()
        out.header = msg.header
        cloud = self._cloud_xy
        r2 = self.cluster_radius * self.cluster_radius

        for obj in msg.objects:
            new_obj = obj  # kinematics/classification は流用、shape のみ埋める
            p = obj.kinematics.pose_with_covariance.pose.position

            fitted = None
            if cloud is not None:
                d2 = (cloud[:, 0] - p.x) ** 2 + (cloud[:, 1] - p.y) ** 2
                near = cloud[d2 <= r2]
                if near.shape[0] >= self.min_points:
                    fitted = fit_l_shape(near)

            shape = Shape()
            shape.type = Shape.BOUNDING_BOX
            if fitted is not None:
                cx, cy, dim_x, dim_y, yaw = fitted
                shape.dimensions.x = float(dim_x)
                shape.dimensions.y = float(dim_y)
                shape.dimensions.z = float(self.default_z)
                # OBB 中心・向きで pose を更新（L字フィットで得た幾何）
                new_obj.kinematics.pose_with_covariance.pose.position.x = float(cx)
                new_obj.kinematics.pose_with_covariance.pose.position.y = float(cy)
                new_obj.kinematics.pose_with_covariance.pose.orientation = yaw_to_quat(yaw)
            else:
                # 点が足りない検出は人サイズの既定 BBox
                shape.dimensions.x = self.default_xy
                shape.dimensions.y = self.default_xy
                shape.dimensions.z = self.default_z

            new_obj.shape = shape
            out.objects.append(new_obj)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ShapeEstimationNode()
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
