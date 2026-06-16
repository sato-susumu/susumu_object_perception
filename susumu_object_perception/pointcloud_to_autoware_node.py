#!/usr/bin/env python3
"""Gazebo の PointXYZI 点群を Autoware の PointXYZIRC 形式に変換する前処理ノード。

【なぜ必要か（ライブ起動で判明した実問題）】
Autoware の autoware_ground_filter は入力点群が `PointXYZIRC` / `PointXYZIRCAEDT`
（ring/channel フィールドを持つ Autoware 独自型）であることを要求し、それ以外だと
"The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC. Aborting"
を出して処理を中断する。一方 Gazebo の velodyne プラグイン
(libgazebo_ros_ray_sensor) が出す /velodyne_points は古典的な `PointXYZI`
（x,y,z=float, intensity=float, point_step=16）で channel を持たない。

そこで本ノードが間に入り、各点に `return_type` と `channel`(=ring) を付与して
Autoware 互換の PointXYZIRC へ変換する。Autoware の sensing 前処理（本来
pointcloud_preprocessor が担うが apt 版に該当ノードが無い）を Python で補完する位置づけ。

【PointXYZIRC のレイアウト】（autoware_point_types/types.hpp より、point_step=16）
    float  x          @0
    float  y          @4
    float  z          @8
    uint8  intensity  @12   ← Gazebo の float intensity を 0..255 に丸めて格納
    uint8  return_type@13   ← 単一エコー前提で 1(FIRST) 固定
    uint16 channel    @14   ← 仰角から求めた ring 番号(0..N-1)

channel(ring) は VLP-16 の各レーザーの仰角に対応する。Gazebo 点群は ring 情報を
持たないので、点の仰角 atan2(z, sqrt(x^2+y^2)) を [min_angle, max_angle] の N 等分に
量子化して ring を復元する（ground_filter は ring 自体の厳密値より「層構造がある」
ことを使うので、この近似で十分機能する）。
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


# PointXYZIRC のフィールド定義（offset は types.hpp と一致させること）。
AUTOWARE_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
    PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
    PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
]
POINT_STEP = 16
RETURN_TYPE_FIRST = 1  # autoware_point_types ReturnType::FIRST 相当


class PointcloudToAutowareNode(Node):

    def __init__(self):
        super().__init__('pointcloud_to_autoware')

        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_autoware')
        # VLP-16 のレーザー本数と仰角範囲 [deg]（VLP-16 は -15°..+15° の 16 層）。
        self.declare_parameter('num_rings', 16)
        self.declare_parameter('min_elev_deg', -15.0)
        self.declare_parameter('max_elev_deg', 15.0)

        self.num_rings = int(self.get_parameter('num_rings').value)
        self.min_elev = math.radians(float(self.get_parameter('min_elev_deg').value))
        self.max_elev = math.radians(float(self.get_parameter('max_elev_deg').value))
        self._elev_span = max(1e-6, self.max_elev - self.min_elev)

        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value,
            qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self.on_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            f'pointcloud_to_autoware started. '
            f'{self.get_parameter("input_topic").value} (PointXYZI) -> '
            f'{self.get_parameter("output_topic").value} (PointXYZIRC, '
            f'{self.num_rings} rings)')

    def on_cloud(self, msg: PointCloud2):
        # x,y,z,intensity を一括取り出し（numpy 構造化配列）。
        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z', 'intensity'),
            skip_nans=True)
        if pts.shape[0] == 0:
            return

        x = pts[:, 0].astype(np.float32)
        y = pts[:, 1].astype(np.float32)
        z = pts[:, 2].astype(np.float32)
        inten_f = pts[:, 3]

        # intensity を uint8 に丸める（Gazebo は 0..1 か 0..255 のことがあるので
        # 1.0 以下なら 255 倍してスケールを合わせる）。
        if np.nanmax(inten_f) <= 1.0 + 1e-6:
            inten_f = inten_f * 255.0
        intensity = np.clip(inten_f, 0, 255).astype(np.uint8)

        # 仰角 → ring 番号(0..num_rings-1)。
        rng_xy = np.sqrt(x * x + y * y)
        elev = np.arctan2(z, np.maximum(rng_xy, 1e-6))
        ring = np.floor(
            (elev - self.min_elev) / self._elev_span * self.num_rings)
        ring = np.clip(ring, 0, self.num_rings - 1).astype(np.uint16)

        n = x.shape[0]
        # PointXYZIRC レイアウトを numpy 構造化 dtype で一括構築する（点ごとの
        # Python ループは VLP-16 の数万点では遅すぎるため）。dtype の itemsize は
        # 自然に 16 バイト（x,y,z=4*3 + intensity,return_type=1*2 + channel=2）に
        # なり POINT_STEP と一致する。
        structured = np.zeros(n, dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.uint8), ('return_type', np.uint8),
            ('channel', np.uint16),
        ])
        structured['x'] = x
        structured['y'] = y
        structured['z'] = z
        structured['intensity'] = intensity
        structured['return_type'] = RETURN_TYPE_FIRST
        structured['channel'] = ring

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n
        out.fields = AUTOWARE_FIELDS
        out.is_bigendian = False
        out.point_step = POINT_STEP
        out.row_step = POINT_STEP * n
        out.data = structured.tobytes()
        out.is_dense = True
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudToAutowareNode()
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
