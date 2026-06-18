#!/usr/bin/env python3
"""2D 占有格子地図と照合して検出物体を絞り込む ROI フィルタ（自作）。

【役割】
Autoware の perception は本来 HD 地図（lanelet2 の drivable area / ROI）で「道路上に
ない検出」を捨てる。本シミュレーターには HD 地図が無いので、代わりに **2D 占有格子
地図**（AMCL/Nav2 が使う `/map`）と照合し、地図上で「壁(占有セル)」や「地図の外／
未知領域」に当たる検出を除外する。地図内のフリースペースにある物体だけを通す。

これにより、壁そのものをクラスタ化した検出（壁上の緑ボックス）や、建物外を拾った
検出（地図範囲外の赤ボックス）が落ち、残るのは「地図に無い＝動的に現れた物体（人）」
に近づく。Autoware の map-based object filter の 2D 版という位置づけ。

【入出力】
  in : /perception/detected_objects (DetectedObjects, frame=lidar_link)
       /map (OccupancyGrid, latched)
  out: /perception/detected_objects_in_map (DetectedObjects, frame=lidar_link)

下流の object_tracker はこの絞り込み済みトピックを購読する。

【判定】
各物体の重心を map 座標へ TF 変換し、その点が乗る地図セルの占有値で判定:
  - セル値 >= occupied_thresh（壁）          → 除外
  - セル値 == -1（未知）または 地図範囲外     → 除外（keep_unknown=false 時）
  - それ以外（フリースペース）                → 通す
壁ぴったりに立つ人を誤って消さないよう、判定はセルそのもの（膨張なし）を既定とし、
必要なら wall_margin_cells で壁周辺の許容を調整できる。
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import TransformException

from nav_msgs.msg import OccupancyGrid
from autoware_perception_msgs.msg import DetectedObjects


class MapRoiFilterNode(Node):

    def __init__(self):
        super().__init__('map_roi_filter')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('input_topic', '/perception/detected_objects')
        self.declare_parameter('output_topic', '/perception/detected_objects_in_map')
        self.declare_parameter('map_frame', 'map')
        # 占有とみなすセル値の下限（OccupancyGrid は 0..100, -1=unknown）。
        self.declare_parameter('occupied_thresh', 50)
        # 未知セル(-1)・地図範囲外の検出を通すか（既定 false=除外）。
        self.declare_parameter('keep_unknown', False)
        # 壁セルの周囲このセル数ぶんも占有扱いにする（0=膨張なし）。壁に貼り付いた
        # クラスタを落としたいなら 1〜2 に。人を消し過ぎるなら 0 のまま。
        self.declare_parameter('wall_margin_cells', 0)

        self.map_frame = self.get_parameter('map_frame').value
        self.occ_thresh = int(self.get_parameter('occupied_thresh').value)
        self.keep_unknown = bool(self.get_parameter('keep_unknown').value)
        self.margin = int(self.get_parameter('wall_margin_cells').value)

        self.map = None          # OccupancyGrid
        self.grid = None         # np.int8 [height, width]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 地図は latched(transient_local) で配信される。
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.history = HistoryPolicy.KEEP_LAST

        io_qos = QoSProfile(depth=10)
        io_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.on_map, map_qos)
        self.pub = self.create_publisher(
            DetectedObjects, self.get_parameter('output_topic').value, io_qos)
        self.create_subscription(
            DetectedObjects, self.get_parameter('input_topic').value,
            self.on_detections, io_qos)

        self.get_logger().info(
            f'map_roi_filter started. {self.get_parameter("input_topic").value} '
            f'-> {self.get_parameter("output_topic").value} '
            f'(occupied_thresh={self.occ_thresh}, keep_unknown={self.keep_unknown})')

    def on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.get_logger().info(
            f'map received: {msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f} '
            f'origin=({msg.info.origin.position.x:.1f},'
            f'{msg.info.origin.position.y:.1f})')

    def _cell_blocked(self, mx, my):
        """map 座標 (mx,my) が壁/未知/範囲外なら True（=除外すべき）。"""
        info = self.map.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        cx = int((mx - ox) / res)
        cy = int((my - oy) / res)
        h, w = self.grid.shape

        # 地図範囲外
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return not self.keep_unknown

        # マージン窓内に壁があれば占有扱い
        m = self.margin
        x0, x1 = max(0, cx - m), min(w, cx + m + 1)
        y0, y1 = max(0, cy - m), min(h, cy + m + 1)
        window = self.grid[y0:y1, x0:x1]
        if np.any(window >= self.occ_thresh):
            return True  # 壁

        # 中心セルが未知
        if self.grid[cy, cx] == -1:
            return not self.keep_unknown

        return False  # フリースペース

    def on_detections(self, msg: DetectedObjects):
        if self.map is None or self.grid is None:
            # 地図未受信のうちは素通し（perception を止めない）。
            self.pub.publish(msg)
            return

        src = msg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, src, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {self.map_frame}<-{src} unavailable: {ex}',
                throttle_duration_sec=2.0)
            self.pub.publish(msg)  # 変換できないときも素通し
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        c, s = math.cos(yaw), math.sin(yaw)

        out = DetectedObjects()
        out.header = msg.header  # frame は据え置き（lidar_link）。下流は据え置きで動く
        for obj in msg.objects:
            p = obj.kinematics.pose_with_covariance.pose.position
            mx = c * p.x - s * p.y + t.x
            my = s * p.x + c * p.y + t.y
            if not self._cell_blocked(mx, my):
                out.objects.append(obj)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapRoiFilterNode()
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
