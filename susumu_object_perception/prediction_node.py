#!/usr/bin/env python3
"""TrackedObjects の将来軌跡を 2D 占有格子地図を使って予測する自作ノード。

Autoware の `map_based_prediction` の **2D 占有格子版**。Autoware は HD 地図のレーン /
crosswalk に沿って予測パスを出すが、屋内には crosswalk が無いので、HD 地図要素の
代わりに **2D 占有格子地図 `/map` の free/occupied** を使う。

第1段（本実装）のアルゴリズム（Autoware の基本部 = 等速予測 を踏襲）:
  1. CV 予測: 各トラックの現在速度 (vx, vy) で等速直線。prediction_horizon[s] を
     time_step[s] 刻みでサンプリングして Pose 列にする（Autoware の既定挙動と同じ
     「現在速度で予測パス長を計算」）。
  2. 2D マップによる壁回避: 予測点をたどり、occupied セルに入ったらそこで打ち切る
     （壁にめり込む非現実的な予測を防ぐ）。地図外/未知は通す。
  3. マルチモーダル: 進行方向を中心に複数角度 (angle_offsets_deg) で扇状に複数パスを
     出す（Autoware の crosswalk マルチパスの 2D 版）。分岐路で複数候補を表現できる。
  4. confidence: 直進(off=0)を最高に、外側ほど減衰。free で最後まで伸びたパスは加点、
     壁で切れたパスは減点し、全候補で正規化する。静止物体は現在位置のみ。

入力:
  /perception/tracked_objects (TrackedObjects, frame=odom)
  /map                        (OccupancyGrid, frame=map)
出力:
  /perception/predicted_objects (PredictedObjects, frame=tracked と同じ=odom)
  /perception/predicted_costmap (OccupancyGrid, frame=map) … Nav2 連携用
    人の「現在位置」と「これから行く先」を表すコストマップ。静的 /map と同じ解像度・原点・
    サイズで、**毎フレーム全セル 0 から作り直し**、(a) 全トラックの現在位置 (b) 移動トラックの
    最有力予測パス、が通るセルだけ 100 にする。Nav2 costmap の自作 C++ 層
    susumu_object_perception::PredictedCostmapLayer が max 合成で読み込む（他層を壊さず・蓄積せず）。
    **STVL 層は廃止し、人の現在位置の障害物化もこの予測層が担う**（STVL は人の通過跡を
    voxel_decay 秒残して「移動軌跡のコスト」が出る問題があった。毎フレーム全消去のこの方式なら
    軌跡が残らない）。現在位置・進路先とも人幅ぶん膨張、進路先は最有力 1 本・近傍のみで過剰に
    塞がない。

座標系: tracked は odom、map は map。各予測点(odom)を map セルに変換して occupied を
見るため、map<-odom の TF（AMCL/Nav2 提供）が要る。TF 不在時は壁回避を無効化して
素の CV 予測だけ出す（perception は止めない設計。map_roi_filter と同じ方針）。
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import tf2_ros
from tf2_ros import TransformException

from nav_msgs.msg import OccupancyGrid
from autoware_perception_msgs.msg import TrackedObjects, PredictedObjects, PredictedObject
from autoware_perception_msgs.msg import PredictedPath
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration


def yaw_to_quat(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PredictionNode(Node):

    def __init__(self):
        super().__init__('prediction')

        self.declare_parameter('input_topic', '/perception/tracked_objects')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/perception/predicted_objects')
        self.declare_parameter('map_frame', 'map')
        # 予測ホライズン [s] と刻み [s]
        self.declare_parameter('prediction_horizon', 3.0)
        self.declare_parameter('time_step', 0.5)
        # 静止とみなす速度しきい [m/s]（これ未満は予測パスを出さない）
        self.declare_parameter('min_speed', 0.1)
        # occupied 判定しきい（OccupancyGrid 0..100、-1=unknown）
        self.declare_parameter('occupied_thresh', 50)
        # マルチモーダル予測の角度オフセット [deg]。進行方向を中心に扇状に複数パスを出す
        # （Autoware の crosswalk マルチパスの 2D 版）。0 は直進。左右に振って分岐を表現。
        self.declare_parameter('angle_offsets_deg', [-40.0, -20.0, 0.0, 20.0, 40.0])
        # 予測パスとして採用する最小ステップ数（壁ですぐ切れたパスは捨てる）
        self.declare_parameter('min_path_steps', 2)

        # --- Nav2 連携: 予測コストマップ（StaticLayer 用 OccupancyGrid）---
        self.declare_parameter('predicted_costmap_topic', '/perception/predicted_costmap')
        # 予測パスを costmap に焼く先の最大時間 [s]。遠い予測ほど不確かなので近傍だけ。
        # （confidence しきいは設けない＝移動トラックなら進路を必ず全部焼く方針。出たり
        #   出なかったりを無くすため。）
        self.declare_parameter('predcost_max_horizon', 2.0)
        # 予測セルを人幅ぶん膨張させる半径 [セル]。1 点だと 1 セルしか踏まないので、
        # 人幅 + 予測方向ズレ吸収ぶん（0.4m=res0.05 で 8 セル）円形に膨らませる。
        # 線分で点間は繋ぐが、tracker 推定方向と実進行方向のズレを吸収するため少し広めに。
        # 大きすぎると進路前方が太くなり経路を塞ぐので 8 程度に留める。
        self.declare_parameter('predcost_inflate_cells', 8)
        # 焼くコスト値（0..100）。100=LETHAL。
        self.declare_parameter('predcost_value', 100)

        self.map_frame = self.get_parameter('map_frame').value
        self.horizon = float(self.get_parameter('prediction_horizon').value)
        self.dt = float(self.get_parameter('time_step').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.occ_thresh = int(self.get_parameter('occupied_thresh').value)
        self.angle_offsets = [math.radians(a) for a in
                              self.get_parameter('angle_offsets_deg').value]
        self.min_path_steps = int(self.get_parameter('min_path_steps').value)

        self.predcost_max_horizon = float(self.get_parameter('predcost_max_horizon').value)
        self.predcost_inflate = int(self.get_parameter('predcost_inflate_cells').value)
        self.predcost_value = int(self.get_parameter('predcost_value').value)

        self.grid = None      # np.int8 (H,W)
        self.map_info = None   # OccupancyGrid.info

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched map
        self.pub = self.create_publisher(
            PredictedObjects, self.get_parameter('output_topic').value, 10)
        # Nav2 連携用の予測コストマップ publisher（StaticLayer が読む。latched）。
        predcost_qos = QoSProfile(depth=1)
        predcost_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.predcost_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('predicted_costmap_topic').value,
            predcost_qos)
        # 予測セル膨張用の円盤オフセット（半径 predcost_inflate セル内の (dr,dc)）。
        self._disk = self._make_disk(self.predcost_inflate)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self.on_map, map_qos)
        self.create_subscription(
            TrackedObjects, self.get_parameter('input_topic').value, self.on_tracked, 10)

        self.get_logger().info(
            'prediction started (2D map-based, CV + wall avoidance, '
            'predicted costmap for Nav2).')

    @staticmethod
    def _make_disk(r):
        """半径 r セル以内の (dr, dc) オフセット一覧（円盤）を返す。予測セル膨張用。"""
        offs = []
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc <= r * r:
                    offs.append((dr, dc))
        return offs

    @staticmethod
    def _line_cells(r0, c0, r1, c1):
        """(r0,c0)→(r1,c1) を結ぶセル列（端点除く中間セル）を Bresenham で返す。
        予測ポリラインの点間を埋めて隙間を防ぐのに使う。"""
        cells = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            if (r, c) != (r0, c0) and (r, c) != (r1, c1):
                cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc
        return cells

    def on_map(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

    def _map_tf(self, src_frame):
        """map<-src の (tx, ty, yaw) を返す。取得不可なら None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, src_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _is_occupied_map_xy(self, mx, my):
        """map 座標 (mx,my) が occupied セルか。地図外/未知は False（通す）。"""
        if self.grid is None or self.map_info is None:
            return False
        info = self.map_info
        col = int((mx - info.origin.position.x) / info.resolution)
        row = int((my - info.origin.position.y) / info.resolution)
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return False
        v = self.grid[row, col]
        if v < 0:  # unknown
            return False
        return v >= self.occ_thresh

    def _predict_one_path(self, p0, speed, yaw, map_tf, n_steps):
        """指定方向(yaw)へ等速(CV)で n_steps 予測し、壁で打ち切った Pose 列を返す。

        各予測点を map 座標に変換し occupied なら打ち切る。戻り値は (poses, reached)
        — reached は「最後まで free で伸びたか」。confidence の重み付けに使う。
        """
        vx, vy = speed * math.cos(yaw), speed * math.sin(yaw)
        poses = []
        reached = True
        for k in range(1, n_steps + 1):
            t = k * self.dt
            px = p0.x + vx * t
            py = p0.y + vy * t
            if map_tf is not None:
                tx, ty, tyaw = map_tf
                c, s = math.cos(tyaw), math.sin(tyaw)
                mx = tx + c * px - s * py
                my = ty + s * px + c * py
                if self._is_occupied_map_xy(mx, my):
                    reached = False
                    break
            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = p0.z
            pose.orientation = yaw_to_quat(yaw)
            poses.append(pose)
        return poses, reached

    def on_tracked(self, msg: TrackedObjects):
        out = PredictedObjects()
        out.header = msg.header
        src = msg.header.frame_id
        map_tf = self._map_tf(src)  # None なら壁回避無効（CV のみ）

        n_steps = max(1, int(round(self.horizon / self.dt)))

        # Nav2 連携: 予測コストマップに焼くポリライン（odom 系の点列）を人ごとに集める。
        # 各ポリラインは「現在位置 → 予測進路」の連続点列。後で map 座標に変換し、点列を
        # 線分補間してセルを連続して焼く（点を独立した円で焼くと、点間隔 > 円径のとき
        # 隙間が空いて「円が飛び石状に並ぶ」ため）。
        obstacle_polylines = []

        for obj in msg.objects:
            po = PredictedObject()
            po.object_id = obj.object_id
            po.existence_probability = obj.existence_probability
            po.classification = obj.classification
            po.shape = obj.shape
            po.kinematics.initial_pose_with_covariance = \
                obj.kinematics.pose_with_covariance
            po.kinematics.initial_twist_with_covariance = \
                obj.kinematics.twist_with_covariance

            p0 = obj.kinematics.pose_with_covariance.pose.position
            vx = obj.kinematics.twist_with_covariance.twist.linear.x
            vy = obj.kinematics.twist_with_covariance.twist.linear.y
            speed = math.hypot(vx, vy)

            ts = Duration(sec=int(self.dt), nanosec=int((self.dt % 1.0) * 1e9))

            # Nav2 連携: 予測コストマップ用ポリラインの起点は**現在位置**（全トラック）。
            # STVL 層を廃止したので、人の現在位置を costmap に入れるのはこの予測層が担う。
            # 予測層は毎フレーム全消去するので、STVL のように移動軌跡が残らない。
            # 移動トラックはこの後、最有力予測パスを同じポリラインに繋いで「現在位置→進路先」を
            # 連続した帯として焼く（confidence しきいは設けず、移動なら必ず進路全部を焼く）。
            polyline = [(p0.x, p0.y)]

            if speed >= self.min_speed:
                base_yaw = math.atan2(vy, vx)
                # マルチモーダル: 進行方向を中心に複数角度で扇状に予測（Autoware の
                # crosswalk マルチパスの 2D 版）。壁で打ち切り、伸びた長さで confidence。
                cand = []
                for off in self.angle_offsets:
                    poses, reached = self._predict_one_path(
                        p0, speed, base_yaw + off, map_tf, n_steps)
                    if len(poses) >= self.min_path_steps:
                        cand.append((off, poses, reached))
                if not cand:
                    # 全方向すぐ壁 → 直進の最短でも 1 本だけ出す（消さない）
                    poses, reached = self._predict_one_path(
                        p0, speed, base_yaw, map_tf, n_steps)
                    cand = [(0.0, poses, reached)] if poses else []

                # confidence: 直進(off=0)を最高に、外側ほど減衰。free で最後まで伸びた
                # パスは加点。全候補で正規化して合計 1 付近にする。
                raw = []
                for off, poses, reached in cand:
                    w = math.cos(off)                  # 直進ほど大
                    w *= len(poses) / float(n_steps)   # 伸びた長さ
                    w *= 1.0 if reached else 0.6        # 壁で切れたら減点
                    raw.append(max(0.05, w))
                ssum = sum(raw) if raw else 1.0
                best = None  # costmap に焼くのは最有力 1 本だけ
                best_conf = -1.0
                for (off, poses, reached), w in zip(cand, raw):
                    conf = float(min(1.0, w / ssum))
                    path = PredictedPath()
                    path.time_step = ts
                    path.path = poses
                    path.confidence = conf
                    po.kinematics.predicted_paths.append(path)
                    if conf > best_conf:
                        best_conf = conf
                        best = poses
                # Nav2 連携: costmap には**最有力 1 本だけ**を、近い数ステップぶん焼く。
                # 可視化はマルチモーダル全パスを出すが、costmap に全 5 方向×全点を焼くと
                # 進路前方が扇状に広がって埋まる（「ぐちゃぐちゃ」の主因）ので 1 本に絞る。
                # confidence しきいは設けない（移動トラックなら進路を必ず焼く＝出たり
                # 出なかったりを無くす）。現在位置からの連続ポリラインに繋ぐ（線分で焼くので
                # 点間隔が広くても隙間が空かない）。
                max_steps = max(1, int(round(self.predcost_max_horizon / self.dt)))
                if best is not None:
                    for pose in best[:max_steps]:
                        polyline.append((pose.position.x, pose.position.y))
            else:
                # 静止物体: 現在位置のみ（実質予測なし）
                path = PredictedPath()
                path.time_step = ts
                pose = Pose()
                pose.position = p0
                pose.orientation = \
                    obj.kinematics.pose_with_covariance.pose.orientation
                path.path.append(pose)
                path.confidence = 1.0
                po.kinematics.predicted_paths.append(path)

            obstacle_polylines.append(polyline)
            out.objects.append(po)

        self.pub.publish(out)
        self._publish_predicted_costmap(obstacle_polylines, map_tf, msg.header.stamp)

    def _stamp_disk(self, grid, cy, cx, val):
        """セル (cy,cx) を中心に円盤膨張で val を焼く。"""
        h, w = grid.shape
        for dr, dc in self._disk:
            r2 = cy + dr
            c2 = cx + dc
            if 0 <= r2 < h and 0 <= c2 < w:
                grid[r2, c2] = val

    def _publish_predicted_costmap(self, polylines_odom, map_tf, stamp):
        """人ごとのポリライン (odom 点列) を map 格子に線分補間して焼き、毎フレーム作り直した
        OccupancyGrid を publish する。自作 PredictedCostmapLayer が max 合成で読む。空セルは
        0(free) に戻り蓄積しない。点列を線分で繋ぐので、点間隔が円径より広くても隙間が
        空かない（飛び石状にならない）。地図 info 未受信時は出さない。"""
        if self.map_info is None:
            return
        info = self.map_info
        h, w = info.height, info.width
        grid = np.zeros((h, w), dtype=np.int8)  # 毎フレーム全セル free から作り直す

        if polylines_odom and map_tf is not None:
            tx, ty, tyaw = map_tf
            c, s = math.cos(tyaw), math.sin(tyaw)
            res = info.resolution
            ox = info.origin.position.x
            oy = info.origin.position.y
            val = self.predcost_value
            for poly in polylines_odom:
                # ポリラインの各点を map セルに変換。
                cells = []
                for (px, py) in poly:
                    mx = tx + c * px - s * py
                    my = ty + s * px + c * py
                    cells.append((int((my - oy) / res), int((mx - ox) / res)))  # (row,col)
                # 各セルを円盤で焼き、連続セル間を線分で埋める（隙間防止）。
                for i, (cy, cx) in enumerate(cells):
                    self._stamp_disk(grid, cy, cx, val)
                    if i + 1 < len(cells):
                        ny, nx = cells[i + 1]
                        for (ly, lx) in self._line_cells(cy, cx, ny, nx):
                            self._stamp_disk(grid, ly, lx, val)

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info = info
        msg.data = grid.reshape(-1).tolist()
        self.predcost_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
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
