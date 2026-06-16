#!/usr/bin/env python3
"""DetectedObjects を時系列で追跡し TrackedObjects を発行する自作トラッカー。

Autoware の perception パイプラインのうち、apt で入手できない multi_object_tracker
（追跡）を Python で代替する。Autoware の euclidean_cluster が出す
`autoware_perception_msgs/DetectedObjects`（フレーム間で対応関係を持たない、その瞬間
の検出）を購読し、フレーム間でアソシエーションして ID と速度を付与した
`autoware_perception_msgs/TrackedObjects` を発行する。

────────────────────────────────────────────────────────────────────────────
設計の根拠（Autoware autoware_multi_object_tracker のソースを参照して縮小実装）:

  - 追跡フレーム: Autoware は world_frame_id=map で追跡する
    (multi_object_tracker_node.param.yaml)。本実装は地図前提を避け odom で追跡する
    （ロボット自己移動を見かけ速度から除くのが目的なので固定フレームなら可）。

  - モーションモデル: Autoware の歩行者は CTRV だが、屋内・低速・点群クラスタ重心の
    追跡では向き推定が不安定なので等速 CV [x, y, vx, vy] を採用。ただし Autoware に
    倣い速度を上限 max_vel でクランプし、外れ値の発散を防ぐ
    (cv_motion_model.hpp の max_vx/max_vy 相当)。

  - データアソシエーション: Autoware は GNN/Mu-SSP。本実装は同じ「大域最適割り当て」
    の性質を持つハンガリアン法 (scipy.optimize.linear_sum_assignment) を使う。
    貪欲法より交差ケースに強い。

  - 距離尺度・ゲーティング: Autoware は非車両でマハラノビス距離 + χ²(2自由度) 閾値
    11.62 (99.6%) でゲートする (bev_assignment_scoring.cpp)。本実装も予測共分散から
    マハラノビス距離を計算し、同じ閾値でゲートする。

  - existence_probability: Autoware は Bayes 更新（測定時 TP=0.9/FP=0.2、非測定時は
    半減期 0.5s で指数減衰）し、下限を下回ると削除する (tracker_base.cpp)。本実装も
    同式で更新し、確率と経過時間でトラックを削除する。

  - is_stationary: Autoware はトラッカー型 (StaticTracker) で決める。本実装は型分割を
    持たないので、速度しきい値かつ累積変位しきい値の二段判定で代替する（静止什器を
    動的と誤判定しない）。

独自メッセージは作らず標準型のみ使う（AGENTS.md の方針）。
────────────────────────────────────────────────────────────────────────────
"""

import math

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import TransformException

from autoware_perception_msgs.msg import (
    DetectedObjects,
    TrackedObjects,
    TrackedObject,
    TrackedObjectKinematics,
    ObjectClassification,
    Shape,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy
from unique_identifier_msgs.msg import UUID


# Autoware bev_assignment_scoring.cpp の χ²(2自由度) 99.6% ゲート閾値。
MAHALANOBIS_GATE = 11.62


def yaw_to_quat(yaw):
    """Z 軸回りの yaw [rad] を quaternion (x, y, z, w) に変換。"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class KalmanTrack:
    """等速 CV モデルの 1 トラック。状態 x = [px, py, vx, vy]^T。

    Autoware の cv_motion_model.hpp に倣い、プロセスノイズと速度上限を持つ。
    """

    _next_id = 1

    def __init__(self, xy, stamp_sec, shape, label, params):
        self.id = KalmanTrack._next_id
        KalmanTrack._next_id += 1
        # 16 バイトの UUID を ID から決定的に作る（毎フレーム同じ ID -> 同じ UUID）。
        self.uuid = np.zeros(16, dtype=np.uint8)
        self.uuid[0:4] = np.frombuffer(np.uint32(self.id).tobytes(), dtype=np.uint8)

        self.x = np.array([xy[0], xy[1], 0.0, 0.0], dtype=float)
        # 状態共分散。初期速度は不確かなので大きめ。
        self.P = np.diag([0.5, 0.5, 2.0, 2.0])
        self.shape = shape          # geometry_msgs/Vector3 dimensions 相当 (x, y, z)
        self.label = label
        self.last_stamp = stamp_sec
        self.hits = 1               # 観測で更新された回数
        # 移動判定は「直近の窓での実移動量」で見る。初期位置からの累積だと長寿命の
        # 静止トラックが推定ドリフトで徐々に動いて見え、誤って移動判定されるため。
        # (stamp, x, y) の履歴を持ち、窓より古いものは捨てる。
        self.history = [(stamp_sec, float(xy[0]), float(xy[1]))]

        # Autoware tracker_base.cpp の existence_probability。初期は控えめに。
        self.existence = 0.3

        self._p = params

    @property
    def pos(self):
        return self.x[0:2]

    @property
    def vel(self):
        return self.x[2:4]

    def predict(self, stamp_sec):
        dt = max(1e-3, stamp_sec - self.last_stamp)
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=float)
        # Autoware: 位置プロセスノイズ小、速度プロセスノイズ大（加速度を吸収）。
        Q = np.diag([
            self._p['q_pos'], self._p['q_pos'],
            self._p['q_vel'] * dt, self._p['q_vel'] * dt,
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_stamp = stamp_sec
        self._clamp_velocity()
        # 非測定フレームでは existence を半減期 decay_half_life で指数減衰。
        self.existence *= math.exp(-math.log(2.0) * dt / self._p['decay_half_life'])

    def _clamp_velocity(self):
        vmax = self._p['max_vel']
        speed = math.hypot(self.x[2], self.x[3])
        if speed > vmax:
            scale = vmax / speed
            self.x[2] *= scale
            self.x[3] *= scale

    def mahalanobis2(self, xy):
        """予測位置共分散に基づく検出 xy へのマハラノビス距離の二乗。"""
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = np.diag([self._p['r_meas'], self._p['r_meas']])
        S = H @ self.P @ H.T + R
        d = np.array(xy, dtype=float) - self.x[0:2]
        return float(d.T @ np.linalg.inv(S) @ d)

    def update(self, xy, shape, label):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = np.diag([self._p['r_meas'], self._p['r_meas']])
        z = np.array(xy, dtype=float)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.shape = shape
        self.label = label
        self.hits += 1
        self._clamp_velocity()
        # 移動判定用に位置履歴へ追加し、窓 (disp_window) より古い点を捨てる。
        self.history.append((self.last_stamp, float(self.x[0]), float(self.x[1])))
        win = self._p['disp_window']
        cutoff = self.last_stamp - win
        self.history = [h for h in self.history if h[0] >= cutoff]
        # Autoware tracker_base.cpp の Bayes 更新（測定された＝確率を引き上げる）。
        tp, fp = self._p['tp'], self._p['fp']
        p = self.existence
        self.existence = (p * tp) / (p * tp + (1.0 - p) * fp)
        self.existence = min(0.999, max(0.001, self.existence))

    def displacement(self):
        """直近 disp_window 秒の窓内での実移動量（最古点から現在位置までの距離）。"""
        if len(self.history) < 2:
            return 0.0
        x0, y0 = self.history[0][1], self.history[0][2]
        return float(math.hypot(self.pos[0] - x0, self.pos[1] - y0))


class ObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('object_tracker')

        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('input_topic', '/perception/detected_objects')
        self.declare_parameter('output_topic', '/perception/tracked_objects')
        # マハラノビスゲートに加える保険の距離ゲート [m]（重心が飛んだ検出を弾く）。
        self.declare_parameter('association_max_dist', 1.5)
        # 出力に乗せる最小ヒット数（ちらつき抑制）。
        self.declare_parameter('min_hits', 2)
        # existence_probability の下限（これ未満で削除）と削除までの最大未更新時間 [s]。
        self.declare_parameter('min_existence', 0.05)
        self.declare_parameter('max_age_sec', 1.0)
        # 移動判定: 速度しきい値 [m/s] かつ 累積変位しきい値 [m]。
        # ライブ確認の結果、cafe の静止什器/壁がクラスタの揺れで重心が ±0.5m ほど
        # ふらつき「移動」と誤判定されたため、変位閾値を 0.7m に上げて実移動（歩行者）
        # だけを残す。速度閾値も歩行者下限に合わせ 0.3m/s に。
        self.declare_parameter('moving_vel_thresh', 0.3)
        self.declare_parameter('moving_disp_thresh', 0.7)
        # 変位を測る時間窓 [s]（この窓内の実移動量で判定。長寿命静止トラックの
        # ドリフト誤判定を避けるため累積ではなく窓ベースにする）。
        self.declare_parameter('disp_window', 2.0)
        # CV モーション/観測パラメータ（Autoware cv_motion_model.hpp を屋内向けに調整）。
        self.declare_parameter('max_vel', 2.78)        # 歩行者上限 [m/s]
        self.declare_parameter('q_pos', 0.025)
        self.declare_parameter('q_vel', 2.0)
        self.declare_parameter('r_meas', 0.08)
        self.declare_parameter('tp', 0.9)
        self.declare_parameter('fp', 0.2)
        self.declare_parameter('decay_half_life', 0.5)
        # 出力時の 2D 地図照合: トラック位置が地図上で壁/地図外に当たれば出力しない。
        # 方針: 「壁の近傍に張り付く静止ゴースト（緑ボックス）だけ消し、人も机も残す」。
        # 机は地図から除去済み（maps/cafe.pgm の机5卓周辺 0.5m を free 化、
        # clear_tables.py）なので地図照合では消えない。
        #
        # 壁 margin を「静止トラックには広く・移動トラックには狭く」二段にする:
        #  - 静止トラック（人でない＝動かない）: 壁から wall_margin_static_cells 以内を
        #    壁扱いにして消す。壁際に張り付く不動ゴーストは机と hits/existence/disp が
        #    区別できないが、「壁からの距離」で分離できる（ground truth 実測: ゴースト
        #    は壁 0.5〜1.4m、机は壁 1.5m 以上）。1.1m(22cell) で机0/5巻込・ゴースト4/5消去。
        #  - 移動トラック（歩行者）: wall_margin_moving_cells のみ。壁ぎりぎりを歩く人を
        #    取りこぼさないよう狭く保つ（0.3m=6cell）。
        self.declare_parameter('use_map_filter', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('occupied_thresh', 50)
        self.declare_parameter('wall_margin_moving_cells', 6)    # 0.30m
        self.declare_parameter('wall_margin_static_cells', 22)   # 1.10m

        self.tracking_frame = self.get_parameter('tracking_frame').value
        self.assoc_dist = float(self.get_parameter('association_max_dist').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.min_existence = float(self.get_parameter('min_existence').value)
        self.max_age = float(self.get_parameter('max_age_sec').value)
        self.moving_vel = float(self.get_parameter('moving_vel_thresh').value)
        self.moving_disp = float(self.get_parameter('moving_disp_thresh').value)
        self.track_params = {
            'max_vel': float(self.get_parameter('max_vel').value),
            'q_pos': float(self.get_parameter('q_pos').value),
            'q_vel': float(self.get_parameter('q_vel').value),
            'r_meas': float(self.get_parameter('r_meas').value),
            'tp': float(self.get_parameter('tp').value),
            'fp': float(self.get_parameter('fp').value),
            'decay_half_life': float(self.get_parameter('decay_half_life').value),
            'disp_window': float(self.get_parameter('disp_window').value),
        }

        self.tracks = []
        self.last_stamp_sec = None

        self.use_map_filter = bool(self.get_parameter('use_map_filter').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.occ_thresh = int(self.get_parameter('occupied_thresh').value)
        self.margin_moving = int(
            self.get_parameter('wall_margin_moving_cells').value)
        self.margin_static = int(
            self.get_parameter('wall_margin_static_cells').value)
        self.map = None
        self.grid = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        if self.use_map_filter:
            map_qos = QoSProfile(depth=1)
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            map_qos.reliability = ReliabilityPolicy.RELIABLE
            self.create_subscription(
                OccupancyGrid, self.get_parameter('map_topic').value,
                self.on_map, map_qos)

        self.pub = self.create_publisher(
            TrackedObjects,
            self.get_parameter('output_topic').value, qos)
        self.sub = self.create_subscription(
            DetectedObjects,
            self.get_parameter('input_topic').value,
            self.on_detections, qos)

        self.get_logger().info(
            f'object_tracker started. {self.get_parameter("input_topic").value} '
            f'-> {self.get_parameter("output_topic").value} '
            f'(tracking_frame={self.tracking_frame})')

    def on_detections(self, msg: DetectedObjects):
        stamp = msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9

        # 検出を tracking_frame へ変換した (xy, shape, label) のリストにする。
        src_frame = msg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tracking_frame, src_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {self.tracking_frame}<-{src_frame} unavailable: {ex}',
                throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        # 2D の同次変換（z 回転のみ使う。屋内平面なので十分）。
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        c, s = math.cos(yaw), math.sin(yaw)

        dets = []
        for obj in msg.objects:
            p = obj.kinematics.pose_with_covariance.pose.position
            gx = c * p.x - s * p.y + t.x
            gy = s * p.x + c * p.y + t.y
            label = (obj.classification[0].label
                     if obj.classification else ObjectClassification.UNKNOWN)
            dets.append(((gx, gy), obj.shape.dimensions, label))

        self._predict_all(stamp_sec)
        self._associate_and_update(dets, stamp_sec)
        self._prune(stamp_sec)
        self._publish(stamp)

    def _predict_all(self, stamp_sec):
        for tr in self.tracks:
            tr.predict(stamp_sec)

    def _associate_and_update(self, dets, stamp_sec):
        """ハンガリアン法でトラックと検出を大域最適に割り当て、ゲート外は不採用。"""
        matched_dets = set()
        if self.tracks and dets:
            nt, nd = len(self.tracks), len(dets)
            # コスト行列。ゲート外は大コスト BIG にして割り当てから実質除外する。
            BIG = 1e6
            cost = np.full((nt, nd), BIG, dtype=float)
            for ti, tr in enumerate(self.tracks):
                for di, (xy, _, _) in enumerate(dets):
                    euclid = math.hypot(tr.pos[0] - xy[0], tr.pos[1] - xy[1])
                    if euclid > self.assoc_dist:
                        continue  # 保険の距離ゲート
                    m2 = tr.mahalanobis2(xy)
                    if m2 > MAHALANOBIS_GATE:
                        continue  # Autoware と同じ χ² ゲート
                    cost[ti, di] = m2

            row_idx, col_idx = linear_sum_assignment(cost)
            for ti, di in zip(row_idx, col_idx):
                if cost[ti, di] >= BIG:
                    continue  # ゲート外の割り当ては破棄
                xy, shape, label = dets[di]
                self.tracks[ti].update(xy, shape, label)
                matched_dets.add(di)

        # 対応の取れなかった検出は新規トラックとして起こす。
        for di, (xy, shape, label) in enumerate(dets):
            if di not in matched_dets:
                self.tracks.append(
                    KalmanTrack(xy, stamp_sec, shape, label, self.track_params))

    def on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def _track_blocked_on_map(self, tr):
        """トラック位置(tracking_frame)を map 座標へ変換し、壁/地図外なら True。

        地図が無い・TF が引けないときは False（=出力する。地図照合を諦めて
        perception を止めない）。
        """
        if not self.use_map_filter or self.grid is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.tracking_frame, rclpy.time.Time())
        except TransformException:
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        c, s = math.cos(yaw), math.sin(yaw)
        mx = c * tr.pos[0] - s * tr.pos[1] + t.x
        my = s * tr.pos[0] + c * tr.pos[1] + t.y

        info = self.map.info
        res = info.resolution
        cx = int((mx - info.origin.position.x) / res)
        cy = int((my - info.origin.position.y) / res)
        h, w = self.grid.shape
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return True  # 地図外
        # 静止トラックは壁から遠くまで（margin_static）壁扱いにして壁際ゴーストを消す。
        # 移動トラック（歩行者）は壁ぎりぎりを歩いても残せるよう margin_moving に絞る。
        m = self.margin_static if self._is_stationary(tr) else self.margin_moving
        x0, x1 = max(0, cx - m), min(w, cx + m + 1)
        y0, y1 = max(0, cy - m), min(h, cy + m + 1)
        if np.any(self.grid[y0:y1, x0:x1] >= self.occ_thresh):
            return True  # 壁（近傍含む）
        return False

    def _prune(self, stamp_sec):
        """Autoware tracker_base.cpp の isExpired 相当: 確率下限 or 経過時間で削除。"""
        kept = []
        for tr in self.tracks:
            age = stamp_sec - tr.last_stamp
            if tr.existence < self.min_existence:
                continue
            if age > self.max_age:
                continue
            kept.append(tr)
        self.tracks = kept

    def _is_stationary(self, tr):
        speed = float(np.linalg.norm(tr.vel))
        return not (speed >= self.moving_vel and tr.displacement() >= self.moving_disp)

    def _publish(self, stamp):
        out = TrackedObjects()
        out.header.stamp = stamp
        out.header.frame_id = self.tracking_frame

        for tr in self.tracks:
            if tr.hits < self.min_hits:
                continue  # まだ安定していないトラックは出さない
            if self._track_blocked_on_map(tr):
                continue  # 地図上で壁/地図外に居るトラックは出さない（壁上の緑ボックス対策）

            to = TrackedObject()
            to.object_id = UUID(uuid=tr.uuid.tolist())
            to.existence_probability = float(tr.existence)

            cls = ObjectClassification()
            # 2D 地図ベースの classification（Autoware の HD マップ walkable-area で
            # 歩行者を推定する処理の 2D 占有格子版）。ここまで来たトラックは
            # _track_blocked_on_map を通過済み = 地図の free space にいる。検出器が
            # ラベルを付けていれば尊重し、UNKNOWN のときだけ地図+運動で推定する:
            #   free space で移動 → PEDESTRIAN / 静止 → UNKNOWN（什器の可能性を残す）
            if tr.label != ObjectClassification.UNKNOWN:
                cls.label = tr.label
                cls.probability = 1.0
            elif not self._is_stationary(tr):
                cls.label = ObjectClassification.PEDESTRIAN
                cls.probability = 0.7
            else:
                cls.label = ObjectClassification.UNKNOWN
                cls.probability = 1.0
            to.classification = [cls]

            kin = TrackedObjectKinematics()
            kin.pose_with_covariance.pose.position.x = float(tr.pos[0])
            kin.pose_with_covariance.pose.position.y = float(tr.pos[1])
            kin.pose_with_covariance.pose.position.z = 0.0
            # 進行方向を向きにする（速度がほぼ 0 なら無向き扱い）。
            speed = float(np.linalg.norm(tr.vel))
            if speed > 1e-3:
                yaw = math.atan2(tr.vel[1], tr.vel[0])
                qx, qy, qz, qw = yaw_to_quat(yaw)
                kin.pose_with_covariance.pose.orientation.x = qx
                kin.pose_with_covariance.pose.orientation.y = qy
                kin.pose_with_covariance.pose.orientation.z = qz
                kin.pose_with_covariance.pose.orientation.w = qw
                kin.orientation_availability = TrackedObjectKinematics.SIGN_UNKNOWN
            else:
                kin.pose_with_covariance.pose.orientation.w = 1.0
                kin.orientation_availability = TrackedObjectKinematics.UNAVAILABLE
            kin.twist_with_covariance.twist.linear.x = float(tr.vel[0])
            kin.twist_with_covariance.twist.linear.y = float(tr.vel[1])
            kin.is_stationary = self._is_stationary(tr)
            to.kinematics = kin

            to.shape = make_shape(tr.shape)
            out.objects.append(to)

        self.pub.publish(out)


def make_shape(dimensions):
    """トラックの dimensions(Vector3) を Shape(BOUNDING_BOX) に詰める。"""
    sh = Shape()
    sh.type = Shape.BOUNDING_BOX
    # dimensions が空（0）なら人サイズの既定値を入れて可視化を破綻させない。
    dx = dimensions.x if dimensions.x > 0.01 else 0.5
    dy = dimensions.y if dimensions.y > 0.01 else 0.5
    dz = dimensions.z if dimensions.z > 0.01 else 1.7
    sh.dimensions.x = float(dx)
    sh.dimensions.y = float(dy)
    sh.dimensions.z = float(dz)
    return sh


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
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
