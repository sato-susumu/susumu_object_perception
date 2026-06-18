#!/usr/bin/env python3
"""検出物体を map 座標で永続記憶し、無くなったものは消す「物体メモリ」ノード。

docs/semantic_object_memory_research.md の「軽量 self-built object map」方式の MVP 実装。
研究記録では RGB-D 追加が前提だったが、本実装は既存 perception が出す
`tracked_objects_classified`（map 照合済み・YOLO 分類済み・existence_probability 付き）を
入力にすることで RGB-D を省き、MVP を軽くしている。

────────────────────────────────────────────────────────────────────────────
やること（research.md の ①②）:

  ① 座標を記憶 = semantic / object-level mapping
     - tracked_objects_classified の各物体を tracking_frame → map 座標に変換し、
       永続 DB（SQLite）の object と data association（同クラス・近傍距離ゲート τ）で
       照合。一致すれば座標を移動平均で更新、無ければ新規 object を起こす。
       (LTC-Mapping の「最近傍ペアの平均ユークリッド距離 τmax=1m」を踏襲)

  ② 無くなったら消す = object permanence / change detection
     - existence 確率をベイズ更新する。検出された object は引き上げ（TP/FP）、
       「視野内に居るはずなのに検出されない」（negative observation）object は
       引き下げる。視野内かどうかは全天球カメラの全周 FOV + レンジ + 壁遮蔽
       （map の occupied セルが間にあるか）で判定する。閾値割れで DB から削除。
       (Dengler et al. の hits/misses ベイズ更新 + LTC-Mapping の遮蔽判定の縮小版)

既存 object_tracker_node.py の existence_probability ベイズ更新式をそのまま流用する
（research.md が「②の核心は tracker が既に持っている」と指摘した部分）。
独自 .msg は作らず、出力は MarkerArray（可視化）と semantic_query_node が読む DB のみ。
────────────────────────────────────────────────────────────────────────────
"""

import math
import os
import sqlite3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from tf2_ros import TransformException

from autoware_perception_msgs.msg import TrackedObjects, ObjectClassification
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from diagnostic_msgs.msg import DiagnosticArray


# Autoware ObjectClassification.label → 人間可読クラス名。DB に文字列で保存し、
# semantic_query_node の固定辞書（「椅子」→ chair 等）と突き合わせる。検出器が COCO
# 由来でも Autoware label に丸められているので、ここでは Autoware の語彙で持つ。
LABEL_NAMES = {
    ObjectClassification.UNKNOWN: 'unknown',
    ObjectClassification.CAR: 'car',
    ObjectClassification.TRUCK: 'truck',
    ObjectClassification.BUS: 'bus',
    ObjectClassification.TRAILER: 'trailer',
    ObjectClassification.MOTORCYCLE: 'motorcycle',
    ObjectClassification.BICYCLE: 'bicycle',
    ObjectClassification.PEDESTRIAN: 'pedestrian',
}


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ObjectMemoryNode(Node):

    def __init__(self):
        super().__init__('object_memory')

        # 入力は分類済みトラック（無ければ素の tracked_objects でも動く）。
        self.declare_parameter(
            'input_topic', '/perception/tracked_objects_classified')
        # COCO 細クラス名の副チャネル（object_classifier が出す DiagnosticArray）。
        # Autoware label が UNKNOWN に丸める什器(chair/couch/diningtable 等)を区別して
        # 記憶するために使う。来ていなければ label ベースの記憶になるだけで無害。
        self.declare_parameter(
            'fine_class_topic', '/perception/object_fine_classes')
        # 副チャネルの UUID→COCO名 をこの秒数だけ保持（古い対応は捨てる）。
        self.declare_parameter('fine_class_ttl', 3.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        # 視野判定の原点（全天球カメラ）。視野内非検出の negative observation に使う。
        self.declare_parameter('sensor_frame', 'omni_camera_link')
        # data association: 同クラスで近傍ゲート [m]（LTC-Mapping τmax=1m）。
        self.declare_parameter('assoc_dist', 1.0)
        # 座標更新の移動平均係数（小さいほど過去を重視。揺れを抑える）。
        self.declare_parameter('pos_lpf_alpha', 0.3)
        # 視野内非検出を「見えるはず」とみなす最大レンジ [m]（全天球は全周なので角度ゲート無し）。
        self.declare_parameter('visible_range', 8.0)
        # existence ベイズ更新（object_tracker と同式）。
        self.declare_parameter('tp', 0.9)
        self.declare_parameter('fp', 0.2)
        # negative observation 1 回あたりの引き下げ（miss を fp 寄りに更新）。
        self.declare_parameter('miss_tp', 0.2)
        self.declare_parameter('miss_fp', 0.6)
        # 削除しきい値（Dengler et al. の Li<0.25）と、可視化に出す下限。
        self.declare_parameter('delete_thresh', 0.25)
        self.declare_parameter('publish_min_existence', 0.3)
        # 確定（記憶として信頼）に必要な最小検出回数。
        self.declare_parameter('min_hits', 3)
        # negative observation の評価周期 [s]（毎フレームやると過剰に消える）。
        self.declare_parameter('decay_period', 1.0)
        # DB ファイル。空なら in-memory（再起動で消える）。
        default_db = os.path.expanduser('~/.ros/object_memory.sqlite3')
        self.declare_parameter('db_path', default_db)
        # 起動時に既存 DB を消すか（デモを毎回まっさらにしたいとき True）。
        self.declare_parameter('reset_db', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.assoc_dist = float(self.get_parameter('assoc_dist').value)
        self.pos_alpha = float(self.get_parameter('pos_lpf_alpha').value)
        self.visible_range = float(self.get_parameter('visible_range').value)
        self.tp = float(self.get_parameter('tp').value)
        self.fp = float(self.get_parameter('fp').value)
        self.miss_tp = float(self.get_parameter('miss_tp').value)
        self.miss_fp = float(self.get_parameter('miss_fp').value)
        self.delete_thresh = float(self.get_parameter('delete_thresh').value)
        self.pub_min_exist = float(
            self.get_parameter('publish_min_existence').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.decay_period = float(self.get_parameter('decay_period').value)
        db_path = self.get_parameter('db_path').value
        reset_db = bool(self.get_parameter('reset_db').value)

        self.map = None
        self.grid = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._open_db(db_path, reset_db)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.on_map, map_qos)

        self.sub = self.create_subscription(
            TrackedObjects, self.get_parameter('input_topic').value,
            self.on_objects, qos)

        # COCO 細クラス副チャネル: uuid_hex -> (coco_name, stamp_sec)。
        self.fine_class_ttl = float(self.get_parameter('fine_class_ttl').value)
        self.fine_classes = {}
        self.create_subscription(
            DiagnosticArray, self.get_parameter('fine_class_topic').value,
            self.on_fine_classes, qos)

        self.pub_markers = self.create_publisher(
            MarkerArray, '/semantic_memory/markers', 10)

        self.last_decay = None

        self.get_logger().info(
            f'object_memory started. {self.get_parameter("input_topic").value} '
            f'-> SQLite({db_path}) + /semantic_memory/markers '
            f'(map_frame={self.map_frame}, assoc_dist={self.assoc_dist}m)')

    # ── DB ───────────────────────────────────────────────────────────────
    def _open_db(self, db_path, reset_db):
        if db_path:
            os.makedirs(os.path.dirname(os.path.abspath(db_path)), exist_ok=True)
            if reset_db and os.path.exists(db_path):
                os.remove(db_path)
            self.db = sqlite3.connect(db_path)
        else:
            self.db = sqlite3.connect(':memory:')
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS objects (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                label INTEGER,        -- Autoware ObjectClassification.label
                class_name TEXT,      -- 人間可読クラス名
                x REAL, y REAL, z REAL,
                size_x REAL, size_y REAL, size_z REAL,
                existence REAL,
                hits INTEGER,
                last_seen REAL        -- stamp [s]
            )
        """)
        self.db.commit()

    def _all_objects(self):
        cur = self.db.execute(
            "SELECT id, label, class_name, x, y, z, "
            "size_x, size_y, size_z, existence, hits, last_seen FROM objects")
        cols = ['id', 'label', 'class_name', 'x', 'y', 'z',
                'size_x', 'size_y', 'size_z', 'existence', 'hits', 'last_seen']
        return [dict(zip(cols, row)) for row in cur.fetchall()]

    # ── コールバック ──────────────────────────────────────────────────────
    def on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def on_fine_classes(self, msg: DiagnosticArray):
        """object_classifier の COCO 細クラス副チャネルを取り込む。

        status: name=UUID hex, message=COCO名。古いエントリは fine_class_ttl で捨てる。
        """
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for st in msg.status:
            if st.name and st.message:
                self.fine_classes[st.name] = (st.message, stamp_sec)
        cutoff = stamp_sec - self.fine_class_ttl
        self.fine_classes = {k: v for k, v in self.fine_classes.items()
                             if v[1] >= cutoff}

    def _fine_name(self, uuid_bytes):
        """object_id(UUID) に対応する COCO 細クラス名を返す（無ければ None）。"""
        entry = self.fine_classes.get(bytes(uuid_bytes).hex())
        return entry[0] if entry else None

    def on_objects(self, msg: TrackedObjects):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        src_frame = msg.header.frame_id

        # 検出を map 座標へ変換する 2D 同次変換を引く。
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, src_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {self.map_frame}<-{src_frame} unavailable: {ex}',
                throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)

        seen_ids = set()
        for obj in msg.objects:
            p = obj.kinematics.pose_with_covariance.pose.position
            mx = c * p.x - s * p.y + t.x
            my = s * p.x + c * p.y + t.y
            label = (obj.classification[0].label
                     if obj.classification else ObjectClassification.UNKNOWN)
            # class_name は COCO 細クラス(chair 等)を優先し、無ければ Autoware label 名。
            # これで什器を区別して記憶でき、クエリ「椅子」で引ける。
            class_name = (self._fine_name(obj.object_id.uuid)
                          or LABEL_NAMES.get(label, 'unknown'))
            dims = obj.shape.dimensions
            oid = self._associate_and_update(
                label, class_name, mx, my, p.z, dims, stamp_sec)
            seen_ids.add(oid)

        # negative observation は decay_period ごとに評価する。
        if self.last_decay is None or \
                (stamp_sec - self.last_decay) >= self.decay_period:
            self._negative_observation(seen_ids, stamp_sec)
            self.last_decay = stamp_sec

        self.db.commit()
        self._publish_markers(stamp_sec)

    # ── ① data association + existence 引き上げ ──────────────────────────
    def _associate_and_update(self, label, class_name, x, y, z, dims, stamp_sec):
        """同クラス・近傍ゲート内の既存 object を探し、あれば更新・無ければ新規。

        class_name で照合する（chair と diningtable を別物として扱う）。ただし片方が
        'unknown' なら一致とみなす: 同一物体が最初 unknown で登録され、後で chair と
        分類が確定したときに二重登録せず、class_name を具体名へ昇格させるため。
        """
        cur = self.db.execute(
            "SELECT id, x, y, class_name FROM objects")
        best_id, best_d, best_cn = None, self.assoc_dist, None
        for oid, ox, oy, cn in cur.fetchall():
            if cn != class_name and cn != 'unknown' and class_name != 'unknown':
                continue  # 別クラス同士は統合しない
            d = math.hypot(ox - x, oy - y)
            if d < best_d:
                best_d, best_id, best_cn = d, oid, cn

        if best_id is None:
            # 新規 object。existence は object_tracker に倣い控えめに開始。
            self.db.execute(
                "INSERT INTO objects "
                "(label, class_name, x, y, z, size_x, size_y, size_z, "
                "existence, hits, last_seen) VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                (label, class_name,
                 x, y, z, dims.x, dims.y, dims.z, 0.3, 1, stamp_sec))
            return self.db.execute("SELECT last_insert_rowid()").fetchone()[0]

        # 既存 object を更新: 位置は LPF、existence は Bayes 引き上げ。
        row = self.db.execute(
            "SELECT x, y, z, existence, hits FROM objects WHERE id = ?",
            (best_id,)).fetchone()
        ox, oy, oz, exist, hits = row
        a = self.pos_alpha
        nx, ny, nz = ox + a * (x - ox), oy + a * (y - oy), oz + a * (z - oz)
        new_exist = (exist * self.tp) / (exist * self.tp + (1.0 - exist) * self.fp)
        new_exist = min(0.999, max(0.001, new_exist))
        # class_name は具体名へ昇格（unknown → chair 等）。具体名同士なら新しい観測を採る。
        merged_cn = class_name if best_cn == 'unknown' else best_cn
        if class_name != 'unknown':
            merged_cn = class_name
        self.db.execute(
            "UPDATE objects SET x=?, y=?, z=?, size_x=?, size_y=?, size_z=?, "
            "label=?, class_name=?, existence=?, hits=?, last_seen=? WHERE id=?",
            (nx, ny, nz, dims.x, dims.y, dims.z,
             label, merged_cn, new_exist, hits + 1, stamp_sec, best_id))
        return best_id

    # ── ② negative observation + 削除 ──────────────────────────────────────
    def _negative_observation(self, seen_ids, stamp_sec):
        """今フレーム見えなかった object のうち「見えるはず」のものを引き下げる。

        見えるはず = センサ原点から visible_range 以内、かつ間に壁(occupied)が無い。
        それ以外（遠い・遮蔽）は negative にカウントせず存在確率を維持する
        （LTC-Mapping の遮蔽 vs 不在の区別の簡易版）。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.sensor_frame, rclpy.time.Time())
        except TransformException:
            return  # センサ位置不明なら negative 判定を見送る（誤消去回避）
        sx = tf.transform.translation.x
        sy = tf.transform.translation.y

        to_delete = []
        for o in self._all_objects():
            if o['id'] in seen_ids:
                continue
            d = math.hypot(o['x'] - sx, o['y'] - sy)
            if d > self.visible_range:
                continue                       # 遠くて見えない → 維持
            if self._occluded(sx, sy, o['x'], o['y']):
                continue                       # 壁の陰 → 維持
            # 見えるはずなのに非検出 → existence を miss 方向に Bayes 更新。
            exist = o['existence']
            new_exist = (exist * self.miss_tp) / \
                (exist * self.miss_tp + (1.0 - exist) * self.miss_fp)
            new_exist = min(0.999, max(0.001, new_exist))
            if new_exist < self.delete_thresh:
                to_delete.append(o['id'])
            else:
                self.db.execute(
                    "UPDATE objects SET existence=? WHERE id=?",
                    (new_exist, o['id']))

        for oid in to_delete:
            self.db.execute("DELETE FROM objects WHERE id=?", (oid,))
            self.get_logger().info(f'object #{oid} forgotten (existence below thresh)')

    def _occluded(self, sx, sy, ox, oy):
        """センサ(sx,sy)→物体(ox,oy) の視線上に occupied セルがあれば True（遮蔽）。

        map が無い・TF が引けない場合は False（=遮蔽なし扱い）。Bresenham 風の
        等間隔サンプリングで占有セルを拾う。終点近傍は物体自身なので除外する。
        """
        if self.grid is None:
            return False
        info = self.map.info
        res = info.resolution
        ox0, oy0 = info.origin.position.x, info.origin.position.y
        h, w = self.grid.shape
        dist = math.hypot(ox - sx, oy - sy)
        n = max(1, int(dist / res))
        for i in range(1, n):
            f = i / float(n)
            wx = sx + f * (ox - sx)
            wy = sy + f * (oy - sy)
            # 終点 0.3m 手前まで（物体自体のセルを壁と誤認しない）。
            if (1.0 - f) * dist < 0.3:
                break
            cx = int((wx - ox0) / res)
            cy = int((wy - oy0) / res)
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                continue
            if self.grid[cy, cx] >= 50:
                return True
        return False

    # ── 可視化 ───────────────────────────────────────────────────────────
    def _publish_markers(self, stamp_sec):
        arr = MarkerArray()
        # 毎回作り直すので、まず DELETEALL で古いマーカーを消す。
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        mid = 0
        for o in self._all_objects():
            if o['existence'] < self.pub_min_exist or o['hits'] < self.min_hits:
                continue
            # 確信度で色を変える（緑=高 / 黄=低）。
            g = max(0.0, min(1.0, (o['existence'] - self.pub_min_exist) /
                             max(1e-3, 1.0 - self.pub_min_exist)))
            color = ColorRGBA(r=float(1.0 - g), g=float(g), b=0.2, a=0.85)

            box = Marker()
            box.header.frame_id = self.map_frame
            box.ns = 'memory_box'
            box.id = mid
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = o['x']
            box.pose.position.y = o['y']
            box.pose.position.z = o['z']
            box.pose.orientation.w = 1.0
            box.scale.x = max(0.2, o['size_x'])
            box.scale.y = max(0.2, o['size_y'])
            box.scale.z = max(0.2, o['size_z'])
            box.color = color
            arr.markers.append(box)
            mid += 1

            text = Marker()
            text.header.frame_id = self.map_frame
            text.ns = 'memory_label'
            text.id = mid
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = o['x']
            text.pose.position.y = o['y']
            text.pose.position.z = o['z'] + max(0.2, o['size_z']) * 0.5 + 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"#{o['id']} {o['class_name']} {o['existence']:.2f}"
            arr.markers.append(text)
            mid += 1

        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMemoryNode()
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
