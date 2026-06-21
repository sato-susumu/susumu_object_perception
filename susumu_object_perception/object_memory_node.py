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


MEMORY_CLASS_NORMALIZATION = {
    # COCO は観葉植物の鉢を vase と見ることが多い。セマンティック地図では
    # 「植物」として扱った方が重複登録もクエリも安定する。
    'vase': 'potted plant',
    'person': 'pedestrian',
    'sofa': 'couch',
    'table': 'dining table',
    'fridge': 'refrigerator',
    # Webots indoor の PottedTree は、全天球クロップで樹冠が umbrella として
    # 出ることがある。静的認識メモリでは plant として扱う。
    'umbrella': 'potted plant',
}

SEMANTIC_CLASS_KEYS = {
    'potted plant': 'plant',
    'vase': 'plant',
    'couch': 'couch',
    'sofa': 'couch',
    'dining table': 'table',
    'table': 'table',
    'refrigerator': 'refrigerator',
    'fridge': 'refrigerator',
    'pedestrian': 'pedestrian',
    'person': 'pedestrian',
}


# Recognition-task oriented static-object sanity checks. These are disabled by
# default because moving-object memory should accept partial observations.
STATIC_CLASS_GEOMETRY_RULES = {
    # key: semantic_class_key -> (min planar area [m^2], max planar area [m^2], max aspect)
    'plant': (0.04, 0.65, 3.0),
    'couch': (0.45, 3.0, 3.0),
    'table': (0.25, 3.5, 5.0),
    'chair': (0.12, 0.6, 4.0),
    'refrigerator': (0.15, 2.5, 5.0),
}


def normalize_class_name(name):
    name = str(name or 'unknown').strip().lower().replace('_', ' ')
    while '  ' in name:
        name = name.replace('  ', ' ')
    return MEMORY_CLASS_NORMALIZATION.get(name, name)


def semantic_class_key(name):
    name = normalize_class_name(name)
    return SEMANTIC_CLASS_KEYS.get(name, name)


def string_list(value):
    if value is None:
        return []
    if isinstance(value, str):
        return [value] if value else []
    try:
        return [str(v) for v in value if str(v)]
    except TypeError:
        return []


def class_name_list(value):
    names = []
    for item in string_list(value):
        for part in item.replace('|', ',').split(','):
            part = part.strip()
            if part:
                names.append(normalize_class_name(part))
    return names


def class_distance_map(value):
    distances = {}
    for item in string_list(value):
        for part in item.split(','):
            part = part.strip()
            if not part:
                continue
            if '=' in part:
                name, dist = part.split('=', 1)
            elif ':' in part:
                name, dist = part.split(':', 1)
            else:
                continue
            try:
                distances[semantic_class_key(name)] = float(dist)
            except ValueError:
                continue
    return distances


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
        # 認識タスクの最終成果物では、tracker の幾何推定だけで付いた pedestrian 等を
        # 記憶せず、YOLO の fine class が来た物体だけを記憶する。
        self.declare_parameter('require_fine_class', False)
        self.declare_parameter('min_fine_conf', 0.0)
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
        # 静的物体の記憶用: 物体中心が保存地図の占有セルから離れすぎている場合は
        # 背景誤分類や空間ゴーストとみなし、DB に登録しない。動的物体記憶では False のまま使う。
        self.declare_parameter('require_map_support', False)
        self.declare_parameter('map_support_dist', 0.55)
        self.declare_parameter('map_support_class_distances', '')
        self.declare_parameter('map_support_occupied_threshold', 50)
        # 静的物体の認識成果物向け: クラスごとの平面形状として明らかに不自然な
        # 候補を DB 登録前に落とす。評価後処理ではなく、認識メモリ側の誤登録抑制。
        self.declare_parameter('static_class_geometry_filter', False)
        # 静的物体の認識成果物向け: LiDAR クラスタ分割や視点差で同一物体が
        # 近接した複数 DB object になった場合、同じ semantic class の近傍候補を
        # DB 内で統合する。通常の動的物体メモリでは 0.0 のまま無効。
        self.declare_parameter('static_duplicate_merge_dist', 0.0)
        # 静的物体の認識成果物向け: YOLO が chair/couch/table など近い語彙で
        # 揺れた場合も、指定した互換クラス群の近接候補を同一物体として統合する。
        # 例: ["chair,couch,dining table"]。
        self.declare_parameter('static_cross_class_merge_dist', 0.0)
        self.declare_parameter('static_compatible_class_groups', '')
        # 統合候補の hits/existence が同等の場合の class_name 優先順。
        self.declare_parameter('static_merge_class_priority', '')
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
        self.require_fine_class = bool(
            self.get_parameter('require_fine_class').value)
        self.min_fine_conf = float(self.get_parameter('min_fine_conf').value)
        self.visible_range = float(self.get_parameter('visible_range').value)
        self.tp = float(self.get_parameter('tp').value)
        self.fp = float(self.get_parameter('fp').value)
        self.miss_tp = float(self.get_parameter('miss_tp').value)
        self.miss_fp = float(self.get_parameter('miss_fp').value)
        self.delete_thresh = float(self.get_parameter('delete_thresh').value)
        self.pub_min_exist = float(
            self.get_parameter('publish_min_existence').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.require_map_support = bool(
            self.get_parameter('require_map_support').value)
        self.map_support_dist = float(
            self.get_parameter('map_support_dist').value)
        self.map_support_class_distances = class_distance_map(
            self.get_parameter('map_support_class_distances').value)
        self.map_support_occ_thresh = int(
            self.get_parameter('map_support_occupied_threshold').value)
        self.static_class_geometry_filter = bool(
            self.get_parameter('static_class_geometry_filter').value)
        self.static_duplicate_merge_dist = float(
            self.get_parameter('static_duplicate_merge_dist').value)
        self.static_cross_class_merge_dist = float(
            self.get_parameter('static_cross_class_merge_dist').value)
        self.static_compatible_class_groups = self._parse_class_groups(
            self.get_parameter('static_compatible_class_groups').value)
        self.static_merge_class_priority = class_name_list(
            self.get_parameter('static_merge_class_priority').value)
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

        # COCO 細クラス副チャネル: uuid_hex -> (coco_name, conf, stamp_sec)。
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
            f'(map_frame={self.map_frame}, assoc_dist={self.assoc_dist}m, '
            f'require_fine_class={self.require_fine_class}, '
            f'require_map_support={self.require_map_support}, '
            f'map_support_class_distances={self.map_support_class_distances}, '
            f'static_class_geometry_filter={self.static_class_geometry_filter}, '
            f'static_duplicate_merge_dist={self.static_duplicate_merge_dist}, '
            f'static_cross_class_merge_dist={self.static_cross_class_merge_dist})')

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

        status: name=UUID hex, message=COCO名。values に conf が入る。
        古いエントリは fine_class_ttl で捨てる。
        """
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for st in msg.status:
            if not st.name:
                continue
            if not st.message:
                self.fine_classes.pop(st.name, None)
                continue
            conf = 0.0
            for kv in st.values:
                if kv.key == 'conf':
                    try:
                        conf = float(kv.value)
                    except ValueError:
                        conf = 0.0
                    break
            self.fine_classes[st.name] = (
                normalize_class_name(st.message), conf, stamp_sec)
        cutoff = stamp_sec - self.fine_class_ttl
        self.fine_classes = {k: v for k, v in self.fine_classes.items()
                             if v[2] >= cutoff}

    def _fine_class(self, uuid_bytes):
        """object_id(UUID) に対応する (COCO細クラス名, conf) を返す。"""
        entry = self.fine_classes.get(bytes(uuid_bytes).hex())
        return (entry[0], entry[1]) if entry else (None, 0.0)

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
            fine_name, fine_conf = self._fine_class(obj.object_id.uuid)
            if self.require_fine_class and (
                    fine_name is None or fine_conf < self.min_fine_conf):
                continue
            # class_name は COCO 細クラス(chair 等)を優先し、無ければ Autoware label 名。
            # これで什器を区別して記憶でき、クエリ「椅子」で引ける。
            class_name = normalize_class_name(
                fine_name or LABEL_NAMES.get(label, 'unknown'))
            if self.require_map_support and not self._has_map_support(
                    mx, my, class_name):
                continue
            dims = obj.shape.dimensions
            if self.static_class_geometry_filter and \
                    not self._passes_static_class_geometry(class_name, dims):
                continue
            oid = self._associate_and_update(
                label, class_name, mx, my, p.z, dims, stamp_sec)
            seen_ids.add(oid)

        # negative observation は decay_period ごとに評価する。
        if self.last_decay is None or \
                (stamp_sec - self.last_decay) >= self.decay_period:
            self._negative_observation(seen_ids, stamp_sec)
            self.last_decay = stamp_sec
        if self.static_duplicate_merge_dist > 0.0 or \
                self.static_cross_class_merge_dist > 0.0:
            self._merge_static_duplicates()

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
        class_key = semantic_class_key(class_name)
        for oid, ox, oy, cn in cur.fetchall():
            cn_key = semantic_class_key(cn)
            if cn_key != class_key and cn != 'unknown' and class_name != 'unknown':
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

    def _passes_static_class_geometry(self, class_name, dims):
        """クラスごとの平面サイズとして明らかに不自然な候補を落とす。

        YOLO がクロップ内の背景を拾うと、LiDAR 側は壁片や家具の一部クラスタの
        まま `potted plant` / `couch` などとして記憶される。静的認識タスクでは
        そうした候補を登録前に抑える。
        """
        rule = STATIC_CLASS_GEOMETRY_RULES.get(semantic_class_key(class_name))
        if rule is None:
            return False
        sx = abs(float(dims.x))
        sy = abs(float(dims.y))
        if sx <= 1e-3 or sy <= 1e-3:
            return False
        area = sx * sy
        aspect = max(sx, sy) / max(1e-3, min(sx, sy))
        min_area, max_area, max_aspect = rule
        return min_area <= area <= max_area and aspect <= max_aspect

    def _merge_static_duplicates(self):
        """近接した同一/互換 semantic class の DB object を統合する。

        認識レビューでは、同じ静的物体が LiDAR の過分割や視点差で複数の
        memory object として残ることがある。さらに屋内家具は、同じ物体が
        chair/couch/table のような近い COCO 語彙へ揺れる場合がある。これは
        評価後処理ではなく、map 上の物体記憶を作る段階で同一物体候補として
        統合する。
        """
        while True:
            objects = self._all_objects()
            components = self._static_merge_components(objects)
            components = [c for c in components if len(c) >= 2]
            if not components:
                return
            # 大きい成分から畳む。1回畳んだ後に重心が動いて別成分と繋がる
            # 場合があるため、外側の while で再評価する。
            components.sort(key=len, reverse=True)
            self._merge_object_component(components[0])

    def _static_merge_components(self, objects):
        n = len(objects)
        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i, j):
            ri = find(i)
            rj = find(j)
            if ri != rj:
                parent[rj] = ri

        for i, a in enumerate(objects):
            for j in range(i + 1, n):
                b = objects[j]
                limit = self._static_merge_limit(
                    a['class_name'], b['class_name'])
                if limit <= 0.0:
                    continue
                d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                if d < limit:
                    union(i, j)

        grouped = {}
        for i, o in enumerate(objects):
            grouped.setdefault(find(i), []).append(o)
        return list(grouped.values())

    def _merge_object_component(self, objects):
        """DB 上の object 連結成分を hit-weighted average で 1 件へ畳む。"""
        objects = sorted(
            objects, key=lambda o: (int(o['hits']), float(o['existence'])),
            reverse=True)
        base = objects[0]
        total_hits = max(1, sum(max(1, int(o['hits'])) for o in objects))
        weights = [max(1, int(o['hits'])) / float(total_hits)
                   for o in objects]
        class_name, label = self._merged_component_class_and_label(objects)
        existence = 0.001
        for o in objects:
            existence = self._combined_existence(
                existence, float(o['existence']))
        self.db.execute(
            "UPDATE objects SET x=?, y=?, z=?, size_x=?, size_y=?, size_z=?, "
            "label=?, class_name=?, existence=?, hits=?, last_seen=? WHERE id=?",
            (sum(w * o['x'] for w, o in zip(weights, objects)),
             sum(w * o['y'] for w, o in zip(weights, objects)),
             sum(w * o['z'] for w, o in zip(weights, objects)),
             sum(w * o['size_x'] for w, o in zip(weights, objects)),
             sum(w * o['size_y'] for w, o in zip(weights, objects)),
             sum(w * o['size_z'] for w, o in zip(weights, objects)),
             label,
             class_name,
             existence,
             total_hits,
             max(o['last_seen'] for o in objects),
             base['id']))
        for o in objects[1:]:
            self.db.execute("DELETE FROM objects WHERE id=?", (o['id'],))

    def _static_merge_limit(self, class_a, class_b):
        key_a = semantic_class_key(class_a)
        key_b = semantic_class_key(class_b)
        if key_a == 'unknown' or key_b == 'unknown':
            return 0.0
        if key_a == key_b:
            return self.static_duplicate_merge_dist
        if self.static_cross_class_merge_dist <= 0.0:
            return 0.0
        for group in self.static_compatible_class_groups:
            if key_a in group and key_b in group:
                return self.static_cross_class_merge_dist
        return 0.0

    def _parse_class_groups(self, value):
        groups = []
        for item in string_list(value):
            for group_text in item.split(';'):
                parts = group_text.replace('|', ',').split(',')
                keys = {semantic_class_key(p) for p in parts if p.strip()}
                keys.discard('unknown')
                if len(keys) >= 2:
                    groups.append(keys)
        return groups

    def _merged_component_class_and_label(self, objects):
        known = [o for o in objects if o['class_name'] != 'unknown']
        if not known:
            return 'unknown', ObjectClassification.UNKNOWN

        def key(o):
            rank = self._class_priority_rank(o['class_name'])
            return (int(o['hits']), float(o['existence']), -rank)

        chosen = max(known, key=key)
        return chosen['class_name'], chosen['label']

    def _class_priority_rank(self, class_name):
        name = normalize_class_name(class_name)
        if name in self.static_merge_class_priority:
            return self.static_merge_class_priority.index(name)
        key = semantic_class_key(name)
        for i, item in enumerate(self.static_merge_class_priority):
            if semantic_class_key(item) == key:
                return i
        return len(self.static_merge_class_priority)

    @staticmethod
    def _combined_existence(pa, pb):
        """同一物体として統合する2候補の存在確率を合成する。"""
        pa = min(0.999, max(0.001, float(pa)))
        pb = min(0.999, max(0.001, float(pb)))
        return min(0.999, max(pa, pb, 1.0 - (1.0 - pa) * (1.0 - pb)))

    def _map_support_dist_for(self, class_name):
        return self.map_support_class_distances.get(
            semantic_class_key(class_name), self.map_support_dist)

    def _has_map_support(self, x, y, class_name='unknown'):
        """物体中心の近くに occupied セルがあるかを見る。

        認識タスクの静的物体メモリ向けのゲート。地図から大きく離れた点は
        画像クロップ内の背景を誤分類した可能性が高いため登録しない。
        """
        if self.grid is None or self.map is None:
            return False
        info = self.map.info
        res = info.resolution
        ox0, oy0 = info.origin.position.x, info.origin.position.y
        h, w = self.grid.shape
        cx = int((x - ox0) / res)
        cy = int((y - oy0) / res)
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return False
        support_dist = self._map_support_dist_for(class_name)
        if support_dist < 0.0:
            return True
        radius_cells = max(1, int(math.ceil(support_dist / res)))
        min_x = max(0, cx - radius_cells)
        max_x = min(w - 1, cx + radius_cells)
        min_y = max(0, cy - radius_cells)
        max_y = min(h - 1, cy + radius_cells)
        for yy in range(min_y, max_y + 1):
            for xx in range(min_x, max_x + 1):
                if self.grid[yy, xx] < self.map_support_occ_thresh:
                    continue
                wx = ox0 + (xx + 0.5) * res
                wy = oy0 + (yy + 0.5) * res
                if math.hypot(wx - x, wy - y) <= support_dist:
                    return True
        return False

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
