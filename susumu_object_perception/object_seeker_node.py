#!/usr/bin/env python3
"""指定クラスの物体を「追従」または「探索して接近」するノード。

セマンティック物体メモリの上に載る行動層。2 つのモードを 1 つの状態機械で扱う:

  FOLLOW（人追従, 案7）: tracked_objects から対象クラスの最近傍をリアルタイムに選び、
    その手前（follow_distance）へ周期的に NavigateToPose を再送して動く対象についていく。
    対象を一定時間見失ったら停止する。

  SEARCH（物体探索, 案8）: まずメモリ DB と現フレームを見て対象が居れば接近(APPROACH)。
    居なければ巡回ウェイポイント(patrol_waypoints)を回りながら探し、対象が現れたら
    接近に遷移して報告する。

  状態: IDLE → (SEARCHING) → APPROACHING → FOLLOWING/ARRIVED

入力 : /object_seek (std_msgs/String)。書式「<動詞> <クラス>」または「<クラス>」。
        例「人を追って」「椅子を探して」「人」。動詞で FOLLOW/SEARCH を切替（既定 SEARCH）。
出力 : /object_seek/status (std_msgs/String) 状態・対象の通知。
        NavigateToPose で移動、見失い時は /cmd_vel に停止を出す。

独自 .msg は作らない（AGENTS.md の方針）。クラス語の辞書は semantic_query_node と共有する。
"""

import math
import os
import sqlite3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import TransformException

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from autoware_perception_msgs.msg import TrackedObjects, ObjectClassification

from susumu_object_perception.patrol_waypoints import PATROL_WAYPOINTS
from susumu_object_perception.semantic_query_node import QUERY_DICT


# クラス名 → Autoware label（tracked_objects 側の label 照合用。COCO 細クラス名で来る
# 什器は label=UNKNOWN なので、その場合は class 文字列照合にフォールバックする）。
LABEL_OF = {
    'pedestrian': ObjectClassification.PEDESTRIAN,
    'car': ObjectClassification.CAR,
    'truck': ObjectClassification.TRUCK,
    'bus': ObjectClassification.BUS,
    'bicycle': ObjectClassification.BICYCLE,
    'motorcycle': ObjectClassification.MOTORCYCLE,
}

# FOLLOW を表す動詞（含まれていれば追従モード）。無ければ SEARCH（探して接近）。
FOLLOW_VERBS = ('追', 'follow', 'ついて', 'ついていけ', 'つけ')


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ObjectSeekerNode(Node):

    def __init__(self):
        super().__init__('object_seeker')

        default_db = os.path.expanduser('~/.ros/object_memory.sqlite3')
        self.declare_parameter('db_path', default_db)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter(
            'tracks_topic', '/perception/tracked_objects_classified')
        # 追従/接近で対象の手前にゴールを置く距離 [m]。
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('approach_distance', 0.8)
        # FOLLOW ゴール再送周期 [s]（動く対象を追い直す間隔）。
        self.declare_parameter('follow_resend_sec', 1.0)
        # 対象を見失ったと判断するまでの猶予 [s]。
        self.declare_parameter('lost_timeout_sec', 3.0)
        # 接近完了（ARRIVED）とみなす対象までの距離 [m]。
        self.declare_parameter('arrive_dist', 1.2)
        # SEARCH の 1 ウェイポイント滞在上限 [s]。
        self.declare_parameter('waypoint_timeout_sec', 20.0)

        self.db_path = self.get_parameter('db_path').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.follow_dist = float(self.get_parameter('follow_distance').value)
        self.approach_dist = float(self.get_parameter('approach_distance').value)
        self.follow_resend = float(self.get_parameter('follow_resend_sec').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout_sec').value)
        self.arrive_dist = float(self.get_parameter('arrive_dist').value)
        self.wp_timeout = float(self.get_parameter('waypoint_timeout_sec').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 状態機械。
        self.state = 'IDLE'
        self.mode = None              # 'FOLLOW' or 'SEARCH'
        self.target_class = None      # 例 'pedestrian' / 'chair'
        self.target_label = None      # Autoware label or None（什器）
        self.latest_target_xy = None  # 直近に見た対象の map 座標
        self.last_seen_t = None       # 直近に対象を見た monotonic 時刻
        self.last_goal_t = None       # 直近に FOLLOW ゴールを送った時刻
        self.wp_index = 0
        self.wp_sent_t = None
        self.nav_busy = False
        self.nav_handle = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(
            TrackedObjects, self.get_parameter('tracks_topic').value,
            self.on_tracks, qos)
        self.create_subscription(String, '/object_seek', self.on_command, 10)
        self.pub_status = self.create_publisher(String, '/object_seek/status', 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # 行動ループ（対象の有無・タイムアウト・ゴール再送を周期評価）。
        self.create_timer(0.5, self.on_tick)

        self.get_logger().info(
            'object_seeker started. /object_seek (例「人を追って」「椅子を探して」) '
            f'-> follow/search, tracks={self.get_parameter("tracks_topic").value}')

    def _status(self, text):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=text))

    # ── コマンド受信 ──────────────────────────────────────────────────────
    def on_command(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            return
        if raw in ('停止', 'stop', 'やめ', 'キャンセル'):
            self._stop_all('コマンドで停止')
            return

        # 動詞で FOLLOW / SEARCH を決める。
        mode = 'FOLLOW' if any(v in raw for v in FOLLOW_VERBS) else 'SEARCH'
        # クラス語を辞書で引く（最長一致で語を拾う）。
        class_name = self._parse_class(raw)
        if class_name is None:
            self._status(f'「{raw}」からクラスを特定できない（対応: '
                         f'{sorted(set(QUERY_DICT.values()))}）')
            return

        self.mode = mode
        self.target_class = class_name
        self.target_label = LABEL_OF.get(class_name)  # None なら什器（文字列照合）
        self.latest_target_xy = None
        self.last_seen_t = None
        self.wp_index = 0
        self.wp_sent_t = None
        self.state = 'FOLLOWING' if mode == 'FOLLOW' else 'SEARCHING'
        self._status(f'{mode} 開始: 対象=「{class_name}」')
        # SEARCH はまずメモリで既知座標を確認して即接近を試みる。
        if mode == 'SEARCH':
            self._seed_from_memory()

    def _parse_class(self, raw):
        """クエリ文からクラス名を取り出す。QUERY_DICT のキーで部分一致を見る。"""
        low = raw.lower()
        best = None
        for key, cls in QUERY_DICT.items():
            if key in raw or key in low:
                # 最長キー優先（「自転車」が「車」より優先されるように）。
                if best is None or len(key) > best[0]:
                    best = (len(key), cls)
        return best[1] if best else None

    def _seed_from_memory(self):
        """メモリ DB に対象クラスがあれば latest_target_xy に入れて接近に入る。"""
        if not os.path.exists(self.db_path):
            return
        db = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True)
        try:
            row = db.execute(
                "SELECT x, y FROM objects WHERE class_name = ? "
                "ORDER BY existence DESC LIMIT 1", (self.target_class,)).fetchone()
        finally:
            db.close()
        if row:
            self.latest_target_xy = (row[0], row[1])
            self.state = 'APPROACHING'
            self._status(f'記憶から「{self.target_class}」を発見 → 接近')

    # ── トラック受信（リアルタイム対象位置の更新）─────────────────────────
    def on_tracks(self, msg: TrackedObjects):
        if self.target_class is None or self.state in ('IDLE', 'ARRIVED'):
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException:
            return
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)

        # ロボット位置（最近傍選択用）。
        rob = self._robot_xy()

        best_xy, best_d = None, float('inf')
        for obj in msg.objects:
            label = (obj.classification[0].label
                     if obj.classification else ObjectClassification.UNKNOWN)
            # 対象判定: label が分かるクラスは label で、什器は label=UNKNOWN なので
            # この経路では拾えない（什器は SEARCH のメモリ seed と APPROACH で扱う）。
            if self.target_label is not None and label != self.target_label:
                continue
            if self.target_label is None:
                continue  # 什器はトラック側 label で識別不可。メモリ経由のみ。
            p = obj.kinematics.pose_with_covariance.pose.position
            mx = c * p.x - s * p.y + t.x
            my = s * p.x + c * p.y + t.y
            if rob is not None:
                d = math.hypot(mx - rob[0], my - rob[1])
            else:
                d = 0.0
            if d < best_d:
                best_d, best_xy = d, (mx, my)

        if best_xy is not None:
            self.latest_target_xy = best_xy
            self.last_seen_t = self._now()
            if self.state == 'SEARCHING':
                self.state = 'APPROACHING'
                self._status(f'巡回中に「{self.target_class}」を発見 → 接近')

    # ── 行動ループ ────────────────────────────────────────────────────────
    def on_tick(self):
        if self.state in ('IDLE', 'ARRIVED'):
            return
        rob = self._robot_xy()

        if self.state == 'SEARCHING':
            self._do_search_patrol()
            return

        # APPROACHING / FOLLOWING: 対象の手前へ向かう。
        if self.latest_target_xy is None:
            return
        tx, ty = self.latest_target_xy

        # 見失い判定（FOLLOW のみ。SEARCH 接近中は最後の座標へ向かい続ける）。
        if self.mode == 'FOLLOW' and self.last_seen_t is not None and \
                (self._now() - self.last_seen_t) > self.lost_timeout:
            self.pub_cmd.publish(Twist())  # 止める
            self.state = 'SEARCHING'
            self.wp_index = 0
            self.wp_sent_t = None
            self._status(f'「{self.target_class}」を見失った → 探索に戻る')
            return

        # 到達判定。
        if rob is not None:
            d = math.hypot(tx - rob[0], ty - rob[1])
            if self.state == 'APPROACHING' and d <= self.arrive_dist:
                self._cancel_nav()
                self.pub_cmd.publish(Twist())
                self.state = 'ARRIVED'
                self._status(
                    f'「{self.target_class}」に到達 (距離 {d:.2f}m)')
                return

        # ゴール再送（FOLLOW は周期再送、APPROACH は未送 or 大きく動いたら再送）。
        offset = self.follow_dist if self.mode == 'FOLLOW' else self.approach_dist
        need_send = (self.last_goal_t is None or
                     (self._now() - self.last_goal_t) >= self.follow_resend)
        if need_send and rob is not None:
            self._send_approach_goal(rob, (tx, ty), offset)

    def _do_search_patrol(self):
        """巡回ウェイポイントを回りながら対象を探す。"""
        if self.nav_busy:
            # タイムアウトしたら次のウェイポイントへ。
            if self.wp_sent_t is not None and \
                    (self._now() - self.wp_sent_t) > self.wp_timeout:
                self._cancel_nav()
                self.wp_index = (self.wp_index + 1) % len(PATROL_WAYPOINTS)
            return
        if not self.nav_client.server_is_ready():
            return
        x, y = PATROL_WAYPOINTS[self.wp_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self.nav_busy = True
        self.wp_sent_t = self._now()
        self.nav_client.send_goal_async(goal).add_done_callback(
            self._on_patrol_response)

    def _on_patrol_response(self, future):
        h = future.result()
        if not h.accepted:
            self.nav_busy = False
            self.wp_index = (self.wp_index + 1) % len(PATROL_WAYPOINTS)
            return
        self.nav_handle = h
        h.get_result_async().add_done_callback(self._on_patrol_result)

    def _on_patrol_result(self, future):
        self.nav_busy = False
        self.nav_handle = None
        # まだ探索中なら次のウェイポイントへ。
        if self.state == 'SEARCHING':
            self.wp_index = (self.wp_index + 1) % len(PATROL_WAYPOINTS)

    # ── 接近ゴール送信 ────────────────────────────────────────────────────
    def _send_approach_goal(self, rob, target, offset):
        tx, ty = target
        dx, dy = tx - rob[0], ty - rob[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            ux, uy = 1.0, 0.0
        else:
            ux, uy = dx / dist, dy / dist
        gx = tx - ux * offset
        gy = ty - uy * offset
        yaw = math.atan2(dy, dx)
        qx, qy, qz, qw = yaw_to_quat(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        if not self.nav_client.server_is_ready():
            return
        # 既存ゴールはキャンセルして新しい追従先へ差し替える。
        self._cancel_nav()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.last_goal_t = self._now()
        self.nav_client.send_goal_async(goal).add_done_callback(
            self._on_approach_response)

    def _on_approach_response(self, future):
        h = future.result()
        if h.accepted:
            self.nav_handle = h

    # ── ユーティリティ ────────────────────────────────────────────────────
    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _cancel_nav(self):
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()
            self.nav_handle = None
        self.nav_busy = False

    def _stop_all(self, reason):
        self._cancel_nav()
        self.pub_cmd.publish(Twist())
        self.state = 'IDLE'
        self.mode = None
        self.target_class = None
        self._status(f'停止: {reason}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSeekerNode()
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
