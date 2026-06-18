#!/usr/bin/env python3
"""自然語クエリで物体メモリを引き、その物体の手前へ Nav2 で移動するノード。

docs/semantic_object_memory_research.md の ③（Object Goal Navigation + LLM）の MVP。
research.md は最終的に CLIP 埋め込み + LLM で曖昧語を解決する構想だが、MVP では
**固定辞書**（「椅子」→ chair 等）でクラス名を引き、object_memory_node の SQLite DB から
最有力（existence 最大）の object 座標を取り、その手前の approach pose を作って
`NavigateToPose` に投げる。

  クエリ受信 : /semantic_query (std_msgs/String, 例 "椅子")
  DB 参照    : object_memory_node と同じ SQLite ファイル（read-only で開く）
  移動       : nav2_msgs/NavigateToPose（teleop_gui_node.py の送信作法を踏襲）
  結果通知   : /semantic_query/result (std_msgs/String)

独自 .msg は作らない（AGENTS.md の方針）。クエリは String、結果も String で返す。
"""

import math
import os
import sqlite3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros
from tf2_ros import TransformException

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# 日本語・英語のクエリ語 → object_memory_node の class_name。複数語を 1 クラスに寄せる。
# COCO/Autoware の語彙が pedestrian/car 等に丸められているので、それに合わせる。
QUERY_DICT = {
    '人': 'pedestrian', 'ひと': 'pedestrian', '歩行者': 'pedestrian',
    'person': 'pedestrian', 'people': 'pedestrian', 'pedestrian': 'pedestrian',
    '車': 'car', 'くるま': 'car', '自動車': 'car', 'car': 'car',
    'トラック': 'truck', 'truck': 'truck',
    'バス': 'bus', 'bus': 'bus',
    '自転車': 'bicycle', 'bicycle': 'bicycle', 'bike': 'bicycle',
    'バイク': 'motorcycle', 'オートバイ': 'motorcycle', 'motorcycle': 'motorcycle',
    # 什器（COCO 細クラス。object_classifier の副チャネル経由でメモリに記憶される）。
    # Autoware label には無いクラスなので class_name は COCO 名そのままで照合する。
    '椅子': 'chair', 'いす': 'chair', 'イス': 'chair', 'chair': 'chair',
    'ソファ': 'couch', 'ソファー': 'couch', 'couch': 'couch', 'sofa': 'couch',
    '机': 'dining table', '食卓': 'dining table', 'テーブル': 'dining table',
    'table': 'dining table', 'dining table': 'dining table',
    '観葉植物': 'potted plant', '植物': 'potted plant', 'plant': 'potted plant',
    'potted plant': 'potted plant',
    'テレビ': 'tv', 'tv': 'tv', 'モニター': 'tv',
    'ノートパソコン': 'laptop', 'laptop': 'laptop', 'パソコン': 'laptop',
    '冷蔵庫': 'refrigerator', 'refrigerator': 'refrigerator', 'fridge': 'refrigerator',
}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class SemanticQueryNode(Node):

    def __init__(self):
        super().__init__('semantic_query')

        default_db = os.path.expanduser('~/.ros/object_memory.sqlite3')
        self.declare_parameter('db_path', default_db)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        # approach pose を物体からどれだけ手前に置くか [m]（衝突しない停止距離）。
        self.declare_parameter('approach_offset', 0.8)
        # クエリ時に object_memory の確信度がこれ未満の物体は採らない。
        self.declare_parameter('min_existence', 0.4)
        self.declare_parameter('query_topic', '/semantic_query')
        self.declare_parameter('result_topic', '/semantic_query/result')

        self.db_path = self.get_parameter('db_path').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.approach_offset = float(self.get_parameter('approach_offset').value)
        self.min_existence = float(self.get_parameter('min_existence').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.pub_result = self.create_publisher(
            String, self.get_parameter('result_topic').value, 10)
        self.create_subscription(
            String, self.get_parameter('query_topic').value, self.on_query, 10)

        self.get_logger().info(
            f'semantic_query started. {self.get_parameter("query_topic").value} '
            f'-> DB({self.db_path}) -> NavigateToPose')

    def _result(self, text):
        self.get_logger().info(text)
        self.pub_result.publish(String(data=text))

    def on_query(self, msg: String):
        raw = msg.data.strip()
        key = raw.lower()
        class_name = QUERY_DICT.get(raw) or QUERY_DICT.get(key)
        if class_name is None:
            self._result(f'クエリ「{raw}」は辞書に無い（対応: {sorted(set(QUERY_DICT.values()))}）')
            return

        target = self._lookup(class_name)
        if target is None:
            self._result(f'「{raw}」({class_name}) は記憶に無い')
            return

        oid, ox, oy, oz, exist = target
        goal = self._approach_pose(ox, oy)
        if goal is None:
            self._result(f'#{oid} {class_name} の手前 pose を作れない（TF 未取得）')
            return

        self._result(
            f'「{raw}」→ #{oid} {class_name} '
            f'(existence={exist:.2f}, map=({ox:.2f},{oy:.2f})) へ移動')
        self._send_nav_goal(goal)

    def _lookup(self, class_name):
        """DB から該当クラスの existence 最大の object を返す（無ければ None）。"""
        if not os.path.exists(self.db_path):
            self.get_logger().warn(f'DB が無い: {self.db_path}')
            return None
        # object_memory が書き込み中でも壊さないよう read-only で開く。
        db = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True)
        try:
            row = db.execute(
                "SELECT id, x, y, z, existence FROM objects "
                "WHERE class_name = ? AND existence >= ? "
                "ORDER BY existence DESC LIMIT 1",
                (class_name, self.min_existence)).fetchone()
        finally:
            db.close()
        return row

    def _approach_pose(self, ox, oy):
        """物体(ox,oy)から approach_offset だけロボット側に寄った PoseStamped を作る。

        向きはロボット→物体方向（到達後に物体を正面に見る）。ロボット位置が
        引けない場合は None。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'robot TF unavailable: {ex}')
            return None
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y

        dx, dy = ox - rx, oy - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            ux, uy = 1.0, 0.0
        else:
            ux, uy = dx / dist, dy / dist
        # 物体から offset 手前（=ロボット側）に goal を置く。
        gx = ox - ux * self.approach_offset
        gy = oy - uy * self.approach_offset
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
        return pose

    def _send_nav_goal(self, pose):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._result('Nav2 navigate_to_pose サーバが応答しない')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self._result('Nav2 がゴールを拒否した')
            return
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._result('Nav2 移動完了（到達 or 終了）')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticQueryNode()
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
