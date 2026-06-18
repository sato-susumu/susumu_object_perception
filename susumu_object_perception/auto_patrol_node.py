#!/usr/bin/env python3
"""SLAM 地図から巡回ルートを自動生成して Nav2 で巡回するノード。

固定ウェイポイント（patrol_waypoints）とは別に、SLAM が育てる /map(OccupancyGrid) の
自由空間からルートを毎回自動算出して回る。city_robot.wbt のように事前マップが無く、
世界の形が分からない環境で「地図を見て経路を決めて巡回する」用途。

アルゴリズム:
  1. /map(OccupancyGrid) を購読。free(=0) セルを抽出。
  2. ロボット周囲が安全マージン(robot_radius)分クリアな free セルを「安全セル」とする
     （未知/占有セルに近すぎる点は候補から外す）。
  3. 安全セルを sample_step[m] グリッドに間引いて候補ウェイポイントにする。
  4. ロボット現在位置(TF map->robot_frame)を起点に最近傍貪欲法で巡回順を決める（簡易TSP）。
  5. Nav2 FollowWaypoints に投げる。完走したら、地図が育って候補が増えていれば
     再計算して次の周回を出す（replan_on_finish）。

入力 : /map (nav_msgs/OccupancyGrid)、TF map->robot_frame
出力 : FollowWaypoints アクション（/follow_waypoints）。/auto_patrol/markers に可視化。
       /auto_patrol/status (std_msgs/String) で状態通知。

独自 .msg は作らない（AGENTS.md の方針）。Nav2 標準 FollowWaypoints を使う。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

import tf2_ros
from tf2_ros import TransformException

from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import FollowWaypoints, Spin
from visualization_msgs.msg import Marker, MarkerArray


class AutoPatrolNode(Node):

    def __init__(self):
        super().__init__('auto_patrol')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        # 候補ウェイポイントを間引くグリッド間隔 [m]（密だと巡回が長すぎる）。
        self.declare_parameter('sample_step', 1.5)
        # 候補点の周囲この半径 [m] が free でないと採用しない（壁際を避ける安全マージン）。
        self.declare_parameter('robot_radius', 0.35)
        # 1 周回の最大ウェイポイント数（多すぎると 1 周が終わらない）。
        self.declare_parameter('max_waypoints', 24)
        # 地図がこのセル数以上 free にならないと巡回を始めない（起動直後の空マップ対策）。
        self.declare_parameter('min_free_cells', 200)
        # 完走後に地図を読み直してルートを再生成し次の周回を出すか。
        self.declare_parameter('replan_on_finish', True)
        # 自動で巡回を開始するか（False なら /auto_patrol/start を待つ）。
        self.declare_parameter('autostart', True)
        # 巡回開始を待つ起動猶予 [s]（SLAM が地図を出し始めるまで待つ）。
        self.declare_parameter('start_delay_sec', 8.0)
        # 地図が育たないとき、その場回転して周囲をスキャンし地図を広げる
        # （鶏卵問題対策: 巡回には地図が要るが、地図を育てるには動く必要がある）。
        # 回転は Nav2 Spin アクションで行う（生 /cmd_vel は velocity_smoother と
        # 競合して打ち消し合うため使わない）。
        self.declare_parameter('bootstrap_rotate', True)
        # 1 回の bootstrap で回す角度 [rad]（既定 ~360°）と許容時間 [s]。
        self.declare_parameter('bootstrap_yaw', 6.283)
        self.declare_parameter('bootstrap_time_allowance', 20.0)
        # bootstrap を繰り返す最大回数（これでも地図が育たなければ諦めて緩く再試行）。
        self.declare_parameter('bootstrap_max_tries', 3)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.sample_step = float(self.get_parameter('sample_step').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.max_waypoints = int(self.get_parameter('max_waypoints').value)
        self.min_free_cells = int(self.get_parameter('min_free_cells').value)
        self.replan_on_finish = bool(
            self.get_parameter('replan_on_finish').value)
        self.start_delay_sec = float(
            self.get_parameter('start_delay_sec').value)
        self.bootstrap_rotate = bool(
            self.get_parameter('bootstrap_rotate').value)
        self.bootstrap_yaw = float(
            self.get_parameter('bootstrap_yaw').value)
        self.bootstrap_time_allowance = float(
            self.get_parameter('bootstrap_time_allowance').value)
        self.bootstrap_max_tries = int(
            self.get_parameter('bootstrap_max_tries').value)

        self._map = None
        self._busy = False          # FollowWaypoints 実行中か
        self._goal_handle = None
        self._bootstrapping = False
        self._bootstrap_tries = 0

        # TF（map->robot 起点の取得）。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /map は latched 相当（TRANSIENT_LOCAL）で来るので合わせる。
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos)

        # 外部からの開始/停止トリガ（autostart=False のとき or 手動で回したいとき）。
        self.create_subscription(
            String, '/auto_patrol/start', self._on_start_cmd, 1)

        self._status_pub = self.create_publisher(
            String, '/auto_patrol/status', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/auto_patrol/markers', 1)

        self._nav_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')
        # bootstrap 回転は Nav2 Spin で行う（smoother と競合しない）。
        self._spin_client = ActionClient(self, Spin, 'spin')

        autostart = bool(self.get_parameter('autostart').value)
        if autostart:
            # SLAM が地図を出し始めるまで待ってから 1 回目を蹴る。
            self._start_timer = self.create_timer(
                self.start_delay_sec, self._kick_start_once)

        self.get_logger().info(
            'auto_patrol started '
            f'(step={self.sample_step}m radius={self.robot_radius}m '
            f'max_wp={self.max_waypoints} autostart={autostart})')

    # ---- callbacks -------------------------------------------------------

    def _on_map(self, msg):
        self._map = msg

    def _on_start_cmd(self, msg):
        self._publish_status('start command received')
        self.start_patrol()

    def _kick_start_once(self):
        # one-shot: タイマを止めて巡回を開始。
        self._start_timer.cancel()
        self.start_patrol()

    # ---- patrol driver ---------------------------------------------------

    def start_patrol(self):
        if self._busy or self._bootstrapping:
            return
        if self._map is None:
            self._publish_status('no map yet; waiting')
            # 地図到着後に再試行。
            self.create_timer(2.0, self._retry_start_once)
            return

        waypoints = self._plan_waypoints()
        if not waypoints:
            # 鶏卵問題: 地図が育っていないと候補が無い。その場回転で周囲を
            # スキャンして地図を広げてから再計画する。
            if self.bootstrap_rotate:
                self._begin_bootstrap()
            else:
                self._publish_status('no reachable waypoints yet; retrying')
                self.create_timer(2.0, self._retry_start_once)
            return

        self._publish_markers(waypoints)
        self._send_follow_waypoints(waypoints)

    # ---- bootstrap rotation（地図ブートストラップ; Nav2 Spin）-------------

    def _begin_bootstrap(self):
        """Nav2 Spin で 1 周回転し、周囲を LiDAR スキャンして地図を広げる。"""
        if self._bootstrapping:
            return
        if self._bootstrap_tries >= self.bootstrap_max_tries:
            # これ以上回っても育たない。緩く再試行に切り替える。
            self._publish_status(
                'bootstrap exhausted; retrying plan with current map')
            self.create_timer(2.0, self._retry_start_once)
            return
        if not self._spin_client.wait_for_server(timeout_sec=5.0):
            self._publish_status('spin server unavailable; retrying')
            self.create_timer(3.0, self._retry_start_once)
            return

        self._bootstrapping = True
        self._bootstrap_tries += 1
        self._publish_status(
            f'bootstrap: spinning to grow the map '
            f'(try {self._bootstrap_tries}/{self.bootstrap_max_tries})')

        goal = Spin.Goal()
        goal.target_yaw = self.bootstrap_yaw
        sec = int(self.bootstrap_time_allowance)
        goal.time_allowance.sec = sec
        goal.time_allowance.nanosec = int(
            (self.bootstrap_time_allowance - sec) * 1e9)
        fut = self._spin_client.send_goal_async(goal)
        fut.add_done_callback(self._on_spin_goal_response)

    def _on_spin_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self._bootstrapping = False
            self._publish_status('spin goal rejected; retrying')
            self.create_timer(2.0, self._retry_start_once)
            return
        gh.get_result_async().add_done_callback(self._on_spin_result)

    def _on_spin_result(self, future):
        self._bootstrapping = False
        # 回転で地図が育ったはず。再計画（候補が出れば巡回、まだなら再度 spin）。
        self._publish_status('bootstrap spin done; replanning')
        self.create_timer(1.0, self._retry_start_once)

    def _retry_start_once(self):
        # create_timer は周期タイマなので 1 回で止める運用にする。
        for t in list(self.timers):
            if t.callback == self._retry_start_once:
                t.cancel()
        if not self._busy:
            self.start_patrol()

    def _plan_waypoints(self):
        """地図の free 空間から安全な候補点を作り、最近傍順に並べて返す。"""
        m = self._map
        info = m.info
        w, h = info.width, info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        data = m.data

        free_total = sum(1 for v in data if 0 <= v <= 20)
        if free_total < self.min_free_cells:
            self.get_logger().info(
                f'map too small ({free_total} free cells < '
                f'{self.min_free_cells}); skip')
            return []

        # 安全半径（セル数）。この範囲が全部 free でないと候補にしない。
        rad_cells = max(1, int(math.ceil(self.robot_radius / res)))
        # サンプリング間隔（セル数）。
        step_cells = max(1, int(round(self.sample_step / res)))

        def idx(cx, cy):
            return cy * w + cx

        def is_clear(cx, cy):
            for dy in range(-rad_cells, rad_cells + 1):
                ny = cy + dy
                if ny < 0 or ny >= h:
                    return False
                for dx in range(-rad_cells, rad_cells + 1):
                    nx = cx + dx
                    if nx < 0 or nx >= w:
                        return False
                    v = data[idx(nx, ny)]
                    # free は 0 付近のみ。未知(-1)・占有(>20)が混じれば不採用。
                    if v < 0 or v > 20:
                        return False
            return True

        candidates = []
        for cy in range(rad_cells, h - rad_cells, step_cells):
            for cx in range(rad_cells, w - rad_cells, step_cells):
                v = data[idx(cx, cy)]
                if v < 0 or v > 20:
                    continue
                if not is_clear(cx, cy):
                    continue
                wx = ox + (cx + 0.5) * res
                wy = oy + (cy + 0.5) * res
                candidates.append((wx, wy))

        if not candidates:
            return []

        # 起点（ロボット現在位置）。取れなければ原点。
        sx, sy = self._robot_xy()

        # 最近傍貪欲法で巡回順を決める（簡易 TSP）。
        ordered = self._greedy_order(candidates, sx, sy)
        if len(ordered) > self.max_waypoints:
            # 多すぎる場合は等間隔に間引いて 1 周の長さを抑える。
            stepf = len(ordered) / float(self.max_waypoints)
            ordered = [ordered[int(i * stepf)]
                       for i in range(self.max_waypoints)]
        return ordered

    def _greedy_order(self, points, sx, sy):
        remaining = list(points)
        ordered = []
        cx, cy = sx, sy
        while remaining:
            best_i = 0
            best_d = float('inf')
            for i, (px, py) in enumerate(remaining):
                d = (px - cx) ** 2 + (py - cy) ** 2
                if d < best_d:
                    best_d = d
                    best_i = i
            nx, ny = remaining.pop(best_i)
            ordered.append((nx, ny))
            cx, cy = nx, ny
        return ordered

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return (tf.transform.translation.x,
                    tf.transform.translation.y)
        except TransformException:
            return (0.0, 0.0)

    # ---- Nav2 FollowWaypoints --------------------------------------------

    def _send_follow_waypoints(self, waypoints):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._publish_status('follow_waypoints server unavailable')
            self.create_timer(3.0, self._retry_start_once)
            return

        goal = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        for (wx, wy) in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = now
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            goal.poses.append(ps)

        self._busy = True
        self._publish_status(f'patrolling {len(waypoints)} waypoints')
        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self._busy = False
            self._publish_status('goal rejected')
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback):
        cur = feedback.feedback.current_waypoint
        self._publish_status(f'heading to waypoint #{cur}')

    def _on_result(self, future):
        self._busy = False
        self._goal_handle = None
        self._publish_status('patrol lap finished')
        if self.replan_on_finish:
            # 地図が育っているはずなので読み直して次の周回。
            self.create_timer(2.0, self._replan_once)

    def _replan_once(self):
        for t in list(self.timers):
            if t.callback == self._replan_once:
                t.cancel()
        self.start_patrol()

    # ---- viz / status ----------------------------------------------------

    def _publish_status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _publish_markers(self, waypoints):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = self.map_frame
        line.header.stamp = now
        line.ns = 'auto_patrol_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.9)
        line.pose.orientation.w = 1.0
        for (wx, wy) in waypoints:
            line.points.append(Point(x=wx, y=wy, z=0.05))
        arr.markers.append(line)

        for i, (wx, wy) in enumerate(waypoints):
            sph = Marker()
            sph.header.frame_id = self.map_frame
            sph.header.stamp = now
            sph.ns = 'auto_patrol_wp'
            sph.id = i + 1
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = wx
            sph.pose.position.y = wy
            sph.pose.position.z = 0.05
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.2
            sph.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=0.9)
            arr.markers.append(sph)

        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AutoPatrolNode()
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
