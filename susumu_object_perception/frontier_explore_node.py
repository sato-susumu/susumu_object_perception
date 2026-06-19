#!/usr/bin/env python3
"""フロンティアベース自律探索で未知環境の地図を作るノード。

事前地図の無いシミュレータ環境で「動きながら地図を作る」最終手段。SLAM(slam_toolbox)
が育てる /map(OccupancyGrid) を見て、未知領域との境界（フロンティア）へ Nav2 で
向かい続け、フロンティアが尽きたら探索完了として地図を保存する。

アルゴリズム（Yamauchi 1997 frontier-based exploration / explore_lite と同系）:
  1. /map を購読。
  2. フロンティアセル = free(0) かつ 4近傍に unknown(-1) を持つセル を抽出。
  3. フロンティアセルを連結成分でクラスタリング（BFS）。小さすぎる塊は捨てる。
  4. 各クラスタの重心を候補ゴールとし、コスト（距離 - 利得*サイズ）最小を選ぶ。
  5. NavigateToPose でそこへ向かう。到達/中断したら再評価して次のフロンティアへ。
  6. フロンティアが規定回数連続で見つからなければ探索完了 → map_saver で保存。

鶏卵問題（地図が無いとフロンティアも無い）対策に Nav2 Spin で初期スキャンする。
独自 .msg は作らない。Nav2 標準 NavigateToPose / Spin / map_saver を使う。

入力 : /map (nav_msgs/OccupancyGrid)、TF map->robot_frame
出力 : NavigateToPose / Spin アクション、/frontier_explore/markers 可視化、
       /frontier_explore/status (std_msgs/String)。完了時 maps/<map_name> を保存。
"""

import math
import os
import subprocess
from collections import deque

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
from nav2_msgs.action import NavigateToPose, Spin
from visualization_msgs.msg import Marker, MarkerArray


# OccupancyGrid のセル意味。
FREE_MAX = 20       # 0..20 を free 扱い（slam_toolbox の occ しきい値に余裕を持たせる）
OCC_MIN = 65        # 65.. を occupied 扱い
UNKNOWN = -1


class FrontierExploreNode(Node):

    def __init__(self):
        super().__init__('frontier_explore')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        # フロンティアクラスタの最小セル数（ノイズ除去）。小さすぎる散在フロンティア
        # も拾うため小さめ（3）。大きいと地図外周の巨大クラスタしか残らず探索が早期に
        # 終わってしまう。
        # 小さいフロンティア（ノイズ由来の小片）は無視して安定したゴールだけ追う。
        # 小さいと細かいノイズへ向かい、ロボットが頻繁に向きを変えて slam マッチが乱れ
        # 斜めノイズ→地図分断の原因に。8 セル(=0.4m級)以上のまとまりだけ狙う。
        self.declare_parameter('min_frontier_cells', 8)
        # 情報利得スコア score = size_weight*log(size) - dist_weight*distance。
        # 【未開拓を広く開拓する方針】最優先は「未踏領域を残さず広く開拓」すること
        # （/scan が疎でも気にしない）。大きな未踏領域(size 大)を強く優先し、多少遠くても
        # 向かわせる。size_weight を大きく・dist_weight を小さくして、近場に滞留せず
        # 遠方の大未踏へ積極的に展開する。
        self.declare_parameter('size_weight', 2.5)
        self.declare_parameter('dist_weight', 0.3)
        # 旧 nearest 法の gain（互換のため残すが未使用）。
        self.declare_parameter('gain', 0.30)
        # 近すぎるフロンティア（既に居る場所）は無視する最小距離 [m]。
        self.declare_parameter('min_goal_dist', 0.6)
        # フロンティア重心そのものは壁/未知の際にあり Nav2 プランが通らないことが多い。
        # ロボット→フロンティアの線上で、重心の手前 approach_setback[m] にゴールを
        # 引く（自由空間側に寄せて到達可能にする）。マッピング中に壁へ寄りすぎて衝突→
        # 自己位置ズレで地図が崩れるのを防ぐため 1.0（costmap の robot_radius 0.22 +
        # inflation 0.5 と併せて壁から離れる）。
        self.declare_parameter('approach_setback', 1.0)
        # ゴール到達判定/タイムアウト [s]（1 ゴールに留まり続けない保険）。
        # planner が失敗するゴールに長く粘らず、早めに別フロンティアへ移る。
        self.declare_parameter('goal_timeout_sec', 15.0)
        # この回数連続でフロンティアが無ければ探索完了とみなす。
        self.declare_parameter('done_after_empty', 3)
        # 開始前の起動猶予 [s]（SLAM が地図を出し始めるまで待つ）。
        self.declare_parameter('start_delay_sec', 10.0)
        # 初期スキャンの Spin（地図ブートストラップ）を行うか・回す角度/許容時間。
        self.declare_parameter('bootstrap_spin', True)
        self.declare_parameter('bootstrap_yaw', 6.283)
        self.declare_parameter('bootstrap_time_allowance', 25.0)
        # 完了時に地図を保存するか・保存先（map_saver_cli）。
        self.declare_parameter('save_map', True)
        self.declare_parameter(
            'map_save_path',
            os.path.expanduser('~/ros2_ws/src/susumu_object_perception/maps/city'))

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_frontier_cells = int(
            self.get_parameter('min_frontier_cells').value)
        self.gain = float(self.get_parameter('gain').value)
        self.size_weight = float(self.get_parameter('size_weight').value)
        self.dist_weight = float(self.get_parameter('dist_weight').value)
        self.min_goal_dist = float(self.get_parameter('min_goal_dist').value)
        self.approach_setback = float(
            self.get_parameter('approach_setback').value)
        self.goal_timeout_sec = float(
            self.get_parameter('goal_timeout_sec').value)
        self.done_after_empty = int(
            self.get_parameter('done_after_empty').value)
        self.start_delay_sec = float(
            self.get_parameter('start_delay_sec').value)
        self.bootstrap_spin = bool(
            self.get_parameter('bootstrap_spin').value)
        self.bootstrap_yaw = float(self.get_parameter('bootstrap_yaw').value)
        self.bootstrap_time_allowance = float(
            self.get_parameter('bootstrap_time_allowance').value)
        self.save_map = bool(self.get_parameter('save_map').value)
        self.map_save_path = self.get_parameter('map_save_path').value

        self._map = None
        self._busy = False
        self._empty_count = 0
        self._done = False
        self._did_bootstrap = False
        self._goal_timer = None
        self._resched_timer = None
        # 到達不能だったゴール近傍のブラックリスト（同じ場所に粘らない）。
        # (gx, gy) を blacklist_cell[m] グリッドに丸めたキーで持つ。
        self._blacklist = set()
        self._last_goal = None
        # 直近の到達失敗回数（連続失敗で setback を一時的に増やす）。
        self._fail_streak = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        self._status_pub = self.create_publisher(
            String, '/frontier_explore/status', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/frontier_explore/markers', 1)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')

        self._start_timer = self.create_timer(
            self.start_delay_sec, self._kick_start_once)

        self.get_logger().info(
            'frontier_explore started '
            f'(min_cells={self.min_frontier_cells} gain={self.gain} '
            f'save={self.save_map} -> {self.map_save_path})')

    # ---- callbacks -------------------------------------------------------

    def _on_map(self, msg):
        self._map = msg

    def _kick_start_once(self):
        self._start_timer.cancel()
        self._step()

    # ---- main step -------------------------------------------------------

    def _step(self):
        """1 サイクル: フロンティアを探し、あれば向かう。無ければ完了判定。"""
        if self._done or self._busy:
            return
        if self._map is None:
            self._publish_status('no map yet; waiting')
            self._reschedule(2.0)
            return

        # 地図がまだ小さいときだけ 1 周 Spin して周囲をスキャンする。地図が既に
        # 十分育っていれば spin せず直接フロンティア探索に入る（spin は Nav2 behavior
        # に依存し応答待ちでハングしうるので、必要なときだけ最小限に使う）。
        if (not self._did_bootstrap and self.bootstrap_spin
                and self._map_is_tiny()):
            self._did_bootstrap = True
            self._do_bootstrap_spin()
            return
        self._did_bootstrap = True

        frontiers = self._find_frontiers()
        if not frontiers:
            self._empty_count += 1
            self._publish_status(
                f'no frontier ({self._empty_count}/{self.done_after_empty})')
            if self._empty_count >= self.done_after_empty:
                self._finish()
                return
            # まだ地図が育つ余地があるかもしれないので軽く回って再評価。
            if not self._did_bootstrap:
                self._did_bootstrap = True
                self._do_bootstrap_spin()
            else:
                self._reschedule(2.0)
            return

        self._publish_markers(frontiers)
        goal = self._choose_goal(frontiers)
        if goal is None:
            # フロンティアはあるが、どれも到達済み/ブラックリスト＝実質探索しきった。
            # 完了判定を進める（連続で続けば _finish で地図保存）。
            self._empty_count += 1
            self._publish_status(
                f'no reachable frontier '
                f'({self._empty_count}/{self.done_after_empty})')
            if self._empty_count >= self.done_after_empty:
                self._finish()
                return
            self._reschedule(2.0)
            return
        self._empty_count = 0
        self._navigate_to(goal)

    def _reschedule(self, sec):
        # 単一タイマで管理する（多重生成すると _step が高頻度で連発し、同じゴールを
        # 何度も再送してしまう）。前のタイマを必ずキャンセルしてから張り直す。
        self._cancel_resched_timer()
        self._resched_timer = self.create_timer(sec, self._reschedule_fire)

    def _reschedule_fire(self):
        self._cancel_resched_timer()
        self._step()

    def _cancel_resched_timer(self):
        t = getattr(self, '_resched_timer', None)
        if t is not None:
            t.cancel()
            self._resched_timer = None

    # ---- frontier detection ----------------------------------------------

    def _map_is_tiny(self):
        m = self._map
        free = sum(1 for v in m.data if 0 <= v <= FREE_MAX)
        return free < 150

    def _find_frontiers(self):
        """フロンティアセルを連結成分でまとめ、(wx, wy, size) のリストを返す。"""
        m = self._map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data
        rx, ry = self._robot_xy()  # 代表点をロボットに最も近いセルにするため

        def at(x, y):
            return data[y * w + x]

        def is_free(v):
            return 0 <= v <= FREE_MAX

        # フロンティアセル判定: free かつ 4近傍に unknown。
        is_frontier = bytearray(w * h)
        for y in range(1, h - 1):
            base = y * w
            for x in range(1, w - 1):
                v = data[base + x]
                if not is_free(v):
                    continue
                if (data[base + x - 1] == UNKNOWN or
                        data[base + x + 1] == UNKNOWN or
                        data[base - w + x] == UNKNOWN or
                        data[base + w + x] == UNKNOWN):
                    is_frontier[base + x] = 1

        # 連結成分（8近傍 BFS）でクラスタ化。
        visited = bytearray(w * h)
        clusters = []
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                idx = y * w + x
                if not is_frontier[idx] or visited[idx]:
                    continue
                q = deque([(x, y)])
                visited[idx] = 1
                cells = []
                while q:
                    cx, cy = q.popleft()
                    cells.append((cx, cy))
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            nx, ny = cx + dx, cy + dy
                            if nx < 1 or nx >= w - 1 or ny < 1 or ny >= h - 1:
                                continue
                            nidx = ny * w + nx
                            if is_frontier[nidx] and not visited[nidx]:
                                visited[nidx] = 1
                                q.append((nx, ny))
                if len(cells) >= self.min_frontier_cells:
                    # 代表点は「クラスタ重心に最も近いクラスタ内セル」にする。
                    # 旧実装は「ロボットに最も近いセル」だったが、それだと遠方の大きな
                    # 未踏クラスタでも代表点がロボット至近に来てしまい、ロボットが常に
                    # 目の前へ向かい遠方へ展開しない（屋外で開拓が広がらない主因だった）。
                    # 重心に最も近い実セルなら、クラスタの中心＝未踏の本体方向を指すので、
                    # 大未踏ほど遠方の代表点になり、未開拓を広く開拓できる。重心そのもの
                    # でなく「クラスタ内の実フロンティアセル」を選ぶので、必ず未踏との
                    # 境界の到達しやすい点を指す（重心が既踏領域に落ちる問題も回避）。
                    gcx = sum(c[0] for c in cells) / len(cells)
                    gcy = sum(c[1] for c in cells) / len(cells)
                    best_c = min(
                        cells,
                        key=lambda c: (c[0] - gcx) ** 2 + (c[1] - gcy) ** 2)
                    wx = ox + (best_c[0] + 0.5) * res
                    wy = oy + (best_c[1] + 0.5) * res
                    clusters.append((wx, wy, len(cells)))
        return clusters

    def _blkey(self, x, y):
        # 0.5m グリッドに丸めてブラックリストキーにする。
        return (round(x * 2.0), round(y * 2.0))

    def _choose_goal(self, frontiers):
        rx, ry = self._robot_xy()
        # 情報利得スコアリング: 大きな未踏領域（=size 大）を優先しつつ、近すぎ
        # （既踏で利得小）・遠すぎ（移動コスト大）を避ける。
        #   score = size_weight*log(size) - dist_weight*distance
        # size は対数で効かせ、巨大クラスタ1個に偏らないようにする。
        # 近傍フロンティアばかり選ぶ最近傍法だと局所に留まり地図が広がらないため、
        # size をしっかり効かせて遠方の大領域へ向かわせる。
        best = None
        best_score = -float('inf')
        for (wx, wy, size) in frontiers:
            d = math.hypot(wx - rx, wy - ry)
            if d < self.min_goal_dist:
                continue
            if self._blkey(wx, wy) in self._blacklist:
                continue
            score = self.size_weight * math.log(size + 1) - self.dist_weight * d
            if score > best_score:
                best_score = score
                best = (wx, wy)
        if best is None:
            return None
        # 代表点（ロボットに最も近いフロンティアセル）の手前に setback してゴール
        # にする。セルは未踏との境界なので、そこへ直接プランすると planner が失敗
        # しやすい。連続失敗中は setback を増やしてさらに手前を狙う。
        gx, gy = best
        setback = self.approach_setback + 0.4 * min(self._fail_streak, 3)
        d = math.hypot(gx - rx, gy - ry)
        if d > setback:
            t = (d - setback) / d
            gx = rx + (gx - rx) * t
            gy = ry + (gy - ry) * t
        # setback 後のゴールが現在地とほぼ同じ＝その境界の手前にはもう居る。Nav2 が
        # 即「到達済み」を返し同じゴールを連発するので、この代表点はブラックリストに
        # 入れて次回は別のフロンティアを選ぶ（探索は終了させず継続する）。
        if math.hypot(gx - rx, gy - ry) < self.min_goal_dist:
            self._blacklist.add(self._blkey(*best))
            return None
        self._last_goal = best
        return (gx, gy)

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            return (0.0, 0.0)

    # ---- bootstrap spin --------------------------------------------------

    def _do_bootstrap_spin(self):
        if not self._spin_client.wait_for_server(timeout_sec=5.0):
            self._publish_status('spin server unavailable; retrying')
            self._reschedule(3.0)
            return
        self._busy = True
        self._publish_status('bootstrap: spinning to seed the map')
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
            self._busy = False
            self._publish_status('spin rejected; continuing')
            self._reschedule(1.0)
            return
        gh.get_result_async().add_done_callback(self._on_spin_result)

    def _on_spin_result(self, future):
        self._busy = False
        self._publish_status('bootstrap spin done')
        self._reschedule(1.0)

    # ---- navigate --------------------------------------------------------

    def _navigate_to(self, goal_xy):
        # 即座に busy ロックして再入を防ぐ（wait_for_server 中に別の _step が
        # 走ると同じゴールを連発してしまう）。
        self._busy = True
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._busy = False
            self._publish_status('navigate_to_pose server unavailable')
            self._reschedule(3.0)
            return
        wx, wy = goal_xy
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        # 進行方向に向ける（ロボット位置からの方位）。
        rx, ry = self._robot_xy()
        yaw = math.atan2(wy - ry, wx - rx)
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)

        goal = NavigateToPose.Goal()
        goal.pose = ps
        self._busy = True
        self._publish_status(
            f'exploring frontier ({wx:.1f}, {wy:.1f})')
        # ゴールに留まり続けないようタイムアウトで打ち切って再評価。
        self._goal_timer = self.create_timer(
            self.goal_timeout_sec, self._on_goal_timeout)
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self._busy = False
            self._cancel_goal_timer()
            self._publish_status('nav goal rejected; re-evaluating')
            self._reschedule(1.0)
            return
        self._nav_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self._busy = False
        self._cancel_goal_timer()
        # ここまで来たら（タイムアウトでなく）ゴール処理が完了した＝多くは到達成功。
        # 連続失敗カウンタをリセットして setback を通常値に戻す。
        self._fail_streak = 0
        self._reschedule(0.5)

    def _on_goal_timeout(self):
        self._cancel_goal_timer()
        # 1 ゴールに時間をかけすぎ＝到達困難。そのフロンティアをブラックリストに
        # 入れて二度と選ばないようにし、別方向へ移る。
        gh = getattr(self, '_nav_goal_handle', None)
        if gh is not None:
            gh.cancel_goal_async()
        if self._last_goal is not None:
            self._blacklist.add(self._blkey(*self._last_goal))
        self._fail_streak += 1
        self._publish_status(
            f'goal timeout; blacklisted, trying another '
            f'(blacklist={len(self._blacklist)})')
        self._busy = False
        self._reschedule(0.5)

    def _cancel_goal_timer(self):
        if self._goal_timer is not None:
            self._goal_timer.cancel()
            self._goal_timer = None

    # ---- finish / save ---------------------------------------------------

    def _finish(self):
        self._done = True
        self._publish_status('exploration complete')
        if not self.save_map:
            return
        # nav2 map_saver_cli で OccupancyGrid を pgm+yaml に保存。
        os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)
        self._publish_status(f'saving map to {self.map_save_path}')
        try:
            subprocess.Popen(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', self.map_save_path,
                 '--ros-args', '-p', 'save_map_timeout:=20.0'])
            self._publish_status(f'map_saver launched: {self.map_save_path}')
        except Exception as e:  # noqa: BLE001
            self._publish_status(f'map save failed: {e}')

    # ---- viz / status ----------------------------------------------------

    def _publish_status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _publish_markers(self, frontiers):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (wx, wy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            s = min(0.6, 0.15 + 0.01 * size)
            m.scale.x = m.scale.y = m.scale.z = s
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.8, a=0.9)
            arr.markers.append(m)
        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploreNode()
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
