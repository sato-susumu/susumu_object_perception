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
       /frontier_explore/status (std_msgs/String)。完了時 outputs/mapping_*/<map_name> を保存。
"""

import json
import math
import os
import subprocess
import threading
from collections import deque

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from rcl_interfaces.msg import ParameterDescriptor

import tf2_ros
from tf2_ros import TransformException

from action_msgs.msg import GoalStatus
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, Spin, ComputePathToPose
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray


# OccupancyGrid のセル意味。
FREE_MAX = 20       # 0..20 を free 扱い（slam_toolbox の occ しきい値に余裕を持たせる）
OCC_MIN = 65        # 65.. を occupied 扱い
UNKNOWN = -1


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FrontierExploreNode(Node):

    def __init__(self):
        super().__init__('frontier_explore')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('world_name', '')
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
        # 近すぎるフロンティア（既に居る場所）は無視する最小距離 [m]。屋内で実績のある 0.6。
        # ※ 屋外開放空間で「2.0 に上げて至近フロンティアを除外し遠征させる」を試したが、
        #   /scan が数mしか届かず遠方に手がかりが無いため効果なし（原点±2-3mから出られない）。
        #   特徴の乏しい開放空間での frontier 探索の困難は学術的にも既知の難問で、標準的な
        #   frontier+slam_toolbox では原理的に解決できない。屋内向けの 0.6 を採用。
        self.declare_parameter('min_goal_dist', 0.6)
        # 到達距離以上のフロンティアが無いとき、最大フロンティア方向へ実際に前進する距離[m]。
        # spin でなく前進で新領域を既知化し「動き回る」探索にするための一歩。屋外で free が
        # 小円に留まり前進しない問題への対策。
        self.declare_parameter('forward_step', 2.0)
        # フロンティア重心そのものは壁/未知の際にあり Nav2 プランが通らないことが多い。
        # ロボット→フロンティアの線上で、重心の手前 approach_setback[m] にゴールを
        # 引く（自由空間側に寄せて到達可能にする）。マッピング中に壁へ寄りすぎて衝突→
        # 自己位置ズレで地図が崩れるのを防ぐため 1.0（costmap の robot_radius 0.22 +
        # inflation 0.5 と併せて壁から離れる）。
        self.declare_parameter('approach_setback', 1.3)
        # ゴール到達判定/タイムアウト [s]（1 ゴールに留まり続けない保険）。
        # 広い world の sweep は隣接点でも 5m 程度走るため、短すぎると到達前に
        # タイムアウトして探索が縮こまる。
        self.declare_parameter('goal_timeout_sec', 60.0)
        # 【経路事前検証】ゴールを NavigateToPose で送る前に ComputePathToPose で「そこへ
        # 経路が引けるか」を検証する（Nav2 標準のフロンティア探索のベストプラクティス。
        # 参照: AniArka/Autonomous-Explorer 等）。引けなければ即 blacklist して次候補を検証
        # する。これにより「到達不能ゴールへ送る→timeout→blacklist」の無駄な往復
        # （no valid path 多発・停滞の主因）を無くし、最初から到達可能なフロンティアだけへ
        # 向かう。検証は軽量なのでタイムアウトは短く。
        self.declare_parameter('validate_path', True)
        self.declare_parameter('validate_timeout_sec', 5.0)
        # 【屋外向け staged frontier】ComputePathToPose で長い経路が引けても、その終端を
        # そのまま NavigateToPose に投げると DWB が長い曲がり角/狭路で進捗判定に落ちやすい。
        # 0 より大きい場合、planner が返した経路上で max_path_goal_distance[m] だけ先の中間点を
        # 実際のゴールにする。経路そのものに沿って切るので、単純な直線クリップで壁を跨がない。
        # 既定 0.0 は無効（屋内挙動を変えない）。
        self.declare_parameter('max_path_goal_distance', 0.0)
        # staged frontier の中間ゴールに求める SLAM map 上の占有セルクリアランス[m]。
        # Nav2 InflationLayer はロボット内接半径内を lethal にするため、経路上で切った点が
        # 壁・フェンス・細い occupied に近いと、到着直後の次計画で start cell lethal になる。
        # 0.0 は無効。屋外 launch だけで有効化する。
        self.declare_parameter('staged_goal_clearance', 0.0)
        self.declare_parameter('staged_goal_backtrack_step', 0.2)
        # 【done 判定】本来の完了条件は「探索可能な領域が壁/障害物で閉じている＆その領域に
        # 未開拓(unknown)が無い」。これは「到達可能なフロンティアの総セル数が十分小さい」と
        # ほぼ等価。フロンティア総セル数が done_frontier_cells 未満が done_after_empty 回連続
        # で完了とする。全フロンティアが blacklist でも、まだ大きなフロンティアが残っていれば
        # 完了にしない（早期完了で未踏を残すのを防ぐ）。
        self.declare_parameter('done_after_empty', 3)
        self.declare_parameter('done_frontier_cells', 15)
        # 【打ち切り】無限探索を防ぐ。既知面積(free+occ)が stall_timeout_sec 間ほとんど
        # 増えなければ（進展が無ければ）強制終了する。
        # ※ かつて break_room で「全ゴールに着いているのに yaw 調整で oscillate→到達失敗扱い
        #   →地図が育たず stall 打ち切り」が起きたが、それは真因(yaw_goal_tolerance が厳しい)
        #   を params 側で直したので、ここは正常時に十分な 120s に戻す。stall は「真に進展が
        #   無い」ときの最後の保険であって、ゴール到達ペースの遅さを吸収する場所ではない。
        self.declare_parameter('stall_timeout_sec', 120.0)
        self.declare_parameter('stall_min_growth_cells', 150)
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
            os.path.expanduser('~/ros2_ws/src/susumu_object_perception/outputs/mapping_outdoor/city'))
        self.declare_parameter('map_saver_timeout_sec', 20.0)
        self.declare_parameter('map_saver_transient_local', True)
        # 保存完了後に world 真値と地図の重ね合わせ画像を自動生成する。world_file が空でなければ
        # check_map_vs_world.py を呼び、保存地図と同じディレクトリに <map_name>_vs_world.{png,json}
        # を出す。world_file は webots_worlds/<name>.wbt の絶対パスを期待する。
        self.declare_parameter('world_file', '')
        self.declare_parameter(
            'vs_world_script',
            os.path.expanduser(
                '~/ros2_ws/src/susumu_object_perception/scripts/check_map_vs_world.py'))
        self.declare_parameter('vs_world_timeout_sec', 30.0)
        # 【非frontier的な特殊探索: sweep モード】屋外の特徴が乏しい開放空間では frontier が
        # ロボット至近にしか出ず原点付近から動けない。そこで frontier 探索の前に、外周/spiral
        # の遠征ゴールを順に送り、未知でも構わず領域を舐めて回る（coverage 型）。全体を一巡
        # したら frontier 探索に移る。360度 LiDAR では各ゴール後の spin は既定 OFF。
        self.declare_parameter(
            'sweep_mode', False,
            ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('sweep_radius', 8.0)
        self.declare_parameter('sweep_spacing', 4.0)
        self.declare_parameter('sweep_dirs', 8)
        self.declare_parameter('sweep_pattern', 'perimeter')
        self.declare_parameter('spin_after_goal', False)
        # 【探索範囲制限】ロボット初期位置から半径 R[m] 以内の frontier だけ採用する。
        # 0 以下なら無制限（既定）。広大な world で「特徴の多い局所領域だけマッピング」したい
        # ときに使う（例: village_center の Cypress + Fence エリア周辺だけ）。
        self.declare_parameter('explore_radius', 0.0)
        # 【屋外向け yaw watchdog】段差・縁石・接触で 2D SLAM の map yaw が実機姿勢から
        # 急に外れた場合、現在の NavigateToPose を中断し、周辺を hazard として避ける。
        # ロボット側 IMU と map->base TF の差だけを見るため、GPS/正解地図は制御に戻さない。
        # 既定 OFF。屋外 launch だけで有効化する。
        self.declare_parameter('yaw_watchdog', False)
        self.declare_parameter('yaw_watchdog_imu_topic', '/imu')
        self.declare_parameter('yaw_watchdog_max_error_deg', 8.0)
        self.declare_parameter('yaw_watchdog_blacklist_radius', 1.2)
        self.declare_parameter('yaw_watchdog_cooldown_sec', 5.0)
        # step_detector 連携。 /step_detector/event を購読し、 ENTER step / stuck
        # 時のロボット位置周辺を hazard としてブラックリスト化する。 屋外の段差・
        # 縁石・植え込み縁で同じ goal を繰り返し試行するのを防ぐ。
        self.declare_parameter('step_detector_avoid', False)
        self.declare_parameter('step_detector_event_topic', '/step_detector/event')
        # 段差 hazard の半径 [m]。 yaw 用と分けて屋外向けにやや広め (1.5m) 既定。
        self.declare_parameter('step_detector_blacklist_radius', 1.5)
        # 同種イベント連発時の最小間隔 [s]。 1 回の段差通過で多数追加されないように。
        self.declare_parameter('step_detector_cooldown_sec', 3.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.world_name = str(self.get_parameter('world_name').value).lower()
        self.min_frontier_cells = int(
            self.get_parameter('min_frontier_cells').value)
        self.gain = float(self.get_parameter('gain').value)
        self.size_weight = float(self.get_parameter('size_weight').value)
        self.dist_weight = float(self.get_parameter('dist_weight').value)
        self.min_goal_dist = float(self.get_parameter('min_goal_dist').value)
        self.forward_step = float(self.get_parameter('forward_step').value)
        self.sweep_mode = self._sweep_enabled_param()
        self.sweep_radius = float(self.get_parameter('sweep_radius').value)
        self.sweep_spacing = float(self.get_parameter('sweep_spacing').value)
        self.sweep_dirs = int(self.get_parameter('sweep_dirs').value)
        self.sweep_pattern = str(self.get_parameter('sweep_pattern').value)
        self.spin_after_goal = self._bool_param('spin_after_goal')
        self.explore_radius = float(self.get_parameter('explore_radius').value)
        self.yaw_watchdog = self._bool_param('yaw_watchdog')
        self.yaw_watchdog_imu_topic = str(
            self.get_parameter('yaw_watchdog_imu_topic').value)
        self.yaw_watchdog_max_error_deg = float(
            self.get_parameter('yaw_watchdog_max_error_deg').value)
        self.yaw_watchdog_blacklist_radius = float(
            self.get_parameter('yaw_watchdog_blacklist_radius').value)
        self.yaw_watchdog_cooldown_sec = float(
            self.get_parameter('yaw_watchdog_cooldown_sec').value)
        self.step_detector_avoid = self._bool_param('step_detector_avoid')
        self.step_detector_event_topic = str(
            self.get_parameter('step_detector_event_topic').value)
        self.step_detector_blacklist_radius = float(
            self.get_parameter('step_detector_blacklist_radius').value)
        self.step_detector_cooldown_sec = float(
            self.get_parameter('step_detector_cooldown_sec').value)
        self._sweep_idx = 0           # 次に向かう sweep 方向のインデックス
        self._sweep_origin = None     # sweep 起点（ロボット初期位置）
        self._sweep_goals = []        # sweep 起点基準で生成した遠征ゴール列
        self._sweep_done = False      # sweep を一巡したか
        self._explore_origin = None   # explore_radius 制限用、初回 _robot_xy で確定
        self.approach_setback = float(
            self.get_parameter('approach_setback').value)
        self.goal_timeout_sec = float(
            self.get_parameter('goal_timeout_sec').value)
        self.validate_path = self._bool_param('validate_path')
        self.validate_timeout_sec = float(
            self.get_parameter('validate_timeout_sec').value)
        self.max_path_goal_distance = float(
            self.get_parameter('max_path_goal_distance').value)
        self.staged_goal_clearance = float(
            self.get_parameter('staged_goal_clearance').value)
        self.staged_goal_backtrack_step = float(
            self.get_parameter('staged_goal_backtrack_step').value)
        self.done_after_empty = int(
            self.get_parameter('done_after_empty').value)
        self.done_frontier_cells = int(
            self.get_parameter('done_frontier_cells').value)
        self.stall_timeout_sec = float(
            self.get_parameter('stall_timeout_sec').value)
        self.stall_min_growth_cells = int(
            self.get_parameter('stall_min_growth_cells').value)
        self.start_delay_sec = float(
            self.get_parameter('start_delay_sec').value)
        self.bootstrap_spin = self._bool_param('bootstrap_spin')
        self.bootstrap_yaw = float(self.get_parameter('bootstrap_yaw').value)
        self.bootstrap_time_allowance = float(
            self.get_parameter('bootstrap_time_allowance').value)
        self.save_map = self._bool_param('save_map')
        self.map_save_path = self.get_parameter('map_save_path').value
        self.map_saver_timeout_sec = float(
            self.get_parameter('map_saver_timeout_sec').value)
        self.map_saver_transient_local = self._bool_param(
            'map_saver_transient_local')
        self.world_file = str(self.get_parameter('world_file').value or '')
        self.vs_world_script = str(
            self.get_parameter('vs_world_script').value or '')
        self.vs_world_timeout_sec = float(
            self.get_parameter('vs_world_timeout_sec').value)

        self._map = None
        self._busy = False
        self._empty_count = 0
        self._done = False
        self._did_bootstrap = False
        self._goal_timer = None
        self._resched_timer = None
        # 打ち切り判定用: 既知セル数(free+occ)が伸びた最後の時刻と、そのときの既知セル数。
        # stall_timeout_sec 間 stall_min_growth_cells 以上伸びなければ進展なしで打ち切る。
        self._max_known = 0
        self._last_progress_t = None
        # 到達不能だったゴール近傍のブラックリスト（同じ場所に粘らない）。
        # (gx, gy) を blacklist_cell[m] グリッドに丸めたキーで持つ。
        self._blacklist = set()
        self._last_goal = None
        self._active_goal_kind = None
        self._active_goal_xy = None
        # 直近の到達失敗回数（連続失敗で setback を一時的に増やす）。
        self._fail_streak = 0
        self._nav_goal_handle = None
        self._nav_token = 0
        self._active_nav_token = 0
        self._map_save_thread = None
        self._ignore_nav_tokens = set()
        self._latest_imu_yaw = None
        self._yaw_offset = None
        self._last_yaw_watchdog_t = None
        self._yaw_hazards = []
        # 段差検知ノードからのイベントで blacklist 化した hazard 一覧。
        self._step_hazards = []
        self._last_step_event_t = None

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
        if self.yaw_watchdog:
            imu_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(
                Imu, self.yaw_watchdog_imu_topic, self._on_imu, imu_qos)
            self.create_timer(0.5, self._check_yaw_watchdog)
        if self.step_detector_avoid:
            self.create_subscription(
                String, self.step_detector_event_topic,
                self._on_step_event, 10)
            self.get_logger().info(
                f'step_detector_avoid enabled: subscribe '
                f'{self.step_detector_event_topic} '
                f'radius={self.step_detector_blacklist_radius:.2f}m')

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')
        # 経路事前検証用。nav2_planner が提供する ComputePathToPose アクション。
        self._plan_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')

        self._start_timer = self.create_timer(
            self.start_delay_sec, self._kick_start_once)

        self.get_logger().info(
            'frontier_explore started '
            f'(min_cells={self.min_frontier_cells} gain={self.gain} '
            f'save={self.save_map} -> {self.map_save_path})')

    def _bool_param(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _sweep_enabled_param(self):
        value = self.get_parameter('sweep_mode').value
        if isinstance(value, str):
            mode = value.strip().lower()
            if mode == 'auto':
                # The Webots include may rewrite the launch 'world' argument to
                # a temporary generated WBT path. Keep map_save_path as a second
                # hint so map_name:=city/outdoor still enables wide-world sweep.
                hint = ' '.join([
                    self.world_name,
                    str(self.get_parameter('map_save_path').value).lower(),
                ])
                return ('city' in hint) or ('outdoor' in hint)
            return mode in ('1', 'true', 'yes', 'on')
        return bool(value)

    # ---- callbacks -------------------------------------------------------

    def _on_map(self, msg):
        self._map = msg

    def _on_imu(self, msg):
        self._latest_imu_yaw = quat_to_yaw(msg.orientation)

    def _kick_start_once(self):
        self._start_timer.cancel()
        self._step()

    def _check_yaw_watchdog(self):
        if (not self.yaw_watchdog or self._done
                or self._latest_imu_yaw is None):
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return
        map_yaw = quat_to_yaw(tf.transform.rotation)
        if self._yaw_offset is None:
            self._yaw_offset = wrap_angle(map_yaw - self._latest_imu_yaw)
            return
        imu_map_yaw = wrap_angle(self._latest_imu_yaw + self._yaw_offset)
        yaw_error_deg = abs(math.degrees(wrap_angle(map_yaw - imu_map_yaw)))
        if yaw_error_deg <= self.yaw_watchdog_max_error_deg:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self._last_yaw_watchdog_t is not None
                and now - self._last_yaw_watchdog_t
                < self.yaw_watchdog_cooldown_sec):
            return
        self._last_yaw_watchdog_t = now
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        radius = max(0.2, self.yaw_watchdog_blacklist_radius)
        self._yaw_hazards.append((x, y, radius))
        # 古い hazard で候補が過剰に詰まらないよう上限を持つ。
        self._yaw_hazards = self._yaw_hazards[-40:]
        if self._last_goal is not None:
            self._blacklist.add(self._blkey(*self._last_goal))
        if self._active_goal_xy is not None:
            self._blacklist.add(self._blkey(*self._active_goal_xy))
        self._publish_status(
            f'yaw watchdog: error={yaw_error_deg:.1f}deg > '
            f'{self.yaw_watchdog_max_error_deg:.1f}deg; '
            f'cancel current goal and blacklist around ({x:.2f}, {y:.2f})')
        self._cancel_active_nav_for_watchdog()

    def _cancel_active_nav_for_watchdog(self):
        self._cancel_goal_timer()
        gh = getattr(self, '_nav_goal_handle', None)
        if gh is not None:
            self._ignore_nav_tokens.add(self._active_nav_token)
            gh.cancel_goal_async()
        self._nav_goal_handle = None
        self._active_nav_token = 0
        self._busy = False
        self._fail_streak += 1
        self._reschedule(1.0)

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
            self._do_observation_spin('bootstrap: spinning to seed the map')
            return
        self._did_bootstrap = True

        # 【sweep モード】frontier 探索の前に、各方向へ遠征ゴールを順に送って領域を舐める。
        # 一巡し終えたら（_sweep_done）通常の frontier 探索へ移る。
        if self.sweep_mode and not self._sweep_done:
            self._do_sweep_step()
            return

        # 進展(既知面積の増加)を監視し、停滞が続けば打ち切る（無限探索防止）。
        if self._check_stall_and_maybe_finish():
            return

        frontiers = self._find_frontiers()
        # 本来の done 条件: 探索可能領域が壁/障害物で閉じ、その中に未開拓が無い。
        # ＝ 到達可能なフロンティアの「総セル数」が十分小さい。総セル数が
        # done_frontier_cells 未満なら「未開拓の縁がほぼ無い」とみなす。
        total_frontier_cells = sum(size for (_, _, size) in frontiers)
        if total_frontier_cells < self.done_frontier_cells:
            self._empty_count += 1
            self._publish_status(
                f'frontier nearly gone ({total_frontier_cells} cells, '
                f'{self._empty_count}/{self.done_after_empty})')
            if self._empty_count >= self.done_after_empty:
                self._finish()
                return
            # まだ地図が育つ余地があるかもしれないので軽く回って再評価。
            if self.bootstrap_spin:
                self._do_observation_spin('frontier sparse; spinning to rescan')
            else:
                self._reschedule(2.0)
            return

        # 候補があるかどうかだけでは reset しない。 _try_candidates 経由で実際に
        # NavigateToPose を発行できたとき (= 進捗の見込みがあるとき) だけ
        # _try_candidates 側で reset する。 lethal pose 等で全候補が unreachable に
        # なる状況でも empty_count が積み上がって done_after_empty に到達できるようにする。
        # （以前は無条件にここでリセットしていたが、 _try_candidates の +1 が次 step
        # 冒頭で毎回打ち消され、 done_after_empty に届かない無限ループになっていた。）
        self._publish_markers(frontiers)
        cands = self._rank_goals(frontiers)
        if not cands:
            # 候補ゼロ = 全フロンティアがブラックリスト済み＝到達できない未開拓が残って
            # いる（家具で塞がれた隙間など物理的に行けない領域）。blacklist を消すと同じ
            # 到達不能ゴールへ無限に再挑戦して高速ループ→CPU 枯渇→webots 落ちになるので
            # blacklist は維持する。empty_count を進めて done_after_empty 回で完了扱いに
            # する（5秒間隔で詰めるので高速ループにならない）。
            self._empty_count += 1
            self._publish_status(
                f'unreachable frontier remains ({total_frontier_cells} cells); '
                f'({self._empty_count}/{self.done_after_empty})')
            if self._empty_count >= self.done_after_empty:
                self._finish()
                return
            self._reschedule(5.0)
            return
        # 候補を「スコア降順」で持ち、先頭から ComputePathToPose で経路検証して
        # 到達可能な最初の候補へ向かう。検証 OFF なら先頭をそのまま送る（従来動作）。
        if self.validate_path:
            self._busy = True
            self._try_candidates(cands, 0, total_frontier_cells)
        else:
            self._navigate_to(cands[0][0], repr_xy=cands[0][1])

    def _check_stall_and_maybe_finish(self):
        """既知面積(free+occ)の伸びを監視し、進展が無ければ打ち切る。

        無限探索を防ぐための保険。stall_timeout_sec 間に既知セルが stall_min_growth_cells
        以上増えなければ「進展なし」とみなして _finish する。done 判定(フロンティア枯渇)が
        正しく働けば通常はこちらに来ないが、到達不能な未開拓が残り続ける等で frontier が
        消えないケースを打ち切る。戻り値 True なら打ち切った（呼び出し側は return）。
        """
        m = self._map
        known = sum(1 for v in m.data if v == 0 or v == 100)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_progress_t is None:
            self._last_progress_t = now
            self._max_known = known
            return False
        if known >= self._max_known + self.stall_min_growth_cells:
            # 十分伸びた＝進展あり。基準を更新。
            self._max_known = known
            self._last_progress_t = now
            return False
        if now - self._last_progress_t >= self.stall_timeout_sec:
            self._publish_status(
                f'no progress for {self.stall_timeout_sec:.0f}s '
                f'(known={known}); finishing')
            self._finish()
            return True
        return False

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

    def _rank_goals(self, frontiers):
        """到達候補をスコア降順で返す。各要素 = (goal_xy, repr_xy)。

        goal_xy は setback 適用後の実際に送るゴール、repr_xy は元のフロンティア
        代表点（blacklist キー算出用）。情報利得スコア
          score = size_weight*log(size) - dist_weight*distance
        で大きな未踏領域を優先。blacklist 済み・近すぎ(min_goal_dist 未満)は除外。
        到達距離以上のフロンティアが 1 つも無ければ、最大クラスタ方向への前進
        (forward push)ゴールを 1 つだけ末尾に積む。

        旧 _choose_goal は「最良 1 点だけ返す」同期実装だったが、これだと到達不能でも
        送ってしまい goal_timeout_sec を待つ羽目になる。リストで返し、呼び出し側が先頭から
        ComputePathToPose で検証して到達可能な最初の候補へ向かうことで停滞を無くす。
        """
        rx, ry = self._robot_xy()
        # explore_radius が有効なら、初回の _robot_xy を中心として固定し、
        # 以後 frontier の代表点も setback 後ゴールも中心から半径内に収める。
        if self.explore_radius > 0.0 and self._explore_origin is None:
            self._explore_origin = (rx, ry)
            self._publish_status(
                f'explore_radius={self.explore_radius:.1f}m '
                f'around origin ({rx:.2f}, {ry:.2f})')
        scored = []
        for (wx, wy, size) in frontiers:
            d = math.hypot(wx - rx, wy - ry)
            if d < self.min_goal_dist:
                continue
            if self._blkey(wx, wy) in self._blacklist:
                continue
            # explore_radius の外にある frontier 代表点は除外する。
            if self._explore_origin is not None:
                ox, oy = self._explore_origin
                if math.hypot(wx - ox, wy - oy) > self.explore_radius:
                    continue
            if self._near_yaw_hazard(wx, wy):
                continue
            score = self.size_weight * math.log(size + 1) - self.dist_weight * d
            # 代表点の手前に setback したゴールにする（境界そのものは planner が失敗
            # しやすい）。連続失敗中は setback を増やしてさらに手前を狙う。
            gx, gy = wx, wy
            setback = self.approach_setback + 0.4 * min(self._fail_streak, 3)
            if d > setback:
                t = (d - setback) / d
                gx = rx + (gx - rx) * t
                gy = ry + (gy - ry) * t
            # setback 後のゴールが現在地とほぼ同じ＝もう手前に居る。送ると Nav2 が即
            # 到達済みを返し連発するので、この代表点は blacklist して候補から外す。
            if math.hypot(gx - rx, gy - ry) < self.min_goal_dist:
                self._blacklist.add(self._blkey(wx, wy))
                continue
            # setback 後ゴールが explore_radius の外なら、半径境界上にクリップする。
            if self._explore_origin is not None:
                ox, oy = self._explore_origin
                d_og = math.hypot(gx - ox, gy - oy)
                if d_og > self.explore_radius:
                    t = self.explore_radius / d_og
                    gx = ox + (gx - ox) * t
                    gy = oy + (gy - oy) * t
            if self._near_yaw_hazard(gx, gy):
                continue
            scored.append((score, (gx, gy), (wx, wy)))
        scored.sort(key=lambda e: e[0], reverse=True)
        cands = [(g, r) for (_, g, r) in scored]
        if cands:
            return cands
        # 到達距離以上のフロンティアが無い＝目の前にしか境界が無い（屋外で free が
        # 小円に留まる典型）。最大クラスタ方向へ前進する push ゴールを 1 つ作る。
        # 前進ゴールは代表点を持たない（repr=None）→検証失敗しても blacklist しない。
        cand = [(wx, wy, size) for (wx, wy, size) in frontiers
                if self._blkey(wx, wy) not in self._blacklist]
        if self._explore_origin is not None:
            ox, oy = self._explore_origin
            cand = [(wx, wy, size) for (wx, wy, size) in cand
                    if math.hypot(wx - ox, wy - oy) <= self.explore_radius]
        if not cand:
            return []
        tx, ty, _ = max(cand, key=lambda f: f[2])
        ang = math.atan2(ty - ry, tx - rx)
        fx = rx + self.forward_step * math.cos(ang)
        fy = ry + self.forward_step * math.sin(ang)
        # 前進ゴールも explore_radius 内に収める。
        if self._explore_origin is not None:
            ox, oy = self._explore_origin
            d_of = math.hypot(fx - ox, fy - oy)
            if d_of > self.explore_radius:
                t = self.explore_radius / d_of
                fx = ox + (fx - ox) * t
                fy = oy + (fy - oy) * t
        if self._near_yaw_hazard(fx, fy):
            return []
        self._publish_status(
            f'no distant frontier; push forward {self.forward_step:.1f}m '
            f'toward largest frontier')
        return [((fx, fy), None)]

    def _near_yaw_hazard(self, x, y):
        for hx, hy, radius in self._yaw_hazards:
            if math.hypot(x - hx, y - hy) <= radius:
                return True
        for hx, hy, radius in self._step_hazards:
            if math.hypot(x - hx, y - hy) <= radius:
                return True
        return False

    def _on_step_event(self, msg):
        """step_detector からのイベントで現在位置周辺を hazard 化する。

        ENTER step (tilt) や stuck で発火。 ロボット現在位置 (map 座標) を中心に
        半径 step_detector_blacklist_radius の hazard を追加し、 同じ場所を
        繰り返し goal にしないようにする。
        """
        if not self.step_detector_avoid:
            return
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        event_type = payload.get('type', '')
        # tilt_recover / accel_jolt は無視 (検出だけで blacklist 不要)
        if event_type not in ('tilt', 'stuck'):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self._last_step_event_t is not None and
                now - self._last_step_event_t < self.step_detector_cooldown_sec):
            return
        self._last_step_event_t = now
        # ロボット現在位置を取得 (map -> robot_frame)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        radius = max(0.3, self.step_detector_blacklist_radius)
        # 既存 hazard の半径内であれば新規追加せず、 ログだけに留める。
        # iter28 ライブで同一位置 62 回連続追加を観測したため。 hazard リスト数を
        # ノイズで埋めず、 「ロボットが詰まって動けない状態」 を肥大化させない。
        merged = False
        for hx, hy, hr in self._step_hazards:
            if math.hypot(x - hx, y - hy) <= max(hr, radius):
                merged = True
                break
        if not merged:
            self._step_hazards.append((x, y, radius))
            # 上限制御 (古い hazard は捨てる)
            self._step_hazards = self._step_hazards[-40:]
        # 進行中の goal もキャンセル
        if self._last_goal is not None:
            self._blacklist.add(self._blkey(*self._last_goal))
        if self._active_goal_xy is not None:
            self._blacklist.add(self._blkey(*self._active_goal_xy))
        action = 'blacklist' if not merged else 'merged with existing hazard'
        self._publish_status(
            f'step_detector event={event_type} '
            f'(tilt_deg={payload.get("tilt_deg", "?")}); '
            f'{action} around ({x:.2f}, {y:.2f}) r={radius:.2f}m '
            f'and cancel current goal')
        try:
            self._cancel_active_nav_for_watchdog()
        except Exception as exc:  # 既存実装と互換、 yaw_watchdog 関連が落ちても続行
            self.get_logger().warning(
                f'failed to cancel nav after step event: {exc}')

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            return (0.0, 0.0)

    # ---- bootstrap spin --------------------------------------------------

    def _do_observation_spin(self, status_text):
        if not self._spin_client.wait_for_server(timeout_sec=5.0):
            self._publish_status('spin server unavailable; retrying')
            self._reschedule(3.0)
            return
        self._busy = True
        self._publish_status(status_text)
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
        self._publish_status('observation spin done')
        self._reschedule(1.0)

    # ---- sweep (非frontier的な特殊探索) ----------------------------------

    def _do_sweep_step(self):
        """sweep 起点から coverage ゴールを順に送る。

        frontier に頼らず未知でも構わず遠方へ向かわせて領域を舐める coverage 型探索。既定は
        perimeter で、早めに外周へ出て map bounding box を広げる。spiral は中心から順に外周へ
        広げる補助パターン。旧来の radial 8方向 sweep は遠方の線を数本引くだけで、広い world
        では地図が星形になりやすい。
        """
        if self._sweep_origin is None:
            self._sweep_origin = self._robot_xy()
            self._sweep_goals = self._build_sweep_goals(self._sweep_origin)
            self._publish_status(
                f'sweep {self.sweep_pattern}: {len(self._sweep_goals)} goals '
                f'(radius={self.sweep_radius:.1f}m spacing={self.sweep_spacing:.1f}m)')
        if self._sweep_idx >= len(self._sweep_goals):
            self._sweep_done = True
            self._publish_status('sweep done; switching to frontier search')
            self._reschedule(1.0)
            return
        gx, gy = self._sweep_goals[self._sweep_idx]
        self._publish_status(
            f'sweep {self._sweep_idx + 1}/{len(self._sweep_goals)} '
            f'-> ({gx:.1f}, {gy:.1f})')
        self._sweep_idx += 1
        # 通常の nav 送信を流用（到達/タイムアウトで次サイクル→次方向へ）。
        self._navigate_to((gx, gy), repr_xy=None, goal_kind='sweep')

    def _build_sweep_goals(self, origin):
        ox, oy = origin
        radius = max(0.5, self.sweep_radius)
        if self.sweep_pattern.lower() == 'radial':
            return [
                (ox + radius * math.cos(2.0 * math.pi * i / self.sweep_dirs),
                 oy + radius * math.sin(2.0 * math.pi * i / self.sweep_dirs))
                for i in range(max(1, self.sweep_dirs))
            ]

        if self.sweep_pattern.lower() == 'perimeter':
            # 20m 前後の開放 world では、中心から小さな螺旋を描くより、早めに外周へ出て
            # 縦横の bbox を広げた方が SLAM map の外形が育つ。北端から時計回りに外周の
            # 中点/角を回り、最後に frontier 探索へ渡す。
            local = [
                (0.0, radius),
                (radius, radius),
                (radius, 0.0),
                (radius, -radius),
                (0.0, -radius),
                (-radius, -radius),
                (-radius, 0.0),
                (-radius, radius),
                (0.0, radius),
            ]
            return [(ox + lx, oy + ly) for lx, ly in local]

        step = max(0.5, self.sweep_spacing)
        rings = max(1, int(math.ceil(radius / step)))
        local = []
        for k in range(1, rings + 1):
            a = min(radius, k * step)
            prev = min(radius, (k - 1) * step)
            # Square spiral: east -> north -> west -> south -> east, then expand.
            local.extend([
                (a, -prev),
                (a, a),
                (-a, a),
                (-a, -a),
                (a, -a),
            ])
        goals = []
        last = None
        for lx, ly in local:
            g = (ox + lx, oy + ly)
            if last is None or math.hypot(g[0] - last[0], g[1] - last[1]) > 0.2:
                goals.append(g)
                last = g
        return goals

    # ---- path validation (送る前に到達可能か検証) ------------------------

    def _try_candidates(self, cands, idx, total_cells):
        """候補リスト cands の idx 番目を ComputePathToPose で検証して向かう。

        到達可能なら NavigateToPose で向かう。経路が引けなければ、その候補の代表点を
        blacklist して次の候補(idx+1)を検証する。全候補が到達不能なら empty_count を
        進める（全フロンティア到達不能＝_step 冒頭の候補ゼロ分岐と同じ扱い）。
        _busy は呼び出し側で True 済み。検証〜送信完了まで _busy を保持する。
        """
        if idx >= len(cands):
            # 全候補が到達不能だった。候補ゼロと同様に done に近づける。
            self._busy = False
            self._empty_count += 1
            self._publish_status(
                f'all {len(cands)} candidates unreachable '
                f'({total_cells} cells); ({self._empty_count}/'
                f'{self.done_after_empty})')
            if self._empty_count >= self.done_after_empty:
                self._finish()
                return
            self._reschedule(5.0)
            return

        goal_xy, repr_xy = cands[idx]
        if not self._plan_client.wait_for_server(timeout_sec=3.0):
            # planner サーバが居なければ検証を諦めて先頭をそのまま送る（従来動作）。
            self._publish_status('planner unavailable; sending without validate')
            self._navigate_to(cands[0][0], repr_xy=cands[0][1])
            return

        wx, wy = goal_xy
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        ps.pose.orientation.w = 1.0

        g = ComputePathToPose.Goal()
        g.goal = ps
        g.use_start = False  # 現在位置から計画させる
        self._publish_status(
            f'validating path to ({wx:.1f}, {wy:.1f}) '
            f'[{idx + 1}/{len(cands)}]')
        # 検証が応答しない場合の保険タイムアウト。
        self._validate_timer = self.create_timer(
            self.validate_timeout_sec,
            lambda: self._on_validate_timeout(cands, idx, total_cells))
        fut = self._plan_client.send_goal_async(g)
        fut.add_done_callback(
            lambda f: self._on_validate_goal_response(
                f, cands, idx, total_cells))

    def _on_validate_goal_response(self, future, cands, idx, total_cells):
        gh = future.result()
        if not gh.accepted:
            self._reject_candidate(cands, idx, total_cells, 'rejected')
            return
        self._validate_goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_validate_result(f, cands, idx, total_cells))

    def _on_validate_result(self, future, cands, idx, total_cells):
        self._cancel_validate_timer()
        ok = False
        path = None
        try:
            res = future.result()
            # 経路が返り、ポーズが 2 点以上あれば到達可能とみなす。
            path = res.result.path
            ok = path is not None and len(path.poses) >= 2
        except Exception:  # noqa: BLE001
            ok = False
        if ok:
            goal_xy, repr_xy = cands[idx]
            nav_goal_xy = goal_xy
            self._fail_streak = 0
            clipped, reject_reason = self._clip_goal_to_path(
                path, self.max_path_goal_distance)
            if reject_reason is not None:
                self._reject_candidate(cands, idx, total_cells, reject_reason)
                return
            if clipped is not None:
                nav_goal_xy, step_dist, total_dist, clearance = clipped
                clearance_text = ''
                if clearance is not None:
                    clearance_text = f' clearance>={clearance:.2f}m '
                self._publish_status(
                    f'path valid; staged nav {step_dist:.1f}/{total_dist:.1f}m '
                    f'{clearance_text}[{idx + 1}/{len(cands)}]')
            else:
                self._publish_status(
                    f'path valid; navigating [{idx + 1}/{len(cands)}]')
            # _busy 維持のまま実際のナビへ。
            self._navigate_to(nav_goal_xy, already_busy=True, repr_xy=repr_xy)
        else:
            self._reject_candidate(cands, idx, total_cells, 'no path')

    def _clip_goal_to_path(self, path, max_distance):
        """Planner 経路上の max_distance[m] 先を中間ゴールとして返す。

        戻り値は (clipped, reject_reason)。clipped は
        ((x, y), step_distance, total_distance, clearance)。
        経路が短い、無効、または機能無効なら (None, None) を返す。
        """
        if max_distance <= 0.0 or path is None:
            return None, None
        poses = getattr(path, 'poses', [])
        if len(poses) < 2:
            return None, None
        points = [
            (p.pose.position.x, p.pose.position.y)
            for p in poses
        ]
        segments = []
        total = 0.0
        for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg <= 1e-6:
                continue
            segments.append((x0, y0, x1, y1, seg))
            total += seg
        if not segments:
            return None, None
        step = max(max_distance, self.min_goal_dist)
        if total <= step + 1e-6:
            return None, None
        clearance = None
        if self.staged_goal_clearance > 0.0:
            safe = self._find_safe_staged_goal(segments, step)
            if safe is None:
                return None, 'no safe staged point'
            x, y, step = safe
            clearance = self.staged_goal_clearance
            return ((x, y), step, total, clearance), None
        point = self._point_at_path_distance(segments, step)
        if point is None:
            return None, None
        x, y = point
        return ((x, y), step, total, clearance), None

    def _find_safe_staged_goal(self, segments, target_dist):
        """経路上を target_dist から後退し、占有セルから十分離れた free 点を探す。"""
        min_dist = max(self.min_goal_dist, 0.5)
        step = max(self.staged_goal_backtrack_step, 0.05)
        d = target_dist
        while d >= min_dist:
            point = self._point_at_path_distance(segments, d)
            if (point is not None
                    and not self._near_yaw_hazard(point[0], point[1])
                    and self._has_map_clearance(
                        point[0], point[1], self.staged_goal_clearance)):
                return point[0], point[1], d
            d -= step
        return None

    @staticmethod
    def _point_at_path_distance(segments, target_dist):
        acc = 0.0
        for x0, y0, x1, y1, seg in segments:
            if acc + seg >= target_dist:
                ratio = (target_dist - acc) / seg
                x = x0 + (x1 - x0) * ratio
                y = y0 + (y1 - y0) * ratio
                return x, y
            acc += seg
        if segments:
            return segments[-1][2], segments[-1][3]
        return None

    def _has_map_clearance(self, wx, wy, clearance):
        """現在の OccupancyGrid 上で、中心 free かつ半径内に occupied が無いかを見る。"""
        m = self._map
        if m is None:
            return True
        cell = self._world_to_map_cell(wx, wy)
        if cell is None:
            return False
        mx, my = cell
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        data = m.data
        center = data[my * w + mx]
        if not (0 <= center <= FREE_MAX):
            return False
        radius_cells = int(math.ceil(clearance / res))
        radius2 = clearance * clearance
        for dy in range(-radius_cells, radius_cells + 1):
            cy = my + dy
            if cy < 0 or cy >= h:
                return False
            ydist = dy * res
            for dx in range(-radius_cells, radius_cells + 1):
                cx = mx + dx
                if cx < 0 or cx >= w:
                    return False
                if (dx * res) ** 2 + ydist ** 2 > radius2:
                    continue
                if data[cy * w + cx] >= OCC_MIN:
                    return False
        return True

    def _world_to_map_cell(self, wx, wy):
        m = self._map
        if m is None:
            return None
        res = m.info.resolution
        if res <= 0.0:
            return None
        mx = int(math.floor((wx - m.info.origin.position.x) / res))
        my = int(math.floor((wy - m.info.origin.position.y) / res))
        if mx < 0 or my < 0 or mx >= m.info.width or my >= m.info.height:
            return None
        return mx, my

    def _on_validate_timeout(self, cands, idx, total_cells):
        self._cancel_validate_timer()
        gh = getattr(self, '_validate_goal_handle', None)
        if gh is not None:
            gh.cancel_goal_async()
        self._reject_candidate(cands, idx, total_cells, 'validate timeout')

    def _reject_candidate(self, cands, idx, total_cells, reason):
        """idx 番目の候補を到達不能として blacklist し、次候補を検証する。"""
        goal_xy, repr_xy = cands[idx]
        if repr_xy is not None:
            self._blacklist.add(self._blkey(*repr_xy))
        self._publish_status(
            f'candidate [{idx + 1}/{len(cands)}] {reason}; '
            f'blacklist={len(self._blacklist)}, trying next')
        self._try_candidates(cands, idx + 1, total_cells)

    def _cancel_validate_timer(self):
        t = getattr(self, '_validate_timer', None)
        if t is not None:
            t.cancel()
            self._validate_timer = None

    # ---- navigate --------------------------------------------------------

    def _navigate_to(self, goal_xy, already_busy=False, repr_xy=None,
                     goal_kind='frontier'):
        # 即座に busy ロックして再入を防ぐ（wait_for_server 中に別の _step が
        # 走ると同じゴールを連発してしまう）。
        self._busy = True
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._busy = False
            self._publish_status('navigate_to_pose server unavailable')
            self._reschedule(3.0)
            return
        # 進捗 (= 到達可能 goal を送れる) があったので empty_count をリセット。
        # frontier / sweep / forward push のどれであっても、何かを navigate に投げる
        # ということは「探索可能領域に進む」進捗とみなしてよい。
        self._empty_count = 0
        wx, wy = goal_xy
        self._last_goal = repr_xy
        self._active_goal_kind = goal_kind
        self._active_goal_xy = goal_xy
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
            f'exploring {goal_kind} ({wx:.1f}, {wy:.1f})')
        # ゴールに留まり続けないようタイムアウトで打ち切って再評価。
        self._goal_timer = self.create_timer(
            self.goal_timeout_sec, self._on_goal_timeout)
        self._nav_token += 1
        token = self._nav_token
        self._active_nav_token = token
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_nav_goal_response(f, token))

    def _on_nav_goal_response(self, future, token):
        try:
            gh = future.result()
        except Exception:  # noqa: BLE001
            if token == self._active_nav_token:
                self._active_nav_token = 0
                self._nav_goal_handle = None
                self._busy = False
                self._cancel_goal_timer()
                self._publish_status('nav goal failed before acceptance')
                self._reschedule(1.0)
            return
        if token != self._active_nav_token:
            if gh.accepted:
                self._ignore_nav_tokens.add(token)
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            self._busy = False
            self._active_nav_token = 0
            self._nav_goal_handle = None
            self._cancel_goal_timer()
            self._publish_status('nav goal rejected; re-evaluating')
            self._reschedule(1.0)
            return
        self._nav_goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_nav_result(f, token))

    def _on_nav_result(self, future, token):
        if token != self._active_nav_token:
            if token in self._ignore_nav_tokens:
                self._ignore_nav_tokens.discard(token)
                self._publish_status(
                    'nav result ignored after watchdog/timeout cancel')
            return
        self._busy = False
        self._active_nav_token = 0
        self._nav_goal_handle = None
        self._cancel_goal_timer()
        if token in self._ignore_nav_tokens:
            self._ignore_nav_tokens.discard(token)
            self._publish_status('nav result ignored after yaw watchdog cancel')
            return
        try:
            result = future.result()
            status = result.status
        except Exception:  # noqa: BLE001
            status = GoalStatus.STATUS_UNKNOWN

        if status == GoalStatus.STATUS_SUCCEEDED:
            # ゴール到達。連続失敗カウンタをリセットして setback を通常値に戻す。
            self._fail_streak = 0
            if self.spin_after_goal:
                kind = self._active_goal_kind or 'goal'
                self._do_observation_spin(
                    f'{kind} reached; spinning to update map')
            else:
                self._reschedule(0.5)
            return

        if self._last_goal is not None:
            self._blacklist.add(self._blkey(*self._last_goal))
        self._fail_streak += 1
        self._publish_status(
            f'nav failed status={status}; trying another '
            f'(blacklist={len(self._blacklist)})')
        self._reschedule(0.5)

    def _on_goal_timeout(self):
        self._cancel_goal_timer()
        # 1 ゴールに時間をかけすぎ＝到達困難。そのフロンティアをブラックリストに
        # 入れて二度と選ばないようにし、別方向へ移る。
        gh = getattr(self, '_nav_goal_handle', None)
        if gh is not None:
            self._ignore_nav_tokens.add(self._active_nav_token)
            gh.cancel_goal_async()
        self._nav_goal_handle = None
        self._active_nav_token = 0
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
        self._map_save_thread = threading.Thread(
            target=self._run_map_saver_cli, daemon=True)
        self._map_save_thread.start()

    def _run_map_saver_cli(self):
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', self.map_save_path,
            '--ros-args',
            '-p', f'save_map_timeout:={self.map_saver_timeout_sec:.1f}',
        ]
        if self.map_saver_transient_local:
            cmd.extend(['-p', 'map_subscribe_transient_local:=true'])
        process_timeout = max(30.0, self.map_saver_timeout_sec + 10.0)
        try:
            completed = subprocess.run(
                cmd, check=False, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=process_timeout)
        except subprocess.TimeoutExpired:
            self._publish_status(
                f'map save failed: map_saver_cli timed out after '
                f'{process_timeout:.1f}s')
            return
        except Exception as e:  # noqa: BLE001
            self._publish_status(f'map save failed: {e}')
            return
        output = (completed.stdout or '').strip()
        if completed.returncode != 0:
            suffix = self._tail_for_status(output)
            self._publish_status(
                f'map save failed: map_saver_cli exit '
                f'{completed.returncode}{suffix}')
            return
        ok, detail = self._saved_map_assets_status()
        if ok:
            self._publish_status(f'map saved: {detail}')
            self._run_vs_world_overlay()
        else:
            suffix = self._tail_for_status(output)
            self._publish_status(f'map save failed: {detail}{suffix}')

    def _run_vs_world_overlay(self):
        """保存地図と world 真値の重ね合わせ画像 + アライメント JSON を生成。

        world_file が空、または check_map_vs_world.py が存在しないときは何もしない
        （cafe.wbt のように Webots world が無い world では正しく skip される）。
        """
        if not self.world_file:
            return
        if not os.path.exists(self.world_file):
            self._publish_status(
                f'vs_world skip: world file missing ({self.world_file})')
            return
        if not self.vs_world_script or not os.path.exists(self.vs_world_script):
            self._publish_status(
                f'vs_world skip: script missing ({self.vs_world_script})')
            return
        yaml_path = self.map_save_path
        if not yaml_path.endswith(('.yaml', '.yml')):
            yaml_path = self.map_save_path + '.yaml'
        out_prefix = self.map_save_path
        if out_prefix.endswith(('.yaml', '.yml')):
            out_prefix = os.path.splitext(out_prefix)[0]
        out_png = f'{out_prefix}_vs_world.png'
        out_json = f'{out_prefix}_vs_world.json'
        cmd = [
            'python3', self.vs_world_script,
            '--wbt', self.world_file,
            '--map', yaml_path,
            '--out', out_png,
            '--report', out_json,
        ]
        try:
            completed = subprocess.run(
                cmd, check=False, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, timeout=self.vs_world_timeout_sec)
        except subprocess.TimeoutExpired:
            self._publish_status(
                f'vs_world failed: timed out after '
                f'{self.vs_world_timeout_sec:.1f}s')
            return
        except Exception as e:  # noqa: BLE001
            self._publish_status(f'vs_world failed: {e}')
            return
        if completed.returncode != 0:
            suffix = self._tail_for_status(completed.stdout or '')
            self._publish_status(
                f'vs_world failed: exit {completed.returncode}{suffix}')
            return
        self._publish_status(f'vs_world saved: {out_png}')

    def _tail_for_status(self, text):
        if not text:
            return ''
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        if not lines:
            return ''
        return f' ({lines[-1][:180]})'

    def _saved_map_assets_status(self):
        yaml_path = self.map_save_path
        if not yaml_path.endswith(('.yaml', '.yml')):
            yaml_path = self.map_save_path + '.yaml'
        if not os.path.exists(yaml_path):
            return False, f'map yaml missing after save: {yaml_path}'
        try:
            with open(yaml_path, encoding='utf-8') as f:
                meta = yaml.safe_load(f) or {}
        except Exception as exc:  # noqa: BLE001
            return False, f'map yaml invalid after save: {yaml_path}: {exc}'
        image = meta.get('image')
        if not image:
            return False, f'map yaml has no image field: {yaml_path}'
        image_path = str(image)
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)
        if not os.path.exists(image_path):
            return False, f'map image missing after save: {image_path}'
        return True, f'{yaml_path} -> {image_path}'

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
