#!/usr/bin/env python3
"""ウェイポイント YAML を Nav2 で順に巡回するノード（各点タイムアウト付き）。

generate_waypoints.py が作った <world>_waypoints.yaml を読み、各ウェイポイントへ
NavigateToPose で順に向かう。到達したら次へ、`goal_timeout_sec` 以内に到達できなければ
その点を「スキップ（missed）」して次へ進む。これにより 1 点で詰まっても巡回が止まらず、
全点を一巡できる（FollowWaypoints 丸投げだと 1 点で延々リトライして完走しない問題への対処）。

1 周終わると到達数・スキップ数を報告し、loop:=True なら次の周回を始める。

使い方:
  ros2 run susumu_object_perception waypoint_nav_node.py --ros-args \
    -p waypoints_file:=.../outputs/waypoint_generation/city_waypoints.yaml -p loop:=True -p goal_timeout_sec:=35.0
"""

import csv
import copy
import json
import os
import math
import time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose


def _as_bool(value):
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _grid_value_at(grid, x, y):
    info = grid.info
    if info.width <= 0 or info.height <= 0 or info.resolution <= 0.0:
        return None
    origin = info.origin.position
    yaw = _yaw_from_quat(info.origin.orientation)
    dx = x - origin.x
    dy = y - origin.y
    if abs(yaw) > 1.0e-6:
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        gx = c * dx - s * dy
        gy = s * dx + c * dy
    else:
        gx = dx
        gy = dy
    mx = int(math.floor(gx / info.resolution))
    my = int(math.floor(gy / info.resolution))
    if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
        return None
    offset = my * info.width + mx
    if offset < 0 or offset >= len(grid.data):
        return None
    return int(grid.data[offset])


class WaypointNavNode(Node):

    def __init__(self):
        super().__init__('waypoint_nav')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('loop', True)
        self.declare_parameter('start_delay_sec', 5.0)
        # 各ウェイポイントへの到達猶予 [s]。これを超えたらスキップして次へ。
        # generate_waypoints が測地距離 TSP で巡回順を作るため連続点間は spacing
        # （~1.5m）程度で大ジャンプは無い。低速(~0.2m/s)+Nav2 が壁際で慎重になる分の
        # 余裕を見て 60s。万一 1 点で詰まっても巡回は止まらずスキップして一巡する。
        self.declare_parameter('goal_timeout_sec', 60.0)
        # NavigateToPose action server は lifecycle bringup の途中でも見えることがある。
        # その直後の goal rejection は起動待ち不足なので、同じ点を短くリトライする。
        self.declare_parameter('goal_reject_retries', 8)
        self.declare_parameter('goal_reject_retry_sec', 2.0)
        # 評価用。report_prefix が空なら従来通りログのみ。指定すると各 waypoint 確定時に
        # JSON/CSV/Markdown を更新するため、長い巡回を途中で止めても reached/missed が残る。
        self.declare_parameter('report_prefix', '')
        # 0 以下なら無効。wall clock で判定し、長時間の屋外巡回評価を bounded にする。
        self.declare_parameter('mission_timeout_sec', 0.0)
        # 空なら Nav2 側 default_bt_xml_filename を使う。屋外巡回では recovery で
        # 経路外へ押し出されるかを切り分けるため、専用 BT XML を明示できるようにする。
        self.declare_parameter('behavior_tree', '')
        # 屋外評価用。ロボット姿勢が global costmap の高コストセルへ入ったとき、
        # または goal timeout/failure が起きたときに最後の安全姿勢へ戻してから次へ進む。
        # 既定OFF。屋内巡回や従来評価には影響させない。
        self.declare_parameter('safe_pose_guard', False)
        self.declare_parameter('safe_pose_pose_topic', '/amcl_pose')
        self.declare_parameter(
            'safe_pose_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('safe_pose_cost_threshold', 80)
        self.declare_parameter('safe_pose_safe_threshold', 40)
        self.declare_parameter('safe_pose_hold_sec', 1.0)
        self.declare_parameter('safe_pose_sample_period', 0.5)
        self.declare_parameter('safe_pose_min_goal_elapsed_sec', 1.0)
        self.declare_parameter('safe_pose_recovery_timeout_sec', 25.0)
        # step_detector との連携 (屋外の段差・縁石・スタックで現在 WP を諦め次へ)。
        # 既定 OFF (屋内・既存評価には影響させない)。 屋外 waypoint_nav launch でのみ ON。
        self.declare_parameter('step_detector_avoid', False)
        self.declare_parameter(
            'step_detector_event_topic', '/step_detector/event')
        # 同種イベントの最小間隔。 1 つの段差で連発しないため。
        self.declare_parameter('step_detector_cooldown_sec', 5.0)

        path = os.path.expanduser(
            self.get_parameter('waypoints_file').value)
        self.waypoints_file = path
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = bool(self.get_parameter('loop').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.goal_reject_retries = int(
            self.get_parameter('goal_reject_retries').value)
        self.goal_reject_retry_sec = float(
            self.get_parameter('goal_reject_retry_sec').value)
        self.report_prefix = os.path.expanduser(
            str(self.get_parameter('report_prefix').value)).strip()
        self.mission_timeout = float(
            self.get_parameter('mission_timeout_sec').value)
        self.behavior_tree = os.path.expanduser(
            str(self.get_parameter('behavior_tree').value)).strip()
        self.safe_pose_guard = _as_bool(
            self.get_parameter('safe_pose_guard').value)
        self.safe_pose_pose_topic = str(
            self.get_parameter('safe_pose_pose_topic').value)
        self.safe_pose_costmap_topic = str(
            self.get_parameter('safe_pose_costmap_topic').value)
        self.safe_pose_cost_threshold = int(
            self.get_parameter('safe_pose_cost_threshold').value)
        self.safe_pose_safe_threshold = int(
            self.get_parameter('safe_pose_safe_threshold').value)
        self.safe_pose_hold_sec = float(
            self.get_parameter('safe_pose_hold_sec').value)
        self.safe_pose_sample_period = float(
            self.get_parameter('safe_pose_sample_period').value)
        self.safe_pose_min_goal_elapsed_sec = float(
            self.get_parameter('safe_pose_min_goal_elapsed_sec').value)
        self.safe_pose_recovery_timeout_sec = float(
            self.get_parameter('safe_pose_recovery_timeout_sec').value)
        self.step_detector_avoid = _as_bool(
            self.get_parameter('step_detector_avoid').value)
        self.step_detector_event_topic = str(
            self.get_parameter('step_detector_event_topic').value)
        self.step_detector_cooldown_sec = float(
            self.get_parameter('step_detector_cooldown_sec').value)
        self._last_step_event_wall = 0.0

        self.waypoints = []
        if path and os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            self.frame_id = data.get('frame_id', self.frame_id)
            self.waypoints = [self._parse_waypoint(p)
                              for p in data.get('waypoints', [])]
            self.get_logger().info(
                f'loaded {len(self.waypoints)} waypoints from {path}')
        else:
            self.get_logger().error(f'waypoints_file not found: {path}')

        self._status_pub = self.create_publisher(
            String, '/waypoint_nav/status', 10)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        if self.step_detector_avoid:
            self.create_subscription(
                String, self.step_detector_event_topic,
                self._on_step_event, 10)
            self.get_logger().info(
                f'step_detector_avoid enabled: subscribe '
                f'{self.step_detector_event_topic} '
                f'cooldown={self.step_detector_cooldown_sec:.1f}s')

        self._idx = 0
        self._reached = 0
        self._missed = []
        self._results = []
        # Nav2 feedback リアルタイム表示の状態（間引き用 + 苦戦記録用）。
        self._fb_last_pub_wall = 0.0
        self._fb_recoveries = 0
        self._fb_distance = None
        self._lap = 0
        self._run_started_wall = time.monotonic()
        self._goal_started_wall = None
        self._finished = False
        self._goal_handle = None
        self._goal_timer = None
        self._retry_timer = None
        self._mission_timer = None
        self._reject_retry_count = 0
        # 各ウェイポイント処理の世代トークン。古いコールバックを無視するのに使う。
        self._token = 0
        self._safe_pose_timer = None
        self._safe_recovery_timer = None
        self._latest_pose = None
        self._latest_costmap = None
        self._last_safe_pose = None
        self._unsafe_since = None
        self._safe_pose_last_warning_wall = 0.0
        self._safe_recovery_active = False
        self._safe_recovery_reason = ''
        self._safe_recovery_action_status = None
        self._safe_recovery_before_index = None
        self._safe_recovery_started_wall = None
        self._safe_recovery_trigger_pose = None
        self._safe_recovery_goal_pose = None
        self._safe_pose_recoveries = []

        if self.safe_pose_guard:
            pose_qos = QoSProfile(depth=10)
            pose_qos.reliability = ReliabilityPolicy.RELIABLE
            costmap_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                PoseWithCovarianceStamped,
                self.safe_pose_pose_topic,
                self._on_safe_pose_pose,
                pose_qos)
            self.create_subscription(
                OccupancyGrid,
                self.safe_pose_costmap_topic,
                self._on_safe_pose_costmap,
                costmap_qos)
            self._safe_pose_timer = self.create_timer(
                max(0.1, self.safe_pose_sample_period),
                self._check_safe_pose_guard)
            self.get_logger().info(
                'safe_pose_guard enabled: '
                f'pose={self.safe_pose_pose_topic}, '
                f'costmap={self.safe_pose_costmap_topic}, '
                f'unsafe>={self.safe_pose_cost_threshold}, '
                f'safe<={self.safe_pose_safe_threshold}')

        delay = float(self.get_parameter('start_delay_sec').value)
        self._start_timer = self.create_timer(delay, self._kick_once)
        if self.mission_timeout > 0.0:
            self._mission_timer = self.create_timer(
                1.0, self._check_mission_timeout)

    @staticmethod
    def _parse_waypoint(p):
        yaw = float(p[2]) if len(p) >= 3 and p[2] is not None else None
        return float(p[0]), float(p[1]), yaw

    @staticmethod
    def _set_yaw(pose, yaw):
        if yaw is None:
            pose.orientation.w = 1.0
            return
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)

    def _kick_once(self):
        self._start_timer.cancel()
        if not self._client.wait_for_server(timeout_sec=15.0):
            self._status('navigate_to_pose server unavailable; retry')
            self.create_timer(3.0, self._retry_start_once)
            return
        self._idx = 0
        self._reached = 0
        self._missed = []
        self._go_next()

    def _retry_start_once(self):
        for t in list(self.timers):
            if t.callback == self._retry_start_once:
                t.cancel()
        self._kick_once()

    def _go_next(self):
        if self._finished:
            return
        if self._safe_recovery_active:
            return
        if self._mission_expired():
            self._finish('mission_timeout')
            return
        if self._idx >= len(self.waypoints):
            self._status(
                f'lap finished (reached={self._reached}/{len(self.waypoints)} '
                f'missed={self._missed})')
            self._write_reports('lap_finished')
            if self.loop:
                self.create_timer(2.0, self._loop_once)
            else:
                self._finish('complete')
            return

        x, y, yaw = self.waypoints[self._idx]
        self._goal_started_wall = time.monotonic()
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        self._set_yaw(ps.pose, yaw)
        goal = NavigateToPose.Goal()
        goal.pose = ps
        if self.behavior_tree:
            goal.behavior_tree = self.behavior_tree
        # この点の処理が確定するまでのトークン。コールバックは自分のトークンが
        # 現役のときだけ前進する（タイムアウトと結果の二重前進・周回間の混線を防ぐ）。
        self._token = getattr(self, '_token', 0) + 1
        my_token = self._token
        yaw_text = '' if yaw is None else f', yaw={math.degrees(yaw):.0f}deg'
        self._status(
            f'heading to waypoint #{self._idx} ({x:.1f}, {y:.1f}{yaw_text})')
        # 到達猶予タイマ（周期タイマだが、発火時に自分のトークンを確認して1回だけ動く）。
        self._goal_timer = self.create_timer(
            self.goal_timeout, lambda: self._on_timeout(my_token))
        # Nav2 feedback をリアルタイムに見える化する（残距離・recovery 回数・経過）。
        # 「今どの点へ向かい、どれだけ進んだか、苦戦(recovery 多発)していないか」を
        # /waypoint_nav/status に流すことで、巡回状況がリアルタイムで分かる。
        self._fb_recoveries = 0
        self._fb_distance = None
        fut = self._client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(fb, my_token))
        fut.add_done_callback(
            lambda f: self._on_goal_response(f, my_token))

    def _on_feedback(self, feedback_msg, token):
        if self._finished or token != self._token:
            return
        fb = feedback_msg.feedback
        self._fb_distance = float(getattr(fb, 'distance_remaining', 0.0) or 0.0)
        prev_rec = self._fb_recoveries
        self._fb_recoveries = int(getattr(fb, 'number_of_recoveries', 0) or 0)
        nav_t = getattr(fb, 'navigation_time', None)
        nav_sec = (nav_t.sec + nav_t.nanosec * 1e-9) if nav_t is not None else 0.0
        # 毎フレーム出すと多すぎるので 1s 間隔に間引く。ただし recovery が増えた瞬間は
        # 即出す（苦戦の始まりを取りこぼさない）。
        now = time.monotonic()
        recovery_jumped = self._fb_recoveries > prev_rec
        if not recovery_jumped and now - self._fb_last_pub_wall < 1.0:
            return
        self._fb_last_pub_wall = now
        struggling = '  ⚠苦戦(recovery多発)' if self._fb_recoveries >= 2 else ''
        self._status(
            f'  -> #{self._idx} 進捗: 残り{self._fb_distance:.2f}m '
            f'recovery={self._fb_recoveries} 経過{nav_sec:.0f}s{struggling}')

    def _on_goal_response(self, future, token):
        if self._finished:
            return
        if token != self._token:
            return
        gh = future.result()
        if not gh.accepted:
            # Nav2 lifecycle activation 前の一時的な reject は同じ点を再送する。
            self._cancel_goal_timer()
            if self._reject_retry_count < self.goal_reject_retries:
                self._reject_retry_count += 1
                self._status(
                    f'waypoint #{self._idx} rejected; retry '
                    f'{self._reject_retry_count}/'
                    f'{self.goal_reject_retries}')
                self._token += 1
                self._schedule_retry()
            else:
                self._status(
                    f'waypoint #{self._idx} rejected after '
                    f'{self.goal_reject_retries} retries; skip')
                if self._start_safe_pose_recovery(reason='rejected'):
                    return
                self._advance(reached=False, reason='rejected')
            return
        self._reject_retry_count = 0
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_result(f, token))

    def _schedule_retry(self):
        self._cancel_retry_timer()
        self._retry_timer = self.create_timer(
            self.goal_reject_retry_sec, self._retry_current_once)

    def _retry_current_once(self):
        self._cancel_retry_timer()
        self._go_next()

    def _on_result(self, future, token):
        if self._finished:
            return
        if token != self._token:
            return  # 既にタイムアウト等で次へ進んだ古いゴールの結果は無視。
        self._cancel_goal_timer()
        self._goal_handle = None
        status = future.result().status
        # status 4 = SUCCEEDED。それ以外（中断/失敗）はスキップ扱い。
        if status == 4:
            self._advance(
                reached=True,
                reason='succeeded',
                action_status=int(status))
            return
        reason = f'action_status_{status}'
        if self._start_safe_pose_recovery(
                reason=reason, action_status=int(status)):
            return
        self._advance(reached=False, reason=reason, action_status=int(status))

    def _on_timeout(self, token):
        if self._finished:
            return
        if token != self._token:
            return  # 自分の点でないタイマ発火は無視。
        # トークンを進めて、この点の結果コールバックを無効化する。
        self._token += 1
        self._cancel_goal_timer()
        self._status(f'waypoint #{self._idx} timeout; skip')
        gh = self._goal_handle
        self._goal_handle = None
        if gh is not None:
            gh.cancel_goal_async()
        if self._start_safe_pose_recovery(reason='goal_timeout'):
            return
        self._advance(reached=False, reason='goal_timeout')

    def _on_step_event(self, msg):
        """step_detector からのイベントで現在 WP を諦め次へ進む。

        屋外で段差にハマる/縁石でスタックするとそれ以上前進できない。 タイマー
        満了 (`goal_timeout_sec` 既定 60s) を待つより、 即座に missed として
        次の WP へ移った方が一巡完走率が高い。
        """
        if not self.step_detector_avoid or self._finished:
            return
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        event_type = payload.get('type', '')
        if event_type not in ('tilt', 'stuck'):
            return
        now = time.monotonic()
        if now - self._last_step_event_wall < self.step_detector_cooldown_sec:
            return
        self._last_step_event_wall = now
        # 現在の goal をキャンセル + 次の WP へ
        self._token += 1
        self._cancel_goal_timer()
        gh = self._goal_handle
        self._goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
        reason = f'step_detector_{event_type}'
        self._status(
            f'waypoint #{self._idx}: {event_type} '
            f'(tilt_deg={payload.get("tilt_deg", "?")}); skip and advance')
        self._advance(reached=False, reason=reason)

    def _advance(self, reached, reason='', action_status=None):
        """現在のウェイポイントを到達/スキップとして確定し、次へ進む。"""
        if self._finished:
            return
        self._record_waypoint_result(
            reached=reached, reason=reason, action_status=action_status)
        self._write_reports('in_progress')
        self._reject_retry_count = 0
        self._cancel_retry_timer()
        self._idx += 1
        self._go_next()

    def _record_waypoint_result(self, reached, reason='', action_status=None,
                                idx=None):
        idx = self._idx if idx is None else idx
        if idx < 0 or idx >= len(self.waypoints):
            return
        x, y, yaw = self.waypoints[idx]
        elapsed = None
        if self._goal_started_wall is not None:
            elapsed = time.monotonic() - self._goal_started_wall
        if reached:
            self._reached += 1
        else:
            if idx not in self._missed:
                self._missed.append(idx)
        self._results.append({
            'lap': self._lap,
            'index': idx,
            'x': x,
            'y': y,
            'yaw': yaw,
            'result': 'reached' if reached else 'missed',
            'reason': reason,
            'action_status': action_status,
            'duration_sec': round(elapsed, 3) if elapsed is not None else None,
            'wall_elapsed_sec': round(
                time.monotonic() - self._run_started_wall, 3),
            # Nav2 feedback 由来。後から「どの点で苦戦/スキップしたか」を JSON/CSV で確認できる。
            # recoveries が多い＝spin/backup を繰り返した＝その点へ向かう経路が難しかった。
            'nav_recoveries': self._fb_recoveries,
            'nav_distance_remaining_m': (round(self._fb_distance, 3)
                                         if self._fb_distance is not None else None),
        })

    def _cancel_goal_timer(self):
        if self._goal_timer is not None:
            self._goal_timer.cancel()
            self._goal_timer = None

    def _cancel_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

    def _cancel_safe_recovery_timer(self):
        if self._safe_recovery_timer is not None:
            self._safe_recovery_timer.cancel()
            self._safe_recovery_timer = None

    def _loop_once(self):
        for t in list(self.timers):
            if t.callback == self._loop_once:
                t.cancel()
        self._idx = 0
        self._reached = 0
        self._missed = []
        self._results = []
        self._lap += 1
        self._reject_retry_count = 0
        self._go_next()

    def _status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _mission_expired(self):
        return (self.mission_timeout > 0.0 and
                time.monotonic() - self._run_started_wall >=
                self.mission_timeout)

    def _check_mission_timeout(self):
        if not self._finished and self._mission_expired():
            self._finish('mission_timeout')

    def _finish(self, reason):
        if self._finished:
            return
        self._finished = True
        self._token += 1
        self._cancel_goal_timer()
        self._cancel_retry_timer()
        self._cancel_safe_recovery_timer()
        if self._safe_pose_timer is not None:
            self._safe_pose_timer.cancel()
            self._safe_pose_timer = None
        if self._mission_timer is not None:
            self._mission_timer.cancel()
            self._mission_timer = None
        gh = self._goal_handle
        self._goal_handle = None
        if gh is not None:
            gh.cancel_goal_async()
        if reason == 'mission_timeout':
            completed = {int(r['index']) for r in self._results}
            for idx in range(self._idx, len(self.waypoints)):
                if idx in completed:
                    continue
                if idx not in self._missed:
                    self._missed.append(idx)
                x, y, yaw = self.waypoints[idx]
                duration = None
                if idx == self._idx and self._goal_started_wall is not None:
                    duration = time.monotonic() - self._goal_started_wall
                self._results.append({
                    'lap': self._lap,
                    'index': idx,
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'result': 'missed',
                    'reason': 'mission_timeout',
                    'action_status': None,
                    'duration_sec': (
                        round(duration, 3) if duration is not None else None),
                    'wall_elapsed_sec': round(
                        time.monotonic() - self._run_started_wall, 3),
                })
        self._write_reports(reason)
        self._status(
            f'mission {reason} (reached={self._reached}/{len(self.waypoints)} '
            f'missed={self._missed})')

    def _on_safe_pose_pose(self, msg):
        self._latest_pose = msg
        self._update_last_safe_pose()

    def _on_safe_pose_costmap(self, msg):
        self._latest_costmap = msg
        self._update_last_safe_pose()

    def _current_pose_stamped(self):
        if self._latest_pose is None:
            return None
        ps = PoseStamped()
        ps.header = copy.deepcopy(self._latest_pose.header)
        if not ps.header.frame_id:
            ps.header.frame_id = self.frame_id
        ps.pose = copy.deepcopy(self._latest_pose.pose.pose)
        return ps

    def _update_last_safe_pose(self):
        if not self.safe_pose_guard:
            return None
        ps = self._current_pose_stamped()
        if ps is None or self._latest_costmap is None:
            return None
        cost = _grid_value_at(
            self._latest_costmap,
            ps.pose.position.x,
            ps.pose.position.y)
        if cost is not None and 0 <= cost <= self.safe_pose_safe_threshold:
            self._last_safe_pose = ps
        return cost

    def _check_safe_pose_guard(self):
        if (self._finished or not self.safe_pose_guard or
                self._safe_recovery_active):
            return
        if self._idx < 0 or self._idx >= len(self.waypoints):
            return
        if self._goal_started_wall is None:
            return
        elapsed = time.monotonic() - self._goal_started_wall
        if elapsed < self.safe_pose_min_goal_elapsed_sec:
            return
        cost = self._update_last_safe_pose()
        if cost is None or cost < self.safe_pose_cost_threshold:
            self._unsafe_since = None
            return
        now = time.monotonic()
        if self._unsafe_since is None:
            self._unsafe_since = now
            return
        if now - self._unsafe_since < self.safe_pose_hold_sec:
            return
        if self._last_safe_pose is None:
            if now - self._safe_pose_last_warning_wall >= 5.0:
                self._safe_pose_last_warning_wall = now
                self._status(
                    f'safe_pose_guard: current cost={cost}, '
                    'but no safe pose has been observed yet')
            return
        self._start_safe_pose_recovery(reason=f'safe_pose_cost_{cost}')

    def _start_safe_pose_recovery(self, reason, action_status=None):
        if (not self.safe_pose_guard or self._finished or
                self._safe_recovery_active):
            return False
        if self._last_safe_pose is None:
            return False
        if self._idx < 0 or self._idx >= len(self.waypoints):
            return False
        self._cancel_goal_timer()
        self._cancel_retry_timer()
        gh = self._goal_handle
        self._goal_handle = None
        if gh is not None:
            gh.cancel_goal_async()
        self._token += 1
        token = self._token
        self._safe_recovery_active = True
        self._safe_recovery_reason = reason
        self._safe_recovery_action_status = action_status
        self._safe_recovery_before_index = self._idx
        self._safe_recovery_started_wall = time.monotonic()
        self._safe_recovery_trigger_pose = self._current_pose_stamped()
        self._safe_recovery_goal_pose = copy.deepcopy(self._last_safe_pose)
        self._unsafe_since = None

        goal_pose = copy.deepcopy(self._safe_recovery_goal_pose)
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        if self.behavior_tree:
            goal.behavior_tree = self.behavior_tree
        sx = goal_pose.pose.position.x
        sy = goal_pose.pose.position.y
        self._status(
            f'waypoint #{self._idx} {reason}; recover to safe pose '
            f'({sx:.2f}, {sy:.2f})')
        self._safe_recovery_timer = self.create_timer(
            self.safe_pose_recovery_timeout_sec,
            lambda: self._on_safe_recovery_timeout(token))
        fut = self._client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f: self._on_safe_recovery_goal_response(f, token))
        return True

    def _on_safe_recovery_goal_response(self, future, token):
        if (self._finished or not self._safe_recovery_active or
                token != self._token):
            return
        gh = future.result()
        if not gh.accepted:
            self._complete_safe_recovery(
                success=False,
                result_reason='safe_pose_rejected',
                action_status=None)
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_safe_recovery_result(f, token))

    def _on_safe_recovery_result(self, future, token):
        if (self._finished or not self._safe_recovery_active or
                token != self._token):
            return
        self._cancel_safe_recovery_timer()
        self._goal_handle = None
        status = int(future.result().status)
        self._complete_safe_recovery(
            success=(status == 4),
            result_reason=(
                'safe_pose_succeeded' if status == 4
                else f'safe_pose_action_status_{status}'),
            action_status=status)

    def _on_safe_recovery_timeout(self, token):
        if (self._finished or not self._safe_recovery_active or
                token != self._token):
            return
        self._token += 1
        self._cancel_safe_recovery_timer()
        gh = self._goal_handle
        self._goal_handle = None
        if gh is not None:
            gh.cancel_goal_async()
        self._complete_safe_recovery(
            success=False,
            result_reason='safe_pose_timeout',
            action_status=None)

    @staticmethod
    def _pose_xy(ps):
        if ps is None:
            return None, None
        return ps.pose.position.x, ps.pose.position.y

    def _complete_safe_recovery(self, success, result_reason, action_status):
        if not self._safe_recovery_active:
            return
        self._cancel_safe_recovery_timer()
        idx = self._safe_recovery_before_index
        trigger = self._safe_recovery_reason
        elapsed = None
        if self._safe_recovery_started_wall is not None:
            elapsed = time.monotonic() - self._safe_recovery_started_wall
        tx, ty = self._pose_xy(self._safe_recovery_trigger_pose)
        sx, sy = self._pose_xy(self._safe_recovery_goal_pose)
        self._safe_pose_recoveries.append({
            'waypoint_index': idx,
            'trigger': trigger,
            'result': 'succeeded' if success else 'failed',
            'result_reason': result_reason,
            'action_status': action_status,
            'duration_sec': round(elapsed, 3) if elapsed is not None else None,
            'trigger_pose_x': round(tx, 3) if tx is not None else None,
            'trigger_pose_y': round(ty, 3) if ty is not None else None,
            'safe_pose_x': round(sx, 3) if sx is not None else None,
            'safe_pose_y': round(sy, 3) if sy is not None else None,
            'wall_elapsed_sec': round(
                time.monotonic() - self._run_started_wall, 3),
        })
        original_status = self._safe_recovery_action_status
        self._safe_recovery_active = False
        self._safe_recovery_reason = ''
        self._safe_recovery_action_status = None
        self._safe_recovery_before_index = None
        self._safe_recovery_started_wall = None
        self._safe_recovery_trigger_pose = None
        self._safe_recovery_goal_pose = None
        self._goal_handle = None

        if self._finished:
            return
        if idx != self._idx or idx is None:
            self._write_reports('safe_pose_recovery_out_of_sync')
            return
        if success:
            self._status(
                f'waypoint #{self._idx} {trigger}; safe pose recovered')
            self._record_waypoint_result(
                reached=False,
                reason=f'{trigger}_safe_recovered',
                action_status=original_status)
            self._write_reports('in_progress')
            self._reject_retry_count = 0
            self._idx += 1
            self._go_next()
            return

        self._status(
            f'waypoint #{self._idx} {trigger}; '
            f'safe pose recovery failed: {result_reason}')
        self._record_waypoint_result(
            reached=False,
            reason=f'{trigger}_{result_reason}',
            action_status=original_status)
        self._write_reports('safe_pose_recovery_failed')
        self._idx += 1
        self._finish('safe_pose_recovery_failed')

    def _write_reports(self, reason):
        if not self.report_prefix:
            return
        prefix = self.report_prefix
        out_dir = os.path.dirname(prefix)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        completed = {int(r['index']) for r in self._results}
        pending = [
            i for i in range(len(self.waypoints))
            if i not in completed and i not in self._missed
        ]
        summary = {
            'reason': reason,
            'waypoints_file': self.waypoints_file,
            'frame_id': self.frame_id,
            'total': len(self.waypoints),
            'reached_count': self._reached,
            'missed_count': len(self._missed),
            'missed': list(self._missed),
            'pending': pending,
            'goal_timeout_sec': self.goal_timeout,
            'mission_timeout_sec': self.mission_timeout,
            'safe_pose_guard': self.safe_pose_guard,
            'safe_pose_recovery_count': len(self._safe_pose_recoveries),
            'wall_elapsed_sec': round(
                time.monotonic() - self._run_started_wall, 3),
            'loop': self.loop,
        }
        report = {
            'summary': summary,
            'waypoints': [
                {'index': i, 'x': p[0], 'y': p[1], 'yaw': p[2]}
                for i, p in enumerate(self.waypoints)
            ],
            'results': self._results,
            'safe_pose_recoveries': self._safe_pose_recoveries,
        }
        with open(prefix + '.json', 'w') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
            f.write('\n')
        with open(prefix + '.csv', 'w', newline='') as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    'lap', 'index', 'x', 'y', 'yaw', 'result', 'reason',
                    'action_status', 'duration_sec', 'wall_elapsed_sec',
                    'nav_recoveries', 'nav_distance_remaining_m',
                ])
            writer.writeheader()
            for row in self._results:
                writer.writerow(row)
        lines = [
            '# Waypoint navigation report',
            '',
            f"- reason: `{reason}`",
            f"- reached: `{self._reached}/{len(self.waypoints)}`",
            f"- missed: `{self._missed}`",
            f"- pending: `{pending}`",
            f"- elapsed: `{summary['wall_elapsed_sec']}s`",
            f"- safe_pose_guard: `{self.safe_pose_guard}`",
            f"- safe_pose_recoveries: `{len(self._safe_pose_recoveries)}`",
            f"- waypoints: `{self.waypoints_file}`",
            '',
            '| index | result | reason | duration_sec | x | y |',
            '|---:|---|---|---:|---:|---:|',
        ]
        for row in self._results:
            lines.append(
                f"| {row['index']} | {row['result']} | {row['reason']} | "
                f"{row['duration_sec']} | {row['x']:.3f} | {row['y']:.3f} |")
        if self._safe_pose_recoveries:
            lines += [
                '',
                '## Safe Pose Recoveries',
                '',
                ('| waypoint | trigger | result | reason | duration_sec | '
                 'safe_x | safe_y |'),
                '|---:|---|---|---|---:|---:|---:|',
            ]
            for row in self._safe_pose_recoveries:
                lines.append(
                    f"| {row['waypoint_index']} | {row['trigger']} | "
                    f"{row['result']} | {row['result_reason']} | "
                    f"{row['duration_sec']} | {row['safe_pose_x']} | "
                    f"{row['safe_pose_y']} |")
        with open(prefix + '.md', 'w') as f:
            f.write('\n'.join(lines) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
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
