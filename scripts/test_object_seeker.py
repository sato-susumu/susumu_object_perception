#!/usr/bin/env python3
"""object_seeker_node の状態機械・コマンド解析・ゴール計算の再現可能な単体テスト。

ROS グラフ（Gazebo / Nav2 / TF）を起動せず、ノードの純ロジックだけを検証する。
外部 I/O（TF lookup, NavigateToPose 送信, /cmd_vel publish, SQLite）はスタブに
差し替え、状態遷移とゴール座標が決定論的に正しいことを確認する。

実行: python3 scripts/test_object_seeker.py
期待: 全テストが PASS し、終了コード 0。
"""

import math
import os
import sys

# rclpy を初期化せずにノードクラスのメソッドだけ使うため、site-packages を通す。
sys.path.insert(
    0, '/home/taro/ros2_ws/install/susumu_object_perception/lib/python3.10/site-packages')

from susumu_object_perception import object_seeker_node as S  # noqa: E402
from autoware_perception_msgs.msg import (  # noqa: E402
    TrackedObjects, TrackedObject, ObjectClassification)
from std_msgs.msg import String  # noqa: E402


PASS, FAIL = 0, 0


def check(name, cond):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f'  PASS: {name}')
    else:
        FAIL += 1
        print(f'  FAIL: {name}')


def make_node():
    """ObjectSeekerNode を __init__ を回さず生成し、必要属性をスタブで埋める。"""
    n = object.__new__(S.ObjectSeekerNode)
    # パラメータ既定値を手で設定（declare_parameter を回さない）。
    n.map_frame = 'map'
    n.robot_frame = 'base_footprint'
    n.db_path = '/tmp/_seeker_test_nonexistent.sqlite3'
    n.follow_dist = 1.0
    n.approach_dist = 0.8
    n.follow_resend = 1.0
    n.lost_timeout = 3.0
    n.arrive_dist = 1.2
    n.wp_timeout = 20.0
    # 状態初期化（__init__ と同じ）。
    n.state = 'IDLE'
    n.mode = None
    n.target_class = None
    n.target_label = None
    n.latest_target_xy = None
    n.last_seen_t = None
    n.last_goal_t = None
    n.wp_index = 0
    n.wp_sent_t = None
    n.nav_busy = False
    n.nav_handle = None

    # I/O スタブ。
    n._status_log = []
    n._sent_goals = []     # NavigateToPose に送った PoseStamped
    n._cmd_log = []        # /cmd_vel に出した Twist
    n._fake_time = [0.0]
    n._robot_pos = [(0.0, 0.0)]

    n._status = lambda text: n._status_log.append(text)
    n._now = lambda: n._fake_time[0]
    n._robot_xy = lambda: n._robot_pos[0]

    # Nav 送信スタブ: ゴール座標だけ記録（accept されたとみなす）。
    def fake_send(rob, target, offset):
        # _send_approach_goal の幾何だけ再現して記録する。
        tx, ty = target
        dx, dy = tx - rob[0], ty - rob[1]
        dist = math.hypot(dx, dy)
        ux, uy = (1.0, 0.0) if dist < 1e-3 else (dx / dist, dy / dist)
        gx, gy = tx - ux * offset, ty - uy * offset
        n._sent_goals.append((gx, gy))
        n.last_goal_t = n._now()
    n._send_approach_goal = fake_send

    # _cancel_nav / cmd publish スタブ。
    n._cancel_nav = lambda: setattr(n, 'nav_busy', False)
    n.pub_cmd = type('P', (), {'publish': lambda self, m: n._cmd_log.append(m)})()
    return n


def make_tracks(objs, frame='map'):
    """(label, x, y) のリストから map フレームの TrackedObjects を作る。"""
    msg = TrackedObjects()
    msg.header.frame_id = frame
    for label, x, y in objs:
        o = TrackedObject()
        c = ObjectClassification()
        c.label = label
        o.classification = [c]
        o.kinematics.pose_with_covariance.pose.position.x = float(x)
        o.kinematics.pose_with_covariance.pose.position.y = float(y)
        msg.objects.append(o)
    return msg


# ── テスト本体 ──────────────────────────────────────────────────────────
def test_parse_class():
    print('[test] コマンド解析（_parse_class）')
    n = make_node()
    check('「人を追って」→pedestrian', n._parse_class('人を追って') == 'pedestrian')
    check('「椅子を探して」→chair', n._parse_class('椅子を探して') == 'chair')
    check('「自転車」は車に誤マッチしない', n._parse_class('自転車を探して') == 'bicycle')
    check('辞書に無い語→None', n._parse_class('宇宙船を探して') is None)


def test_command_sets_mode():
    print('[test] コマンドでモード/状態が決まる（on_command）')
    n = make_node()
    n._seed_from_memory = lambda: None  # DB を見ない
    n.on_command(String(data='人を追って'))
    check('FOLLOW モードになる', n.mode == 'FOLLOW')
    check('状態が FOLLOWING', n.state == 'FOLLOWING')
    check('target_label が PEDESTRIAN',
          n.target_label == ObjectClassification.PEDESTRIAN)

    n2 = make_node()
    n2._seed_from_memory = lambda: None
    n2.on_command(String(data='椅子を探して'))
    check('SEARCH モードになる', n2.mode == 'SEARCH')
    check('状態が SEARCHING', n2.state == 'SEARCHING')
    check('什器は target_label=None', n2.target_label is None)


def test_follow_tracks_nearest():
    print('[test] FOLLOW: トラックから最近傍の対象を選ぶ（on_tracks）')
    n = make_node()
    n._seed_from_memory = lambda: None
    # TF lookup をスタブ（map<-map の恒等変換を返す）。
    n.tf_buffer = _identity_tf()
    n.on_command(String(data='人を追って'))
    n._robot_pos[0] = (0.0, 0.0)
    # 2 人: 近い (1,0) と遠い (5,0)。
    n.on_tracks(make_tracks([
        (ObjectClassification.PEDESTRIAN, 5.0, 0.0),
        (ObjectClassification.PEDESTRIAN, 1.0, 0.0),
    ]))
    check('最近傍 (1,0) を対象に選ぶ', n.latest_target_xy == (1.0, 0.0))
    check('last_seen_t が更新される', n.last_seen_t is not None)
    # 別クラス(車)は無視される。
    n.latest_target_xy = None
    n.on_tracks(make_tracks([(ObjectClassification.CAR, 2.0, 0.0)]))
    check('対象外クラスは選ばない', n.latest_target_xy is None)


def test_approach_goal_geometry():
    print('[test] 接近ゴールは対象の手前に置かれる（on_tick→_send_approach_goal）')
    n = make_node()
    n._seed_from_memory = lambda: None
    n.tf_buffer = _identity_tf()
    n.on_command(String(data='人を追って'))
    n.state = 'FOLLOWING'
    n._robot_pos[0] = (0.0, 0.0)
    n.latest_target_xy = (3.0, 0.0)
    n.last_seen_t = 0.0
    n._fake_time[0] = 0.0
    n.last_goal_t = None
    n.on_tick()
    # 対象(3,0) の手前 follow_dist=1.0 → ゴールは (2,0)。
    check('ゴールは対象の手前 follow_distance',
          n._sent_goals and abs(n._sent_goals[-1][0] - 2.0) < 1e-6
          and abs(n._sent_goals[-1][1]) < 1e-6)


def test_arrive_and_lost():
    print('[test] 到達(ARRIVED)と見失い(SEARCH復帰)')
    n = make_node()
    n._seed_from_memory = lambda: None
    n.tf_buffer = _identity_tf()
    n.on_command(String(data='人を追って'))
    # APPROACHING で対象が arrive_dist 以内 → ARRIVED。
    n.state = 'APPROACHING'
    n._robot_pos[0] = (0.0, 0.0)
    n.latest_target_xy = (1.0, 0.0)   # 距離 1.0 < arrive_dist 1.2
    n.last_seen_t = 0.0
    n._fake_time[0] = 0.0
    n.on_tick()
    check('対象に近いと ARRIVED', n.state == 'ARRIVED')

    # FOLLOW で lost_timeout 超過 → SEARCHING + 停止。
    n2 = make_node()
    n2._seed_from_memory = lambda: None
    n2.tf_buffer = _identity_tf()
    n2.on_command(String(data='人を追って'))
    n2.state = 'FOLLOWING'
    n2.mode = 'FOLLOW'
    n2._robot_pos[0] = (0.0, 0.0)
    n2.latest_target_xy = (3.0, 0.0)
    n2.last_seen_t = 0.0
    n2._fake_time[0] = 5.0            # 5s > lost_timeout 3s
    n2.on_tick()
    check('見失うと SEARCHING に戻る', n2.state == 'SEARCHING')
    check('見失い時に停止 Twist を出す', len(n2._cmd_log) >= 1)


def test_stop_command():
    print('[test] 停止コマンドで IDLE に戻る')
    n = make_node()
    n._seed_from_memory = lambda: None
    n.on_command(String(data='人を追って'))
    n.on_command(String(data='停止'))
    check('停止で IDLE', n.state == 'IDLE')
    check('停止で target クリア', n.target_class is None)


def _identity_tf():
    """map<-任意フレーム を恒等変換で返す TF バッファのスタブ。"""
    class _Tf:
        def lookup_transform(self, target, src, t):
            class T:
                class transform:
                    class translation:
                        x = y = z = 0.0
                    class rotation:
                        x = y = z = 0.0
                        w = 1.0
            return T()
    return _Tf()


def main():
    for t in (test_parse_class, test_command_sets_mode, test_follow_tracks_nearest,
              test_approach_goal_geometry, test_arrive_and_lost, test_stop_command):
        t()
    print(f'\n=== 結果: {PASS} passed, {FAIL} failed ===')
    sys.exit(1 if FAIL else 0)


if __name__ == '__main__':
    main()
