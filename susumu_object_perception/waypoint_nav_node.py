#!/usr/bin/env python3
"""ウェイポイント YAML を Nav2 で順に巡回するノード（各点タイムアウト付き）。

generate_waypoints.py が作った <world>_waypoints.yaml を読み、各ウェイポイントへ
NavigateToPose で順に向かう。到達したら次へ、`goal_timeout_sec` 以内に到達できなければ
その点を「スキップ（missed）」して次へ進む。これにより 1 点で詰まっても巡回が止まらず、
全点を一巡できる（FollowWaypoints 丸投げだと 1 点で延々リトライして完走しない問題への対処）。

1 周終わると到達数・スキップ数を報告し、loop:=True なら次の周回を始める。

使い方:
  ros2 run susumu_object_perception waypoint_nav_node.py --ros-args \
    -p waypoints_file:=.../maps/city_waypoints.yaml -p loop:=True -p goal_timeout_sec:=35.0
"""

import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


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

        path = os.path.expanduser(
            self.get_parameter('waypoints_file').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = bool(self.get_parameter('loop').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_sec').value)

        self.waypoints = []
        if path and os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            self.frame_id = data.get('frame_id', self.frame_id)
            self.waypoints = [(float(p[0]), float(p[1]))
                              for p in data.get('waypoints', [])]
            self.get_logger().info(
                f'loaded {len(self.waypoints)} waypoints from {path}')
        else:
            self.get_logger().error(f'waypoints_file not found: {path}')

        self._status_pub = self.create_publisher(
            String, '/waypoint_nav/status', 10)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._idx = 0
        self._reached = 0
        self._missed = []
        self._goal_handle = None
        self._goal_timer = None
        # 各ウェイポイント処理の世代トークン。古いコールバックを無視するのに使う。
        self._token = 0

        delay = float(self.get_parameter('start_delay_sec').value)
        self._start_timer = self.create_timer(delay, self._kick_once)

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
        if self._idx >= len(self.waypoints):
            self._status(
                f'lap finished (reached={self._reached}/{len(self.waypoints)} '
                f'missed={self._missed})')
            if self.loop:
                self.create_timer(2.0, self._loop_once)
            return

        x, y = self.waypoints[self._idx]
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = ps
        # この点の処理が確定するまでのトークン。コールバックは自分のトークンが
        # 現役のときだけ前進する（タイムアウトと結果の二重前進・周回間の混線を防ぐ）。
        self._token = getattr(self, '_token', 0) + 1
        my_token = self._token
        self._status(f'heading to waypoint #{self._idx} ({x:.1f}, {y:.1f})')
        # 到達猶予タイマ（周期タイマだが、発火時に自分のトークンを確認して1回だけ動く）。
        self._goal_timer = self.create_timer(
            self.goal_timeout, lambda: self._on_timeout(my_token))
        fut = self._client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f: self._on_goal_response(f, my_token))

    def _on_goal_response(self, future, token):
        if token != self._token:
            return
        gh = future.result()
        if not gh.accepted:
            # 受理されない点はスキップ。
            self._cancel_goal_timer()
            self._advance(reached=False)
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_result(f, token))

    def _on_result(self, future, token):
        if token != self._token:
            return  # 既にタイムアウト等で次へ進んだ古いゴールの結果は無視。
        self._cancel_goal_timer()
        self._goal_handle = None
        status = future.result().status
        # status 4 = SUCCEEDED。それ以外（中断/失敗）はスキップ扱い。
        self._advance(reached=(status == 4))

    def _on_timeout(self, token):
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
        self._advance(reached=False)

    def _advance(self, reached):
        """現在のウェイポイントを到達/スキップとして確定し、次へ進む。"""
        if reached:
            self._reached += 1
        else:
            if self._idx not in self._missed:
                self._missed.append(self._idx)
        self._idx += 1
        self._go_next()

    def _cancel_goal_timer(self):
        if self._goal_timer is not None:
            self._goal_timer.cancel()
            self._goal_timer = None

    def _loop_once(self):
        for t in list(self.timers):
            if t.callback == self._loop_once:
                t.cancel()
        self._idx = 0
        self._reached = 0
        self._missed = []
        self._go_next()

    def _status(self, text):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))


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
