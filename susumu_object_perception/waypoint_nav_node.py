#!/usr/bin/env python3
"""ウェイポイント YAML を Nav2 FollowWaypoints で巡回するノード。

generate_waypoints.py が作った <world>_waypoints.yaml を読み、Nav2 の
FollowWaypoints アクションに投げて順番に巡回する。loop:=True なら完走後に
もう一周する。waypoint_viz_node.py と併用すると RViz で経路を見ながら走れる。

使い方:
  ros2 run susumu_object_perception waypoint_nav_node.py --ros-args \
    -p waypoints_file:=~/ros2_ws/src/susumu_object_perception/maps/city_waypoints.yaml \
    -p loop:=True
"""

import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


class WaypointNavNode(Node):

    def __init__(self):
        super().__init__('waypoint_nav')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        # 完走後にもう一周するか。
        self.declare_parameter('loop', True)
        # アクションサーバが立つのを待ってから開始する猶予 [s]。
        self.declare_parameter('start_delay_sec', 5.0)

        path = os.path.expanduser(
            self.get_parameter('waypoints_file').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = bool(self.get_parameter('loop').value)

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
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        delay = float(self.get_parameter('start_delay_sec').value)
        self._start_timer = self.create_timer(delay, self._kick_once)

    def _kick_once(self):
        self._start_timer.cancel()
        self._send()

    def _send(self):
        if not self.waypoints:
            self._status('no waypoints; abort')
            return
        if not self._client.wait_for_server(timeout_sec=10.0):
            self._status('follow_waypoints server unavailable; retry')
            self.create_timer(3.0, self._retry_once)
            return
        goal = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        for (x, y) in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            goal.poses.append(ps)
        self._status(f'following {len(self.waypoints)} waypoints')
        fut = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_response)

    def _retry_once(self):
        for t in list(self.timers):
            if t.callback == self._retry_once:
                t.cancel()
        self._send()

    def _on_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self._status('goal rejected')
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, fb):
        self._status(f'heading to waypoint #{fb.feedback.current_waypoint}')

    def _on_result(self, future):
        result = future.result().result
        missed = list(result.missed_waypoints)
        self._status(f'lap finished (missed={missed})')
        if self.loop:
            self.create_timer(2.0, self._loop_once)

    def _loop_once(self):
        for t in list(self.timers):
            if t.callback == self._loop_once:
                t.cancel()
        self._send()

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
