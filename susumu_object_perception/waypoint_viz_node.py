#!/usr/bin/env python3
"""巡回ウェイポイント YAML を RViz(MarkerArray) で可視化するノード。

generate_waypoints.py が作った <world>_waypoints.yaml を読み、各点を番号付きの
球 + 巡回順を結ぶ経路線(LINE_STRIP)として /waypoints/markers に出す。地図自体は
nav2 の map_server が /map に出すので、RViz で Map Display と MarkerArray Display を
並べれば「地図 + ウェイポイント」が見える。

使い方:
  ros2 run susumu_object_perception waypoint_viz_node.py --ros-args \
    -p waypoints_file:=~/ros2_ws/src/susumu_object_perception/outputs/waypoint_generation/city_waypoints.yaml
"""

import os
import math

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class WaypointVizNode(Node):

    def __init__(self):
        super().__init__('waypoint_viz')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        # latched 相当で出す（RViz が後から繋いでも見える）。
        self.declare_parameter('publish_period_sec', 1.0)

        path = os.path.expanduser(
            self.get_parameter('waypoints_file').value)
        self.frame_id = self.get_parameter('frame_id').value

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

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.history = HistoryPolicy.KEEP_LAST
        self._pub = self.create_publisher(
            MarkerArray, '/waypoints/markers', qos)

        period = float(self.get_parameter('publish_period_sec').value)
        self.create_timer(period, self._publish)
        self._publish()

    @staticmethod
    def _parse_waypoint(p):
        yaw = float(p[2]) if len(p) >= 3 and p[2] is not None else None
        return float(p[0]), float(p[1]), yaw

    def _publish(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 巡回経路線。
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = 'waypoint_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.8)
        line.pose.orientation.w = 1.0
        for (x, y, _yaw) in self.waypoints:
            line.points.append(Point(x=x, y=y, z=0.05))
        # ループを閉じる（最後→最初）。
        if len(self.waypoints) > 1:
            x0, y0, _yaw0 = self.waypoints[0]
            line.points.append(Point(x=x0, y=y0, z=0.05))
        arr.markers.append(line)

        for i, (x, y, yaw) in enumerate(self.waypoints):
            sph = Marker()
            sph.header.frame_id = self.frame_id
            sph.header.stamp = now
            sph.ns = 'waypoint_pts'
            sph.id = i + 1
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = x
            sph.pose.position.y = y
            sph.pose.position.z = 0.05
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.22
            # 始点は緑、他は黄。
            if i == 0:
                sph.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.95)
            else:
                sph.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.95)
            arr.markers.append(sph)

            if yaw is not None:
                arr_mark = Marker()
                arr_mark.header.frame_id = self.frame_id
                arr_mark.header.stamp = now
                arr_mark.ns = 'waypoint_yaw'
                arr_mark.id = 2000 + i
                arr_mark.type = Marker.ARROW
                arr_mark.action = Marker.ADD
                arr_mark.scale.x = 0.45
                arr_mark.scale.y = 0.08
                arr_mark.scale.z = 0.08
                arr_mark.pose.position.x = x
                arr_mark.pose.position.y = y
                arr_mark.pose.position.z = 0.12
                arr_mark.pose.orientation.z = math.sin(yaw * 0.5)
                arr_mark.pose.orientation.w = math.cos(yaw * 0.5)
                arr_mark.color = ColorRGBA(r=1.0, g=0.35, b=0.05, a=0.9)
                arr.markers.append(arr_mark)

            txt = Marker()
            txt.header.frame_id = self.frame_id
            txt.header.stamp = now
            txt.ns = 'waypoint_labels'
            txt.id = 1000 + i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = 0.35
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.3
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = str(i)
            arr.markers.append(txt)

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVizNode()
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
