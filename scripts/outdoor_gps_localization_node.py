#!/usr/bin/env python3
"""GPS-based global localization bridge for sparse outdoor navigation.

This node turns the Webots GPS stream into a local ENU-like ``map`` pose and
publishes ``map -> odom`` so Nav2-compatible consumers can keep using the
standard ``map -> odom -> base_footprint`` chain without SLAM or AMCL.
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


SUPPORTED_GPS_TYPES = {
    'sensor_msgs/msg/NavSatFix': NavSatFix,
    'geometry_msgs/msg/PointStamped': PointStamped,
    'geometry_msgs/msg/Vector3Stamped': Vector3Stamped,
    'nav_msgs/msg/Odometry': Odometry,
}
EARTH_RADIUS_M = 6378137.0


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OutdoorGpsLocalization(Node):
    def __init__(self):
        super().__init__('outdoor_gps_localization')
        self.declare_parameter('gps_topic', 'auto')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('gps_map_origin_x', 0.0)
        self.declare_parameter('gps_map_origin_y', 0.0)
        self.declare_parameter('heading_source', 'imu')  # imu, odom, gps_motion
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('min_motion_heading_m', 0.05)

        self.gps_topic_param = self.get_parameter('gps_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.gps_map_origin_x = float(
            self.get_parameter('gps_map_origin_x').value)
        self.gps_map_origin_y = float(
            self.get_parameter('gps_map_origin_y').value)
        self.heading_source = self.get_parameter('heading_source').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.min_motion_heading = float(
            self.get_parameter('min_motion_heading_m').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.gps_sub = None
        self.gps_topic = None
        self.gps_type = None
        self.gps_origin = None
        self.latlon_origin = None
        self.map_xy = None
        self.prev_map_xy = None
        self.motion_yaw = None
        self.imu_yaw = None
        self.odom_xy = None
        self.odom_yaw = None

        self.odom_pub = self.create_publisher(
            Odometry, '/outdoor_gps/odometry', 10)
        self.status_pub = self.create_publisher(
            String, '/outdoor_gps/status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.create_timer(1.0, self._try_attach_gps)
        self._status('outdoor GPS localization waiting for GPS')

    def _status(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _candidate_topics(self):
        if self.gps_topic_param != 'auto':
            return [self.gps_topic_param]
        names = [name for name, _ in self.get_topic_names_and_types()]
        preferred = ['/gps', '/TurtleBot3Burger/gps']
        out = [topic for topic in preferred if topic in names]
        out.extend(sorted(name for name in names
                          if name.endswith('/gps') and name not in out))
        return out

    def _try_attach_gps(self):
        if self.gps_sub is not None:
            return True
        topics = dict(self.get_topic_names_and_types())
        for topic in self._candidate_topics():
            for type_name in topics.get(topic, []):
                msg_type = SUPPORTED_GPS_TYPES.get(type_name)
                if msg_type is None:
                    continue
                self.gps_topic = topic
                self.gps_type = type_name
                self.gps_sub = self.create_subscription(
                    msg_type, topic, self._gps_cb, 10)
                self._status(f'using GPS topic {topic} ({type_name})')
                return True
        return False

    def _gps_cb(self, msg):
        raw_xy = self._gps_xy(msg)
        if raw_xy is None:
            return
        if self.gps_origin is None:
            self.gps_origin = raw_xy
            self._status(
                f'gps origin set raw=({raw_xy[0]:.3f}, {raw_xy[1]:.3f})')
        rel_x = raw_xy[0] - self.gps_origin[0]
        rel_y = raw_xy[1] - self.gps_origin[1]
        self.prev_map_xy = self.map_xy
        self.map_xy = (
            self.gps_map_origin_x + rel_x,
            self.gps_map_origin_y + rel_y,
        )
        if self.prev_map_xy is not None:
            dx = self.map_xy[0] - self.prev_map_xy[0]
            dy = self.map_xy[1] - self.prev_map_xy[1]
            if math.hypot(dx, dy) >= self.min_motion_heading:
                self.motion_yaw = math.atan2(dy, dx)

    def _gps_xy(self, msg):
        if isinstance(msg, NavSatFix):
            if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
                return None
            lat = math.radians(float(msg.latitude))
            lon = math.radians(float(msg.longitude))
            if self.latlon_origin is None:
                self.latlon_origin = (lat, lon)
            lat0, lon0 = self.latlon_origin
            x = (lon - lon0) * math.cos(lat0) * EARTH_RADIUS_M
            y = (lat - lat0) * EARTH_RADIUS_M
            return x, y
        if isinstance(msg, PointStamped):
            return float(msg.point.x), float(msg.point.y)
        if isinstance(msg, Vector3Stamped):
            return float(msg.vector.x), float(msg.vector.y)
        if isinstance(msg, Odometry):
            p = msg.pose.pose.position
            return float(p.x), float(p.y)
        return None

    def _imu_cb(self, msg):
        self.imu_yaw = quat_to_yaw(msg.orientation)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (float(p.x), float(p.y))
        self.odom_yaw = quat_to_yaw(msg.pose.pose.orientation)

    def _map_yaw(self):
        if self.heading_source == 'odom' and self.odom_yaw is not None:
            return self.odom_yaw
        if self.heading_source == 'gps_motion' and self.motion_yaw is not None:
            return self.motion_yaw
        if self.imu_yaw is not None:
            return self.imu_yaw
        if self.odom_yaw is not None:
            return self.odom_yaw
        return self.motion_yaw

    def _tick(self):
        if self.map_xy is None or self.odom_xy is None:
            return
        map_yaw = self._map_yaw()
        if map_yaw is None:
            return
        odom_yaw = 0.0 if self.odom_yaw is None else self.odom_yaw
        map_to_odom_yaw = wrap_angle(map_yaw - odom_yaw)
        c = math.cos(map_to_odom_yaw)
        s = math.sin(map_to_odom_yaw)
        odom_x, odom_y = self.odom_xy
        tx = self.map_xy[0] - (c * odom_x - s * odom_y)
        ty = self.map_xy[1] - (s * odom_x + c * odom_y)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.map_xy[0]
        odom.pose.pose.position.y = self.map_xy[1]
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(map_yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = tx
        tf.transform.translation.y = ty
        tf.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(map_to_odom_yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OutdoorGpsLocalization()
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
