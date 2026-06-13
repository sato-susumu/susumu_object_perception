#!/usr/bin/env python3
"""Walk on the right-hand side of a chosen person.

Subscribes to LiDAR-derived person detections (/perception/persons, PoseArray in
a fixed frame) — it never reads HuNavSim ground truth. It:

  1. Locks onto one target (the nearest *moving* person at startup).
  2. Each cycle re-finds that target by nearest-neighbour gating around its last
     position, and estimates the person's heading from its motion.
  3. Computes a goal pose offset to the RIGHT of the person's heading (so the
     robot walks abreast on the person's right) and sends it to Nav2
     (NavigateToPose). Nav2 handles path planning + obstacle avoidance.
  4. If the target is lost, it stops and waits, retrying to re-acquire the SAME
     target for a timeout before giving up the lock.

"Right" = right of the person's direction of travel. If the person is (nearly)
stationary, the last known heading is reused.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FollowPerson(Node):
    def __init__(self):
        super().__init__('follow_person')

        self.declare_parameter('persons_topic', '/perception/persons')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('side_offset', 1.0)      # lateral distance on the right [m]
        self.declare_parameter('back_offset', 0.2)      # how far behind abreast [m] (+ = behind)
        self.declare_parameter('min_speed_for_heading', 0.06)  # [m/s] (people walk ~0.2 m/s)
        self.declare_parameter('vel_smoothing', 0.8)    # velocity low-pass (0..1, higher=smoother)
        self.declare_parameter('heading_smoothing', 0.85)  # heading angle low-pass (0..1)
        self.declare_parameter('goal_period', 0.6)      # resend goal every N s
        self.declare_parameter('goal_eps', 0.35)        # only resend if goal moved > eps [m]
        self.declare_parameter('lock_gate', 1.6)        # re-acquire gate [m]
        # People move slowly, so a brief detection gap must NOT drop the lock.
        self.declare_parameter('lost_timeout', 20.0)    # give up lock after [s]
        self.declare_parameter('select_max_range', 8.0)  # only lock targets within [m]

        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Locked target state (in fixed frame).
        self.target_xy = None
        self.target_vel = np.zeros(2)
        self.target_heading = None
        self.last_seen = None
        self.last_goal = None

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.sub = self.create_subscription(
            PoseArray, self.get_parameter('persons_topic').value,
            self.persons_cb, QoSProfile(depth=10))
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/follow/goal_marker', 10)

        self.create_timer(self.get_parameter('goal_period').value, self.tick)

        self.get_logger().info('follow_person up: waiting for persons + Nav2 server...')

    # ------------------------------------------------------------------
    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.base_frame, rclpy.time.Time())
            return np.array([tf.transform.translation.x, tf.transform.translation.y])
        except TransformException:
            return None

    def persons_cb(self, msg: PoseArray):
        now = self.get_clock().now().nanoseconds * 1e-9
        persons = []
        for p in msg.poses:
            xy = np.array([p.position.x, p.position.y])
            q = p.orientation
            heading = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
            persons.append((xy, heading))
        if not persons:
            return

        if self.target_xy is None:
            self._acquire(persons, now)
        else:
            self._reacquire(persons, now)

    def _acquire(self, persons, now):
        """Lock onto the nearest moving person at startup."""
        rxy = self.robot_xy()
        if rxy is None:
            return
        max_rng = self.get_parameter('select_max_range').value
        best, best_d = None, max_rng
        for xy, heading in persons:
            d = float(np.linalg.norm(xy - rxy))
            # Prefer a person with a defined heading (i.e. actually moving), but
            # accept any in range; we still need *a* target.
            if d < best_d:
                best, best_d = (xy, heading), d
        if best is not None:
            self.target_xy, self.target_heading = best[0], best[1]
            self.target_vel = np.zeros(2)
            self.last_seen = now
            self.get_logger().info(
                f'Locked target at ({self.target_xy[0]:.2f}, {self.target_xy[1]:.2f}), '
                f'dist {best_d:.2f} m')

    def _reacquire(self, persons, now):
        """Find the locked target again by nearest-neighbour gating.

        The gate is centred on the *predicted* position (last position advanced by
        the estimated velocity), so a walking person stays matched even at 1+ m/s.
        """
        gate = self.get_parameter('lock_gate').value
        dt_pred = min(max(now - self.last_seen, 0.0), 1.0)
        predicted = self.target_xy + self.target_vel * dt_pred

        best, best_d = None, gate
        for xy, heading in persons:
            d = float(np.linalg.norm(xy - predicted))
            if d < best_d:
                best, best_d = (xy, heading), d
        if best is not None:
            new_xy, _ = best
            min_sp = self.get_parameter('min_speed_for_heading').value
            dt = max(now - self.last_seen, 1e-3)
            raw_vel = (new_xy - self.target_xy) / dt
            # Strongly smooth the velocity estimate (slow people + jitter make the
            # raw direction swing wildly). Heavy low-pass keeps the heading steady.
            vsm = self.get_parameter('vel_smoothing').value
            self.target_vel = (1.0 - vsm) * raw_vel + vsm * self.target_vel
            # Update heading only when clearly moving, and low-pass the ANGLE itself
            # so the right-side goal doesn't whip around (reduces over-turning).
            if float(np.linalg.norm(self.target_vel)) > min_sp:
                target_h = math.atan2(self.target_vel[1], self.target_vel[0])
                if self.target_heading is None:
                    self.target_heading = target_h
                else:
                    self.target_heading = self._smooth_angle(
                        self.target_heading, target_h,
                        self.get_parameter('heading_smoothing').value)
            self.target_xy = new_xy
            self.last_seen = now

    @staticmethod
    def _smooth_angle(prev, new, keep):
        """Angular low-pass: blend toward `new` keeping `keep` of `prev` (wrap-safe)."""
        diff = math.atan2(math.sin(new - prev), math.cos(new - prev))
        return prev + (1.0 - keep) * diff

    # ------------------------------------------------------------------
    def tick(self):
        if self.target_xy is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        # Lost handling: stop + wait, drop lock after timeout.
        if self.last_seen is None or (now - self.last_seen) > \
                self.get_parameter('lost_timeout').value:
            self.get_logger().warn('Target lost — dropping lock, waiting in place.')
            self.target_xy = None
            self.target_vel = np.zeros(2)
            self.target_heading = None
            self.last_goal = None
            return

        goal_xy, goal_yaw = self._right_side_goal()
        # Only resend if the goal moved meaningfully (avoid spamming Nav2).
        eps = self.get_parameter('goal_eps').value
        if self.last_goal is not None and \
                float(np.linalg.norm(goal_xy - self.last_goal)) < eps:
            self._publish_goal_marker(goal_xy, goal_yaw)
            return

        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server(timeout_sec=0.1)
            if not self.nav_client.server_is_ready():
                return

        self._send_goal(goal_xy, goal_yaw)
        self.last_goal = goal_xy
        self._publish_goal_marker(goal_xy, goal_yaw)

    def _right_side_goal(self):
        """Position to the RIGHT of the person's heading, slightly behind abreast."""
        side = self.get_parameter('side_offset').value
        back = self.get_parameter('back_offset').value
        h = self.target_heading if self.target_heading is not None else 0.0
        fwd = np.array([math.cos(h), math.sin(h)])
        # Right-hand normal of the heading vector is (sin h, -cos h).
        right = np.array([math.sin(h), -math.cos(h)])
        goal_xy = self.target_xy + side * right - back * fwd
        # Face the same way the person walks (so the robot stays abreast).
        return goal_xy, h

    def _send_goal(self, goal_xy, goal_yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.fixed_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(goal_xy[0])
        ps.pose.position.y = float(goal_xy[1])
        qx, qy, qz, qw = yaw_to_quat(goal_yaw)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = ps
        self.nav_client.send_goal_async(goal)  # fire-and-forget; superseded next tick

    def _publish_goal_marker(self, goal_xy, goal_yaw):
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'follow_goal'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = float(goal_xy[0])
        m.pose.position.y = float(goal_xy[1])
        m.pose.position.z = 0.1
        _, _, qz, qw = yaw_to_quat(goal_yaw)
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        m.scale.x, m.scale.y, m.scale.z = 0.5, 0.1, 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
        arr = MarkerArray()
        arr.markers.append(m)
        self.goal_marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FollowPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
