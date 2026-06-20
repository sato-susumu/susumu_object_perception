#!/usr/bin/env python3
"""衝突診断ノード — バンパー接触を起点に「なぜ衝突したか」を切り分ける。

break_room の地図がぶれる主因は「ナビが障害物を把握できずロボットが何かに衝突し、
その反動・押し込みで odom がずれ、SLAM のマッチングが破綻する」と推定される。
本ノードはバンパー(/bumper/collision)の立ち上がりを検知した瞬間に、衝突に関わる
状態を一度に集めてログ出力し、原因を以下の 3 系統に切り分ける:

  (A) センサに映っていなかった   … /scan に進行方向の近接点が無い
        → LiDAR の死角・高さ帯（pointcloud_to_laserscan の min/max_height）・
          maxRange/minRange の問題。障害物が 2D scan に乗らない。
  (B) 映ったが costmap に乗らなかった … /scan には近接点があるが local_costmap の
        ロボット周辺が空いている
        → costmap の obstacle_layer 設定（obstacle_range / raytrace / inflation）や
          TF/odom ドリフトでマーキング位置がずれている。
  (C) costmap に乗っていたのに突っ込んだ … local_costmap にロボット前方の障害があり、
        かつ /cmd_vel が前進を命じていた
        → planner/controller・footprint・inflation_radius の問題（回避が効いていない）。

これは「衝突を止める」ためでなく「原因を診断する」ためのノード（ユーザー方針:
移動物体の無い環境での衝突はナビの把握不全かアルゴリズム不良なので切り分けて対処）。

購読:
  /bumper/collision, /bumper/{front,back,left,right} (std_msgs/Bool)
  /cmd_vel (geometry_msgs/Twist)
  /scan (sensor_msgs/LaserScan)
  /local_costmap/costmap (nav2_msgs/.. ではなく nav_msgs/OccupancyGrid)
  TF: map->base_footprint（位置）
出力:
  /collision_diagnostic/event (std_msgs/String, JSON)  … 機械可読な診断イベント
  ログ（人が読む切り分け結果）
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import tf2_ros


# 各バンパー方向のロボット座標系での向き（rad, base_footprint 前方=0, 左=+pi/2）。
_DIR_ANGLE = {
    "front": 0.0,
    "left": math.pi / 2.0,
    "right": -math.pi / 2.0,
    "back": math.pi,
}


class CollisionDiagnosticNode(Node):
    def __init__(self):
        super().__init__("collision_diagnostic")

        self.declare_parameter("near_dist", 0.4)       # 「近接」とみなす距離[m]
        self.declare_parameter("cmd_fwd_thresh", 0.02)  # 前進とみなす linear.x[m/s]
        self.declare_parameter("cone_deg", 45.0)        # 進行方向±この角度を見る
        self.near_dist = self.get_parameter("near_dist").value
        self.cmd_fwd_thresh = self.get_parameter("cmd_fwd_thresh").value
        self.cone = math.radians(self.get_parameter("cone_deg").value)

        # 直近の各種状態（衝突時に参照するためキャッシュ）。
        self._last_cmd = Twist()
        self._last_scan = None
        self._last_costmap = None
        self._prev_collision = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb,
                                 rclpy.qos.qos_profile_sensor_data)
        # costmap は TRANSIENT_LOCAL で latch される。
        cm_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self._costmap_cb, cm_qos)

        # 方向別バンパー（どちら向きの衝突かを scan/costmap の参照角に使う）。
        self._dir_hits = {d: False for d in _DIR_ANGLE}
        for d in _DIR_ANGLE:
            self.create_subscription(
                Bool, f"/bumper/{d}",
                lambda msg, dd=d: self._dir_cb(dd, msg), 10)
        self.create_subscription(Bool, "/bumper/collision", self._collision_cb, 10)

        self.event_pub = self.create_publisher(String, "/collision_diagnostic/event", 10)

        self.collision_count = 0
        self.get_logger().info(
            "衝突診断ノード起動。/bumper/collision を監視し衝突時に原因を切り分けます")

    # ---- キャッシュ更新 ----
    def _cmd_cb(self, msg):
        self._last_cmd = msg

    def _scan_cb(self, msg):
        self._last_scan = msg

    def _costmap_cb(self, msg):
        self._last_costmap = msg

    def _dir_cb(self, d, msg):
        self._dir_hits[d] = msg.data

    # ---- 衝突イベント ----
    def _collision_cb(self, msg):
        collision = msg.data
        # 立ち上がり（非接触→接触）でのみ診断する。
        if collision and not self._prev_collision:
            self._diagnose()
        self._prev_collision = collision

    def _diagnose(self):
        self.collision_count += 1
        hit_dirs = [d for d, v in self._dir_hits.items() if v]
        if not hit_dirs:
            hit_dirs = ["front"]  # 集約のみ True の取りこぼし時は前方を仮定
        # 衝突の代表方向（複数なら front 優先で 1 つ）。
        primary = next((d for d in ("front", "left", "right", "back") if d in hit_dirs),
                       hit_dirs[0])
        angle = _DIR_ANGLE[primary]

        scan_near = self._scan_min_in_cone(angle)
        costmap_blocked = self._costmap_blocked_ahead(angle)
        cmd_fwd = self._last_cmd.linear.x
        cmd_forward = cmd_fwd > self.cmd_fwd_thresh

        # ---- 切り分け ----
        scan_sees = scan_near is not None and scan_near <= self.near_dist
        if not scan_sees:
            cause = "A_not_in_scan"
            advice = ("障害物が /scan に乗っていない。LiDAR 死角 / "
                      "pointcloud_to_laserscan の高さ帯(min/max_height) / "
                      "range_min を疑う")
        elif costmap_blocked is False:
            cause = "B_not_in_costmap"
            advice = ("scan には近接点があるが local_costmap が空。obstacle_layer の "
                      "obstacle_range/raytrace_range、TF/odom ドリフトを疑う")
        elif cmd_forward:
            cause = "C_planned_into_obstacle"
            advice = ("costmap に障害があるのに前進指令。footprint/inflation_radius・"
                      "controller の回避が効いていない")
        else:
            cause = "D_drift_or_residual"
            advice = ("scan/costmap は障害を認識し前進指令も無い。odom ドリフトで実機が"
                      "押し込まれた / 旋回中の接触の可能性")

        scan_str = f"{scan_near:.2f}m" if scan_near is not None else "なし"
        cm_str = {True: "占有", False: "空き", None: "不明"}[costmap_blocked]
        self.get_logger().warn(
            f"[衝突#{self.collision_count}] 方向={primary} 原因={cause}\n"
            f"  scan最近接({primary}方向±{math.degrees(self.cone):.0f}deg)={scan_str} "
            f"(<= {self.near_dist}m を近接とみなす)\n"
            f"  local_costmap前方={cm_str}  cmd_vel.x={cmd_fwd:+.2f}m/s "
            f"(前進={cmd_forward})\n"
            f"  → {advice}")

        event = {
            "count": self.collision_count,
            "direction": primary,
            "all_dirs": hit_dirs,
            "cause": cause,
            "scan_near_m": round(scan_near, 3) if scan_near is not None else None,
            "costmap_blocked": costmap_blocked,
            "cmd_vel_x": round(cmd_fwd, 3),
            "advice": advice,
        }
        self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def _scan_min_in_cone(self, center_angle):
        """進行方向 center_angle を中心に ±cone の範囲で /scan の最小距離を返す。"""
        scan = self._last_scan
        if scan is None or not scan.ranges:
            return None
        best = None
        a = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                # center_angle に対する角度差を [-pi,pi] に正規化。
                d = math.atan2(math.sin(a - center_angle), math.cos(a - center_angle))
                if abs(d) <= self.cone:
                    if best is None or r < best:
                        best = r
            a += scan.angle_increment
        return best

    def _costmap_blocked_ahead(self, center_angle):
        """ロボット前方(近接距離内, center_angle方向)の local_costmap が占有か。

        True=占有 / False=空き / None=判定不能（costmap/TF 未取得）。
        """
        cm = self._last_costmap
        if cm is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                cm.header.frame_id, "base_footprint", rclpy.time.Time())
        except Exception:
            return None

        # ロボット位置と yaw。
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        heading = yaw + center_angle

        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        w = cm.info.width
        h = cm.info.height

        # 近接距離内を細かくサンプリングし、1 セルでも占有(>=lethal閾値)なら True。
        blocked = False
        steps = max(2, int(self.near_dist / max(res, 0.01)))
        for i in range(1, steps + 1):
            dist = self.near_dist * i / steps
            px = rx + dist * math.cos(heading)
            py = ry + dist * math.sin(heading)
            gx = int((px - ox) / res)
            gy = int((py - oy) / res)
            if 0 <= gx < w and 0 <= gy < h:
                val = cm.data[gy * w + gx]
                if val >= 90:  # inscribed/lethal 付近
                    blocked = True
                    break
        return blocked


def main():
    rclpy.init()
    node = CollisionDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
