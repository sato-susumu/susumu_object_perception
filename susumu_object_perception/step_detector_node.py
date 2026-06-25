#!/usr/bin/env python3
"""屋外段差検出ノード。

屋外マッピング/巡回中の段差・縁石・急坂を検出する。 fall_detector (転倒検知 45°)
と区別し、 5-15° の中程度傾きで「段差付近」 と判定する。

検出ロジック (3 軸):
1. **IMU 傾き**: roll/pitch の絶対値が `tilt_warn_deg` 〜 `tilt_critical_deg` の
   範囲で持続したら「段差候補」 (転倒未満)。
2. **odom 進行率**: 直近窓内で /cmd_vel に対する odom 進捗率が著しく低いと
   「車輪空回り」 と判定 (段差にスタック)。
3. **IMU 加速度 z 変動**: 短時間に z 加速度が急変 (乗り上げ・落下) すると
   「段差通過」 イベント。

出力:
- `/step_detector/status` (std_msgs/Bool): 現在段差付近にいるか
- `/step_detector/event` (std_msgs/String): 検出イベント JSON
  例: `{"type": "tilt", "tilt_deg": 8.3, "ts": "..."}`
- `/step_detector/tilt_deg` (Float32): 現在の傾き角

リカバリは別ノードに任せる (例: nav2 collision_monitor、 frontier_explore の
blacklist)。 この ノードは検出のみに専念。

入力 : /imu (sensor_msgs/Imu), /odom (nav_msgs/Odometry), /cmd_vel (Twist)
出力 : /step_detector/status (Bool), /step_detector/event (String),
       /step_detector/tilt_deg (Float32)
"""

import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String


def quat_to_roll_pitch(q):
    """quaternion (x,y,z,w) -> (roll, pitch) in radians."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


class StepDetectorNode(Node):

    def __init__(self):
        super().__init__('step_detector')

        # === パラメータ ===
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # 段差警告閾値 [deg]。 これ以上の傾きで「段差候補」。
        self.declare_parameter('tilt_warn_deg', 5.0)
        # 段差クリティカル閾値 [deg]。 これ以上で「段差通過中」。
        # 転倒 (fall_detector の既定 45°) よりずっと低く。
        self.declare_parameter('tilt_critical_deg', 15.0)
        # 傾きがこの秒数続いたら確定 (一瞬の振動で誤検知しない)。
        self.declare_parameter('confirm_sec', 0.3)
        # cmd_vel に対する odom 進行率がこの値を下回ったらスタック判定。
        # 例: cmd_vel.linear.x=0.2 m/s で odom が 0.05 m/s なら 25%、 stuck。
        self.declare_parameter('stuck_progress_ratio', 0.3)
        # 進行率判定の窓 [s]。
        self.declare_parameter('stuck_window_sec', 2.0)
        # stuck 判定にこの秒数のサンプルが揃うまで発火しない (起動直後/加速時の
        # false positive を避ける)。 窓長の 80% 以上が初期値の目安。
        self.declare_parameter('stuck_min_window_fill_sec', 1.6)
        # stuck 発火後のクールダウン [s]。 1 つの段差で多発しないように。
        self.declare_parameter('stuck_cooldown_sec', 3.0)
        # 加速度 z 急変判定 [m/s^2]。 短時間に重力 (9.81) からこの値以上ズレたら
        # 「段差通過」 イベント。
        self.declare_parameter('accel_z_jolt_threshold', 4.0)
        # 警告ログ周期 [s]。
        self.declare_parameter('warn_period_sec', 3.0)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.tilt_warn = math.radians(
            float(self.get_parameter('tilt_warn_deg').value))
        self.tilt_critical = math.radians(
            float(self.get_parameter('tilt_critical_deg').value))
        self.confirm_sec = float(self.get_parameter('confirm_sec').value)
        self.stuck_progress_ratio = float(
            self.get_parameter('stuck_progress_ratio').value)
        self.stuck_window_sec = float(
            self.get_parameter('stuck_window_sec').value)
        self.stuck_min_window_fill_sec = float(
            self.get_parameter('stuck_min_window_fill_sec').value)
        self.stuck_cooldown_sec = float(
            self.get_parameter('stuck_cooldown_sec').value)
        self.jolt_threshold = float(
            self.get_parameter('accel_z_jolt_threshold').value)
        self.warn_period_sec = float(
            self.get_parameter('warn_period_sec').value)

        # 状態
        self._tilt_above_since = None
        self._in_step = False
        self._last_warn_t = 0.0
        # 直近 cmd_vel と odom サンプル
        self._cmd_vel_samples = deque(maxlen=200)
        self._odom_samples = deque(maxlen=200)
        self._last_stuck_fire_t = 0.0

        # Publisher / Subscriber
        self.pub_status = self.create_publisher(
            Bool, '/step_detector/status', 10)
        self.pub_event = self.create_publisher(
            String, '/step_detector/event', 10)
        self.pub_tilt = self.create_publisher(
            Float32, '/step_detector/tilt_deg', 10)

        self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(
            Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)

        self.get_logger().info(
            f'step_detector started (imu={self.imu_topic} '
            f'tilt warn={math.degrees(self.tilt_warn):.1f}deg '
            f'critical={math.degrees(self.tilt_critical):.1f}deg '
            f'jolt_threshold={self.jolt_threshold:.1f}m/s^2)')

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _emit_event(self, event_type, payload):
        msg = String()
        payload = dict(payload)
        payload['type'] = event_type
        payload['ts'] = self._now_sec()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_event.publish(msg)

    def _on_imu(self, msg):
        roll, pitch = quat_to_roll_pitch(msg.orientation)
        tilt = max(abs(roll), abs(pitch))
        # tilt_deg 出力
        f = Float32()
        f.data = math.degrees(tilt)
        self.pub_tilt.publish(f)
        # 加速度 z (gravity 含む) ジョルト検知
        az = msg.linear_acceleration.z
        if abs(az - 9.81) >= self.jolt_threshold:
            self._emit_event('accel_jolt', {
                'accel_z': az,
                'deviation_from_g': az - 9.81,
            })
            self.get_logger().info(
                f'step_detector: accel_z jolt {az:.2f} m/s^2 (Δ from g='
                f'{az - 9.81:+.2f})')
        # 傾き判定
        now = self._now_sec()
        if tilt >= self.tilt_warn:
            if self._tilt_above_since is None:
                self._tilt_above_since = now
            elif now - self._tilt_above_since >= self.confirm_sec:
                if not self._in_step:
                    self._in_step = True
                    self._emit_event('tilt', {
                        'tilt_deg': math.degrees(tilt),
                        'severity': 'critical' if tilt >= self.tilt_critical
                                    else 'warn',
                    })
                    self.get_logger().info(
                        f'step_detector: ENTER step state '
                        f'(tilt={math.degrees(tilt):.1f}deg)')
                # 持続中の警告ログ
                if now - self._last_warn_t >= self.warn_period_sec:
                    self.get_logger().info(
                        f'step_detector: still in step '
                        f'(tilt={math.degrees(tilt):.1f}deg)')
                    self._last_warn_t = now
        else:
            self._tilt_above_since = None
            if self._in_step:
                self._in_step = False
                self._emit_event('tilt_recover', {
                    'tilt_deg': math.degrees(tilt),
                })
                self.get_logger().info(
                    f'step_detector: EXIT step state '
                    f'(tilt={math.degrees(tilt):.1f}deg)')
        # status publish
        b = Bool()
        b.data = self._in_step
        self.pub_status.publish(b)

    def _on_odom(self, msg):
        now = self._now_sec()
        # 平面速度 [m/s]
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        self._odom_samples.append((now, speed))
        # 古いサンプル除去
        cutoff = now - self.stuck_window_sec
        while self._odom_samples and self._odom_samples[0][0] < cutoff:
            self._odom_samples.popleft()
        self._check_stuck(now)

    def _on_cmd_vel(self, msg):
        now = self._now_sec()
        speed = abs(msg.linear.x)
        self._cmd_vel_samples.append((now, speed))
        cutoff = now - self.stuck_window_sec
        while self._cmd_vel_samples and self._cmd_vel_samples[0][0] < cutoff:
            self._cmd_vel_samples.popleft()

    def _check_stuck(self, now):
        if not self._cmd_vel_samples or not self._odom_samples:
            return
        # 窓のサンプルが十分に貯まるまで判定保留 (起動直後/加速中の false positive
        # 回避)。 両方の系列で「窓内の時間長」 が閾値以上であることを要求する。
        def _series_span(samples):
            if len(samples) < 2:
                return 0.0
            return samples[-1][0] - samples[0][0]
        if (_series_span(self._cmd_vel_samples) < self.stuck_min_window_fill_sec
                or _series_span(self._odom_samples) < self.stuck_min_window_fill_sec):
            return
        # 直近の発火からクールダウン中は判定しない
        if now - self._last_stuck_fire_t < self.stuck_cooldown_sec:
            return
        cmd_avg = sum(s for _, s in self._cmd_vel_samples) / len(self._cmd_vel_samples)
        odom_avg = sum(s for _, s in self._odom_samples) / len(self._odom_samples)
        if cmd_avg < 0.05:
            # 静止指令時はスタック判定しない
            return
        ratio = odom_avg / cmd_avg if cmd_avg > 0 else 1.0
        if ratio < self.stuck_progress_ratio:
            self._last_stuck_fire_t = now
            self._emit_event('stuck', {
                'cmd_vel_avg': cmd_avg,
                'odom_avg': odom_avg,
                'progress_ratio': ratio,
            })
            self.get_logger().warning(
                f'step_detector: STUCK detected '
                f'(cmd={cmd_avg:.2f}m/s odom={odom_avg:.2f}m/s ratio={ratio:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = StepDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
