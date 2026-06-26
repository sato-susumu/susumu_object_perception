#!/usr/bin/env python3
"""ロボットの転倒を検知するノード。

IMU の姿勢（roll/pitch）から機体の傾きを求め、しきい値（既定 45°）を超えた状態が
一定時間続いたら「転倒」と判定して警告を出す。odom の orientation は 2D 前提で
roll/pitch が常に 0 のため転倒を検知できない。IMU(InertialUnit)の 3D 姿勢を使う。

転倒判定は2系統の冗長:
  - 姿勢: quaternion から roll/pitch を算出し、傾き角 = max(|roll|,|pitch|) がしきい値超え
  - 重力: linear_acceleration の z 成分が小さい（重力が z 軸に乗っていない）

検知すると:
  - /fall_detector/status (std_msgs/Bool) に転倒中フラグを publish
  - 転倒に入った/復帰したエッジで WARN/INFO ログ
  - 動作時の監視用に、転倒中は周期的に警告ログを出す

入力 : /imu (sensor_msgs/Imu)
出力 : /fall_detector/status (Bool), /fall_detector/tilt_deg (Float32)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32

from susumu_object_perception.geometry_utils import quat_to_roll_pitch


class FallDetectorNode(Node):

    def __init__(self):
        super().__init__('fall_detector')

        self.declare_parameter('imu_topic', '/imu')
        # この傾き角[deg]を超えたら転倒候補。
        self.declare_parameter('tilt_threshold_deg', 45.0)
        # 転倒候補がこの秒数続いたら確定（一瞬の振動で誤検知しない）。
        self.declare_parameter('confirm_sec', 0.5)
        # 重力 z 成分がこの割合(対 9.81)を下回ったら横倒し補助判定。
        self.declare_parameter('gravity_z_ratio', 0.5)
        # 転倒中の警告ログ周期[s]。
        self.declare_parameter('warn_period_sec', 3.0)

        self.tilt_threshold = float(
            self.get_parameter('tilt_threshold_deg').value)
        self.confirm_sec = float(self.get_parameter('confirm_sec').value)
        self.gravity_z_ratio = float(
            self.get_parameter('gravity_z_ratio').value)
        self.warn_period = float(self.get_parameter('warn_period_sec').value)
        imu_topic = self.get_parameter('imu_topic').value

        self._fallen = False
        self._candidate_since = None
        self._last_warn = None

        self.create_subscription(Imu, imu_topic, self._on_imu,
                                 qos_profile_sensor_data)
        self._status_pub = self.create_publisher(
            Bool, '/fall_detector/status', 10)
        self._tilt_pub = self.create_publisher(
            Float32, '/fall_detector/tilt_deg', 10)

        self.get_logger().info(
            f'fall_detector started (imu={imu_topic} '
            f'threshold={self.tilt_threshold}deg confirm={self.confirm_sec}s)')

    def _on_imu(self, msg):
        roll, pitch = quat_to_roll_pitch(msg.orientation)
        tilt = math.degrees(max(abs(roll), abs(pitch)))

        # 重力補助: z 加速度が小さい = 横倒し。
        az = msg.linear_acceleration.z
        gravity_low = abs(az) < (self.gravity_z_ratio * 9.81)

        self._tilt_pub.publish(Float32(data=float(tilt)))

        now = self.get_clock().now()
        is_candidate = (tilt > self.tilt_threshold) or gravity_low

        if is_candidate:
            if self._candidate_since is None:
                self._candidate_since = now
            elapsed = (now - self._candidate_since).nanoseconds * 1e-9
            if elapsed >= self.confirm_sec and not self._fallen:
                self._fallen = True
                self._last_warn = now
                self.get_logger().error(
                    f'ROBOT FALLEN! tilt={tilt:.1f}deg (accel_z={az:.2f}). '
                    'ナビ/認識は無効です。原点へワープ等で復帰してください。')
        else:
            self._candidate_since = None
            if self._fallen:
                self._fallen = False
                self.get_logger().info(
                    f'recovered from fall (tilt={tilt:.1f}deg)')

        # 転倒中は周期的に警告。
        if self._fallen and self._last_warn is not None:
            if (now - self._last_warn).nanoseconds * 1e-9 >= self.warn_period:
                self._last_warn = now
                self.get_logger().warn(
                    f'still fallen (tilt={tilt:.1f}deg)')

        self._status_pub.publish(Bool(data=self._fallen))


def main(args=None):
    rclpy.init(args=args)
    node = FallDetectorNode()
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
