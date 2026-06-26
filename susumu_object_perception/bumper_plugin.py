#!/usr/bin/env python3
"""Webots ROS 2 driver plugin: 4 方向バンパー(TouchSensor)を ROS トピック化する。

webots_ros2_driver には DistanceSensor / LightSensor 等の static plugin はあるが
**TouchSensor 用の ROS プラグインが存在しない**（cyberbotics/webots_ros2 の
src/plugins/static を確認済み）。そのため URDF の <device> では自動 ROS 化できない。
本 plugin は driver の python plugin 機構（init/step）で 4 つの TouchSensor を
直接読み、標準型 std_msgs/Bool で publish する。独自 .msg は定義しない方針に従い、
診断用途なので「衝突有無の集約」+「方向別」を Bool で出す:

  - /bumper/collision (Bool)  … いずれかのバンパーが接触したら True（集約）
  - /bumper/front    (Bool)
  - /bumper/back     (Bool)
  - /bumper/left     (Bool)
  - /bumper/right    (Bool)

break_room の地図がぶれる原因が「ナビが障害物を把握できずロボットが衝突して
いる」かを切り分けるための計測器。bumper 型 TouchSensor は getValue() が
1.0(接触)/0.0(非接触) を返す。

driver URDF への登録（turtlebot_webots_3d.urdf）:
  <plugin type="susumu_object_perception.bumper_plugin.BumperPlugin" />
"""

import rclpy
from std_msgs.msg import Bool


# wbt の TouchSensor name と publish するトピックサフィックスの対応。
_BUMPERS = [
    ("bumper_front", "front"),
    ("bumper_back", "back"),
    ("bumper_left", "left"),
    ("bumper_right", "right"),
]


class BumperPlugin:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        # basicTimeStep[ms]。TouchSensor.enable に渡すサンプリング周期に使う。
        timestep = int(self.__robot.getBasicTimeStep())

        # plugin は driver プロセス内で動く。driver は既に rclpy.init 済みなので
        # 二重 init を避けてからノードを作る（公式 plugin_example は init するが、
        # 同一プロセスで複数 plugin がいると失敗するため未初期化時のみ）。
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node("bumper_plugin")

        self.__sensors = []
        for dev_name, suffix in _BUMPERS:
            sensor = self.__robot.getDevice(dev_name)
            if sensor is None:
                self.__node.get_logger().warn(
                    f"TouchSensor '{dev_name}' が見つかりません（wbt 未定義?）。スキップ")
                continue
            sensor.enable(timestep)
            pub = self.__node.create_publisher(Bool, f"/bumper/{suffix}", 10)
            self.__sensors.append((sensor, pub, suffix))

        self.__collision_pub = self.__node.create_publisher(Bool, "/bumper/collision", 10)
        # 値が変化したときだけログを出すための直近状態。
        self.__prev_any = False

        self.__node.get_logger().info(
            f"BumperPlugin 起動: {len(self.__sensors)} 個のバンパーを監視")

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        any_hit = False
        hit_dirs = []
        for sensor, pub, suffix in self.__sensors:
            # bumper 型は 1.0(接触)/0.0(非接触)。閾値 0.5 で bool 化。
            hit = sensor.getValue() > 0.5
            pub.publish(Bool(data=hit))
            if hit:
                any_hit = True
                hit_dirs.append(suffix)

        self.__collision_pub.publish(Bool(data=any_hit))

        # 立ち上がり（非接触→接触）でのみ警告ログ。診断ノードが詳細を出す。
        if any_hit and not self.__prev_any:
            self.__node.get_logger().warn(
                f"バンパー接触検知: {', '.join(hit_dirs)}")
        self.__prev_any = any_hit
