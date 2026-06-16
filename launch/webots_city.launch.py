# Webots city_traffic 起動 launch（車・信号・歩行者が動く街、SUMO 連携）。
#
# Webots 標準同梱の city_traffic.wbt を起動する。SUMO（都市交通シミュレータ）連携で
# 車が信号を守って自律走行し（最大100台）、信号機・歩行者も動く。
# このロボット（車）は ROS2 連携なし＝街のデモを眺める用途。ROS2 で TurtleBot3 を
# 走らせたいときは webots_simulation.launch.py / webots_nav.launch.py を使う。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_city.launch.py            # GUI realtime
#   ros2 launch susumu_object_perception webots_city.launch.py mode:=fast # 高速
#   ros2 launch susumu_object_perception webots_city.launch.py world:=village
#
# 罠（docs/webots_simulation.md）:
#   - SUMO_HOME 未設定だと「SUMO not found」になる。本 launch は既定 /usr/share/sumo を
#     プロセス環境に設定する（環境側で別途 export 済みならそちらが優先）。
#   - Webots は GUI（X）を要求する。ヘッドレスなら DISPLAY を環境側で設定しておくこと。

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher


# city_traffic 等は webots 標準の vehicles/worlds 配下にある。world 引数で切替可能。
WORLDS_DIR = '/usr/local/webots/projects/vehicles/worlds'


def _launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    mode = LaunchConfiguration('mode').perform(context)

    # SUMO_HOME 未設定なら既定値を入れる（city_traffic の車走行に必須）。
    os.environ.setdefault('SUMO_HOME', '/usr/share/sumo')

    world_path = os.path.join(WORLDS_DIR, world_name + '.wbt')

    webots = WebotsLauncher(
        world=world_path,
        mode=mode,
        # ロボット ROS2 連携が無い純粋デモなので Ros2Supervisor は起動しない。
        ros2_supervisor=False,
    )
    return [webots]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='city_traffic',
            description=('vehicles/worlds 配下の world 名（拡張子不要）。'
                         'city_traffic / city / village / village_realistic / highway')),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）'),
        OpaqueFunction(function=_launch_setup),
    ])
