# Webots city（車・信号・歩行者が動く街）+ ROS2 認識 launch。
#
# 2 モード:
#   ros2:=true（既定） … city にセンサ付き TurtleBot3 を組み込んだ webots_worlds/city_robot.wbt
#                        を起動し、ROS2 認識（LiDAR perception + 全天球色付き点群 + 信号認識 +
#                        物体画像分類）を回す。車・歩行者・信号を認識する用途。
#   ros2:=false        … Webots 標準同梱の city_traffic.wbt 等をそのまま起動する眺めるだけのデモ
#                        （従来動作、SUMO で車 100 台が走る）。ROS2 連携なし。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_city.launch.py                 # ros2 認識あり(city_robot)
#   ros2 launch susumu_object_perception webots_city.launch.py mode:=fast
#   ros2 launch susumu_object_perception webots_city.launch.py image_recognition:=False  # YOLO/信号認識OFF
#   ros2 launch susumu_object_perception webots_city.launch.py ros2:=false world:=city_traffic  # 眺めるデモ
#
# 罠（docs/webots_simulation.md）:
#   - SUMO_HOME 未設定だと city_traffic で「SUMO not found」。本 launch は既定 /usr/share/sumo を設定。
#   - Webots は GUI(X) を要求。ヘッドレスなら DISPLAY を環境側で設定しておくこと。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher


# city_traffic 等は webots 標準の vehicles/worlds 配下にある（ros2:=false の眺めるデモ用）。
WORLDS_DIR = '/usr/local/webots/projects/vehicles/worlds'


def _demo_setup(context, *args, **kwargs):
    """ros2:=false のとき: 標準 city world をそのまま起動（ROS2 連携なし）。"""
    world_name = LaunchConfiguration('world').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    os.environ.setdefault('SUMO_HOME', '/usr/share/sumo')
    webots = WebotsLauncher(
        world=os.path.join(WORLDS_DIR, world_name + '.wbt'),
        mode=mode,
        ros2_supervisor=False,
    )
    return [webots]


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')
    use_ros2 = LaunchConfiguration('ros2')
    mode = LaunchConfiguration('mode')
    use_image_recognition = LaunchConfiguration('image_recognition')

    # ros2:=true: city にロボットを組み込んだ city_robot.wbt を webots_simulation 経由で起動。
    # webots_simulation.launch.py が driver 配線・perception・全天球色付き点群を担う。
    # 画像認識（YOLO 物体分類 + 全天球信号認識）は webots_simulation の image_recognition に
    # 任せる（個別起動はせず DRY に）。LiDAR perception・全天球色付き点群も同 launch が担う。
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', 'city_robot.wbt'),
            ('mode', mode),
            ('nav', 'False'),       # 街認識が目的なので Nav2 は使わない
            ('slam', 'False'),
            ('perception', 'True'),          # LiDAR perception（検出・追跡・予測）
            ('omni_perception', 'True'),     # 全天球色付き点群
            ('image_recognition', use_image_recognition),  # YOLO 物体分類 + 全天球信号認識
        ],
        condition=IfCondition(use_ros2))

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros2', default_value='True',
            description=('True: city にロボットを置き ROS2 認識を回す（既定）。'
                         'False: 標準 city world を眺めるだけのデモ（ROS2 連携なし）')),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'image_recognition', default_value='True',
            description='YOLO 物体分類 + 全天球信号認識を起動する。重いときは False'),
        DeclareLaunchArgument(
            'world', default_value='city_traffic',
            description=('ros2:=false の眺めるデモで使う標準 world 名（拡張子不要）。'
                         'city_traffic / city / village / highway 等')),
        sim,
        # ros2:=false のときだけ標準 world を眺めるデモ起動（OpaqueFunction は condition を
        # 取れないので内部で ros2 引数を判定する）。
        OpaqueFunction(function=_demo_setup_guarded),
    ])


def _demo_setup_guarded(context, *args, **kwargs):
    """ros2:=false のときだけ標準 world を起動（OpaqueFunction は condition を取れないので内部判定）。"""
    if LaunchConfiguration('ros2').perform(context).lower() == 'true':
        return []
    return _demo_setup(context)
