# Webots city（city_robot.wbt）で SLAM + Nav2 + 自動巡回を 1 コマンドで。
#
# 「事前マップ無しの街で、SLAM が作る地図を見てルートを自動生成し Nav2 で巡回する」構成。
#   - webots_nav.launch.py を world:=city_robot.wbt で include（robot + Webots + Nav2 +
#     slam_toolbox。slam_toolbox が map->odom を供給し、/map を育てる）。
#   - auto_patrol_node が /map(OccupancyGrid) の自由空間からウェイポイントを自動算出し、
#     Nav2 FollowWaypoints で巡回。完走後は地図が育っていれば再計算して次の周回を出す。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_city_patrol.launch.py
#   ros2 launch susumu_object_perception webots_city_patrol.launch.py mode:=fast
#   # 認識が重ければ perception/omni/image_recognition を切る:
#   ros2 launch susumu_object_perception webots_city_patrol.launch.py \
#     perception:=False omni_perception:=False image_recognition:=False
#
# 罠:
#   - Webots は GUI(X) を要求。ヘッドレスなら DISPLAY を環境側で設定する。
#   - city は屋外で広いため、巡回開始まで SLAM が地図を出す猶予が要る（start_delay_sec）。
#   - nav/slam に小文字 true を渡すと launch 評価で NameError になる（webots_nav 内は大文字固定）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz')
    use_perception = LaunchConfiguration('perception')
    use_omni_perception = LaunchConfiguration('omni_perception')
    use_image_recognition = LaunchConfiguration('image_recognition')
    sample_step = LaunchConfiguration('sample_step')
    robot_radius = LaunchConfiguration('robot_radius')
    max_waypoints = LaunchConfiguration('max_waypoints')
    start_delay_sec = LaunchConfiguration('start_delay_sec')

    # robot + Webots + Nav2 + slam_toolbox（webots_nav が二重起動なくまとめる）。
    # city_robot.wbt を world に指定して街を起動する。
    robot_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_nav.launch.py')),
        launch_arguments=[
            ('world', 'city_robot.wbt'),
            ('mode', mode),
            ('rviz', use_rviz),
            ('perception', use_perception),
            ('omni_perception', use_omni_perception),
            ('image_recognition', use_image_recognition),
        ],
    )

    # 自動巡回ノード。SLAM が地図を出し始めてから巡回開始したいので、ノード自体の
    # start_delay_sec に加えて launch でも少し遅らせて起動する（Nav2 のアクション
    # サーバが立つのを待つ）。
    auto_patrol = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='auto_patrol_node.py',
                name='auto_patrol',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'map_frame': 'map',
                    'robot_frame': 'base_footprint',
                    'sample_step': sample_step,
                    'robot_radius': robot_radius,
                    'max_waypoints': max_waypoints,
                    'start_delay_sec': start_delay_sec,
                    'autostart': True,
                    'replan_on_finish': True,
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'perception', default_value='True',
            description='Autoware perception を起動する（重ければ False）'),
        DeclareLaunchArgument(
            'omni_perception', default_value='True',
            description='全天球カメラ連携を起動する（重ければ False）'),
        DeclareLaunchArgument(
            'image_recognition', default_value='True',
            description='YOLO 物体分類 + 全天球信号認識を起動する（重ければ False）'),
        DeclareLaunchArgument(
            'sample_step', default_value='2.0',
            description='巡回ウェイポイントのグリッド間隔 [m]（city は広いので粗め）'),
        DeclareLaunchArgument(
            'robot_radius', default_value='0.35',
            description='候補点の安全マージン半径 [m]'),
        DeclareLaunchArgument(
            'max_waypoints', default_value='20',
            description='1 周回の最大ウェイポイント数'),
        DeclareLaunchArgument(
            'start_delay_sec', default_value='10.0',
            description='auto_patrol が巡回開始を待つ起動猶予 [s]'),
        robot_nav,
        auto_patrol,
    ])
