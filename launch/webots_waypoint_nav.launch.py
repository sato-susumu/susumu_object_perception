# 保存済みウェイポイントに沿って Nav2 で巡回ナビゲーションする launch。
#
# generate_waypoints.py で作った maps/<world>_waypoints.yaml を読み、
#   - webots_nav.launch.py(robot + Webots + Nav2 + slam_toolbox) で地図と TF を供給
#   - waypoint_nav_node が NavigateToPose を各点へ順に送りウェイポイントを巡回
#   - waypoint_viz_node が地図上にウェイポイントと経路を可視化(/waypoints/markers)
#
# ウェイポイントは保存地図(同じ world をロボット起動位置原点で SLAM したもの)の map 座標で
# 作られている。本 launch も同じ world を slam_toolbox で立てるので、ロボット起動位置を原点と
# する map 座標系がほぼ一致し、ウェイポイントがそのまま使える。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
#     world:=city_robot.wbt waypoints:=city_waypoints.yaml
#   ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
#     world:=outdoor.wbt waypoints:=outdoor_waypoints.yaml mode:=fast

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    world = LaunchConfiguration('world')
    waypoints = LaunchConfiguration('waypoints')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz')
    loop = LaunchConfiguration('loop')
    # 物体認識（LiDAR検出/追跡 + 全天球色付き点群 + YOLO分類）。巡回しながら検出・識別を
    # 調査したいときは perception:=True omni_perception:=True image_recognition:=True で起動する。
    use_perception = LaunchConfiguration('perception')
    use_omni_perception = LaunchConfiguration('omni_perception')
    use_image_recognition = LaunchConfiguration('image_recognition')
    indoor_objects = LaunchConfiguration('indoor_objects')

    # ウェイポイント yaml の絶対パス（maps/ 配下）。
    wp_path = PythonExpression([
        "'", os.path.join(pkg, 'maps', ''), "' + '", waypoints, "'"])

    robot_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_nav.launch.py')),
        launch_arguments=[
            ('world', world),
            ('mode', mode),
            ('rviz', use_rviz),
            ('perception', use_perception),
            ('omni_perception', use_omni_perception),
            ('image_recognition', use_image_recognition),
            ('indoor_objects', indoor_objects),
        ],
    )

    # 可視化（latched で出すので早めに起動してよい）。
    viz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='waypoint_viz_node.py',
                name='waypoint_viz',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'waypoints_file': wp_path,
                }],
            ),
        ],
    )

    # ナビ（Nav2 のアクションサーバが立つのを待って遅延起動）。
    nav = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='waypoint_nav_node.py',
                name='waypoint_nav',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'waypoints_file': wp_path,
                    'loop': loop,
                    'start_delay_sec': 3.0,
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='city_robot.wbt',
            description='ナビするワールド（city_robot.wbt / outdoor.wbt / indoor.wbt）'),
        DeclareLaunchArgument(
            'waypoints', default_value='city_waypoints.yaml',
            description='maps/ 配下のウェイポイント yaml ファイル名'),
        DeclareLaunchArgument(
            'mode', default_value='fast',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する（地図+ウェイポイント+ロボットを見る）'),
        DeclareLaunchArgument(
            'loop', default_value='True',
            description='完走後にもう一周する'),
        DeclareLaunchArgument(
            'perception', default_value='False',
            description='物体検出/追跡(Autoware perception)。巡回中の物体識別調査は True'),
        DeclareLaunchArgument(
            'omni_perception', default_value='False',
            description='全天球色付き点群/クロップ補助。物体識別調査は True'),
        DeclareLaunchArgument(
            'image_recognition', default_value='False',
            description='YOLO 物体分類 + 全天球信号認識。物体識別調査は True'),
        DeclareLaunchArgument(
            'indoor_objects', default_value='False',
            description='室内物体検出（高所除外+床付近の家具を検出/識別）。室内 world で True'),
        robot_nav,
        viz,
        nav,
    ])
