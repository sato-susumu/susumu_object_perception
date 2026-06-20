# Webots 屋外マッピング実験用 launch（特徴が多くて狭めの屋外 world 向け、未公式サポート）。
#
# このパッケージのマッピングは原則「屋内専用」（docs/tasks/mapping_indoor.md）。屋外 world は
# docs/tasks/mapping_outdoor.md で「特徴の少ない広域屋外は未対応」と明記しているが、
# 例外として「特徴が多く狭めの屋外」(例: village_center.wbt のロボット周辺)を **explore_radius で
# マッピング範囲を半径 R[m] に制限することで** 屋内 launch とは独立に実験できるようにする。
#
# 屋内 launch (webots_indoor_mapping.launch.py) と完全に分離した実験用 launch である。
# 屋内 launch / 屋内 nav2 params は一切参照しない。屋内設定を壊さないことが第一目的。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_outdoor_mapping.launch.py \
#     world:=village_center.wbt map_name:=village_center mode:=realtime explore_radius:=12.0
#
# 必要条件:
#   - wbt 側で TurtleBot3Burger が「床のある場所」に置かれていること。
#   - explore_radius は frontier 探索の範囲制限（R[m]）。狭めにマッピングしたい屋外で必須。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz')
    use_perception = LaunchConfiguration('perception')
    use_omni_perception = LaunchConfiguration('omni_perception')
    use_image_recognition = LaunchConfiguration('image_recognition')
    use_colored_slam = LaunchConfiguration('colored_slam')
    use_collision_diagnostics = LaunchConfiguration('collision_diagnostics')
    map_name = LaunchConfiguration('map_name')
    save_map = LaunchConfiguration('save_map')
    gain = LaunchConfiguration('gain')
    min_frontier_cells = LaunchConfiguration('min_frontier_cells')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')
    explore_radius = LaunchConfiguration('explore_radius')

    # 屋外専用 Nav2 params (rolling window + Smac planner)。屋内 params は使わない。
    outdoor_params = os.path.join(
        pkg, 'config', 'nav2_params_webots_explore_outdoor.yaml')

    # robot + Webots + Nav2 + slam_toolbox。地図作成の本体（slam_toolbox が /map を出す）。
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
            ('colored_slam', use_colored_slam),
            ('nav_params_file', outdoor_params),
        ],
    )

    # 保存先（maps/<map_name>。map_saver は拡張子を自動付与する）。
    save_path = PythonExpression([
        "'", os.path.expanduser(
            '~/ros2_ws/src/susumu_object_perception/maps/'), "' + '",
        map_name, "'"])

    # フロンティア探索。屋外実験では explore_radius でロボット初期位置からの半径を制限する。
    # village_center のような広大 world でも、特徴の多い周辺だけマッピングできる。
    frontier = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='frontier_explore_node.py',
                name='frontier_explore',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'map_frame': 'map',
                    'robot_frame': 'base_footprint',
                    'world_name': world,
                    'min_frontier_cells': min_frontier_cells,
                    'gain': gain,
                    'save_map': save_map,
                    'map_save_path': save_path,
                    'start_delay_sec': 8.0,
                    'done_after_empty': 12,
                    'goal_timeout_sec': goal_timeout,
                    'forward_step': 2.0,
                    'sweep_mode': False,
                    'spin_after_goal': False,
                    'explore_radius': explore_radius,
                }],
            ),
        ],
    )

    # 衝突診断（屋外実験ではバンパーは付いていない場合が多いので既定 OFF）。
    collision_diag = TimerAction(
        period=22.0,
        condition=IfCondition(use_collision_diagnostics),
        actions=[
            Node(
                package='susumu_object_perception',
                executable='collision_diagnostic_node.py',
                name='collision_diagnostic',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='village_center.wbt',
            description='探索マッピングする屋外 world（特徴が多く狭めの実験用）'),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'perception', default_value='False',
            description='Autoware perception（地図作成に不要なので既定 OFF）'),
        DeclareLaunchArgument(
            'omni_perception', default_value='False',
            description='全天球カメラ連携（地図作成に不要なので既定 OFF）'),
        DeclareLaunchArgument(
            'image_recognition', default_value='False',
            description='YOLO 物体分類 + 全天球信号認識（既定 OFF）'),
        DeclareLaunchArgument(
            'colored_slam', default_value='False',
            description='色付き点群SLAMマップ（既定 OFF）'),
        DeclareLaunchArgument(
            'collision_diagnostics', default_value='False',
            description='衝突診断ノードを起動する'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='60.0',
            description='frontier の 1 ゴール到達猶予[s]'),
        DeclareLaunchArgument(
            'explore_radius', default_value='12.0',
            description='ロボット初期位置から半径 R[m] 以内の frontier だけ探索する '
                        '（狭めにマッピングするための制限。0 以下なら無制限）'),
        DeclareLaunchArgument(
            'map_name', default_value='village_center',
            description='保存する地図名（maps/<map_name>.pgm/.yaml）'),
        DeclareLaunchArgument(
            'save_map', default_value='True',
            description='探索完了時に地図を maps/ に保存する'),
        DeclareLaunchArgument(
            'gain', default_value='0.30',
            description='フロンティア選択の利得'),
        DeclareLaunchArgument(
            'min_frontier_cells', default_value='4',
            description='フロンティアクラスタの最小セル数'),
        robot_nav,
        frontier,
        collision_diag,
    ])
