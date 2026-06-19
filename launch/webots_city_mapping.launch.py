# Webots city（city_robot.wbt）で「事前地図なし→自律探索→地図作成→保存」を 1 コマンドで。
#
# 最終目標は地図を作ること。frontier-based exploration（Yamauchi 1997 / explore_lite と
# 同系）でロボットが未知領域へ向かい続け、slam_toolbox が /map を育てる。フロンティアが
# 尽きたら探索完了として maps/city.{pgm,yaml} に保存する。
#
#   - webots_nav.launch.py を world:=city_robot.wbt で include
#     （robot + Webots + Nav2 + slam_toolbox。slam_toolbox が地図を作る本体）。
#   - frontier_explore_node が /map のフロンティアを検出し NavigateToPose で探索。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_city_mapping.launch.py
#   ros2 launch susumu_object_perception webots_city_mapping.launch.py mode:=fast
#   # 別の world でも探索マッピングできる:
#   ros2 launch susumu_object_perception webots_city_mapping.launch.py world:=outdoor.wbt map_name:=outdoor
#
# 完了後の地図は maps/<map_name>.pgm / .yaml。SLAM 中の手動保存も可能:
#   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/susumu_object_perception/maps/city
#
# 罠:
#   - Webots は GUI(X) を要求。ヘッドレスなら DISPLAY を環境側で設定する。
#   - 認識(perception/omni)は地図作成に不要なので既定 OFF（CPU を SLAM/Nav2 に回す）。

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
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz')
    use_perception = LaunchConfiguration('perception')
    use_omni_perception = LaunchConfiguration('omni_perception')
    map_name = LaunchConfiguration('map_name')
    save_map = LaunchConfiguration('save_map')
    gain = LaunchConfiguration('gain')
    min_frontier_cells = LaunchConfiguration('min_frontier_cells')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')

    # 探索向け Nav2 params（inflation を 0.35 に下げ、フロンティアゴールへの planner が
    # 通るようにする。標準の 1.0 だと 5x4m の自由空間が高コストで埋まり経路が作れない）。
    explore_params = os.path.join(pkg, 'config', 'nav2_params_webots_explore.yaml')

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
            ('nav_params_file', explore_params),
        ],
    )

    # 保存先（maps/<map_name>。map_saver は拡張子を自動付与する）。
    save_path = PythonExpression([
        "'", os.path.expanduser(
            '~/ros2_ws/src/susumu_object_perception/maps/'), "' + '",
        map_name, "'"])

    # フロンティア探索: 実績ある explore_lite(m-explore-ros2 移植)を使う。自作ノードは
    # 屋外の疎な環境で「目の前のフロンティアばかり選び前進しない」問題があり、定番実装の
    # 距離重視スコア(potential_scale)・進捗タイムアウト・spin 復帰に置き換えた。Nav2 へ
    # NavigateToPose でゴールを投げる設計(自作と同じアーキテクチャ)。/map を購読し base_link
    # 基準。frame は slam_toolbox の base_footprint に合わせる。
    # Nav2 のアクションサーバが立つのを待って遅延起動する。
    frontier = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_base_frame': 'base_footprint',
                    'costmap_topic': 'map',
                    'costmap_updates_topic': 'map_updates',
                    'visualize': True,
                    'planner_frequency': 0.33,
                    # 進捗が無いゴールは放棄して別フロンティアへ（屋外で詰まり対策）。
                    'progress_timeout': 30.0,
                    # 距離重視(explore_lite 既定 3.0)。大きいほど近いフロンティアを優先しつつ
                    # ゴールに着いたら次へ連続的に前進＝広く開拓する。
                    'potential_scale': 3.0,
                    'gain_scale': 1.0,
                    'orientation_scale': 0.0,
                    'transform_tolerance': 0.5,
                    # フロンティア最小サイズ[m]（セルでなくメートル基準＝解像度非依存）。
                    'min_frontier_size': 0.5,
                    # 探索完了後に初期位置へ戻る挙動は地図保存タイミングを乱すので無効。
                    'return_to_init': False,
                }],
                remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='city_robot.wbt',
            description='探索マッピングする world（city_robot.wbt / outdoor.wbt 等）'),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する（地図の育ちを見られる）'),
        DeclareLaunchArgument(
            'perception', default_value='False',
            description='Autoware perception（地図作成に不要なので既定 OFF。/scan は生点群'
                        'から作るので perception 非依存）'),
        DeclareLaunchArgument(
            'omni_perception', default_value='False',
            description='全天球カメラ連携（地図作成に不要なので既定 OFF）'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='30.0',
            description='frontier の 1 ゴール到達猶予[s]。短いと狭い屋内で'
                        'ブラックリスト化が多発し探索が縮こまる'),
        DeclareLaunchArgument(
            'map_name', default_value='city',
            description='保存する地図名（maps/<map_name>.pgm/.yaml）'),
        DeclareLaunchArgument(
            'save_map', default_value='True',
            description='探索完了時に地図を maps/ に保存する'),
        DeclareLaunchArgument(
            'gain', default_value='0.30',
            description='フロンティア選択の利得（大きいほど広い未踏領域を優先）'),
        DeclareLaunchArgument(
            'min_frontier_cells', default_value='4',
            description='フロンティアクラスタの最小セル数（小さいと細かい未踏も追い'
                        'ワールド全体を探索しきる。大きいと早期完了で地図が狭くなる）'),
        robot_nav,
        frontier,
    ])
