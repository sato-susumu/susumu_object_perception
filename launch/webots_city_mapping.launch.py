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
    forward_step = LaunchConfiguration('forward_step')
    sweep_mode = LaunchConfiguration('sweep_mode')
    sweep_radius = LaunchConfiguration('sweep_radius')

    # 探索向け Nav2 params。屋内は static追従 costmap + Navfn の安定版、屋外(sweep_mode:=True)
    # は rolling window(40m) + Smac planner の遠征版を使う（屋外は遠方ゴールが costmap 外に
    # ならないよう rolling、Navfn の未知領域経路失敗を避けるため Smac）。sweep_mode で切替。
    explore_params = PythonExpression([
        "'", os.path.join(pkg, 'config', 'nav2_params_webots_explore_outdoor.yaml'),
        "' if '", sweep_mode, "'.lower() == 'true' else '",
        os.path.join(pkg, 'config', 'nav2_params_webots_explore.yaml'), "'"])

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

    # フロンティア探索（自作 frontier_explore_node）。explore_lite(m-explore-ros2)を試したが、
    # 屋外で「最小ゴール距離チェックが無く至近フロンティアで即到達→前進しない」「初期 spin が
    # 無く free が小さいまま鶏卵問題に陥る」弱点があり（ソース確認・公式 troubleshooting でも
    # spin 推奨）、自作ノードの方が min_goal_dist と bootstrap_spin でこれに対処できるので自作に
    # 戻す。Nav2 へ NavigateToPose で投げる設計は同じ。完了時に地図も自動保存する。
    # Nav2 のアクションサーバが立つのを待って遅延起動する。
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
                    'min_frontier_cells': min_frontier_cells,
                    'gain': gain,
                    'save_map': save_map,
                    'map_save_path': save_path,
                    'start_delay_sec': 8.0,
                    # ワールド全体を探索しきるまで粘る。
                    'done_after_empty': 12,
                    'goal_timeout_sec': goal_timeout,
                    # 至近フロンティアしか無い時の前進距離[m]。屋外の開放空間では大きくして
                    # 植木の間を抜け遠くへ一気に展開させる（forward_step:=4.0 等）。
                    'forward_step': forward_step,
                    # 非frontier的な sweep 探索（屋外の開放空間向け）。frontier 前に各方向へ
                    # 遠征し領域を舐める。sweep:=True sweep_radius:=7.0 で有効化。
                    'sweep_mode': sweep_mode,
                    'sweep_radius': sweep_radius,
                }],
            ),
        ],
    )

    # 衝突診断ノード（break_room のバンパー /bumper/collision を監視）。移動物体の無い
    # 環境での衝突は「ナビが障害物を把握できていない or アルゴリズム不良」なので、衝突時に
    # scan/costmap/cmd_vel を突き合わせて原因(A:scan非検出 / B:costmap非マーク /
    # C:回避失敗 / D:ドリフト)を切り分ける。バンパーの無い world では /bumper/* が来ない
    # だけで無害なので常時起動する。Nav2/costmap が立ってから遅延起動。
    collision_diag = TimerAction(
        period=22.0,
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
            'forward_step', default_value='2.0',
            description='至近フロンティアしか無い時に前進する距離[m]。屋外の開放空間は'
                        '4.0 程度に大きくすると遠くへ展開しやすい'),
        DeclareLaunchArgument(
            'sweep_mode', default_value='False',
            description='非frontier的な sweep 探索。屋外の開放空間で True にすると'
                        'frontier 前に各方向へ遠征し領域を舐める'),
        DeclareLaunchArgument(
            'sweep_radius', default_value='7.0',
            description='sweep の遠征半径[m]。20m world なら 7〜8'),
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
        collision_diag,
    ])
