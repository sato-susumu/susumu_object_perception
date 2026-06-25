# Webots 屋内 world（indoor.wbt / break_room.wbt 等）で「事前地図なし→自律探索→地図作成→保存」
# を 1 コマンドで実行する屋内マッピング専用 launch。
#
# 屋内マッピングと屋外マッピングはタスクとして完全に分離されている（docs/tasks/mapping_indoor.md と
# docs/tasks/mapping_outdoor.md を参照）。この launch は屋内 world 専用で、屋外 world
# （outdoor.wbt / city_robot.wbt）には対応しない。屋外向けの sweep_mode auto 判定や、
# 屋外専用 Nav2 params（rolling window + Smac）への分岐は意図的に持たない。
#
# 最終目標は地図を作ること。frontier-based exploration（Yamauchi 1997 / explore_lite と
# 同系）でロボットが未知領域へ向かい続け、slam_toolbox が /map を育てる。フロンティアが
# 尽きたら探索完了として outputs/mapping_indoor/<map_name>.{pgm,yaml} に保存する。
#
#   - webots_nav.launch.py を world:=<屋内wbt> で include
#     （robot + Webots + Nav2 + slam_toolbox。slam_toolbox が地図を作る本体）。
#   - frontier_explore_node が /map のフロンティアを検出し NavigateToPose で探索。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_indoor_mapping.launch.py world:=indoor.wbt map_name:=indoor
#   ros2 launch susumu_object_perception webots_indoor_mapping.launch.py world:=break_room.wbt map_name:=break_room
#
# 完了後の地図は outputs/mapping_indoor/<map_name>.pgm / .yaml。SLAM 中の手動保存も可能:
#   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/susumu_object_perception/outputs/mapping_indoor/<map_name>
#
# 罠:
#   - Webots は GUI(X) を要求。ヘッドレスなら DISPLAY を環境側で設定する。
#   - 認識(perception/omni)は地図作成に不要なので既定 OFF（CPU を SLAM/Nav2 に回す）。
#   - mode は realtime 必須。fast は odom が過大積算しドリフトする。

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

    # 屋内専用 Nav2 params（static追従 costmap + NavfnPlanner の安定版）。屋外専用 params
    # (nav2_params_webots_explore_outdoor.yaml) はここからは絶対に参照しない（屋内に副作用を
    # 持ち込まないため）。
    indoor_params = os.path.join(
        pkg, 'config', 'nav2_params_webots_explore.yaml')

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
            ('nav_params_file', indoor_params),
        ],
    )

    # 保存先（outputs/mapping_indoor/<map_name>。map_saver は拡張子を自動付与する）。
    save_path = PythonExpression([
        "'", os.path.expanduser(
            '~/ros2_ws/src/susumu_object_perception/outputs/mapping_indoor/'), "' + '",
        map_name, "'"])
    # vs_world 自動生成用に world ファイルの絶対パスを組み立てる。
    # source 側の webots_worlds/ を優先（install 経由でも同じ内容）。
    world_file_path = PythonExpression([
        "'", os.path.expanduser(
            '~/ros2_ws/src/susumu_object_perception/webots_worlds/'), "' + '",
        world, "'"])

    # フロンティア探索（自作 frontier_explore_node）。屋内 world では純 frontier で十分。
    # sweep_mode は False（屋外用の perimeter sweep は使わない）、forward_step は屋内向けの
    # 控えめな値で固定する。Nav2 のアクションサーバが立つのを待って遅延起動する。
    #
    # 【粘り強い探索（屋内向け）】 break_room (7.7×12.86m) のような細長い世界で「曲がり角の
    # 先の半分が未探索のまま完了する」のを防ぐため、done 条件を厳しく・stall も粘り強く設定:
    #   - done_frontier_cells=5 (既定 15)：少しでも未踏縁が残っていれば走り続ける
    #   - done_after_empty=20 (旧 12)：empty 連続判定を長く取って早期完了を避ける
    #   - stall_timeout_sec=180 (既定 120)：恐ろしく狭い通路でも諦めない
    #   - approach_setback=1.0 (既定 1.3)：曲がり角の先のフロンティアにも届かせる
    # これらは launch 引数 done_frontier_cells / done_after_empty / stall_timeout_sec /
    # approach_setback で上書き可能。
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
                    'world_file': world_file_path,
                    'start_delay_sec': 8.0,
                    'done_after_empty': LaunchConfiguration('done_after_empty'),
                    'done_frontier_cells': LaunchConfiguration('done_frontier_cells'),
                    'stall_timeout_sec': LaunchConfiguration('stall_timeout_sec'),
                    'approach_setback': LaunchConfiguration('approach_setback'),
                    'goal_timeout_sec': goal_timeout,
                    'forward_step': 2.0,
                    # 屋外専用の sweep_mode は明示的に無効化する。屋内では純 frontier で
                    # 部屋全体を回り尽くせる。
                    'sweep_mode': False,
                    'spin_after_goal': False,
                }],
            ),
        ],
    )

    # 衝突診断ノード（break_room のバンパー /bumper/collision を監視）。移動物体の無い
    # 環境での衝突は「ナビが障害物を把握できていない or アルゴリズム不良」なので、衝突時に
    # scan/costmap/cmd_vel を突き合わせて原因(A:scan非検出 / B:costmap非マーク /
    # C:回避失敗 / D:ドリフト)を切り分ける。広域マッピング検証では CPU を SLAM/Nav2 に
    # 回すため既定 OFF。必要なときだけ collision_diagnostics:=True で起動する。
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
            'world', default_value='indoor.wbt',
            description='探索マッピングする屋内 world（indoor.wbt / break_room.wbt）。'
                        '屋外 world は未対応（docs/tasks/mapping_outdoor.md 参照）'),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）。'
                        'マッピング品質を評価するときは realtime 必須'),
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
            'image_recognition', default_value='False',
            description='YOLO 物体分類 + 全天球信号認識（地図作成に不要なので既定 OFF）'),
        DeclareLaunchArgument(
            'colored_slam', default_value='False',
            description='色付き点群SLAMマップ（地図作成検証では重いので既定 OFF）'),
        DeclareLaunchArgument(
            'collision_diagnostics', default_value='False',
            description='衝突診断ノードを起動する。break_room 等で衝突原因を切り分ける時だけ True'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='30.0',
            description='frontier の 1 ゴール到達猶予[s]。短いと狭い屋内で'
                        'ブラックリスト化が多発し探索が縮こまる'),
        DeclareLaunchArgument(
            'map_name', default_value='indoor',
            description='保存する地図名（outputs/mapping_indoor/<map_name>.pgm/.yaml）'),
        DeclareLaunchArgument(
            'save_map', default_value='True',
            description='探索完了時に地図を outputs/mapping_indoor/ に保存する'),
        DeclareLaunchArgument(
            'gain', default_value='0.30',
            description='フロンティア選択の利得（大きいほど広い未踏領域を優先）'),
        DeclareLaunchArgument(
            'min_frontier_cells', default_value='2',
            description='フロンティアクラスタの最小セル数（小さいと細かい未踏も追い'
                        'ワールド全体を探索しきる。大きいと早期完了で地図が狭くなる）'),
        DeclareLaunchArgument(
            'done_frontier_cells', default_value='5',
            description='完了判定の閾値[セル数]。フロンティア総セル数がこれ未満のとき '
                        '「未踏縁ほぼ無し」と見なす。小さいほど粘る (旧既定 15)'),
        DeclareLaunchArgument(
            'done_after_empty', default_value='20',
            description='done 連続検知回数。フロンティアがほぼ無い状態が連続でこの回数 '
                        '続けば完了。大きいほど早期完了を防ぐ (旧既定 12)'),
        DeclareLaunchArgument(
            'stall_timeout_sec', default_value='180.0',
            description='既知面積の伸びが stall_min_growth_cells に満たない時間がこの '
                        '秒数超えたら強制終了。大きいほど粘る (旧既定 120)'),
        DeclareLaunchArgument(
            'approach_setback', default_value='1.0',
            description='フロンティア代表点の手前 setback[m] にゴールを置く。小さいほど '
                        '曲がり角の先のフロンティアにも届かせやすい (旧既定 1.3)'),
        robot_nav,
        frontier,
        collision_diag,
    ])
