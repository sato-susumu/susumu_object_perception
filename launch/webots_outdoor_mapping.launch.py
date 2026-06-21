# Webots 屋外マッピング launch（特徴が多い trimmed outdoor world 向け）。
#
# このパッケージのマッピングは原則「屋内専用」（docs/tasks/mapping_indoor.md）。屋外 world は
# docs/tasks/mapping_outdoor.md で定義する屋外の本線は、都市部・公園のように MID360 が
# 常に複数特徴を拾える小さめの屋外区画。屋内 launch とは独立させ、屋外用 world / params /
# waypoint wrapper だけで試行錯誤する。
#
# 屋内 launch (webots_indoor_mapping.launch.py) と完全に分離した実験用 launch である。
# 屋内 launch / 屋内 nav2 params は一切参照しない。屋内設定を壊さないことが第一目的。
#
# 使い方:
#   ros2 launch susumu_object_perception webots_outdoor_mapping.launch.py \
#     world:=village_square_trimmed.wbt map_name:=village_square_trimmed mode:=realtime \
#     explore_radius:=14.0 goal_timeout_sec:=120.0
#
# 必要条件:
#   - wbt 側で TurtleBot3Burger が「床のある場所」に置かれていること。
#   - explore_radius は frontier 探索の範囲制限（R[m]）。trimmed world の外周は通行止めで閉じる。

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
    use_truth_monitor = LaunchConfiguration('truth_monitor')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')
    scan_angle_increment = LaunchConfiguration('scan_angle_increment')
    scan_range_min = LaunchConfiguration('scan_range_min')
    scan_range_max = LaunchConfiguration('scan_range_max')
    scan_use_inf = LaunchConfiguration('scan_use_inf')
    map_name = LaunchConfiguration('map_name')
    save_map = LaunchConfiguration('save_map')
    gain = LaunchConfiguration('gain')
    min_frontier_cells = LaunchConfiguration('min_frontier_cells')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')
    explore_radius = LaunchConfiguration('explore_radius')
    max_path_goal_distance = LaunchConfiguration('max_path_goal_distance')
    staged_goal_clearance = LaunchConfiguration('staged_goal_clearance')
    yaw_watchdog = LaunchConfiguration('yaw_watchdog')
    yaw_watchdog_max_error_deg = LaunchConfiguration(
        'yaw_watchdog_max_error_deg')
    yaw_watchdog_blacklist_radius = LaunchConfiguration(
        'yaw_watchdog_blacklist_radius')
    truth_max_aligned_error = LaunchConfiguration('truth_max_aligned_error')
    truth_max_heading_error_deg = LaunchConfiguration(
        'truth_max_heading_error_deg')
    truth_max_yaw_error_deg = LaunchConfiguration('truth_max_yaw_error_deg')

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
            ('scan_min_height', scan_min_height),
            ('scan_max_height', scan_max_height),
            ('scan_angle_increment', scan_angle_increment),
            ('scan_range_min', scan_range_min),
            ('scan_range_max', scan_range_max),
            ('scan_use_inf', scan_use_inf),
        ],
    )

    # 保存先（maps/<map_name>。map_saver は拡張子を自動付与する）。
    save_path = PythonExpression([
        "'", os.path.expanduser(
            '~/ros2_ws/src/susumu_object_perception/maps/'), "' + '",
        map_name, "'"])
    truth_report_prefix = PythonExpression([
        "'", os.path.expanduser(
            '~/ros2_ws/src/susumu_object_perception/maps/'), "' + '",
        map_name, "' + '_truth_monitor'"])

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
                    'max_path_goal_distance': max_path_goal_distance,
                    'staged_goal_clearance': staged_goal_clearance,
                    'yaw_watchdog': yaw_watchdog,
                    'yaw_watchdog_imu_topic': '/imu',
                    'yaw_watchdog_max_error_deg': yaw_watchdog_max_error_deg,
                    'yaw_watchdog_blacklist_radius':
                        yaw_watchdog_blacklist_radius,
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

    # Webots GPS truth と SLAM/Nav2 の map->base_footprint をリアルタイム比較する監視。
    # 監視専用で、正解データを SLAM / Nav2 へ戻さない。大きなズレをイベントとして
    # maps/<map_name>_truth_monitor.{json,csv,md} に残す。
    truth_monitor = TimerAction(
        period=22.0,
        condition=IfCondition(use_truth_monitor),
        actions=[
            Node(
                package='susumu_object_perception',
                executable='live_slam_truth_monitor.py',
                name='live_slam_truth_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'gps_topic': 'auto',
                    'imu_topic': '/imu',
                    'estimate_frame': 'map',
                    'robot_frame': 'base_footprint',
                    'sample_period': 0.5,
                    'min_align_samples': 8,
                    'min_align_path_length': 1.0,
                    'max_aligned_error': truth_max_aligned_error,
                    'max_heading_error_deg': truth_max_heading_error_deg,
                    'max_yaw_error_deg': truth_max_yaw_error_deg,
                    'report_prefix': truth_report_prefix,
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='village_square_trimmed.wbt',
            description='探索マッピングする屋外 world（特徴が多い trimmed 区画）'),
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
            'truth_monitor', default_value='True',
            description='Webots GPS truth と map->base_footprint をリアルタイム比較して drift report を残す'),
        DeclareLaunchArgument(
            'truth_max_aligned_error', default_value='0.60',
            description='truth monitor の剛体合わせ後位置ずれイベント閾値[m]'),
        DeclareLaunchArgument(
            'truth_max_heading_error_deg', default_value='30.0',
            description='truth monitor の移動方向ずれイベント閾値[deg]'),
        DeclareLaunchArgument(
            'truth_max_yaw_error_deg', default_value='8.0',
            description='truth monitor の IMU 真値 yaw と map->base yaw の絶対方位ずれイベント閾値[deg]'),
        DeclareLaunchArgument(
            'scan_min_height', default_value='0.0',
            description='屋外専用 pointcloud_to_laserscan min_height[m]。'
                        'lidar_link 基準で地面(z≈-0.2)を避けつつ低いフェンス/コーンを拾う'),
        DeclareLaunchArgument(
            'scan_max_height', default_value='2.5',
            description='屋外専用 pointcloud_to_laserscan max_height[m]。街灯・樹幹を拾い、屋根上部は抑える'),
        DeclareLaunchArgument(
            'scan_angle_increment', default_value='0.00698',
            description='屋外専用 pointcloud_to_laserscan 角度分解能[rad]。約0.4deg'),
        DeclareLaunchArgument(
            'scan_range_min', default_value='0.3',
            description='屋外専用 pointcloud_to_laserscan range_min[m]'),
        DeclareLaunchArgument(
            'scan_range_max', default_value='18.0',
            description='屋外専用 pointcloud_to_laserscan range_max[m]。slam_toolbox max_laser_range と近い値にする'),
        DeclareLaunchArgument(
            'scan_use_inf', default_value='True',
            description='屋外専用 pointcloud_to_laserscan use_inf'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='120.0',
            description='frontier の 1 ゴール到達猶予[s]。trimmed 屋外は長距離ゴールが出るため屋内より長め'),
        DeclareLaunchArgument(
            'max_path_goal_distance', default_value='4.0',
            description='ComputePathToPose の経路が長い場合、経路上この距離[m]だけ先を中間ゴールにする。'
                        '0 以下なら無効'),
        DeclareLaunchArgument(
            'staged_goal_clearance', default_value='0.45',
            description='staged frontier の中間ゴールに要求する SLAM map 上の occupied clearance[m]。'
                        '0 以下なら無効'),
        DeclareLaunchArgument(
            'yaw_watchdog', default_value='True',
            description='屋外 frontier 中に IMU yaw と map yaw の差が大きくなったらゴールを中断する'),
        DeclareLaunchArgument(
            'yaw_watchdog_max_error_deg', default_value='8.0',
            description='yaw watchdog の中断閾値[deg]'),
        DeclareLaunchArgument(
            'yaw_watchdog_blacklist_radius', default_value='1.2',
            description='yaw watchdog 発火位置周辺を frontier 候補から除外する半径[m]'),
        DeclareLaunchArgument(
            'explore_radius', default_value='14.0',
            description='ロボット初期位置から半径 R[m] 以内の frontier だけ探索する '
                        '（狭めにマッピングするための制限。0 以下なら無制限）'),
        DeclareLaunchArgument(
            'map_name', default_value='village_square_trimmed',
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
        truth_monitor,
    ])
