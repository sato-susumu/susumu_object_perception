# 保存済みウェイポイントに沿って Nav2 で巡回ナビゲーションする launch。
#
# generate_waypoints.py で作った outputs/waypoint_generation/<world>_waypoints.yaml を読み、
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
from launch.conditions import IfCondition
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
    object_yolo_weights = LaunchConfiguration('object_yolo_weights')
    object_yolo_imgsz = LaunchConfiguration('object_yolo_imgsz')
    object_yolo_conf = LaunchConfiguration('object_yolo_conf')
    object_min_accept_conf = LaunchConfiguration('object_min_accept_conf')
    object_min_accept_conf_overrides = LaunchConfiguration(
        'object_min_accept_conf_overrides')
    object_crop_fovs_deg = LaunchConfiguration('object_crop_fovs_deg')
    object_crop_yaw_offsets_deg = LaunchConfiguration(
        'object_crop_yaw_offsets_deg')
    object_crop_pitch_offsets_deg = LaunchConfiguration(
        'object_crop_pitch_offsets_deg')
    object_crop_shape_center_height_fracs = LaunchConfiguration(
        'object_crop_shape_center_height_fracs')
    object_crop_shape_bbox_margins_deg = LaunchConfiguration(
        'object_crop_shape_bbox_margins_deg')
    object_classifier_debug = LaunchConfiguration('object_classifier_debug')
    object_debug_crop_dir = LaunchConfiguration('object_debug_crop_dir')
    object_debug_crop_max_per_track = LaunchConfiguration(
        'object_debug_crop_max_per_track')
    object_tracker_debug = LaunchConfiguration('object_tracker_debug')
    object_tracker_min_hits = LaunchConfiguration('object_tracker_min_hits')
    object_tracker_wall_margin_moving_cells = LaunchConfiguration(
        'object_tracker_wall_margin_moving_cells')
    object_tracker_wall_margin_static_cells = LaunchConfiguration(
        'object_tracker_wall_margin_static_cells')
    object_memory_delete_thresh = LaunchConfiguration(
        'object_memory_delete_thresh')
    object_memory_miss_tp = LaunchConfiguration('object_memory_miss_tp')
    object_memory_miss_fp = LaunchConfiguration('object_memory_miss_fp')
    indoor_objects = LaunchConfiguration('indoor_objects')
    use_slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map_file')
    use_colored_slam = LaunchConfiguration('colored_slam')
    use_stationary_only = LaunchConfiguration('stationary_only')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
    nav_params_file = LaunchConfiguration('nav_params_file')
    ros2_control_params_file = LaunchConfiguration('ros2_control_params_file')
    nav_start_delay_sec = LaunchConfiguration('nav_start_delay_sec')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')
    report_prefix = LaunchConfiguration('report_prefix')
    mission_timeout = LaunchConfiguration('mission_timeout_sec')
    behavior_tree = LaunchConfiguration('behavior_tree')
    safe_pose_guard = LaunchConfiguration('safe_pose_guard')
    safe_pose_cost_threshold = LaunchConfiguration('safe_pose_cost_threshold')
    safe_pose_safe_threshold = LaunchConfiguration('safe_pose_safe_threshold')
    safe_pose_hold_sec = LaunchConfiguration('safe_pose_hold_sec')
    safe_pose_recovery_timeout = LaunchConfiguration(
        'safe_pose_recovery_timeout_sec')
    step_detector_avoid = LaunchConfiguration('step_detector_avoid')
    use_truth_monitor = LaunchConfiguration('truth_monitor')
    truth_report_prefix = LaunchConfiguration('truth_report_prefix')
    truth_max_aligned_error = LaunchConfiguration('truth_max_aligned_error')
    truth_max_heading_error_deg = LaunchConfiguration(
        'truth_max_heading_error_deg')
    truth_max_yaw_error_deg = LaunchConfiguration('truth_max_yaw_error_deg')
    truth_odom_frame = LaunchConfiguration('truth_odom_frame')
    use_ekf_odom = LaunchConfiguration('ekf_odom')
    ekf_odom_params_file = LaunchConfiguration('ekf_odom_params_file')
    ekf_odom_start_sec = LaunchConfiguration('ekf_odom_start_sec')
    filtered_odom_topic = LaunchConfiguration('filtered_odom_topic')

    # ウェイポイント yaml の絶対パス。ファイル名だけなら outputs/waypoint_generation/、
    # 絶対パスなら指定されたパスをそのまま使う。
    wp_path = PythonExpression([
        "'", waypoints, "' if '", waypoints, "'.startswith('/') else "
        "'", os.path.join(pkg, 'outputs', 'waypoint_generation', ''), "' + '", waypoints, "'"])

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
            ('object_yolo_weights', object_yolo_weights),
            ('object_yolo_imgsz', object_yolo_imgsz),
            ('object_yolo_conf', object_yolo_conf),
            ('object_min_accept_conf', object_min_accept_conf),
            ('object_min_accept_conf_overrides',
                object_min_accept_conf_overrides),
            ('object_crop_fovs_deg', object_crop_fovs_deg),
            ('object_crop_yaw_offsets_deg', object_crop_yaw_offsets_deg),
            ('object_crop_pitch_offsets_deg', object_crop_pitch_offsets_deg),
            ('object_crop_shape_center_height_fracs',
             object_crop_shape_center_height_fracs),
            ('object_crop_shape_bbox_margins_deg',
             object_crop_shape_bbox_margins_deg),
            ('object_classifier_debug', object_classifier_debug),
            ('object_debug_crop_dir', object_debug_crop_dir),
            ('object_debug_crop_max_per_track',
             object_debug_crop_max_per_track),
            ('object_tracker_debug', object_tracker_debug),
            ('object_tracker_min_hits', object_tracker_min_hits),
            ('object_tracker_wall_margin_moving_cells',
             object_tracker_wall_margin_moving_cells),
            ('object_tracker_wall_margin_static_cells',
             object_tracker_wall_margin_static_cells),
            ('object_memory_delete_thresh', object_memory_delete_thresh),
            ('object_memory_miss_tp', object_memory_miss_tp),
            ('object_memory_miss_fp', object_memory_miss_fp),
            ('indoor_objects', indoor_objects),
            ('slam', use_slam),
            ('map_file', map_file),
            ('nav_params_file', nav_params_file),
            ('ros2_control_params_file', ros2_control_params_file),
            ('nav_start_delay_sec', nav_start_delay_sec),
            ('colored_slam', use_colored_slam),
            ('stationary_only', use_stationary_only),
            ('omni_calibration_json', omni_calibration_json),
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
                    'goal_timeout_sec': goal_timeout,
                    'report_prefix': report_prefix,
                    'mission_timeout_sec': mission_timeout,
                    'behavior_tree': behavior_tree,
                    'safe_pose_guard': safe_pose_guard,
                    'safe_pose_cost_threshold': safe_pose_cost_threshold,
                    'safe_pose_safe_threshold': safe_pose_safe_threshold,
                    'safe_pose_hold_sec': safe_pose_hold_sec,
                    'safe_pose_recovery_timeout_sec':
                        safe_pose_recovery_timeout,
                    'step_detector_avoid': step_detector_avoid,
                }],
            ),
        ],
    )

    # Webots GPS/IMU truth と AMCL/SLAM の map->base_footprint を比較する評価専用監視。
    # 真値は評価レポートにだけ使い、自己位置推定や Nav2 へは戻さない。
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
                    'odom_frame': truth_odom_frame,
                    'filtered_odom_topic': filtered_odom_topic,
                    'robot_frame': 'base_footprint',
                    'sample_period': 0.5,
                    'min_align_samples': 8,
                    'min_align_path_length': 1.0,
                    'max_aligned_error': truth_max_aligned_error,
                    'max_heading_error_deg': truth_max_heading_error_deg,
                    'max_yaw_error_deg': truth_max_yaw_error_deg,
                    'report_prefix': truth_report_prefix,
                    'stop_status_topic': '/waypoint_nav/status',
                    'stop_status_patterns':
                        'mission complete,mission_timeout',
                }],
            ),
        ],
    )

    # robot_localization EKF。既定 params は publish_tf:false の評価用だが、
    # ros2_control 側 enable_odom_tf:false と publish_tf:true の params を組み合わせると
    # odom->base_link TF 発行元としても切り分けできる。
    ekf_odom = TimerAction(
        period=ekf_odom_start_sec,
        condition=IfCondition(use_ekf_odom),
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_odom_params_file,
                    {'use_sim_time': True},
                ],
                remappings=[
                    ('odometry/filtered', filtered_odom_topic),
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='indoor.wbt',
            description='ナビするワールド（indoor.wbt / break_room.wbt / city_robot.wbt / outdoor.wbt）。 '
                        'iter89 で default を city_robot から indoor に変更 (city_waypoints.yaml が '
                        'contracts から legacy に移動されたため、 動く default ペアにする)'),
        DeclareLaunchArgument(
            'waypoints', default_value='indoor_waypoints.yaml',
            description='outputs/waypoint_generation/ 配下のウェイポイント yaml ファイル名、または絶対パス。 '
                        'iter89 で default を city_waypoints.yaml → indoor_waypoints.yaml に変更 '
                        '(city_waypoints.yaml は experiments/waypoint_generation/legacy/ に移動済み)'),
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
            'object_yolo_weights', default_value='yolov8s-seg.pt',
            description='object_classifier_node.py の YOLO weight。例: yolov8m-seg.pt'),
        DeclareLaunchArgument(
            'object_yolo_imgsz', default_value='640',
            description='object_classifier_node.py の YOLO 推論画像サイズ。例: 960'),
        DeclareLaunchArgument(
            'object_yolo_conf', default_value='0.15',
            description='object_classifier_node.py の YOLO predict conf。既定 0.15'),
        DeclareLaunchArgument(
            'object_min_accept_conf', default_value='0.15',
            description='object_classifier_node.py の分類採用conf。既定 0.15'),
        DeclareLaunchArgument(
            'object_min_accept_conf_overrides', default_value='',
            description='クラス別 min_accept_conf。"class1=0.10,class2=0.30" 形式'),
        DeclareLaunchArgument(
            'object_crop_fovs_deg', default_value='',
            description='object_classifier_node.py の複数FOVクロップ。例: 75,55,40'),
        DeclareLaunchArgument(
            'object_crop_yaw_offsets_deg', default_value='',
            description='object_classifier_node.py の crop yaw offset[deg]。例: -18,0,18'),
        DeclareLaunchArgument(
            'object_crop_pitch_offsets_deg', default_value='',
            description='object_classifier_node.py の crop pitch offset[deg]。例: -10,0,10'),
        DeclareLaunchArgument(
            'object_crop_shape_center_height_fracs', default_value='',
            description='object_classifier_node.py の shape 高さ方向 crop 中心。例: 0,0.5,0.75'),
        DeclareLaunchArgument(
            'object_crop_shape_bbox_margins_deg', default_value='',
            description='object_classifier_node.py の 3D bbox 投影 crop margin[deg]。例: 4,10,18'),
        DeclareLaunchArgument(
            'object_classifier_debug', default_value='False',
            description='True で /perception/object_classifier/debug に YOLO 候補の採否理由を出す'),
        DeclareLaunchArgument(
            'object_debug_crop_dir', default_value='',
            description='空でなければ object_classifier_node.py の raw crop と metadata.jsonl を保存する'),
        DeclareLaunchArgument(
            'object_debug_crop_max_per_track', default_value='3',
            description='各 track で保存する debug crop 上限。負値で無制限'),
        DeclareLaunchArgument(
            'object_tracker_debug', default_value='False',
            description='True で /perception/object_tracker/debug に track publish/reject 理由を出す'),
        DeclareLaunchArgument(
            'object_tracker_min_hits', default_value='2',
            description='object_tracker_node.py の出力最小hit数。既定 2'),
        DeclareLaunchArgument(
            'object_tracker_wall_margin_moving_cells',
            default_value='6',
            description='object_tracker_node.py の移動track向け壁margin[cell]。既定 6'),
        DeclareLaunchArgument(
            'object_tracker_wall_margin_static_cells',
            default_value='22',
            description='object_tracker_node.py の静止track向け壁margin[cell]。既定 22'),
        DeclareLaunchArgument(
            'object_memory_delete_thresh', default_value='0.25',
            description=('object_memory の Bayes 削除しきい値 (既定 0.25=Dengler et al.)。 '
                         '巡回中に DB が空になる時は 0.05〜0.10 に下げて存続を許す '
                         '(memory feedback_recog_db_empty_issue 参照)')),
        DeclareLaunchArgument(
            'object_memory_miss_tp', default_value='0.2',
            description=('object_memory の miss 観測時の TP 確率 (既定 0.2)。 '
                         '上げると減衰が緩む')),
        DeclareLaunchArgument(
            'object_memory_miss_fp', default_value='0.6',
            description=('object_memory の miss 観測時の FP 確率 (既定 0.6)。 '
                         '下げると減衰が緩む')),
        DeclareLaunchArgument(
            'indoor_objects', default_value='False',
            description='室内物体検出（高所除外+床付近の家具を検出/識別）。室内 world で True'),
        DeclareLaunchArgument(
            'slam', default_value='True',
            description='True: slam_toolbox で巡回 / False: map_file の保存地図 + AMCL で巡回'),
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='slam:=False で Nav2/AMCL に読ませる地図 yaml'),
        DeclareLaunchArgument(
            'colored_slam', default_value='False',
            description='色付き点群SLAMマップを /slam/colorized_points_map に出す（カラー点群出力タスク）'),
        DeclareLaunchArgument(
            'stationary_only', default_value='False',
            description='色付き点群を静止時のみ蓄積する (iter12 で実証、 約 10 倍シャープ)。 '
                        '巡回 + 停止時のみ蓄積で点数と質を両立する用途'),
        DeclareLaunchArgument(
            'omni_calibration_json', default_value='',
            description='LiDAR-camera 外部キャリブ calib.json。空なら初期TF'),
        DeclareLaunchArgument(
            'nav_params_file', default_value='',
            description='Nav2 params 差し替え。保存地図AMCL評価では config/nav2_params.yaml を指定'),
        DeclareLaunchArgument(
            'ros2_control_params_file', default_value='',
            description='ros2_control params 差し替え。EKF TF 評価では diffdrive odom TF 無効設定を指定'),
        DeclareLaunchArgument(
            'nav_start_delay_sec',
            default_value='0.0',
            description='Webots controller 接続後に Nav2 起動を遅らせる秒数。EKF TF 評価では 2.5 など'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='60.0',
            description='各 waypoint の NavigateToPose 到達猶予[s]'),
        DeclareLaunchArgument(
            'report_prefix', default_value='',
            description='空でなければ waypoint_nav_node の JSON/CSV/Markdown report prefix'),
        DeclareLaunchArgument(
            'mission_timeout_sec', default_value='0.0',
            description='0 以下なら無効。wall-clock で巡回評価全体を打ち切る秒数'),
        DeclareLaunchArgument(
            'behavior_tree', default_value='',
            description='NavigateToPose goal に渡す BT XML。空なら Nav2 既定'),
        DeclareLaunchArgument(
            'safe_pose_guard',
            default_value='False',
            description='True で現在姿勢が global costmap 高コストセルに入ったら最後の安全姿勢へ戻る'),
        DeclareLaunchArgument(
            'safe_pose_cost_threshold',
            default_value='80',
            description='safe_pose_guard の危険判定 costmap 値'),
        DeclareLaunchArgument(
            'safe_pose_safe_threshold',
            default_value='40',
            description='safe_pose_guard が最後の安全姿勢として記録する最大 costmap 値'),
        DeclareLaunchArgument(
            'safe_pose_hold_sec',
            default_value='1.0',
            description='危険 cost が継続したとみなす保持時間[s]'),
        DeclareLaunchArgument(
            'safe_pose_recovery_timeout_sec',
            default_value='25.0',
            description='最後の安全姿勢へ戻る NavigateToPose の timeout[s]'),
        DeclareLaunchArgument(
            'step_detector_avoid',
            default_value='False',
            description=(
                'True で waypoint_nav が /step_detector/event を購読し、 '
                '段差/縁石/スタック検知時に現在 WP を諦め次へ進む。 '
                '屋外巡回で goal_timeout_sec 満了を待たず即座に skip する。 '
                '既定 False (屋内・既存評価には影響させない)。')),
        DeclareLaunchArgument(
            'truth_monitor',
            default_value='False',
            description='True で Webots GPS/IMU truth と map->base_footprint を比較し、評価レポートだけ残す'),
        DeclareLaunchArgument(
            'truth_report_prefix',
            default_value='/tmp/susumu_waypoint_truth_monitor',
            description='truth monitor の JSON/CSV/Markdown report prefix'),
        DeclareLaunchArgument(
            'truth_max_aligned_error',
            default_value='0.30',
            description='truth monitor の剛体合わせ後位置ずれイベント閾値[m]'),
        DeclareLaunchArgument(
            'truth_max_heading_error_deg',
            default_value='20.0',
            description='truth monitor の移動方向ずれイベント閾値[deg]'),
        DeclareLaunchArgument(
            'truth_max_yaw_error_deg',
            default_value='12.0',
            description='truth monitor の IMU 真値 yaw と map->base yaw の絶対方位ずれイベント閾値[deg]'),
        DeclareLaunchArgument(
            'truth_odom_frame',
            default_value='odom',
            description='truth monitor が同時評価する odom frame。空なら odom 評価を無効化'),
        DeclareLaunchArgument(
            'ekf_odom',
            default_value='False',
            description='True で robot_localization EKF を起動する。publish_tf は params yaml 側で決める'),
        DeclareLaunchArgument(
            'ekf_odom_params_file',
            default_value=os.path.join(
                pkg, 'config', 'ekf_odom_twist_imu_eval.yaml'),
            description='評価専用 robot_localization EKF params yaml'),
        DeclareLaunchArgument(
            'ekf_odom_start_sec',
            default_value='18.0',
            description='robot_localization EKF 起動時刻[s]。EKF TF 評価では Nav2 より前に起動する'),
        DeclareLaunchArgument(
            'filtered_odom_topic',
            default_value='/odometry/filtered',
            description='EKF 出力 odometry topic。truth monitor も同 topic を評価する'),
        robot_nav,
        viz,
        nav,
        ekf_odom,
        truth_monitor,
    ])
