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
    object_yolo_weights = LaunchConfiguration('object_yolo_weights')
    object_yolo_imgsz = LaunchConfiguration('object_yolo_imgsz')
    object_crop_fovs_deg = LaunchConfiguration('object_crop_fovs_deg')
    object_classifier_debug = LaunchConfiguration('object_classifier_debug')
    indoor_objects = LaunchConfiguration('indoor_objects')
    use_slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
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

    # ウェイポイント yaml の絶対パス。ファイル名だけなら install/share/maps 配下、
    # 絶対パスなら生成直後の source 側 maps/ もそのまま読めるようにする。
    wp_path = PythonExpression([
        "'", waypoints, "' if '", waypoints, "'.startswith('/') else "
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
            ('object_yolo_weights', object_yolo_weights),
            ('object_yolo_imgsz', object_yolo_imgsz),
            ('object_crop_fovs_deg', object_crop_fovs_deg),
            ('object_classifier_debug', object_classifier_debug),
            ('indoor_objects', indoor_objects),
            ('slam', use_slam),
            ('map_file', map_file),
            ('nav_params_file', nav_params_file),
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
            description='maps/ 配下のウェイポイント yaml ファイル名、または絶対パス'),
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
            'object_crop_fovs_deg', default_value='',
            description='object_classifier_node.py の複数FOVクロップ。例: 75,55,40'),
        DeclareLaunchArgument(
            'object_classifier_debug', default_value='False',
            description='True で /perception/object_classifier/debug に YOLO 候補の採否理由を出す'),
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
            'nav_params_file', default_value='',
            description='Nav2 params 差し替え。保存地図AMCL評価では config/nav2_params.yaml を指定'),
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
        robot_nav,
        viz,
        nav,
    ])
