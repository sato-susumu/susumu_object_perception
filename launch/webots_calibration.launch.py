# Webots LiDAR-camera calibration scene shortcut.
#
# High-contrast panels and colored 3D landmarks are placed around the robot so
# /perception/colorized_points alignment and object crop projection can be
# checked from all bearings.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav')
    use_rviz = LaunchConfiguration('rviz')
    use_perception = LaunchConfiguration('perception')
    use_omni_perception = LaunchConfiguration('omni_perception')
    use_image_recognition = LaunchConfiguration('image_recognition')
    use_colored_slam = LaunchConfiguration('colored_slam')
    lidar_model = LaunchConfiguration('lidar_model')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_apriltag_calib = LaunchConfiguration('apriltag_calib')
    apriltag_calib_json = LaunchConfiguration('apriltag_calib_json')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', 'calibration.wbt'),
            ('mode', mode),
            ('nav', use_nav),
            ('slam', 'False'),
            ('rviz', use_rviz),
            ('perception', use_perception),
            ('omni_perception', use_omni_perception),
            ('image_recognition', use_image_recognition),
            ('colored_slam', use_colored_slam),
            ('lidar_model', lidar_model),
            ('omni_calibration_json', omni_calibration_json),
            ('use_sim_time', use_sim_time),
        ],
    )

    # AprilTag 外部キャリブノード（opt-in）。Webots / perception 起動が落ち着いてから遅延起動。
    # 全天球画像と LiDAR からタグを検出し T_lidar_camera を calib.json に書く。
    apriltag_calib = TimerAction(
        period=20.0,
        condition=IfCondition(use_apriltag_calib),
        actions=[
            Node(
                package='susumu_object_perception',
                executable='apriltag_extrinsic_calib_node.py',
                name='apriltag_extrinsic_calib',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'output_json': apriltag_calib_json,
                    # Webots driver の実 LiDAR トピックは /lidar/points/point_cloud
                    # （webots_simulation.launch.py と同じ）。
                    'input_cloud': '/lidar/points/point_cloud',
                    'lidar_z_use_range_mid': LaunchConfiguration(
                        'lidar_z_use_range_mid'),
                    'board_height_assumption': LaunchConfiguration(
                        'board_height_assumption'),
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='realtime',
                              description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument('nav', default_value='False',
                              description='キャリブレーション時は既定でNav2を起動しない'),
        DeclareLaunchArgument('rviz', default_value='True',
                              description='RViz2 を起動する'),
        DeclareLaunchArgument('perception', default_value='True',
                              description='Autoware perception を起動する'),
        DeclareLaunchArgument('omni_perception', default_value='True',
                              description='全天球カメラ連携を起動する'),
        DeclareLaunchArgument('image_recognition', default_value='False',
                              description='YOLO 物体分類 + 全天球信号認識を起動する。キャリブレーションだけなら False'),
        DeclareLaunchArgument('colored_slam', default_value='True',
                              description='色付き点群SLAMマップを /slam/colorized_points_map に出す'),
        DeclareLaunchArgument('lidar_model', default_value='mid360',
                              description='3D LiDAR model metadata: mid360 / vlp16'),
        DeclareLaunchArgument('omni_calibration_json', default_value='',
                              description='direct_visual_lidar_calibration の calib.json。空なら初期TF'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Webots はシミュレーション時刻のため True'),
        DeclareLaunchArgument('apriltag_calib', default_value='False',
                              description='AprilTag で全天球+LiDAR 外部キャリブを実行する'),
        DeclareLaunchArgument(
            'apriltag_calib_json',
            default_value=os.path.expanduser(
                '~/ros2_ws/src/susumu_object_perception/outputs/extrinsic_calibration/calib.json'),
            description='AprilTag キャリブ結果 calib.json の出力先'),
        DeclareLaunchArgument(
            'lidar_z_use_range_mid', default_value='False',
            description='LiDAR 板抽出で z 座標を「点群の z 範囲中央 (max+min)/2」に '
                        '置換する。 板下半分しか点が来ない場合 (MID-360 上向き FOV) '
                        'の補正候補'),
        DeclareLaunchArgument(
            'board_height_assumption', default_value='0.0',
            description='板の物理高 [m] を渡すと、 LiDAR 板抽出で z 座標を '
                        '「点群 z の最上端 - board_height_assumption / 2」 に置換する。 '
                        'iter25 で実装した代替補正案 (上端 z は LiDAR 上向き FOV でも '
                        '捉えやすい想定)。 0.0 で無効 (既定)。 既知の calibration.wbt '
                        'では 0.3 推奨。'),
        sim,
        apriltag_calib,
    ])
