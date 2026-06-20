# Webots Nav2 フルスタック launch（robot + Nav2 + SLAM を1コマンドで）。
#
# docs/webots_simulation.md §4「推奨手順」の端末1（robot+Nav2）と端末2（slam_toolbox を
# 1個だけ）を 1 launch にまとめたもの。TF 二重起動を避けるため:
#   - webots_simulation.launch.py を nav:=True slam:=False で include（Nav2 のみ。SLAM は起動しない）
#   - slam_toolbox は webots_slam.launch.py 経由で「1個だけ」起動（map->odom を供給）
# あとは別端末で NavigateToPose にゴールを送れば自律走行する（§4 端末3 参照）:
#   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
#     "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
#
# 使い方:
#   ros2 launch susumu_object_perception webots_nav.launch.py world:=outdoor.wbt
#   ros2 launch susumu_object_perception webots_nav.launch.py world:=indoor.wbt
#
# 罠: nav/slam の小文字 true は launch 評価時に NameError でクラッシュする（大文字必須）。
#     本 launch は内部で大文字 True/False を固定で渡すので、利用者は world だけ意識すればよい。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz')
    use_perception = LaunchConfiguration('perception')
    use_omni_perception = LaunchConfiguration('omni_perception')
    use_image_recognition = LaunchConfiguration('image_recognition')
    indoor_objects = LaunchConfiguration('indoor_objects')
    use_colored_slam = LaunchConfiguration('colored_slam')
    lidar_model = LaunchConfiguration('lidar_model')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_params_file = LaunchConfiguration('nav_params_file')

    # robot + Webots + Nav2 + SLAM。webots_simulation を nav:=True slam:=True で呼ぶだけ。
    # Nav2 の bringup が slam:=True のとき slam_toolbox を 1 個起動し map->odom を供給する
    # （AMCL は起動しない）。以前は別途 webots_slam を足していたが、bringup に委譲して
    # slam_toolbox を一本化したので不要になった（二重起動・TF 競合の根絶）。
    robot_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', world),
            ('mode', mode),
            ('nav', 'True'),
            ('slam', 'True'),
            ('rviz', use_rviz),
            ('perception', use_perception),
            ('omni_perception', use_omni_perception),
            ('image_recognition', use_image_recognition),
            ('indoor_objects', indoor_objects),
            ('colored_slam', use_colored_slam),
            ('lidar_model', lidar_model),
            ('omni_calibration_json', omni_calibration_json),
            ('use_sim_time', use_sim_time),
            ('nav_params_file', nav_params_file),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='outdoor.wbt',
            description='webots_worlds/ の world ファイル名（outdoor.wbt / indoor.wbt、拡張子込み）'),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'perception', default_value='True',
            description='Autoware perception を起動する'),
        DeclareLaunchArgument(
            'indoor_objects', default_value='False',
            description='室内物体検出（高所除外+床付近の家具を検出/識別）。室内 world で True'),
        DeclareLaunchArgument(
            'omni_perception', default_value='True',
            description='全天球カメラ連携（色付き点群/クロップ補助）を起動する'),
        DeclareLaunchArgument(
            'image_recognition', default_value='True',
            description='YOLO 物体分類 + 全天球信号認識を起動する。重いときは False'),
        DeclareLaunchArgument(
            'colored_slam', default_value='True',
            description='色付き点群SLAMマップを /slam/colorized_points_map に出す'),
        DeclareLaunchArgument(
            'lidar_model', default_value='mid360',
            description='3D LiDAR model metadata: mid360 / vlp16'),
        DeclareLaunchArgument(
            'omni_calibration_json', default_value='',
            description='direct_visual_lidar_calibration の calib.json。空なら初期TF'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Webots はシミュレーション時刻のため true 必須'),
        DeclareLaunchArgument(
            'nav_params_file', default_value='',
            description='Nav2 params 差し替え（空なら標準。探索は inflation を下げた '
                        'config/nav2_params_webots_explore.yaml を指定）'),
        robot_nav,
    ])
