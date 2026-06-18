# Webots 屋外（outdoor.wbt）起動ショートカット launch。
#
# webots_simulation.launch.py world:=outdoor.wbt と等価。world 指定が面倒なので分離した。
# nav / slam / mode / use_sim_time はそのまま渡せる（既定は webots_simulation と同じ）。
#
#   ros2 launch susumu_object_perception webots_outdoor.launch.py
#   ros2 launch susumu_object_perception webots_outdoor.launch.py nav:=True   # Nav2 付き
#   ros2 launch susumu_object_perception webots_outdoor.launch.py mode:=fast

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav')
    use_slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('rviz')
    use_perception = LaunchConfiguration('perception')
    use_omni_perception = LaunchConfiguration('omni_perception')
    use_colored_slam = LaunchConfiguration('colored_slam')
    lidar_model = LaunchConfiguration('lidar_model')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', 'outdoor.wbt'),
            ('mode', mode),
            ('nav', use_nav),
            ('slam', use_slam),
            ('rviz', use_rviz),
            ('perception', use_perception),
            ('omni_perception', use_omni_perception),
            ('colored_slam', use_colored_slam),
            ('lidar_model', lidar_model),
            ('omni_calibration_json', omni_calibration_json),
            ('use_sim_time', use_sim_time),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='realtime',
                              description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument('nav', default_value='True',
                              description='Nav2 を起動する（既定 True。見るだけなら nav:=False）'),
        DeclareLaunchArgument('slam', default_value='False',
                              description='SLAM(slam_toolbox)で地図生成しつつ自律走行（AMCL無効。大文字）'),
        DeclareLaunchArgument('rviz', default_value='True',
                              description='RViz2 を起動する（既定 True）'),
        DeclareLaunchArgument('perception', default_value='True',
                              description='Autoware perception を起動する（既定 True）'),
        DeclareLaunchArgument('omni_perception', default_value='True',
                              description='全天球カメラ連携を起動する（色付き点群・物体クロップ）'),
        DeclareLaunchArgument('colored_slam', default_value='True',
                              description='色付き点群SLAMマップを /slam/colorized_points_map に出す'),
        DeclareLaunchArgument('lidar_model', default_value='mid360',
                              description='3D LiDAR model metadata: mid360 / vlp16'),
        DeclareLaunchArgument('omni_calibration_json', default_value='',
                              description='direct_visual_lidar_calibration の calib.json。空なら初期TF'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Webots はシミュレーション時刻のため True'),
        sim,
    ])
