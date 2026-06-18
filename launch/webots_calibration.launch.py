# Webots LiDAR-camera calibration scene shortcut.
#
# High-contrast panels and colored 3D landmarks are placed around the robot so
# /perception/colorized_points alignment and object crop projection can be
# checked from all bearings.

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
            ('world', 'calibration.wbt'),
            ('mode', mode),
            ('nav', use_nav),
            ('slam', 'False'),
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
        DeclareLaunchArgument('nav', default_value='False',
                              description='キャリブレーション時は既定でNav2を起動しない'),
        DeclareLaunchArgument('rviz', default_value='True',
                              description='RViz2 を起動する'),
        DeclareLaunchArgument('perception', default_value='True',
                              description='Autoware perception を起動する'),
        DeclareLaunchArgument('omni_perception', default_value='True',
                              description='全天球カメラ連携を起動する'),
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
