# Webots colored point cloud SLAM shortcut.
#
# Starts Webots + Nav2/slam_toolbox + omni camera colorization + accumulated
# colored point cloud map. The output cloud is /slam/colorized_points_map.

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
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_model = LaunchConfiguration('lidar_model')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', world),
            ('mode', mode),
            ('nav', 'True'),
            ('slam', 'True'),
            ('rviz', use_rviz),
            ('perception', use_perception),
            ('omni_perception', 'True'),
            ('colored_slam', 'True'),
            ('lidar_model', lidar_model),
            ('omni_calibration_json', omni_calibration_json),
            ('use_sim_time', use_sim_time),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='calibration.wbt',
            description='webots_worlds/ の world ファイル名'),
        DeclareLaunchArgument(
            'mode', default_value='fast',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'perception', default_value='False',
            description='Autoware perception を起動する。SLAM検証だけなら False'),
        DeclareLaunchArgument(
            'omni_calibration_json', default_value='',
            description='direct_visual_lidar_calibration の calib.json。空なら初期TF'),
        DeclareLaunchArgument(
            'lidar_model', default_value='mid360',
            description='3D LiDAR model metadata: mid360 / vlp16'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Webots はシミュレーション時刻のため True'),
        sim,
    ])
