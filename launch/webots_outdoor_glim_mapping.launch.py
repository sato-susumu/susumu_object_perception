# Outdoor mapping entry point for the GLIM-first route.
#
# Route:
#   1. Run Webots + MID360 + GLIM and drive the robot through the outdoor area.
#   2. Stop GLIM so it writes /tmp/dump, then export a PLY point cloud from
#      GLIM offline_viewer.
#   3. Convert the exported PLY + optional traj_lidar.txt into a Nav2 2D map
#      with scripts/glim_cloud_to_2d_map.py.
#
# This launch intentionally does not start slam_toolbox or Nav2 AMCL. GLIM
# stays in the independent glim_* TF tree and outdoor experiments do not touch
# the indoor mapping launch/params.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    teleop_gui = LaunchConfiguration('teleop_gui')
    glim_config_path = LaunchConfiguration('glim_config_path')
    lidar_model = LaunchConfiguration('lidar_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')

    glim_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_glim_colored_slam.launch.py')),
        launch_arguments=[
            ('world', world),
            ('mode', mode),
            ('rviz', rviz),
            ('perception', 'False'),
            ('image_recognition', 'False'),
            ('glim_config_path', glim_config_path),
            ('lidar_model', lidar_model),
            ('use_sim_time', use_sim_time),
            ('omni_calibration_json', omni_calibration_json),
        ],
    )

    teleop = TimerAction(
        period=8.0,
        condition=IfCondition(teleop_gui),
        actions=[
            Node(
                package='susumu_object_perception',
                executable='teleop_gui_node.py',
                name='teleop_gui',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='village_square_trimmed.wbt',
            description='GLIM で点群地図を作る屋外 world'),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（GLIM評価は realtime 推奨）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'teleop_gui', default_value='True',
            description='手動走行用 teleop GUI を起動する'),
        DeclareLaunchArgument(
            'glim_config_path',
            default_value=os.path.join(pkg, 'config', 'glim_webots'),
            description='GLIM config.json があるディレクトリ'),
        DeclareLaunchArgument(
            'lidar_model', default_value='mid360',
            description='LiDAR model metadata: mid360 / vlp16'),
        DeclareLaunchArgument(
            'omni_calibration_json', default_value='',
            description='direct_visual_lidar_calibration の calib.json。空なら初期TF'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Webots はシミュレーション時刻のため True'),
        glim_mapping,
        teleop,
    ])
