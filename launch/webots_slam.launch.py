# Webots 用 SLAM 補助 launch（slam_toolbox を「1個だけ」起動する）。
#
# webots_ros2_turtlebot の robot_launch.py（や本パッケージの webots_simulation.launch.py
# / webots_nav.launch.py）で slam:=True にすると、同梱の Cartographer 等と二重起動して
# map->odom TF が競合し壊れることがある（docs/webots_simulation.md「ハマりどころ」）。
# そのため Nav2 側は slam:=False で起動し、map->odom を供給する SLAM はこの launch で
# 1 プロセスだけ立てる、という分離運用にする。
#
# 使い方（別端末で robot+nav を起動済みの状態で）:
#   ros2 launch susumu_object_perception webots_slam.launch.py
#
# docs/webots_simulation.md §4「推奨手順」の端末2に相当する。

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        # Webots はシミュレーション時刻なので use_sim_time は true 必須。
        launch_arguments=[('use_sim_time', use_sim_time)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Webots はシミュレーション時刻のため true 必須'),
        slam,
    ])
