# Webots 用 SLAM 補助 launch（slam_toolbox を「1個だけ」起動する）。
#
# 通常は webots_simulation/outdoor/indoor の slam:=True を使えば Nav2 の bringup が
# slam_toolbox を 1 個起動するので、この launch は不要。
# この launch は「robot+nav を別 launch で起動済みで、SLAM だけ後から足したい」等の
# 単独運用向けの補助。robot_launch.py 直叩き等で map->odom を供給する SLAM が要るときに使う。
#
# 使い方（別端末で robot を起動済みの状態で）:
#   ros2 launch susumu_object_perception webots_slam.launch.py

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
