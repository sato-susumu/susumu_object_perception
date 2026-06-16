# Webots 屋内（indoor.wbt）起動ショートカット launch。
#
# webots_simulation.launch.py world:=indoor.wbt と等価。world 指定が面倒なので分離した。
# nav / slam / mode / use_sim_time はそのまま渡せる（既定は webots_simulation と同じ）。
#
#   ros2 launch susumu_object_perception webots_indoor.launch.py
#   ros2 launch susumu_object_perception webots_indoor.launch.py nav:=True   # Nav2 付き
#   ros2 launch susumu_object_perception webots_indoor.launch.py mode:=fast

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
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', 'indoor.wbt'),
            ('mode', mode),
            ('nav', use_nav),
            ('slam', use_slam),
            ('use_sim_time', use_sim_time),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='realtime',
                              description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument('nav', default_value='False',
                              description='Nav2 を起動する（大文字 True/False）'),
        DeclareLaunchArgument('slam', default_value='False',
                              description='Cartographer SLAM を起動する（大文字 True/False）'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Webots はシミュレーション時刻のため True'),
        sim,
    ])
