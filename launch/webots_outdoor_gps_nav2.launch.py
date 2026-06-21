"""Outdoor GPS/IMU localization with Nav2 navigation only.

This launch keeps AMCL and SLAM out of the sparse outdoor path. Webots
provides odom/IMU/GPS, outdoor_gps_localization_node.py publishes map->odom,
and Nav2 navigation_launch.py consumes that TF with rolling obstacle costmaps.
"""

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
    nav2_bringup = get_package_share_directory('nav2_bringup')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    run_waypoints = LaunchConfiguration('run_waypoints')
    waypoints = LaunchConfiguration('waypoints')
    output_prefix = LaunchConfiguration('output_prefix')
    nav2_params = LaunchConfiguration('nav2_params')
    goal_timeout_sec = LaunchConfiguration('goal_timeout_sec')
    mission_timeout_sec = LaunchConfiguration('mission_timeout_sec')

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', world),
            ('mode', mode),
            ('nav', 'False'),
            ('slam', 'False'),
            ('rviz', 'False'),
            ('perception', 'False'),
            ('omni_perception', 'False'),
            ('image_recognition', 'False'),
            ('colored_slam', 'False'),
        ])

    localization = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='outdoor_gps_localization_node.py',
                name='outdoor_gps_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'gps_topic': 'auto',
                    'imu_topic': '/imu',
                    'odom_topic': '/odom',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'heading_source': 'imu',
                    'publish_tf': True,
                }])
        ])

    navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    nav2_bringup, 'launch', 'navigation_launch.py')),
                launch_arguments=[
                    ('use_sim_time', 'True'),
                    ('params_file', nav2_params),
                    ('autostart', 'True'),
                    ('use_composition', 'False'),
                ])
        ])

    waypoint_runner = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='outdoor_nav2_waypoint_nav_node.py',
                name='outdoor_nav2_waypoint_nav',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'waypoints_file': waypoints,
                    'frame_id': 'map',
                    'robot_frame': 'base_footprint',
                    'output_prefix': output_prefix,
                    'start_delay_sec': 0.0,
                    'goal_timeout_sec': goal_timeout_sec,
                    'mission_timeout_sec': mission_timeout_sec,
                    'sample_period_sec': 0.5,
                }],
                condition=IfCondition(run_waypoints))
        ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='outdoor.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime'),
        DeclareLaunchArgument('run_waypoints', default_value='True'),
        DeclareLaunchArgument(
            'waypoints',
            default_value=os.path.join(
                pkg, 'maps', 'outdoor_gps_smoke_waypoints.yaml')),
        DeclareLaunchArgument(
            'output_prefix',
            default_value='/tmp/outdoor_nav2_gps_nav'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(
                pkg, 'config', 'nav2_params_outdoor_gps.yaml')),
        DeclareLaunchArgument('goal_timeout_sec', default_value='90.0'),
        DeclareLaunchArgument('mission_timeout_sec', default_value='300.0'),
        webots,
        localization,
        navigation,
        waypoint_runner,
    ])
