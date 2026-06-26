"""Outdoor GPS navigation prototype without SLAM/AMCL.

This launch keeps the sparse outdoor path separate from indoor mapping/nav:
Webots publishes GPS/IMU/odom, outdoor_gps_localization_node.py publishes
map->odom, and outdoor_gps_waypoint_nav_node.py can optionally drive a small
relative GPS waypoint loop for smoke testing.
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
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    run_follower = LaunchConfiguration('run_follower')
    waypoints = LaunchConfiguration('waypoints')
    output_prefix = LaunchConfiguration('output_prefix')

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
                    'gps_topic': 'auto',
                    'imu_topic': '/imu',
                    'odom_topic': '/odom',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'heading_source': 'imu',
                    'publish_tf': True,
                    'use_sim_time': True,
                }])
        ])

    follower = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='outdoor_gps_waypoint_nav_node.py',
                name='outdoor_gps_waypoint_nav',
                output='screen',
                parameters=[{
                    'waypoints_file': waypoints,
                    'output_prefix': output_prefix,
                    'mission_timeout_sec': 90.0,
                    'waypoint_timeout_sec': 25.0,
                    'goal_tolerance_m': 0.35,
                    'max_linear_mps': 0.18,
                    'obstacle_stop_range_m': 0.0,
                }],
                condition=IfCondition(run_follower))
        ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='outdoor.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime'),
        DeclareLaunchArgument('run_follower', default_value='True'),
        DeclareLaunchArgument(
            'waypoints',
            default_value=os.path.join(
                pkg, 'outputs', 'waypoint_generation',
                'outdoor_gps_smoke_waypoints.yaml')),
        DeclareLaunchArgument(
            'output_prefix',
            default_value='/tmp/outdoor_gps_nav'),
        webots,
        localization,
        follower,
    ])
