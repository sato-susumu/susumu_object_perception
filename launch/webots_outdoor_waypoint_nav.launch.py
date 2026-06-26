"""Outdoor saved-map waypoint navigation.

This is intentionally separate from webots_waypoint_nav.launch.py defaults so
outdoor map/AMCL/Nav2 experiments do not change indoor patrol behavior.
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
    map_file = LaunchConfiguration('map_file')
    waypoints = LaunchConfiguration('waypoints')
    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    loop = LaunchConfiguration('loop')
    perception = LaunchConfiguration('perception')
    omni_perception = LaunchConfiguration('omni_perception')
    image_recognition = LaunchConfiguration('image_recognition')
    nav_params_file = LaunchConfiguration('nav_params_file')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')
    report_prefix = LaunchConfiguration('report_prefix')
    mission_timeout = LaunchConfiguration('mission_timeout_sec')
    costmap_monitor = LaunchConfiguration('costmap_monitor')
    costmap_monitor_prefix = LaunchConfiguration('costmap_monitor_prefix')
    behavior_tree = LaunchConfiguration('behavior_tree')
    safe_pose_guard = LaunchConfiguration('safe_pose_guard')
    safe_pose_cost_threshold = LaunchConfiguration('safe_pose_cost_threshold')
    safe_pose_safe_threshold = LaunchConfiguration('safe_pose_safe_threshold')
    safe_pose_hold_sec = LaunchConfiguration('safe_pose_hold_sec')
    safe_pose_recovery_timeout = LaunchConfiguration(
        'safe_pose_recovery_timeout_sec')

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_waypoint_nav.launch.py')),
        launch_arguments=[
            ('world', world),
            ('waypoints', waypoints),
            ('mode', mode),
            ('rviz', rviz),
            ('loop', loop),
            ('perception', perception),
            ('omni_perception', omni_perception),
            ('image_recognition', image_recognition),
            ('slam', 'False'),
            ('map_file', map_file),
            ('nav_params_file', nav_params_file),
            ('indoor_objects', 'False'),
            ('goal_timeout_sec', goal_timeout),
            ('report_prefix', report_prefix),
            ('mission_timeout_sec', mission_timeout),
            ('behavior_tree', behavior_tree),
            ('safe_pose_guard', safe_pose_guard),
            ('safe_pose_cost_threshold', safe_pose_cost_threshold),
            ('safe_pose_safe_threshold', safe_pose_safe_threshold),
            ('safe_pose_hold_sec', safe_pose_hold_sec),
            ('safe_pose_recovery_timeout_sec', safe_pose_recovery_timeout),
            # 屋外巡回では既定で段差検知連携を ON にする (iter19)。
            ('step_detector_avoid', 'True'),
        ],
    )

    # 屋外専用: step_detector_node を起動 (iter16 で追加、 mapping と同じノード)。
    # waypoint_nav は /step_detector/event を購読し、 段差時に現在 WP を skip する。
    step_detector = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='susumu_object_perception',
                executable='step_detector_node.py',
                name='step_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'imu_topic': '/imu',
                    'odom_topic': '/odom',
                    'cmd_vel_topic': '/cmd_vel',
                    'tilt_warn_deg': 5.0,
                    'tilt_critical_deg': 15.0,
                }],
            ),
        ],
    )

    monitor = TimerAction(
        period=18.0,
        condition=IfCondition(costmap_monitor),
        actions=[
            Node(
                package='susumu_object_perception',
                executable='nav2_pose_costmap_monitor_node.py',
                name='nav2_pose_costmap_monitor',
                output='screen',
                parameters=[{
                    'report_prefix': costmap_monitor_prefix,
                    'waypoints_file': waypoints,
                    'sample_period': 0.5,
                    'report_period': 5.0,
                    'robot_frame': 'base_link',
                    'fallback_robot_frame': 'base_footprint',
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='village_square_trimmed.wbt',
            description='Outdoor trimmed world to patrol'),
        DeclareLaunchArgument(
            'map_file', default_value='village_square_trimmed.yaml',
            description='Saved map YAML in outputs/mapping_outdoor/ or an absolute path. '
                        'NOTE (iter88): default value points to a not-yet-existing file '
                        '(mapping_outdoor is not yet completed, only `_gt.yaml` truth maps '
                        'are in contracts). Pass map_file:=<your_map>.yaml explicitly until '
                        'mapping_outdoor produces a saved map.'),
        DeclareLaunchArgument(
            'waypoints', default_value='village_square_trimmed_waypoints.yaml',
            description='Outdoor waypoint YAML in outputs/waypoint_generation/ or an absolute path. '
                        'NOTE (iter88): default value points to a not-yet-existing file. '
                        'Pass waypoints:=<your_waypoints>.yaml explicitly.'),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）。 屋外巡回は realtime 推奨'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する'),
        DeclareLaunchArgument(
            'loop', default_value='False',
            description='巡回完走後にもう一周する。 屋外既定 False = 1 周で停止'),
        DeclareLaunchArgument(
            'perception', default_value='False',
            description='Autoware perception (LiDAR 検出・追跡・予測) を起動する。 屋外 patrol 既定 False'),
        DeclareLaunchArgument(
            'omni_perception', default_value='False',
            description='全天球色付き点群 / 全天球クロップ補助を起動する。 屋外 patrol 既定 False'),
        DeclareLaunchArgument(
            'image_recognition', default_value='False',
            description='YOLO 物体分類 + 全天球信号認識を起動する。 屋外 patrol 既定 False (CPU 節約)'),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value='nav2_params_webots_explore_outdoor.yaml',
            description='Outdoor-only Nav2 params for saved-map patrol'),
        DeclareLaunchArgument(
            'goal_timeout_sec',
            default_value='120.0',
            description='Timeout for each outdoor waypoint NavigateToPose goal'),
        DeclareLaunchArgument(
            'report_prefix',
            default_value='',
            description='Optional JSON/CSV/Markdown report prefix for outdoor waypoint evaluation'),
        DeclareLaunchArgument(
            'mission_timeout_sec',
            default_value='0.0',
            description='Optional wall-clock mission timeout for bounded outdoor evaluation'),
        DeclareLaunchArgument(
            'costmap_monitor',
            default_value='False',
            description='Run outdoor-only pose/costmap diagnostic monitor'),
        DeclareLaunchArgument(
            'costmap_monitor_prefix',
            default_value='',
            description='Optional JSON/CSV/Markdown/PNG prefix for costmap monitor'),
        DeclareLaunchArgument(
            'behavior_tree',
            default_value='',
            description='Optional outdoor patrol BT XML. Empty uses Nav2 default recovery BT'),
        DeclareLaunchArgument(
            'safe_pose_guard',
            default_value='False',
            description='True: before continuing after unsafe pose/timeout, navigate back to the last safe AMCL pose'),
        DeclareLaunchArgument(
            'safe_pose_cost_threshold',
            default_value='80',
            description='Global costmap value treated as unsafe by safe_pose_guard'),
        DeclareLaunchArgument(
            'safe_pose_safe_threshold',
            default_value='40',
            description='Maximum global costmap value recorded as a safe pose'),
        DeclareLaunchArgument(
            'safe_pose_hold_sec',
            default_value='1.0',
            description='Unsafe-cost hold time before triggering safe pose recovery'),
        DeclareLaunchArgument(
            'safe_pose_recovery_timeout_sec',
            default_value='25.0',
            description='Timeout for safe-pose NavigateToPose recovery'),
        nav,
        step_detector,
        monitor,
    ])
