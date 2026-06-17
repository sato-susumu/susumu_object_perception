# Webots + GLIM 3D loop-closure backend + colorized point cloud mapper.
#
# GLIM is kept in an isolated TF subtree (glim_map -> glim_odom -> glim_lidar)
# so it does not conflict with Webots/Nav2 odom -> base_link. The colorized
# mapper reads /perception/colorized_points coordinates as LiDAR-frame points
# and looks up GLIM's glim_lidar pose for global color map integration.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz')
    use_perception = LaunchConfiguration('perception')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
    use_sim_time = LaunchConfiguration('use_sim_time')
    glim_config_path = LaunchConfiguration('glim_config_path')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'webots_simulation.launch.py')),
        launch_arguments=[
            ('world', world),
            ('mode', mode),
            ('nav', 'False'),
            ('slam', 'False'),
            ('rviz', use_rviz),
            ('perception', use_perception),
            ('omni_perception', 'True'),
            ('colored_slam', 'True'),
            ('colored_slam_target_frame', 'glim_map'),
            ('colored_slam_fallback_frame', ''),
            ('colored_slam_source_frame_override', 'glim_lidar'),
            ('colored_slam_output_cloud', '/slam/glim_colorized_points_map'),
            ('omni_calibration_json', omni_calibration_json),
            ('use_sim_time', use_sim_time),
        ],
    )

    # The installed CUDA GLIM stack on this machine is linked against the
    # /usr/local GTSAM/gtsam_points set. Keep the override local to this node.
    glim_ld_library_path = ':'.join([
        '/usr/local/lib',
        '/opt/ros/humble/lib',
        '/opt/ros/humble/lib/x86_64-linux-gnu',
        os.environ.get('LD_LIBRARY_PATH', ''),
    ])
    glim = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_path': glim_config_path,
        }],
        additional_env={
            'LD_LIBRARY_PATH': glim_ld_library_path,
        },
    )

    glim_pose_tf = Node(
        package='susumu_object_perception',
        executable='pose_stamped_tf_bridge_node.py',
        name='glim_pose_tf_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_pose': '/glim_ros/pose_corrected',
            'child_frame_id': 'glim_imu',
            'parent_frame_id_override': 'glim_map',
        }],
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
            description='Autoware perception を起動する。GLIM検証だけなら False'),
        DeclareLaunchArgument(
            'omni_calibration_json', default_value='',
            description='direct_visual_lidar_calibration の calib.json。空なら初期TF'),
        DeclareLaunchArgument(
            'glim_config_path',
            default_value=os.path.join(pkg, 'config', 'glim_webots'),
            description='GLIM の config.json があるディレクトリ'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Webots はシミュレーション時刻のため True'),
        sim,
        glim,
        glim_pose_tf,
    ])
