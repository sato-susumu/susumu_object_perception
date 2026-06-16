# 起動中の Gazebo に 3D-LiDAR TurtleBot3（waffle + Velodyne VLP-16）を spawn し、
# TF 用に robot_state_publisher を起動する。
#
# 再利用可能な部品: 全部入りシミュレーション launch から include されるほか、
# 既に動いている Gazebo に対して単体で動作確認用に使うこともできる。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')

    xacro_file = os.path.join(pkg, 'urdf', 'turtlebot3_waffle_3d.urdf.xacro')
    sdf_file = os.path.join(pkg, 'models', 'turtlebot3_waffle_3d', 'model.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x_pose')
    y = LaunchConfiguration('y_pose')
    z = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    entity = LaunchConfiguration('entity_name')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z = DeclareLaunchArgument('z_pose', default_value='0.05')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    # HuNav プラグインが追跡する robot_name と一致させること（既定 'turtlebot3'）。
    declare_entity = DeclareLaunchArgument('entity_name', default_value='turtlebot3')

    # robot_state_publisher: URDF から TF を生成（base_link -> velodyne_link 含む）
    robot_description = Command(['xacro ', xacro_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}])

    # SDF モデルを spawn（Gazebo プラグインを含む: diff_drive,
    # 3D velodyne gpu_ray, imu, camera）。2D LiDAR は搭載していない。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', entity,
                   '-file', sdf_file,
                   '-x', x, '-y', y, '-z', z, '-Y', yaw])

    # 2D LiDAR を載せない代わりに、3D LiDAR の点群 /velodyne_points を
    # pointcloud_to_laserscan で 2D スキャン /scan に変換する。
    # AMCL と Nav2 の obstacle_layer はこの /scan を使う。
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/velodyne_points'),
                    ('scan', '/scan')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'velodyne_link',  # /scan の出力フレーム
            'transform_tolerance': 0.01,
            # velodyne_link 基準の高さ帯 [m]（velodyne_link は地面 +0.21m）。
            # 地面(z≈-0.21)を拾うと costmap obstacle_layer が床を障害物化して
            # 自動巡回できなくなるため、地面より明確に上(0.0=地面+0.21m)から取る。
            # 上端 1.0(地面+1.21m) は壁・人の胴体をカバーし AMCL の自己位置にも十分。
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,  # -180°
            'angle_max': 3.14159,   # +180°
            'angle_increment': 0.0087,  # ~0.5°
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 30.0,
            'use_inf': True,
        }])

    ld = LaunchDescription()
    for a in (declare_use_sim_time, declare_x, declare_y, declare_z,
              declare_yaw, declare_entity):
        ld.add_action(a)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(pointcloud_to_laserscan)
    return ld
