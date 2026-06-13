# Spawns the 3D-LiDAR TurtleBot3 (waffle + Velodyne VLP-16) into a running
# Gazebo, and starts robot_state_publisher for TF.
#
# Reusable building block: included by the full simulation launch, and also
# usable standalone against an already-running Gazebo for quick checks.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('susumu_sim')

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
    # Must match the robot_name the HuNav plugin tracks (default 'turtlebot3').
    declare_entity = DeclareLaunchArgument('entity_name', default_value='turtlebot3')

    # robot_state_publisher: TF from the URDF (incl. base_link -> velodyne_link)
    robot_description = Command(['xacro ', xacro_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}])

    # Spawn the SDF model (carries the Gazebo plugins: diff_drive, laser,
    # 3D velodyne gpu_ray, imu, camera).
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', entity,
                   '-file', sdf_file,
                   '-x', x, '-y', y, '-z', z, '-Y', yaw])

    ld = LaunchDescription()
    for a in (declare_use_sim_time, declare_x, declare_y, declare_z,
              declare_yaw, declare_entity):
        ld.add_action(a)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    return ld
