# Phase B test: launch an empty Gazebo world and spawn the 3D-LiDAR TurtleBot3.
# Used to verify /velodyne_points (PointCloud2) and TF independently of HuNav/Nav2.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    susumu_pkg = get_package_share_directory('susumu_sim')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Resolve model:// meshes (turtlebot3_common) and our 3D model.
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(tb3_gazebo, 'models') + os.pathsep
              + os.path.join(susumu_pkg, 'models'))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': os.path.join(gazebo_ros, 'worlds', 'empty.world'),
                          'verbose': 'true'}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')))

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch the Gazebo client GUI'),
        set_model_path, gzserver, gzclient, spawn])
