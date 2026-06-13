# Phase D: full simulation.
#
#   Gazebo (house world) + 5 HuNavSim pedestrians      (from hunav_house.launch.py)
#   + 3D-LiDAR TurtleBot3 (waffle + Velodyne VLP-16)   (from spawn_robot.launch.py)
#   + Nav2 (AMCL localization + 3D-pointcloud obstacle avoidance)
#   + RViz2
#
# Give a goal in RViz2 ("2D Goal Pose") and the robot navigates the house while
# avoiding the moving people (the 3D LiDAR marks them in the costmaps).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    susumu_pkg = get_package_share_directory('susumu_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    follow = LaunchConfiguration('follow')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_use_nav2 = DeclareLaunchArgument('use_nav2', default_value='True',
        description='Launch the Nav2 stack')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True',
        description='Launch RViz2')
    declare_follow = DeclareLaunchArgument('follow', default_value='False',
        description='Launch the LiDAR person-follow pipeline (walk on a person\'s right)')
    declare_map = DeclareLaunchArgument('map',
        default_value=os.path.join(susumu_pkg, 'maps', 'house.yaml'),
        description='Full path to the map yaml')
    declare_params = DeclareLaunchArgument('params_file',
        default_value=os.path.join(susumu_pkg, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 params yaml (3D-LiDAR obstacle avoidance)')
    # Robot spawn pose. Keep it on free space in the house map.
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # ------------------------------------------------------------------
    # 1) Gazebo house world + 5 HuNavSim pedestrians.
    #    navigation:=True tells the HuNav launch NOT to publish a static
    #    map->odom (Nav2/AMCL will provide it instead).
    # ------------------------------------------------------------------
    hunav_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'hunav_house.launch.py')),
        launch_arguments={
            'robot_name': 'turtlebot3',
            'navigation': use_nav2,
        }.items())

    # ------------------------------------------------------------------
    # 2) Spawn the 3D-LiDAR TurtleBot3 + robot_state_publisher.
    #    Delay so Gazebo (started inside the HuNav launch) is up first.
    # ------------------------------------------------------------------
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'entity_name': 'turtlebot3',
            'x_pose': x_pose, 'y_pose': y_pose, 'yaw': yaw,
        }.items())

    spawn_robot_delayed = TimerAction(period=8.0, actions=[spawn_robot])

    # ------------------------------------------------------------------
    # 3) Nav2 (localization + navigation). Delay so the robot/TF exist.
    # ------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam': 'False',
            'autostart': 'True',
        }.items())

    nav2_delayed = TimerAction(period=12.0, actions=[
        LogInfo(msg='Starting Nav2 (3D-LiDAR obstacle avoidance)...'), nav2])

    # ------------------------------------------------------------------
    # 4) RViz2
    # ------------------------------------------------------------------
    rviz_config = os.path.join(susumu_pkg, 'rviz', 'simulation.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz))
    rviz_delayed = TimerAction(period=12.0, actions=[rviz])

    # ------------------------------------------------------------------
    # 5) Optional: LiDAR person-follow pipeline (walk on a person's right).
    #    Starts after Nav2 so the navigate_to_pose action server is up.
    # ------------------------------------------------------------------
    follow_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'follow_person.launch.py')),
        condition=IfCondition(follow),
        launch_arguments={'use_sim_time': use_sim_time}.items())
    follow_delayed = TimerAction(period=18.0, actions=[
        LogInfo(msg='Starting LiDAR person-follow (walk on person\'s right)...'),
        follow_pipeline])

    ld = LaunchDescription()
    for a in (declare_use_sim_time, declare_use_nav2, declare_use_rviz,
              declare_follow, declare_map, declare_params,
              declare_x, declare_y, declare_yaw):
        ld.add_action(a)

    ld.add_action(hunav_world)
    ld.add_action(spawn_robot_delayed)
    ld.add_action(nav2_delayed)
    ld.add_action(rviz_delayed)
    ld.add_action(follow_delayed)
    return ld
