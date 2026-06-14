# 全部入りシミュレーション。
#
#   Gazebo（house world）+ HuNavSim 歩行者5人          （hunav_house.launch.py より）
#   + 3D-LiDAR TurtleBot3（waffle + Velodyne VLP-16）  （spawn_robot.launch.py より）
#   + Nav2（AMCL 自己位置推定 + 3D点群による障害物回避）
#   + RViz2
#   + Teleop / 自動巡回 GUI
#
# RViz2 の「2D Goal Pose」でゴールを与えると、歩く人（3D LiDAR が costmap に
# マークする）を避けながら家の中を自律移動する。GUI からは手動操縦・部屋の自動巡回もできる。

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
    use_perception = LaunchConfiguration('use_perception')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('gui')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_use_nav2 = DeclareLaunchArgument('use_nav2', default_value='True',
        description='Nav2 スタックを起動する')
    declare_use_perception = DeclareLaunchArgument('use_perception', default_value='True',
        description='Autoware sensing/perception パイプライン（物体検出・追跡・可視化）を起動する')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True',
        description='RViz2 を起動する')
    declare_gui = DeclareLaunchArgument('gui', default_value='True',
        description='Teleop / 自動巡回 GUI ウィンドウを起動する')
    declare_map = DeclareLaunchArgument('map',
        default_value=os.path.join(susumu_pkg, 'maps', 'cafe.yaml'),
        description='マップ yaml のフルパス')
    declare_params = DeclareLaunchArgument('params_file',
        default_value=os.path.join(susumu_pkg, 'config', 'nav2_params.yaml'),
        description='Nav2 パラメータ yaml のフルパス（3D-LiDAR 障害物回避）')
    # ロボットの spawn 姿勢。house マップ上の空きスペースに置くこと。
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # ------------------------------------------------------------------
    # 1) Gazebo house world + HuNavSim 歩行者5人。
    #    navigation:=True は HuNav launch に静的な map->odom を publish させない
    #    ことを伝える（代わりに Nav2/AMCL が提供する）。
    # ------------------------------------------------------------------
    hunav_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'include', 'hunav_house.launch.py')),
        launch_arguments={
            'robot_name': 'turtlebot3',
            'navigation': use_nav2,
        }.items())

    # ------------------------------------------------------------------
    # 2) 3D-LiDAR TurtleBot3 を spawn + robot_state_publisher。
    #    Gazebo（HuNav launch 内で起動）が先に立ち上がるよう遅延させる。
    # ------------------------------------------------------------------
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'include', 'spawn_robot.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'entity_name': 'turtlebot3',
            'x_pose': x_pose, 'y_pose': y_pose, 'yaw': yaw,
        }.items())

    spawn_robot_delayed = TimerAction(period=15.0, actions=[spawn_robot])

    # ------------------------------------------------------------------
    # 3) Nav2（自己位置推定 + ナビゲーション）。robot/TF が揃うよう遅延させる。
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

    nav2_delayed = TimerAction(period=20.0, actions=[
        LogInfo(msg='Starting Nav2 (3D-LiDAR obstacle avoidance)...'), nav2])

    # ------------------------------------------------------------------
    # 3.5) Autoware sensing/perception パイプライン。
    #      /velodyne_points → crop_box → ground_filter → euclidean_cluster
    #      （ここまで Autoware 純正）→ object_tracker → perception_marker（自作）。
    #      追跡は odom←velodyne_link の TF を使うため、robot spawn の後に起動する。
    #      Nav2 とは連携せず（生センサで動く Nav2 はそのまま）、検出結果は可視化のみ。
    # ------------------------------------------------------------------
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'include', 'autoware_perception.launch.py')),
        condition=IfCondition(use_perception),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'input_pointcloud': '/velodyne_points',
        }.items())
    perception_delayed = TimerAction(period=18.0, actions=[
        LogInfo(msg='Starting Autoware perception pipeline...'), perception])

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
    rviz_delayed = TimerAction(period=20.0, actions=[rviz])

    # ------------------------------------------------------------------
    # 5) Teleop / 自動巡回 GUI。矢印ボタン + テンキーで手動操縦し、ON/OFF トグルで
    #    Nav2 経由の部屋自動巡回を行う。navigate_to_pose アクションサーバが存在する
    #    よう Nav2 の後に起動する。
    # ------------------------------------------------------------------
    gui_node = Node(
        package='susumu_sim', executable='teleop_gui_node.py',
        name='teleop_gui', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui))
    gui_delayed = TimerAction(period=24.0, actions=[gui_node])

    ld = LaunchDescription()
    for a in (declare_use_sim_time, declare_use_nav2, declare_use_perception,
              declare_use_rviz, declare_gui, declare_map, declare_params,
              declare_x, declare_y, declare_yaw):
        ld.add_action(a)

    ld.add_action(hunav_world)
    ld.add_action(spawn_robot_delayed)
    ld.add_action(perception_delayed)
    ld.add_action(nav2_delayed)
    ld.add_action(rviz_delayed)
    ld.add_action(gui_delayed)
    return ld
