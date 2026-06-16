# Webots シミュレーション起動 launch（このパッケージ同梱の world を直接使う）。
#
# 従来は webots_ros2_turtlebot/robot_launch.py を手打ちし、自作 world を
# /opt/ros/humble/share/webots_ros2_turtlebot/worlds/ へ sudo cp する必要があった
# （robot_launch.py は world を PathJoinSubstitution([package_dir,'worlds',world]) で
#  解決するため、パッケージ外のフルパス world を受け付けない）。
#
# 本 launch は robot_launch.py の driver 配線（WebotsLauncher + WebotsController +
# robot_state_publisher + ros2_control spawner）を踏襲しつつ、WebotsLauncher の world に
# 本パッケージ susumu_object_perception/webots_worlds/<world>.wbt のフルパスを渡す。
# これで sudo cp が不要になる。URDF/ros2control 等の resource は webots_ros2_turtlebot の
# share をそのまま参照する（複製しない）。
#
#   world:=outdoor （既定）/ world:=indoor          … webots_worlds/<world>.wbt を読む
#   nav:=True   … Nav2（turtlebot3_navigation2）を起動（大文字必須）
#   slam:=True  … Cartographer SLAM（turtlebot3_cartographer）を起動（大文字必須）
#   rviz:=False … （webots_ros2_turtlebot 既定の RViz は本 launch では起動しない）
#
# 罠: nav:=true / slam:=true の小文字は launch 評価時に NameError でクラッシュする。
#     必ず大文字 True/False を渡すこと（docs/webots_simulation.md「ハマりどころ」参照）。

import os

import launch
from ament_index_python.packages import (get_package_share_directory,
                                          get_packages_with_prefixes)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import \
    WaitForControllerConnection


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')
    tb3_pkg = get_package_share_directory('webots_ros2_turtlebot')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_perception = LaunchConfiguration('perception', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # 本パッケージ同梱の world を直接指す（sudo cp 不要）。
    # world 引数は拡張子込みのファイル名（outdoor.wbt / indoor.wbt）で受け、upstream
    # robot_launch.py と同じく PathJoinSubstitution に素直に渡す。
    # ※ 拡張子を launch 側で [world,'.wbt'] と連結すると、WebotsLauncher が作る一時 world 名が
    #   .wbt.wbt と二重化して "could not open file" で落ちる（実測）。拡張子込みで受けるのが確実。
    world_path = PathJoinSubstitution([pkg, 'webots_worlds', world])

    webots = WebotsLauncher(
        world=world_path,
        mode=mode,
        ros2_supervisor=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # 3D LiDAR の frame。world で Lidar を +0.20m に置いたので base_link→velodyne_link を
    # 同じ +0.20m で publish（perception の crop_box が velodyne_link 基準で処理するため）。
    velodyne_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.20', '0', '0', '0', 'base_link', 'velodyne_link'],
    )

    # 2D LiDAR(LDS-01) を廃止したので /scan は 3D 点群から生成する（Gazebo 側と同構成）。
    # Nav2 の AMCL / costmap obstacle_layer がこの /scan を使う。Webots の PointCloud2 は
    # /velodyne_points/point_cloud に出る（driver が /point_cloud サフィックスを付ける）。
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/velodyne_points/point_cloud'),
                    ('scan', '/scan')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'velodyne_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 30.0,
            'use_inf': True,
        }])

    # ROS 2 control spawners（webots_ros2_turtlebot 踏襲）
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner,
                            joint_state_broadcaster_spawner]

    # driver の URDF は本パッケージの 3D LiDAR 拡張版（velodyne→/velodyne_points）を使う。
    # ros2_control 設定は webots_ros2_turtlebot の resource を流用。
    robot_description_path = os.path.join(pkg, 'resource', 'turtlebot_webots_3d.urdf')
    ros2_control_params = os.path.join(tb3_pkg, 'resource', 'ros2control.yml')
    use_twist_stamped = ('ROS_DISTRO' in os.environ
                         and os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    # Nav2（turtlebot3_navigation2）— webots_ros2_turtlebot の map/params を流用。
    navigation_nodes = []
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    nav2_map = os.path.join(tb3_pkg, 'resource', 'turtlebot3_burger_example_map.yaml')
    nav2_params = os.path.join(tb3_pkg, 'resource', 'nav2_params.yaml')
    if 'turtlebot3_navigation2' in get_packages_with_prefixes():
        turtlebot_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch', 'navigation2.launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('params_file', nav2_params),
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_nav))
        navigation_nodes.append(turtlebot_navigation)

    # SLAM（turtlebot3_cartographer）。slam:=True で起動。
    if 'turtlebot3_cartographer' in get_packages_with_prefixes():
        turtlebot_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_cartographer'),
                'launch', 'cartographer.launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_slam))
        navigation_nodes.append(turtlebot_slam)

    # Autoware perception パイプライン（perception:=True で起動）。
    # world に追加した 3D LiDAR が /velodyne_points を出すので、Gazebo 同様に検出できる。
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', 'include', 'autoware_perception.launch.py')),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            # webots_ros2_driver の Lidar は PointCloud2 を <topicName>/point_cloud に出す
            # （URDF topicName=/velodyne_points → 実トピックは /velodyne_points/point_cloud）。
            ('input_pointcloud', '/velodyne_points/point_cloud'),
        ],
        condition=launch.conditions.IfCondition(use_perception))

    # RViz2（rviz:=True で起動）。本パッケージの設定を使う。
    rviz_config = os.path.join(pkg, 'rviz', 'simulation.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=launch.conditions.IfCondition(use_rviz))

    # シミュレータ準備完了（driver 接続）を待ってから nav/control を起動する。
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=navigation_nodes + ros_control_spawners,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='outdoor.wbt',
            description='webots_worlds/ の world ファイル名（outdoor.wbt / indoor.wbt、拡張子込み）'),
        DeclareLaunchArgument(
            'mode', default_value='realtime',
            description='Webots 起動モード（realtime / fast / pause）'),
        DeclareLaunchArgument(
            'nav', default_value='True',
            description='Nav2 を起動する（既定 True。見るだけなら nav:=False。大文字必須、小文字は NameError）'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='Cartographer SLAM を起動する（大文字 True/False）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する（既定 True）'),
        DeclareLaunchArgument(
            'perception', default_value='True',
            description='Autoware perception パイプラインを起動する（既定 True。'
                        '3D LiDAR /velodyne_points を入力に検出・追跡・可視化）'),
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        velodyne_tf,
        pointcloud_to_laserscan,
        turtlebot_driver,
        perception,
        rviz,
        waiting_nodes,
        # Webots 終了時に全ノードを落とす
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
