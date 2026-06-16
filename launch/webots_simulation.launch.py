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

    # driver の URDF / ros2_control 設定は webots_ros2_turtlebot の resource を流用。
    robot_description_path = os.path.join(tb3_pkg, 'resource', 'turtlebot_webots.urdf')
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
            'nav', default_value='False',
            description='Nav2 を起動する（大文字 True/False。小文字は NameError）'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='Cartographer SLAM を起動する（大文字 True/False）'),
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        turtlebot_driver,
        waiting_nodes,
        # Webots 終了時に全ノードを落とす
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
