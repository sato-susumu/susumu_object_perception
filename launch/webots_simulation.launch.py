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
#   slam:=True  … SLAM(slam_toolbox)で地図生成しつつ自律走行（AMCL は起動しない。大文字必須）
#   rviz:=False … （webots_ros2_turtlebot 既定の RViz は本 launch では起動しない）
#
# 罠: nav:=true / slam:=true の小文字は launch 評価時に NameError でクラッシュする。
#     必ず大文字 True/False を渡すこと（docs/webots_simulation.md「ハマりどころ」参照）。

import os

import launch
from ament_index_python.packages import get_package_share_directory
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
    use_omni_perception = LaunchConfiguration('omni_perception', default=True)
    use_colored_slam = LaunchConfiguration('colored_slam', default=True)
    colored_slam_target_frame = LaunchConfiguration('colored_slam_target_frame')
    colored_slam_fallback_frame = LaunchConfiguration('colored_slam_fallback_frame')
    colored_slam_source_frame_override = LaunchConfiguration(
        'colored_slam_source_frame_override')
    colored_slam_output_cloud = LaunchConfiguration('colored_slam_output_cloud')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
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

    # LiDAR/camera TF。未キャリブレーション時は base->lidar と lidar->camera の
    # 初期値を出し、calib.json 指定時は direct_visual_lidar_calibration の結果を使う。
    omni_sensor_tf = Node(
        package='susumu_object_perception',
        executable='omni_sensor_tf_node.py',
        name='omni_sensor_tf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_link',
            'lidar_frame': 'velodyne_link',
            'camera_frame': 'omni_camera_link',
            'lidar_xyz': [0.0, 0.0, 0.20],
            'camera_xyz_initial': [0.0, 0.0, 0.75],
            'calibration_json': omni_calibration_json,
        }])

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

    # Nav2 は nav2_bringup の bringup_launch.py を直接呼び、slam 引数を渡す。
    # こうすると localization が排他的に切り替わる:
    #   slam:=False → AMCL（事前地図 + 初期位置で自己位置推定）
    #   slam:=True  → slam_toolbox（地図を作りながら map->odom を供給。AMCL は起動しない）
    # 以前は turtlebot3_navigation2(AMCL固定) + 別途 Cartographer/slam_toolbox を足しており、
    # slam:=True で AMCL と SLAM が両方 map->odom を出して競合した。bringup に slam を委ねて
    # 一本化することで二重起動・TF 競合を根絶する（slam_toolbox に統一）。
    navigation_nodes = []
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_map = os.path.join(tb3_pkg, 'resource', 'turtlebot3_burger_example_map.yaml')
    nav2_params = os.path.join(tb3_pkg, 'resource', 'nav2_params.yaml')
    turtlebot_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('slam', use_slam),
            ('map', nav2_map),
            ('params_file', nav2_params),
            ('use_sim_time', use_sim_time),
        ],
        condition=launch.conditions.IfCondition(use_nav))
    navigation_nodes.append(turtlebot_navigation)

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

    colorized_points = Node(
        package='susumu_object_perception',
        executable='colorized_pointcloud_node.py',
        name='colorized_pointcloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cloud': '/velodyne_points/point_cloud',
            'input_image': '/omni_camera/image_raw/image_color',
            'output_cloud': '/perception/colorized_points',
            'camera_frame': 'omni_camera_link',
        }],
        condition=launch.conditions.IfCondition(use_omni_perception))

    pointcloud_intensity = Node(
        package='susumu_object_perception',
        executable='pointcloud_intensity_node.py',
        name='pointcloud_intensity',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cloud': '/velodyne_points/point_cloud',
            'output_cloud': '/velodyne_points/point_cloud_intensity',
        }],
        condition=launch.conditions.IfCondition(use_omni_perception))

    equirect_camera_info = Node(
        package='susumu_object_perception',
        executable='equirect_camera_info_node.py',
        name='equirect_camera_info',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_image': '/omni_camera/image_raw/image_color',
            'output_camera_info': '/omni_camera/equirect/camera_info',
            'camera_frame': 'omni_camera_link',
        }],
        condition=launch.conditions.IfCondition(use_omni_perception))

    omni_image_compress = Node(
        package='susumu_object_perception',
        executable='omni_image_compress_node.py',
        name='omni_image_compress',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_image': '/omni_camera/image_raw/image_color',
            'output_image': '/omni_camera/image_raw/compressed',
            'jpeg_quality': 80,
        }],
        condition=launch.conditions.IfCondition(use_omni_perception))

    object_crops = Node(
        package='susumu_object_perception',
        executable='object_image_crop_node.py',
        name='object_image_crop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_image': '/omni_camera/image_raw/image_color',
            'input_objects': '/perception/tracked_objects',
            'object_type': 'tracked',
            'output_image': '/perception/object_crops/image_rect',
            'camera_frame': 'omni_camera_link',
        }],
        condition=launch.conditions.IfCondition(use_omni_perception))

    colorized_mapper = Node(
        package='susumu_object_perception',
        executable='colorized_pointcloud_mapper_node.py',
        name='colorized_pointcloud_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cloud': '/perception/colorized_points',
            'output_cloud': colored_slam_output_cloud,
            'target_frame': colored_slam_target_frame,
            'fallback_frame': colored_slam_fallback_frame,
            'source_frame_override': colored_slam_source_frame_override,
            'voxel_size': 0.08,
            'max_voxels': 250000,
        }],
        condition=launch.conditions.IfCondition(use_colored_slam))

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
            description='SLAM(slam_toolbox)で地図生成しつつ自律走行（AMCL は無効。大文字 True/False）'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する（既定 True）'),
        DeclareLaunchArgument(
            'perception', default_value='True',
            description='Autoware perception パイプラインを起動する（既定 True。'
                        '3D LiDAR /velodyne_points を入力に検出・追跡・可視化）'),
        DeclareLaunchArgument(
            'omni_perception', default_value='True',
            description='全天球カメラ連携（色付き点群・物体クロップ）を起動する'),
        DeclareLaunchArgument(
            'colored_slam', default_value='True',
            description='SLAM/odom座標に色付き点群を蓄積して /slam/colorized_points_map を出す'),
        DeclareLaunchArgument(
            'colored_slam_target_frame', default_value='map',
            description='色付き点群マップの目標TFフレーム。GLIMでは glim_map を指定する'),
        DeclareLaunchArgument(
            'colored_slam_fallback_frame', default_value='odom',
            description='target_frame が未接続のときのフォールバックTFフレーム。不要なら空文字'),
        DeclareLaunchArgument(
            'colored_slam_source_frame_override', default_value='',
            description='色付き点群のTF lookup用source frame上書き。GLIMでは glim_lidar を指定する'),
        DeclareLaunchArgument(
            'colored_slam_output_cloud', default_value='/slam/colorized_points_map',
            description='蓄積した色付き点群マップの出力トピック'),
        DeclareLaunchArgument(
            'omni_calibration_json', default_value='',
            description='direct_visual_lidar_calibration の calib.json。空なら初期TFを使う'),
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        omni_sensor_tf,
        pointcloud_to_laserscan,
        turtlebot_driver,
        perception,
        colorized_points,
        pointcloud_intensity,
        equirect_camera_info,
        omni_image_compress,
        object_crops,
        colorized_mapper,
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
