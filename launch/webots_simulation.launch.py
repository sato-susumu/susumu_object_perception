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
#   world:=outdoor.wbt （既定）/ world:=indoor.wbt … webots_worlds/<world> を読む
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
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import \
    WaitForControllerConnection


def _resolve_package_file(value, pkg, default_path, subdir):
    if value == '':
        return default_path
    if os.path.isabs(value):
        return value
    if '/' not in value:
        return os.path.join(pkg, subdir, value)
    return os.path.join(pkg, value)


def _make_robot_control_actions(context, *, pkg, tb3_pkg, robot_description_path,
                                use_sim_time, ros2_control_params_file,
                                nav_start_delay_sec, mappings, navigation_nodes,
                                ros_control_spawners):
    default_ros2_control_params = os.path.join(
        tb3_pkg, 'resource', 'ros2control.yml')
    ros2_control_params = _resolve_package_file(
        ros2_control_params_file.perform(context),
        pkg,
        default_ros2_control_params,
        'config')
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
    nav_delay = float(nav_start_delay_sec.perform(context) or 0.0)
    nav_actions = list(navigation_nodes)
    if nav_delay > 0.0:
        nav_actions = [
            TimerAction(period=nav_delay, actions=nav_actions)
        ]
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners + nav_actions,
    )
    return [turtlebot_driver, waiting_nodes]


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')
    tb3_pkg = get_package_share_directory('webots_ros2_turtlebot')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    # Nav2 params 差し替え用（空なら従来の webots_ros2_turtlebot 標準を使う）。
    # 探索マッピングでは inflation を下げた config/nav2_params_webots_explore.yaml を渡す。
    nav_params_file = LaunchConfiguration('nav_params_file', default='')
    # ros2_control params 差し替え用。空なら従来の webots_ros2_turtlebot 標準を使う。
    # EKF が odom->base_link TF を出す評価では diffdrive 側の enable_odom_tf:false 設定を渡す。
    ros2_control_params_file = LaunchConfiguration(
        'ros2_control_params_file', default='')
    nav_start_delay_sec = LaunchConfiguration('nav_start_delay_sec')
    # slam:=False の AMCL/保存地図ナビで Nav2 map_server に読ませる地図。
    # 空なら従来の webots_ros2_turtlebot 標準地図を使う。
    map_file = LaunchConfiguration('map_file', default='')
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_perception = LaunchConfiguration('perception', default=True)
    use_omni_perception = LaunchConfiguration('omni_perception', default=True)
    # 画像認識（LiDAR 検出物体の YOLO 分類 + 全天球信号認識）。YOLO は CPU 負荷が高いが
    # 間引き（トラック ID キャッシュ + レート上限）があるので既定 ON。重いとき image_recognition:=False。
    use_image_recognition = LaunchConfiguration('image_recognition', default=True)
    tl_method = LaunchConfiguration('traffic_light_method')
    tl_weights = LaunchConfiguration('traffic_light_weights')
    om_delete_thresh = LaunchConfiguration('object_memory_delete_thresh')
    om_miss_tp = LaunchConfiguration('object_memory_miss_tp')
    om_miss_fp = LaunchConfiguration('object_memory_miss_fp')
    om_visible_range = LaunchConfiguration('object_memory_visible_range')
    object_yolo_weights = LaunchConfiguration('object_yolo_weights')
    object_yolo_imgsz = LaunchConfiguration('object_yolo_imgsz')
    object_yolo_conf = LaunchConfiguration('object_yolo_conf')
    object_min_accept_conf = LaunchConfiguration('object_min_accept_conf')
    object_crop_fovs_deg = LaunchConfiguration('object_crop_fovs_deg')
    object_crop_yaw_offsets_deg = LaunchConfiguration(
        'object_crop_yaw_offsets_deg')
    object_crop_pitch_offsets_deg = LaunchConfiguration(
        'object_crop_pitch_offsets_deg')
    object_crop_shape_center_height_fracs = LaunchConfiguration(
        'object_crop_shape_center_height_fracs')
    object_crop_shape_bbox_margins_deg = LaunchConfiguration(
        'object_crop_shape_bbox_margins_deg')
    object_classifier_debug = LaunchConfiguration('object_classifier_debug')
    object_debug_crop_dir = LaunchConfiguration('object_debug_crop_dir')
    object_debug_crop_max_per_track = LaunchConfiguration(
        'object_debug_crop_max_per_track')
    object_tracker_debug = LaunchConfiguration('object_tracker_debug')
    object_tracker_min_hits = LaunchConfiguration('object_tracker_min_hits')
    object_tracker_wall_margin_moving_cells = LaunchConfiguration(
        'object_tracker_wall_margin_moving_cells')
    object_tracker_wall_margin_static_cells = LaunchConfiguration(
        'object_tracker_wall_margin_static_cells')
    use_colored_slam = LaunchConfiguration('colored_slam', default=True)
    lidar_model = LaunchConfiguration('lidar_model')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')
    scan_angle_increment = LaunchConfiguration('scan_angle_increment')
    scan_range_min = LaunchConfiguration('scan_range_min')
    scan_range_max = LaunchConfiguration('scan_range_max')
    scan_use_inf = LaunchConfiguration('scan_use_inf')
    colored_slam_target_frame = LaunchConfiguration('colored_slam_target_frame')
    colored_slam_fallback_frame = LaunchConfiguration('colored_slam_fallback_frame')
    colored_slam_source_frame_override = LaunchConfiguration(
        'colored_slam_source_frame_override')
    colored_slam_output_cloud = LaunchConfiguration('colored_slam_output_cloud')
    colored_slam_max_range = LaunchConfiguration('colored_slam_max_range')
    colored_slam_min_z = LaunchConfiguration('colored_slam_min_z')
    colored_slam_max_z = LaunchConfiguration('colored_slam_max_z')
    omni_calibration_json = LaunchConfiguration('omni_calibration_json')
    strict_omni_calibration_json = LaunchConfiguration(
        'strict_omni_calibration_json')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    lidar_frame = 'lidar_link'
    lidar_points_topic = '/lidar/points/point_cloud'
    lidar_points_intensity = '/lidar/points_intensity'
    lidar_num_rings = PythonExpression([
        "'16' if '", lidar_model, "' == 'vlp16' else '64'"
    ])
    lidar_min_elev = PythonExpression([
        "'-15.0' if '", lidar_model, "' == 'vlp16' else '-55.0'"
    ])
    lidar_max_elev = PythonExpression([
        "'15.0' if '", lidar_model, "' == 'vlp16' else '55.0'"
    ])

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
            'lidar_frame': lidar_frame,
            'camera_frame': 'omni_camera_link',
            'lidar_xyz': [0.0, 0.0, 0.20],
            'camera_xyz_initial': [0.0, 0.0, 0.75],
            'calibration_json': omni_calibration_json,
            'strict_calibration_json': strict_omni_calibration_json,
        }])

    # 2D LiDAR(LDS-01) を廃止したので /scan は 3D 点群から生成する（Gazebo 側と同構成）。
    # Nav2 の AMCL / costmap obstacle_layer と slam_toolbox がこの /scan を使う。
    #
    # 入力は生点群 /lidar/points/point_cloud（perception 非依存。OFF でも /scan が出る）。
    #
    # 【前提: wbt の Lidar tiltAngle=0】Webots Lidar は tiltAngle≠0 だと点の高さが過大に
    # なるバグ(cyberbotics/webots #37, 未修正)があり、平地が原点中心の同心円状に持ち上がって
    # 地図に「円形の影」を焼いていた。全 wbt で tiltAngle を 0 にして回避済み（点群解析で、
    # 下向きビームの地上高さが正しく ≈0、上向きビームは空に抜けて消えることを確認）。
    # tiltAngle=0 で FOV は対称（仰角 ±30deg 相当）になる。
    #
    # 高さ帯: lidar_link 基準（LiDAR は地上 0.2m）。地面は z≈-0.2（地上 0）に正しく乗るので、
    # z>=0.1（地上約 0.3m）で地面を除外しつつ、屋内の低い家具〜壁、人を拾う。上限 z=2.0
    # （地上 2.2m）で天井/壁上部を外す。range_min は屋内の近い壁(〜0.7m)を残すため 0.3。
    #
    # 【屋内専用設定】この設定は indoor / break_room / cafe など特徴の多い屋内向けに最適化
    # してある。outdoor / city_robot のような特徴の少ない広域 world（20m級 + 建物/植木のみ）
    # は **未対応**（建物 occupied と SLAM 姿勢安定の両立ができず、地図が崩れるか自己位置喪失）。
    # 詳細は docs/tasks/mapping.md「特徴の少ない広域世界は未対応」を参照。
    # 過去に試した設定（いずれも片方しか満たせない）:
    #   (a) use_inf:False, inf_epsilon:-0.5（15.5m偽hit）→ 広い free 地図は出るが建物 occ 全消失
    #   (b) use_inf:True, min/max_height:0.05/10, range_max:20, slam max_laser_range:18
    #       → 建物 occ は出るが /scan の有効 hit が疎で SLAM scan match が不安定化し自己位置喪失
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/lidar/points/point_cloud'),
                    ('scan', '/scan')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': lidar_frame,
            'transform_tolerance': 0.01,
            # lidar_link 基準。既定値は屋内向け。屋外 launch だけ別値を渡す。
            'min_height': ParameterValue(scan_min_height, value_type=float),
            'max_height': ParameterValue(scan_max_height, value_type=float),
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': ParameterValue(
                scan_angle_increment, value_type=float),
            'scan_time': 0.1,
            'range_min': ParameterValue(scan_range_min, value_type=float),
            'range_max': ParameterValue(scan_range_max, value_type=float),
            'use_inf': ParameterValue(scan_use_inf, value_type=bool),
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

    # driver の URDF は本パッケージの 3D LiDAR 拡張版（/lidar/points）を使う。
    # ros2_control 設定は既定で webots_ros2_turtlebot の resource を流用し、
    # 評価・調整時だけ launch 引数で差し替える。
    robot_description_path = os.path.join(pkg, 'resource', 'turtlebot_webots_3d.urdf')
    use_twist_stamped = ('ROS_DISTRO' in os.environ
                         and os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')]

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
    default_nav2_map = os.path.join(
        tb3_pkg, 'resource', 'turtlebot3_burger_example_map.yaml')
    # Nav2 bringup に渡す map/params は絶対パスで安定させる。
    # launch 実行ディレクトリに依存すると、`map_file:=indoor.yaml` が
    # cd 位置によって読めたり読めなかったりする。相対指定は package share
    # 基準（ファイル名だけなら outputs/mapping_indoor/ → outputs/mapping_outdoor/
    # の順で探索）に解決する。スラッシュを含む相対パスはパッケージ直下基準。
    indoor_prefix = os.path.join(pkg, 'outputs', 'mapping_indoor', '')
    outdoor_prefix = os.path.join(pkg, 'outputs', 'mapping_outdoor', '')
    nav2_map = PythonExpression([
        "'", default_nav2_map, "' if '", map_file, "' == '' else (",
        "'", map_file, "' if '", map_file, "'.startswith('/') else (",
        "('", indoor_prefix, "' + '", map_file,
        "') if ('/' not in '", map_file,
        "' and __import__('os').path.exists('", indoor_prefix, "' + '", map_file, "')) else (",
        "('", outdoor_prefix, "' + '", map_file,
        "') if '/' not in '", map_file, "' else ",
        "'", os.path.join(pkg, ''), "' + '", map_file, "')))"])
    default_nav2_params = os.path.join(tb3_pkg, 'resource', 'nav2_params.yaml')
    # nav_params_file が空なら従来の標準 params、指定があればそれを使う。
    # マッピング（webots_indoor_mapping）は nav2_params_webots_explore.yaml を渡す。
    nav2_params = PythonExpression([
        "'", default_nav2_params, "' if '", nav_params_file, "' == '' else (",
        "'", nav_params_file, "' if '", nav_params_file, "'.startswith('/') else (",
        "'", os.path.join(pkg, 'config', ''), "' + '", nav_params_file,
        "' if '/' not in '", nav_params_file, "' else ",
        "'", os.path.join(pkg, ''), "' + '", nav_params_file, "'))"])
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
    robot_control = OpaqueFunction(
        function=lambda context: _make_robot_control_actions(
            context,
            pkg=pkg,
            tb3_pkg=tb3_pkg,
            robot_description_path=robot_description_path,
            use_sim_time=use_sim_time,
            ros2_control_params_file=ros2_control_params_file,
            nav_start_delay_sec=nav_start_delay_sec,
            mappings=mappings,
            navigation_nodes=navigation_nodes,
            ros_control_spawners=ros_control_spawners))

    # 転倒検知（常時監視）。IMU の姿勢から機体の傾きを見て、しきい値超えで転倒を警告する。
    # odom は 2D 前提で roll/pitch=0 のため転倒を検知できず、IMU(InertialUnit)を使う。
    fall_detector = Node(
        package='susumu_object_perception',
        executable='fall_detector_node.py',
        name='fall_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'imu_topic': '/imu',
        }],
    )

    # Autoware perception パイプライン（perception:=True で起動）。
    # world に追加した 3D LiDAR が /lidar/points を出すので、Gazebo 同様に検出できる。
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', 'include', 'autoware_perception.launch.py')),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            # webots_ros2_driver の Lidar は PointCloud2 を <topicName>/point_cloud に出す。
            ('input_pointcloud', lidar_points_topic),
            ('lidar_frame', lidar_frame),
            ('num_rings', lidar_num_rings),
            ('min_elev_deg', lidar_min_elev),
            ('max_elev_deg', lidar_max_elev),
            ('indoor_objects', LaunchConfiguration('indoor_objects')),
            ('object_tracker_debug', object_tracker_debug),
            ('object_tracker_min_hits', object_tracker_min_hits),
            ('object_tracker_wall_margin_moving_cells',
             object_tracker_wall_margin_moving_cells),
            ('object_tracker_wall_margin_static_cells',
             object_tracker_wall_margin_static_cells),
        ],
        condition=launch.conditions.IfCondition(use_perception))

    colorized_points = Node(
        package='susumu_object_perception',
        executable='colorized_pointcloud_node.py',
        name='colorized_pointcloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cloud': lidar_points_topic,
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
            'input_cloud': lidar_points_topic,
            'output_cloud': lidar_points_intensity,
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

    # === 画像認識（image_recognition:=True で起動）===
    # LiDAR 検出物体を全天球 YOLO で分類（car/person 等）。
    object_classifier = Node(
        package='susumu_object_perception',
        executable='object_classifier_node.py',
        name='object_classifier',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_objects': '/perception/tracked_objects',
            'input_image': '/omni_camera/image_raw/image_color',
            'camera_frame': 'omni_camera_link',
            # 識別率チューニング。seg weight を使い、LiDAR 対象方向の中心に
            # YOLO mask が乗る候補だけを採ることで、クロップ背景の植物誤認を抑える。
            'yolo.weights': object_yolo_weights,
            'yolo.imgsz': ParameterValue(object_yolo_imgsz, value_type=int),
            'yolo.conf': ParameterValue(object_yolo_conf, value_type=float),
            'min_accept_conf': ParameterValue(
                object_min_accept_conf, value_type=float),
            'min_accept_conf_overrides': LaunchConfiguration(
                'object_min_accept_conf_overrides'),
            'crop_fov_deg': 75.0,
            'crop_fovs_deg': ParameterValue(
                object_crop_fovs_deg, value_type=str),
            'crop_yaw_offsets_deg': ParameterValue(
                object_crop_yaw_offsets_deg, value_type=str),
            'crop_pitch_offsets_deg': ParameterValue(
                object_crop_pitch_offsets_deg, value_type=str),
            'crop_shape_center_height_fracs': ParameterValue(
                object_crop_shape_center_height_fracs, value_type=str),
            'crop_shape_bbox_margins_deg': ParameterValue(
                object_crop_shape_bbox_margins_deg, value_type=str),
            # LiDAR 対象方向の中心ROIと YOLO bbox の重なりで fine class を絞る実験用ゲート。
            # 屋内フル巡回では正解候補の hits が伸びにくくなったため、既定は無効。
            'center_window_frac': 0.25,
            'min_center_window_overlap': 0.0,
            'require_mask_center': True,
            'mask_center_window_frac': 0.25,
            'min_mask_center_overlap': 0.04,
            # 植物系ラベルだけ、YOLO bbox 内に緑/花色が一定以上あるかを確認する。
            # 家具・箱・壁片の potted plant 誤登録を減らすための認識本体側ゲート。
            'plant_color_min_frac': 0.02,
            # 定期再分類は実験用。屋内フル巡回評価では一時的な YOLO miss で
            # 正解記憶の hits が伸びにくくなったため、既定は従来どおり無効。
            'reclassify_interval_sec': 0.0,
            'min_consistent_hits': 1,
            'max_class_misses': 1,
            'max_rate_hz': 2.0,
            'max_inferences_per_cycle': 4,
            'publish_unknown_fine_class_clears': False,
            'publish_debug_diagnostics': object_classifier_debug,
            'debug_crop_dir': ParameterValue(
                object_debug_crop_dir, value_type=str),
            'debug_crop_max_per_track': ParameterValue(
                object_debug_crop_max_per_track, value_type=int),
        }],
        condition=launch.conditions.IfCondition(use_image_recognition))

    # 信号認識（全天球を全周 N 分割の透視ビューで検出・色判定）。
    traffic_light_detector = Node(
        package='susumu_object_perception',
        executable='traffic_light_detector_node.py',
        name='traffic_light_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'omni_mode': True,
            'input_image': '/omni_camera/image_raw/image_color',
            'method': tl_method,
            'yolo.weights': tl_weights,
        }],
        condition=launch.conditions.IfCondition(use_image_recognition))

    # 信号の 3D 位置推定（検出方向 × LiDAR 点群）。
    traffic_light_localizer = Node(
        package='susumu_object_perception',
        executable='traffic_light_localizer_node.py',
        name='traffic_light_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'points_topic': lidar_points_topic,
        }],
        condition=launch.conditions.IfCondition(use_image_recognition))

    # 信号認識の可視化（全天球画像に方位帯マーカーを重畳）。
    traffic_light_marker = Node(
        package='susumu_object_perception',
        executable='traffic_light_marker_node.py',
        name='traffic_light_marker',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'omni_mode': True,
            'input_image': '/omni_camera/image_raw/image_color',
        }],
        condition=launch.conditions.IfCondition(use_image_recognition))

    # === セマンティック物体メモリ（image_recognition:=True で起動）===
    # tracked_objects_classified を SQLite DB に記録し、認識タスクの最終評価
    # (render_recognition_overlay.py / evaluate_recognition_vs_world.py) の入力にする。
    # DB の出力先は ~/.ros/object_memory.sqlite3 が既定。run_all_tasks.sh では
    # db_path:= で /tmp/<world>_object_memory.sqlite3 にリダイレクトする。
    object_memory = Node(
        package='susumu_object_perception',
        executable='object_memory_node.py',
        name='object_memory',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/perception/tracked_objects_classified',
            'require_fine_class': True,
            'min_fine_conf': 0.15,
            'require_map_support': True,
            'map_support_dist': 0.45,
            'map_support_class_distances': 'plant=0.55,table=0.55',
            'static_class_geometry_filter': True,
            'static_duplicate_merge_dist': 1.7,
            'static_cross_class_merge_dist': 0.75,
            'static_compatible_class_groups': 'chair,couch',
            'static_merge_class_priority': 'chair,couch',
            'delete_thresh': ParameterValue(om_delete_thresh, value_type=float),
            'miss_tp': ParameterValue(om_miss_tp, value_type=float),
            'miss_fp': ParameterValue(om_miss_fp, value_type=float),
            'visible_range': ParameterValue(om_visible_range, value_type=float),
        }],
        condition=launch.conditions.IfCondition(use_image_recognition))

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
            'voxel_size': 0.05,
            'max_voxels': 400000,
            # 屋内向けブレ低減フィルタ。LiDAR 遠距離点は SLAM 姿勢の小さな回転誤差が
            # 増幅されて放射状の筋（ブレ）になるため近距離だけ採る。床下/天井の散乱も切る。
            # 値は LiDAR(lidar_link)座標基準。屋内 5x10m なので max_range 7m で全域に届く。
            'max_range': colored_slam_max_range,
            'min_z': colored_slam_min_z,
            'max_z': colored_slam_max_z,
            # 静止時のみ蓄積モード (iter4 で実装)。 巡回中の SLAM 2D 姿勢誤差が
            # 色付き点群の壁ブレの真因なので、 動いている間のフレームを除外する。
            'stationary_only': LaunchConfiguration('stationary_only'),
            'stationary_max_lin_velocity': LaunchConfiguration(
                'stationary_max_lin_velocity'),
            'stationary_max_ang_velocity': LaunchConfiguration(
                'stationary_max_ang_velocity'),
            # 色付き点群 PLY をプロジェクト内 outputs/colorized_pointcloud/ に保存。
            # install/share でなくソースを指す（再生成物を手元に残すため）。
            'save_dir': os.path.join(
                os.path.expanduser(
                    '~/ros2_ws/src/susumu_object_perception'),
                'outputs', 'colorized_pointcloud'),
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
            'map_file', default_value='',
            description='slam:=False の AMCL/保存地図ナビで Nav2 に読ませる地図 yaml。空なら TurtleBot3 既定地図'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='RViz2 を起動する（既定 True）'),
        DeclareLaunchArgument(
            'perception', default_value='True',
            description='Autoware perception パイプラインを起動する（既定 True。'
                        '3D LiDAR /lidar/points を入力に検出・追跡・可視化）'),
        DeclareLaunchArgument(
            'omni_perception', default_value='True',
            description='全天球カメラ連携（色付き点群・物体クロップ）を起動する'),
        DeclareLaunchArgument(
            'image_recognition', default_value='True',
            description=('画像認識（LiDAR検出物体のYOLO分類 + 全天球信号認識）を起動する。'
                         'YOLOはCPU負荷が高いが間引きありで既定ON。重いときは False')),
        DeclareLaunchArgument(
            'traffic_light_method', default_value='classic',
            description=('信号認識バックエンド: classic (HSV+円形度、 学習不要、 既定) または '
                         'yolo (YOLOv8、 traffic_light_weights 必須。 初期化失敗で FATAL 終了)')),
        DeclareLaunchArgument(
            'traffic_light_weights', default_value='yolov8n.pt',
            description=('traffic_light_method:=yolo のときに使う重み。 '
                         '相対パスは ultralytics デフォルト探索パスを使用')),
        DeclareLaunchArgument(
            'object_memory_delete_thresh', default_value='0.25',
            description=('object_memory の Bayes 削除しきい値 (既定 0.25=Dengler et al.)。 '
                         '巡回中に DB が空になる時は 0.05〜0.10 に下げて存続を許す '
                         '(memory feedback_recog_db_empty_issue 参照)')),
        DeclareLaunchArgument(
            'object_memory_miss_tp', default_value='0.2',
            description=('object_memory の miss 観測時の TP 確率 (既定 0.2)。 '
                         '上げると減衰が緩む')),
        DeclareLaunchArgument(
            'object_memory_miss_fp', default_value='0.6',
            description=('object_memory の miss 観測時の FP 確率 (既定 0.6)。 '
                         '下げると減衰が緩む')),
        DeclareLaunchArgument(
            'object_memory_visible_range', default_value='8.0',
            description=('object_memory の「見えるはず」 判定レンジ [m] (既定 8.0)。 '
                         '短くする (例 5.0) と遠い物体に対する negative observation を抑え、 '
                         '一度認識した物体が離れても忘却されにくくなる。 ただし完全静止での '
                         '蓄積が増える副作用あり')),
        DeclareLaunchArgument(
            'object_yolo_weights', default_value='yolov8s-seg.pt',
            description='object_classifier_node.py の YOLO weight。認識比較では yolov8m-seg.pt 等へ差し替え可能'),
        DeclareLaunchArgument(
            'object_yolo_imgsz', default_value='640',
            description='object_classifier_node.py の YOLO 推論画像サイズ。大きいほど小物に有利だが重い'),
        DeclareLaunchArgument(
            'object_yolo_conf', default_value='0.15',
            description='object_classifier_node.py の YOLO predict conf。既定 0.15'),
        DeclareLaunchArgument(
            'object_min_accept_conf', default_value='0.15',
            description='object_classifier_node.py の分類採用conf。既定 0.15'),
        DeclareLaunchArgument(
            'object_min_accept_conf_overrides', default_value='',
            description='クラス別 min_accept_conf。"class1=0.10,class2=0.30" 形式。'
                        '例: "refrigerator=0.10,fridge=0.10,dining table=0.30,table=0.30" で '
                        '冷蔵庫を取りやすく / ダイニングテーブルを厳しくして FP を減らす'),
        DeclareLaunchArgument(
            'object_crop_fovs_deg', default_value='',
            description='object_classifier_node.py の複数FOVクロップ（例: 75,55,40）。空なら crop_fov_deg のみ'),
        DeclareLaunchArgument(
            'object_crop_yaw_offsets_deg', default_value='',
            description='object_classifier_node.py のcrop中心yaw offset[deg]（例: -12,0,12）。空なら0のみ'),
        DeclareLaunchArgument(
            'object_crop_pitch_offsets_deg', default_value='',
            description='object_classifier_node.py のcrop中心pitch offset[deg]（例: -8,0,8）。空なら0のみ'),
        DeclareLaunchArgument(
            'object_crop_shape_center_height_fracs', default_value='',
            description='object_classifier_node.py のshape高さ方向crop中心。例: 0,0.5,0.75。空ならpose中心のみ'),
        DeclareLaunchArgument(
            'object_crop_shape_bbox_margins_deg', default_value='',
            description='object_classifier_node.py の3D bbox投影crop margin[deg]。例: 4,10,18'),
        DeclareLaunchArgument(
            'object_classifier_debug', default_value='False',
            description='True で /perception/object_classifier/debug に YOLO 候補の採否理由を出す'),
        DeclareLaunchArgument(
            'object_debug_crop_dir', default_value='',
            description='空でなければ object_classifier_node.py の raw crop と metadata.jsonl を保存する'),
        DeclareLaunchArgument(
            'object_debug_crop_max_per_track', default_value='3',
            description='object_classifier_node.py のdebug crop保存上限/track。-1で無制限'),
        DeclareLaunchArgument(
            'object_tracker_debug', default_value='False',
            description='True で /perception/object_tracker/debug に track publish/reject 理由を出す'),
        DeclareLaunchArgument(
            'object_tracker_min_hits', default_value='2',
            description='object_tracker_node.py の出力最小hit数。既定 2'),
        DeclareLaunchArgument(
            'object_tracker_wall_margin_moving_cells',
            default_value='6',
            description='object_tracker_node.py の移動track向け壁margin[cell]。既定 6'),
        DeclareLaunchArgument(
            'object_tracker_wall_margin_static_cells',
            default_value='22',
            description='object_tracker_node.py の静止track向け壁margin[cell]。既定 22'),
        DeclareLaunchArgument(
            'indoor_objects', default_value='False',
            description=('室内物体検出: map_roi_filter が高所（天井/壁上部）を除外しつつ'
                         '床付近の家具を占有セル上でも検出/識別する。室内 world で True')),
        DeclareLaunchArgument(
            'colored_slam', default_value='True',
            description='SLAM/odom座標に色付き点群を蓄積して /slam/colorized_points_map を出す'),
        DeclareLaunchArgument(
            'lidar_model', default_value='mid360',
            description='3D LiDAR model metadata: mid360 (default) or vlp16. '
                        'world ファイル自体のセンサ形状は world 引数で選ぶ'),
        DeclareLaunchArgument(
            'scan_min_height', default_value='0.1',
            description='pointcloud_to_laserscan の min_height[m]。既定は屋内向け値'),
        DeclareLaunchArgument(
            'scan_max_height', default_value='2.0',
            description='pointcloud_to_laserscan の max_height[m]。既定は屋内向け値'),
        DeclareLaunchArgument(
            'scan_angle_increment', default_value='0.0087',
            description='pointcloud_to_laserscan の角度分解能[rad]。既定は約0.5deg'),
        DeclareLaunchArgument(
            'scan_range_min', default_value='0.3',
            description='pointcloud_to_laserscan の range_min[m]'),
        DeclareLaunchArgument(
            'scan_range_max', default_value='40.0',
            description='pointcloud_to_laserscan の range_max[m]'),
        DeclareLaunchArgument(
            'scan_use_inf', default_value='True',
            description='pointcloud_to_laserscan の use_inf'),
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
            'colored_slam_max_range', default_value='7.0',
            description='色付き点群蓄積の LiDAR 最大距離[m]。屋内はブレ低減のため近距離だけ採る'),
        DeclareLaunchArgument(
            'colored_slam_min_z', default_value='-0.1',
            description='色付き点群蓄積の最小 z[m]（LiDAR 座標）。床下散乱を切る'),
        DeclareLaunchArgument(
            'colored_slam_max_z', default_value='2.0',
            description='色付き点群蓄積の最大 z[m]（LiDAR 座標）。天井外れを切る'),
        DeclareLaunchArgument(
            'stationary_only', default_value='False',
            description='色付き点群を静止時のみ蓄積する (iter4 実装)。 巡回中の '
                        'SLAM 2D 姿勢誤差で生じる壁ブレ低減の有力策'),
        DeclareLaunchArgument(
            'stationary_max_lin_velocity', default_value='0.05',
            description='静止判定の最大並進速度 [m/s]。 既定 0.05 m/s'),
        DeclareLaunchArgument(
            'stationary_max_ang_velocity', default_value='0.2',
            description='静止判定の最大回転速度 [rad/s]。 既定 0.2 rad/s'),
        DeclareLaunchArgument(
            'omni_calibration_json', default_value='',
            description='direct_visual_lidar_calibration の calib.json。空なら初期TFを使う'),
        DeclareLaunchArgument(
            'strict_omni_calibration_json', default_value='False',
            description='Trueなら calibration_json 読み込み失敗時に初期TFへ戻さず停止する'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Webots はシミュレーション時刻のため true 必須'),
        DeclareLaunchArgument(
            'nav_params_file', default_value='',
            description='Nav2 params 差し替え。空なら webots_ros2_turtlebot 標準'),
        DeclareLaunchArgument(
            'ros2_control_params_file', default_value='',
            description='ros2_control params 差し替え。空なら webots_ros2_turtlebot 標準'),
        DeclareLaunchArgument(
            'nav_start_delay_sec', default_value='0.0',
            description='Webots controller 接続後に Nav2 起動を遅らせる秒数。既定 0'),
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        omni_sensor_tf,
        pointcloud_to_laserscan,
        robot_control,
        fall_detector,
        perception,
        colorized_points,
        pointcloud_intensity,
        equirect_camera_info,
        omni_image_compress,
        object_crops,
        object_classifier,
        traffic_light_detector,
        traffic_light_localizer,
        traffic_light_marker,
        object_memory,
        colorized_mapper,
        rviz,
        # Webots 終了時に全ノードを落とす
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
