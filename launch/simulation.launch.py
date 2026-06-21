# 全部入りシミュレーション。
#
#   Gazebo（cafe world）+ HuNavSim 歩行者5人           （hunav_house.launch.py より）
#   + 3D-LiDAR TurtleBot3（waffle + Livox MID-360）    （spawn_robot.launch.py より）
#   + Nav2（AMCL 自己位置推定 + /scan obstacle_layer + predicted_layer）
#   + RViz2
#   + Teleop / 自動巡回 GUI
#
# RViz2 の「2D Goal Pose」でゴールを与えると、/scan の障害物層と予測層を使って
# 歩く人を避けながらカフェ world 内を自律移動する。GUI からは手動操縦・自動巡回もできる。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    susumu_pkg = get_package_share_directory('susumu_object_perception')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_perception = LaunchConfiguration('use_perception')
    use_image_recognition = LaunchConfiguration('image_recognition')
    use_semantic_memory = LaunchConfiguration('semantic_memory')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('gui')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    lidar_model = LaunchConfiguration('lidar_model')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_use_nav2 = DeclareLaunchArgument('use_nav2', default_value='True',
        description='Nav2 スタックを起動する')
    declare_use_perception = DeclareLaunchArgument('use_perception', default_value='True',
        description='Autoware sensing/perception パイプライン（物体検出・追跡・可視化）を起動する')
    declare_use_image_recognition = DeclareLaunchArgument(
        'image_recognition', default_value='True',
        description=('画像認識（6面カメラ→全天球合成 + LiDAR検出物体のYOLO分類 + 全天球信号認識）を起動する。'
                     'YOLOはCPU負荷が高いが間引きありで既定ON。重いときは False'))
    declare_use_semantic_memory = DeclareLaunchArgument(
        'semantic_memory', default_value='False',
        description=('セマンティック物体メモリ（検出物体を map 座標で永続記憶し、'
                     '無くなったら消し、RViz/GUI で一覧表示）を起動する。'
                     '既定 OFF。image_recognition:=True で分類済みトラックを使うのが望ましい'))
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
    declare_lidar_model = DeclareLaunchArgument(
        'lidar_model',
        default_value='mid360',
        description='3D LiDAR model: mid360 (default) or vlp16')

    lidar_points_topic = '/lidar/points'
    lidar_frame = 'lidar_link'
    lidar_num_rings = PythonExpression([
        "'16' if '", lidar_model, "' == 'vlp16' else '64'"
    ])
    lidar_min_elev = PythonExpression([
        "'-15.0' if '", lidar_model, "' == 'vlp16' else '-55.0'"
    ])
    lidar_max_elev = PythonExpression([
        "'15.0' if '", lidar_model, "' == 'vlp16' else '55.0'"
    ])

    # ------------------------------------------------------------------
    # 1) Gazebo cafe world + HuNavSim 歩行者5人。
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
            'lidar_model': lidar_model,
            'x_pose': x_pose, 'y_pose': y_pose, 'yaw': yaw,
        }.items())

    spawn_robot_delayed = TimerAction(period=15.0, actions=[spawn_robot])

    # 転倒検知（常時監視）。IMU の傾きで転倒を警告。robot spawn 後に起動する。
    fall_detector = Node(
        package='susumu_object_perception',
        executable='fall_detector_node.py',
        name='fall_detector',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'imu_topic': '/imu'}],
    )
    fall_detector_delayed = TimerAction(period=17.0, actions=[fall_detector])

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
    #      /lidar/points → crop_box → ground_filter → euclidean_cluster
    #      （ここまで Autoware 純正）→ object_tracker → perception_marker（自作）。
    #      追跡は odom←LiDAR frame の TF を使うため、robot spawn の後に起動する。
    #      Nav2 とは連携せず（生センサで動く Nav2 はそのまま）、検出結果は可視化のみ。
    # ------------------------------------------------------------------
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(susumu_pkg, 'launch', 'include', 'autoware_perception.launch.py')),
        condition=IfCondition(use_perception),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'input_pointcloud': lidar_points_topic,
            'lidar_frame': lidar_frame,
            'num_rings': lidar_num_rings,
            'min_elev_deg': lidar_min_elev,
            'max_elev_deg': lidar_max_elev,
        }.items())
    perception_delayed = TimerAction(period=18.0, actions=[
        LogInfo(msg='Starting Autoware perception pipeline...'), perception])

    # ------------------------------------------------------------------
    # 3.6) 画像認識（image_recognition:=True で起動）。Gazebo は 6 面カメラなので
    #      omni_image_node で全天球（/omni_camera/image_raw）に合成し、その画像で
    #      LiDAR 検出物体の YOLO 分類と全天球信号認識を行う。YOLO は重いが間引きあり。
    # ------------------------------------------------------------------
    omni_image = Node(
        package='susumu_object_perception', executable='omni_image_node.py',
        name='omni_image', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_image_recognition))
    object_classifier = Node(
        package='susumu_object_perception', executable='object_classifier_node.py',
        name='object_classifier', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_objects': '/perception/tracked_objects',
            'input_image': '/omni_camera/image_raw',  # Gazebo は合成後の生トピック
            'camera_frame': 'omni_camera_link',
        }],
        condition=IfCondition(use_image_recognition))
    traffic_light_detector = Node(
        package='susumu_object_perception', executable='traffic_light_detector_node.py',
        name='traffic_light_detector', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 'omni_mode': True,
            'input_image': '/omni_camera/image_raw',
        }],
        condition=IfCondition(use_image_recognition))
    traffic_light_localizer = Node(
        package='susumu_object_perception', executable='traffic_light_localizer_node.py',
        name='traffic_light_localizer', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'points_topic': lidar_points_topic}],
        condition=IfCondition(use_image_recognition))
    traffic_light_marker = Node(
        package='susumu_object_perception', executable='traffic_light_marker_node.py',
        name='traffic_light_marker', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 'omni_mode': True,
            'input_image': '/omni_camera/image_raw',
        }],
        condition=IfCondition(use_image_recognition))
    image_recognition_delayed = TimerAction(period=20.0, actions=[
        LogInfo(msg='Starting image recognition (omni + YOLO classify + traffic light)...'),
        omni_image, object_classifier, traffic_light_detector,
        traffic_light_localizer, traffic_light_marker])

    # ------------------------------------------------------------------
    # 3.7) セマンティック物体メモリ（semantic_memory:=True で起動、既定 OFF）。
    #      検出物体を map 座標で永続記憶し、RViz marker と SQLite DB に出す。
    #      image_recognition が ON なら YOLO 分類済み tracked_objects_classified を、
    #      OFF なら素の tracked_objects を入力に使う。
    # ------------------------------------------------------------------
    memory_input = PythonExpression([
        "'/perception/tracked_objects_classified' if '",
        use_image_recognition,
        "' == 'True' else '/perception/tracked_objects'"
    ])
    object_memory = Node(
        package='susumu_object_perception', executable='object_memory_node.py',
        name='object_memory', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': memory_input,
        }],
        condition=IfCondition(use_semantic_memory))
    semantic_memory_delayed = TimerAction(period=22.0, actions=[
        LogInfo(msg='Starting semantic object memory (memory only)...'),
        object_memory])

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
        package='susumu_object_perception', executable='teleop_gui_node.py',
        name='teleop_gui', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui))
    gui_delayed = TimerAction(period=24.0, actions=[gui_node])

    # 5.5) 記憶物体の一覧 GUI（gui:=True かつ semantic_memory:=True のとき）。
    #      object_memory の DB を読んで「どこに何があるか」を一覧表示し、行クリックで詳細を出す。
    memory_gui_cond = PythonExpression([
        "'", use_gui, "' == 'True' and '", use_semantic_memory, "' == 'True'"])
    memory_gui_node = Node(
        package='susumu_object_perception', executable='object_memory_gui_node.py',
        name='object_memory_gui', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(memory_gui_cond))
    memory_gui_delayed = TimerAction(period=25.0, actions=[memory_gui_node])

    ld = LaunchDescription()
    for a in (declare_use_sim_time, declare_use_nav2, declare_use_perception,
              declare_use_image_recognition, declare_use_semantic_memory,
              declare_use_rviz, declare_gui, declare_map, declare_params,
              declare_x, declare_y, declare_yaw, declare_lidar_model):
        ld.add_action(a)

    ld.add_action(hunav_world)
    ld.add_action(spawn_robot_delayed)
    ld.add_action(fall_detector_delayed)
    ld.add_action(perception_delayed)
    ld.add_action(image_recognition_delayed)
    ld.add_action(semantic_memory_delayed)
    ld.add_action(nav2_delayed)
    ld.add_action(rviz_delayed)
    ld.add_action(gui_delayed)
    ld.add_action(memory_gui_delayed)
    return ld
