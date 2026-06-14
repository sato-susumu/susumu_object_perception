# Autoware の sensing/perception モジュールで 3D LiDAR の点群から物体を検出する
# パイプライン。HD 地図は使わず、点群ジオメトリのみで検出する。
#
#   /velodyne_points (PointCloud2, frame: velodyne_link)
#     └→ [Autoware] crop_box_filter   ROI クロップ        → cropped/pointcloud
#        └→ [Autoware] ground_filter  地面除去(Scan Ground) → no_ground/pointcloud
#           └→ [Autoware] euclidean_cluster クラスタ化      → /perception/detected_objects
#              └→ [自作Py] object_tracker_node  追跡        → /perception/tracked_objects
#                 └→ [自作Py] perception_marker_node 可視化 → /perception/markers
#
# Autoware の 3 モジュールは composable node なので 1 つの component_container に
# まとめてロードする（プロセス間コピーを避け、ゼロコピー intra-process 通信にできる）。
# 自作 Python ノードは通常の Node として別途起動する（rclpy はコンポーネント化しない）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg = get_package_share_directory('susumu_sim')
    cfg = os.path.join(pkg, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    input_pc = LaunchConfiguration('input_pointcloud')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_input = DeclareLaunchArgument(
        'input_pointcloud', default_value='/velodyne_points',
        description='検出パイプラインへの入力点群トピック')

    crop_box_param = os.path.join(cfg, 'autoware_crop_box.param.yaml')
    ground_param = os.path.join(cfg, 'autoware_ground_filter.param.yaml')
    cluster_param = os.path.join(cfg, 'autoware_euclidean_cluster.param.yaml')
    vehicle_param = os.path.join(cfg, 'autoware_vehicle_info.param.yaml')

    sim_time_param = {'use_sim_time': use_sim_time}

    # 0) pointcloud_to_autoware: Gazebo の PointXYZI を Autoware の PointXYZIRC へ変換。
    #    ground_filter は ring/channel を持つ Autoware 独自点群型を要求し、Gazebo の
    #    生 /velodyne_points (PointXYZI) では "layout not compatible ... Aborting" で
    #    止まる。この前処理で channel(ring) を付与して互換にする（ライブ起動で判明）。
    pc_convert = Node(
        package='susumu_sim',
        executable='pointcloud_to_autoware_node.py',
        name='pointcloud_to_autoware',
        output='screen',
        parameters=[sim_time_param, {
            'input_topic': input_pc,
            'output_topic': '/perception/points_autoware',
        }],
    )

    # 1) crop_box_filter: ROI クロップ（velodyne_link 基準）
    crop_box = ComposableNode(
        package='autoware_crop_box_filter',
        plugin='autoware::crop_box_filter::CropBoxFilterNode',
        name='crop_box_filter',
        remappings=[
            ('input', '/perception/points_autoware'),
            ('output', '/perception/cropped/pointcloud'),
        ],
        # frame 系はノードパラメータで個別指定（param file は範囲のみ）。
        # 入力点群は velodyne_link なので変換せず同フレームで処理する。
        parameters=[crop_box_param, sim_time_param, {
            'input_frame': 'velodyne_link',
            'output_frame': 'velodyne_link',
            'input_pointcloud_frame': 'velodyne_link',
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # 2) ground_filter: Scan Ground Filter による地面除去
    ground = ComposableNode(
        package='autoware_ground_filter',
        plugin='autoware::ground_filter::GroundFilterComponent',
        name='ground_filter',
        remappings=[
            ('input', '/perception/cropped/pointcloud'),
            ('output', '/perception/no_ground/pointcloud'),
        ],
        parameters=[ground_param, vehicle_param, sim_time_param],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # 3) euclidean_cluster: 非地面点をクラスタリング → DetectedObjects
    cluster = ComposableNode(
        package='autoware_euclidean_cluster_object_detector',
        plugin='autoware::euclidean_cluster::EuclideanClusterNode',
        name='euclidean_cluster',
        remappings=[
            ('input', '/perception/no_ground/pointcloud'),
            ('output', '/perception/detected_objects'),
        ],
        parameters=[cluster_param, sim_time_param],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='autoware_perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[crop_box, ground, cluster],
        output='screen',
    )

    # 3.5) 自作 Python: 2D 地図照合 ROI フィルタ
    #      DetectedObjects のうち、2D 占有格子地図で壁/地図外/未知に当たるものを除外し、
    #      地図内フリースペースの物体（＝動的に現れた人など）だけ通す。HD 地図の無い
    #      本環境で Autoware の map-based ROI フィルタを 2D 地図で代替する。
    #      map<-velodyne_link の TF（map->odom は AMCL/Nav2 提供）が要るため、Nav2 無し
    #      のときは TF 不在で素通しになる（perception は止めない設計）。
    map_filter = Node(
        package='susumu_sim',
        executable='map_roi_filter_node.py',
        name='map_roi_filter',
        output='screen',
        parameters=[sim_time_param, {
            # 壁セルの周囲 ±3 セル（地図 res 0.05m なら ±15cm）も占有扱いにし、
            # 壁に貼り付いて検出される静止クラスタ（緑ボックス）を落とす。人は壁から
            # 多少離れるので残る。消し過ぎるなら下げる。
            'wall_margin_cells': 3,
        }],
    )

    # 4) 自作 Python: 追跡（地図照合後の DetectedObjects → TrackedObjects）
    tracker = Node(
        package='susumu_sim',
        executable='object_tracker_node.py',
        name='object_tracker',
        output='screen',
        parameters=[sim_time_param, {
            'input_topic': '/perception/detected_objects_in_map',
        }],
    )

    # 5) 自作 Python: 可視化（Detected/Tracked → MarkerArray）
    #    検出マーカー(青)も地図照合後を見せる（生検出だと壁だらけになるため）。
    marker = Node(
        package='susumu_sim',
        executable='perception_marker_node.py',
        name='perception_marker',
        output='screen',
        parameters=[sim_time_param, {
            'detected_topic': '/perception/detected_objects_in_map',
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_input)
    ld.add_action(pc_convert)
    ld.add_action(container)
    ld.add_action(map_filter)
    ld.add_action(tracker)
    ld.add_action(marker)
    return ld
