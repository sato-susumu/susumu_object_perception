# Autoware の sensing/perception モジュールで 3D LiDAR の点群から物体を検出する
# パイプライン。HD 地図は使わず、点群ジオメトリのみで検出する。
#
#   /lidar/points (PointCloud2, frame: LiDAR frame)
#     └→ [Autoware] crop_box_filter   ROI クロップ        → cropped/pointcloud
#        └→ [Autoware] ground_filter  地面除去(Scan Ground) → no_ground/pointcloud
#           └→ [Autoware] euclidean_cluster クラスタ化      → /perception/detected_objects
#              └→ [自作Py] shape_estimation_node OBB推定    → /perception/detected_objects_shaped
#                 （Autoware L字フィット bounding_box.cpp を踏襲、型は標準で自作）
#              └→ [自作Py] detection_by_tracker_node 過分割統合 → /perception/detected_objects_merged
#                 （Autoware Cluster Merger を踏襲、tracker 出力を参照する循環）
#              └→ [自作Py] map_roi_filter_node 壁除去       → /perception/detected_objects_in_map
#                 └→ [自作Py] object_tracker_node  追跡        → /perception/tracked_objects
#                    └→ [自作Py] prediction_node 将来軌跡予測 → /perception/predicted_objects
#                    └→ [自作Py] perception_marker_node 可視化 → /perception/markers
#
# Autoware の 3 モジュールは composable node なので 1 つの component_container に
# まとめてロードする（プロセス間コピーを避け、ゼロコピー intra-process 通信にできる）。
# 自作 Python ノードは通常の Node として別途起動する（rclpy はコンポーネント化しない）。
# 可視化は自作 MarkerArray ノード（表示方法・色を自由に作り込むため。純正プラグインは不使用）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('susumu_object_perception')
    cfg = os.path.join(pkg, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    input_pc = LaunchConfiguration('input_pointcloud')
    lidar_frame = LaunchConfiguration('lidar_frame')
    num_rings = LaunchConfiguration('num_rings')
    min_elev_deg = LaunchConfiguration('min_elev_deg')
    max_elev_deg = LaunchConfiguration('max_elev_deg')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_input = DeclareLaunchArgument(
        'input_pointcloud', default_value='/lidar/points',
        description='検出パイプラインへの入力点群トピック')
    declare_lidar_frame = DeclareLaunchArgument(
        'lidar_frame', default_value='lidar_link',
        description='入力点群の LiDAR frame')
    declare_num_rings = DeclareLaunchArgument(
        'num_rings', default_value='16',
        description='PointXYZIRC channel 近似に使う仮想ring数')
    declare_min_elev = DeclareLaunchArgument(
        'min_elev_deg', default_value='-15.0',
        description='PointXYZIRC channel 近似に使う最小仰角[deg]')
    declare_max_elev = DeclareLaunchArgument(
        'max_elev_deg', default_value='15.0',
        description='PointXYZIRC channel 近似に使う最大仰角[deg]')

    crop_box_param = os.path.join(cfg, 'autoware_crop_box.param.yaml')
    ground_param = os.path.join(cfg, 'autoware_ground_filter.param.yaml')
    cluster_param = os.path.join(cfg, 'autoware_euclidean_cluster.param.yaml')
    vehicle_param = os.path.join(cfg, 'autoware_vehicle_info.param.yaml')

    sim_time_param = {'use_sim_time': use_sim_time}

    # 0) pointcloud_to_autoware: Gazebo の PointXYZI を Autoware の PointXYZIRC へ変換。
    #    ground_filter は ring/channel を持つ Autoware 独自点群型を要求し、Gazebo の
    #    生 /lidar/points (PointXYZI) では "layout not compatible ... Aborting" で
    #    止まる。この前処理で channel(ring) を付与して互換にする（ライブ起動で判明）。
    pc_convert = Node(
        package='susumu_object_perception',
        executable='pointcloud_to_autoware_node.py',
        name='pointcloud_to_autoware',
        output='screen',
        parameters=[sim_time_param, {
            'input_topic': input_pc,
            'output_topic': '/perception/points_autoware',
            'num_rings': ParameterValue(num_rings, value_type=int),
            'min_elev_deg': ParameterValue(min_elev_deg, value_type=float),
            'max_elev_deg': ParameterValue(max_elev_deg, value_type=float),
        }],
    )

    # 1) crop_box_filter: ROI クロップ（lidar_link 基準）
    crop_box = ComposableNode(
        package='autoware_crop_box_filter',
        plugin='autoware::crop_box_filter::CropBoxFilterNode',
        name='crop_box_filter',
        remappings=[
            ('input', '/perception/points_autoware'),
            ('output', '/perception/cropped/pointcloud'),
        ],
        # frame 系はノードパラメータで個別指定（param file は範囲のみ）。
        # 入力点群は LiDAR frame なので変換せず同フレームで処理する。
        parameters=[crop_box_param, sim_time_param, {
            'input_frame': lidar_frame,
            'output_frame': lidar_frame,
            'input_pointcloud_frame': lidar_frame,
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

    # 3.3) 自作 Python: 形状推定（OBB）
    #      euclidean_cluster の DetectedObjects は位置のみで shape が空。no_ground 点群から
    #      各検出近傍の点を集め、Autoware の L字フィット（bounding_box.cpp を踏襲）で
    #      OBB の寸法・向きを推定して shape を埋める。apt 版 shape_estimation は無く、
    #      universe 版は型（tier4_perception_msgs）が世代不整合なため、アルゴリズムのみ
    #      公式踏襲して標準型で自作した。
    shape_est = Node(
        package='susumu_object_perception',
        executable='shape_estimation_node.py',
        name='shape_estimation',
        output='screen',
        parameters=[sim_time_param, {
            'input_objects': '/perception/detected_objects',
            'input_cloud': '/perception/no_ground/pointcloud',
            'output_objects': '/perception/detected_objects_shaped',
        }],
    )

    # 3.4) 自作 Python: detection_by_tracker（過分割統合）
    #      euclidean クラスタリングが 1 人を複数に割る over-segmentation を、前フレームの
    #      tracker の位置・サイズを参照して 1 つに統合する（Autoware Cluster Merger 踏襲）。
    #      tracker 出力を購読する循環構造。tracker 未起動/TF 不在時は素通し。
    det_by_trk = Node(
        package='susumu_object_perception',
        executable='detection_by_tracker_node.py',
        name='detection_by_tracker',
        output='screen',
        parameters=[sim_time_param, {
            'input_objects': '/perception/detected_objects_shaped',
            'input_tracks': '/perception/tracked_objects',
            'output_objects': '/perception/detected_objects_merged',
        }],
    )

    # 3.5) 自作 Python: 2D 地図照合 ROI フィルタ
    #      DetectedObjects のうち、2D 占有格子地図で壁/地図外/未知に当たるものを除外し、
    #      地図内フリースペースの物体（＝動的に現れた人など）だけ通す。HD 地図の無い
    #      本環境で Autoware の map-based ROI フィルタを 2D 地図で代替する。
    #      map<-lidar_link の TF（map->odom は AMCL/Nav2 提供）が要るため、Nav2 無し
    #      のときは TF 不在で素通しになる（perception は止めない設計）。
    #      入力は detection_by_tracker で過分割統合した検出。
    map_filter = Node(
        package='susumu_object_perception',
        executable='map_roi_filter_node.py',
        name='map_roi_filter',
        output='screen',
        parameters=[sim_time_param, {
            'input_topic': '/perception/detected_objects_merged',
            # 壁セルの周囲 ±3 セル（地図 res 0.05m なら ±15cm）も占有扱いにし、
            # 壁に貼り付いて検出される静止クラスタ（緑ボックス）を落とす。人は壁から
            # 多少離れるので残る。消し過ぎるなら下げる。
            'wall_margin_cells': 3,
        }],
    )

    # 4) 自作 Python: 追跡（地図照合後の DetectedObjects → TrackedObjects）
    tracker = Node(
        package='susumu_object_perception',
        executable='object_tracker_node.py',
        name='object_tracker',
        output='screen',
        parameters=[sim_time_param, {
            'input_topic': '/perception/detected_objects_in_map',
        }],
    )

    # 4.5) 自作 Python: 将来軌跡予測（2D map_based_prediction の 2D 占有格子版）
    #      tracked_objects を等速(CV)で予測しつつ、予測点が 2D 地図の occupied セルに
    #      入ったら打ち切る（壁めり込み予測を防ぐ）。Autoware の map_based_prediction の
    #      基本部（CV 予測）を踏襲し、HD 地図要素の代わりに 2D 占有格子を使う。
    prediction = Node(
        package='susumu_object_perception',
        executable='prediction_node.py',
        name='prediction',
        output='screen',
        parameters=[sim_time_param, {
            'input_topic': '/perception/tracked_objects',
        }],
    )

    # 5) 自作 Python: 可視化（Detected/Tracked/Predicted → MarkerArray）
    #    表示方法・色を自由に作り込むため自作する（純正プラグインは使わない）。
    #    検出マーカー(青)も地図照合後を見せる（生検出だと壁だらけになるため）。
    marker = Node(
        package='susumu_object_perception',
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
    ld.add_action(declare_lidar_frame)
    ld.add_action(declare_num_rings)
    ld.add_action(declare_min_elev)
    ld.add_action(declare_max_elev)
    ld.add_action(pc_convert)
    ld.add_action(container)
    ld.add_action(shape_est)
    ld.add_action(det_by_trk)
    ld.add_action(map_filter)
    ld.add_action(tracker)
    ld.add_action(prediction)
    ld.add_action(marker)
    return ld
