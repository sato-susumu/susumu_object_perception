cmake_minimum_required(VERSION 3.8)
project(susumu_object_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 予測コストマップ層（C++ costmap_2d プラグイン）の依存
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
# LCAS livox MID-360 plugin（ODE MultiRayShape 方式）の依存
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Python モジュール（susumu_object_perception/*.py）
ament_python_install_package(${PROJECT_NAME})

# === 予測コストマップ層プラグイン（perception 連携、costmap_2d::Layer）===
# prediction の予測 OccupancyGrid を max 合成で costmap に焼く自作 C++ 層。
# 標準層（ObstacleLayer/STVL=蓄積、StaticLayer=他層上書き）では実現できなかった
# 「他層を壊さず・蓄積せず」を満たす。
add_library(predicted_costmap_layer SHARED
  src/predicted_costmap_layer.cpp)
target_include_directories(predicted_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(predicted_costmap_layer
  rclcpp
  nav2_costmap_2d
  nav_msgs
  pluginlib
  geometry_msgs)

# pluginlib にプラグインを登録（nav2_costmap_2d 名前空間に export）。
pluginlib_export_plugin_description_file(nav2_costmap_2d predicted_costmap_layer.xml)

install(TARGETS predicted_costmap_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# === Gazebo Classic MID-360 LiDAR sensor plugin (LCAS / ODE MultiRayShape) ===
# LCAS/livox_laser_simulation_ros2 由来。ODE MultiRayShape で CSV の各角度に
# 実 ray を撃つため、自作 mid360_livox_sensor の「標準 RaySensor 格子に最近傍」
# 方式で出ていた上向きの自己遮蔽・幾何歪みが原理的に起きない。こちらを標準にする。
# 由来とライセンスは config/mid360_scan_patterns/LCAS_LIVOX_LICENSE 参照。
add_library(livox_mid360_sensor SHARED
  src/livox_ros2/livox_points_plugin.cpp
  src/livox_ros2/livox_ode_multiray_shape.cpp)
target_include_directories(livox_mid360_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/livox_ros2>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS})
target_link_directories(livox_mid360_sensor PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(livox_mid360_sensor
  ${GAZEBO_LIBRARIES}
  RayPlugin
  GpuRayPlugin
  # 元 CMake は libprotobuf.so.9 / libboost_chrono.so.1.71.0 を直指定していたが
  # Ubuntu 22.04 には存在しない。開発版シンボリックリンクで実体(.so.23 / 1.74)へ解決させる。
  protobuf
  boost_chrono)
ament_target_dependencies(livox_mid360_sensor
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_dev
  gazebo_ros
  tf2_ros
  ament_index_cpp
  livox_ros_driver2)

install(TARGETS livox_mid360_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

# Teleop / 自動巡回 GUI ノード、Autoware perception の自作補完ノード
install(PROGRAMS
  susumu_object_perception/teleop_gui_node.py
  susumu_object_perception/pointcloud_to_autoware_node.py
  susumu_object_perception/shape_estimation_node.py
  susumu_object_perception/detection_by_tracker_node.py
  susumu_object_perception/map_roi_filter_node.py
  susumu_object_perception/object_tracker_node.py
  susumu_object_perception/prediction_node.py
  susumu_object_perception/perception_marker_node.py
  susumu_object_perception/traffic_light_detector_node.py
  susumu_object_perception/traffic_light_marker_node.py
  susumu_object_perception/traffic_light_localizer_node.py
  susumu_object_perception/omni_image_node.py
  susumu_object_perception/colorized_pointcloud_node.py
  susumu_object_perception/object_image_crop_node.py
  susumu_object_perception/object_classifier_node.py
  susumu_object_perception/object_memory_node.py
  susumu_object_perception/semantic_query_node.py
  susumu_object_perception/object_seeker_node.py
  susumu_object_perception/auto_patrol_node.py
  susumu_object_perception/frontier_explore_node.py
  susumu_object_perception/waypoint_viz_node.py
  susumu_object_perception/waypoint_nav_node.py
  susumu_object_perception/fall_detector_node.py
  susumu_object_perception/collision_diagnostic_node.py
  susumu_object_perception/object_memory_gui_node.py
  susumu_object_perception/equirect_camera_info_node.py
  susumu_object_perception/pointcloud_intensity_node.py
  susumu_object_perception/omni_image_compress_node.py
  susumu_object_perception/omni_sensor_tf_node.py
  susumu_object_perception/colorized_pointcloud_mapper_node.py
  susumu_object_perception/pose_stamped_tf_bridge_node.py
  scripts/validate_omni_colorization.py
  scripts/record_omni_calibration_bag.sh
  scripts/run_direct_visual_lidar_calibration_docker.sh
  scripts/direct_calib_to_tf.py
  scripts/validate_colorized_pointcloud_quality.py
  scripts/test_object_seeker.py
  scripts/generate_waypoints.py
  scripts/generate_outdoor_waypoints.py
  scripts/expand_waypoint_route.py
  scripts/glim_cloud_to_2d_map.py
  scripts/evaluate_glim_map_variants.py
  scripts/nav2_pose_costmap_monitor_node.py
  scripts/save_pointcloud2_to_ply.py
  scripts/save_pose_trajectory_to_tum.py
  scripts/generate_webots_ground_truth_map.py
  scripts/map_progress_monitor.py
  scripts/eval_map_quality.py
  scripts/check_map_vs_world.py
  scripts/evaluate_gps_vs_slam.py
  scripts/live_slam_truth_monitor.py
  scripts/outdoor_gps_localization_node.py
  scripts/outdoor_nav2_waypoint_nav_node.py
  scripts/outdoor_gps_waypoint_nav_node.py
  scripts/fence_probe_sensor_check_node.py
  scripts/generate_fence_probe_waypoints.py
  scripts/import_verifai_outdoor_worlds.py
  scripts/render_recognition_overlay.py
  scripts/filter_object_memory_db.py
  scripts/evaluate_recognition_vs_world.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf maps rviz models webots_worlds resource behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(predicted_costmap_layer livox_mid360_sensor)
ament_export_dependencies(nav2_costmap_2d nav_msgs pluginlib sensor_msgs gazebo_dev gazebo_ros)

ament_package()
