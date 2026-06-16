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
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf maps rviz models webots_worlds resource
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(predicted_costmap_layer)
ament_export_dependencies(nav2_costmap_2d nav_msgs pluginlib)

ament_package()
