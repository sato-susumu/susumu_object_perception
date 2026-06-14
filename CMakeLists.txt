cmake_minimum_required(VERSION 3.8)
project(susumu_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python モジュール（susumu_sim/*.py）
ament_python_install_package(${PROJECT_NAME})

# Teleop / 自動巡回 GUI ノード、Autoware perception の自作補完ノード
install(PROGRAMS
  susumu_sim/teleop_gui_node.py
  susumu_sim/pointcloud_to_autoware_node.py
  susumu_sim/map_roi_filter_node.py
  susumu_sim/object_tracker_node.py
  susumu_sim/perception_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf maps rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
