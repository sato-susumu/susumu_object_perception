cmake_minimum_required(VERSION 3.8)
project(susumu_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python module (susumu_sim/*.py)
ament_python_install_package(${PROJECT_NAME})

# Python node executables
install(PROGRAMS
  susumu_sim/person_detector_node.py
  susumu_sim/follow_person_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf maps rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
