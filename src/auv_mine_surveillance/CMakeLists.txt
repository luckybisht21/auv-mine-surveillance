cmake_minimum_required(VERSION 3.8)
project(auv_mine_surveillance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  nodes/auv_localization_node.py
  nodes/auv_mission_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch worlds urdf config maps meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
