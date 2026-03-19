cmake_minimum_required(VERSION 3.8)
project(vision_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/object_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_detection_node
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
