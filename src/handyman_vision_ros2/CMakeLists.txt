cmake_minimum_required(VERSION 3.8)
project(handyman_vision_ros2)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/object_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_detection_node
)

install(FILES
  scripts/last.pt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
