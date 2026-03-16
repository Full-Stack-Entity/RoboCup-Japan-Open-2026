cmake_minimum_required(VERSION 3.8)
project(cleanup_vision_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/PointingDirection.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(PROGRAMS
  scripts/cleanup_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleanup_detection_node
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
