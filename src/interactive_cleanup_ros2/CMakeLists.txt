cmake_minimum_required(VERSION 3.8)
project(interactive_cleanup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(cleanup_vision_ros2 REQUIRED)
find_package(interactive_cleanup_msgs REQUIRED)

# Main controller node
add_executable(interactive_cleanup_sample src/interactive_cleanup_sample.cpp)
ament_target_dependencies(interactive_cleanup_sample
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  action_msgs
  cleanup_vision_ros2
  interactive_cleanup_msgs
)

install(TARGETS
  interactive_cleanup_sample
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
