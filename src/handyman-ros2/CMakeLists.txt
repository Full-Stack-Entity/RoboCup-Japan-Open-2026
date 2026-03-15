cmake_minimum_required(VERSION 3.8)
project(handyman)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandymanMsg.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# handyman_sample executable
add_executable(handyman_sample src/handyman_sample.cpp)
ament_target_dependencies(handyman_sample
  rclcpp
  rclcpp_action
  ament_index_cpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  action_msgs
  std_srvs
)
target_link_libraries(handyman_sample "${cpp_typesupport_target}")

# teleop_key_handyman executable
add_executable(teleop_key_handyman src/teleop_key_handyman.cpp)
ament_target_dependencies(teleop_key_handyman
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(teleop_key_handyman "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  handyman_sample
  teleop_key_handyman
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install param files
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install msg
install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
