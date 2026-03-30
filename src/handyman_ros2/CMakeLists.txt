cmake_minimum_required(VERSION 3.8)
project(handyman_ros2)

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
find_package(handyman_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

# ---- 基础辅助库（room_navigation + placement_navigation，已验证可编译）----
add_library(handyman_nav_lib STATIC
  src/room_navigation.cpp
  src/placement_navigation.cpp
)
ament_target_dependencies(handyman_nav_lib
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  handyman_msgs
)
target_include_directories(handyman_nav_lib PUBLIC include)

# ---- LoadMapManager 库（提取自 handyman_sample.cpp）----
add_library(handyman_map_lib STATIC
  src/load_map_manager.cpp
)
ament_target_dependencies(handyman_map_lib
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  action_msgs
  ament_index_cpp
)
target_include_directories(handyman_map_lib PUBLIC include)

add_executable(teleop_key_handyman src/teleop_key_handyman.cpp)
ament_target_dependencies(teleop_key_handyman
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  handyman_msgs
)

# ---- task_orchestrator 可执行文件（模块化状态机入口）----
add_executable(task_orchestrator
  src/task_orchestrator_main.cpp
)
target_link_libraries(task_orchestrator
  handyman_modules
  handyman_nav_lib
  handyman_map_lib
)
ament_target_dependencies(task_orchestrator
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
  handyman_msgs
  ament_index_cpp
)

# ---- 以下为重构模块（task_orchestrator.cpp 已启用）----
set(MODULE_SOURCES
  src/protocol_handler.cpp
  src/map_nav_controller.cpp
  src/vision_manipulation_controller.cpp
  src/task_orchestrator.cpp  # 已修复，可启用
)

add_library(handyman_modules STATIC ${MODULE_SOURCES})
target_link_libraries(handyman_modules
  handyman_nav_lib
  handyman_map_lib
)
ament_target_dependencies(handyman_modules
  rclcpp rclcpp_action std_msgs std_srvs geometry_msgs
  trajectory_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
  nav2_msgs action_msgs handyman_msgs ament_index_cpp
)
target_include_directories(handyman_modules PUBLIC include)

install(TARGETS
  task_orchestrator
  teleop_key_handyman
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(room_navigation_test
    test/room_navigation_test.cpp
    src/room_navigation.cpp
  )
  target_include_directories(room_navigation_test PUBLIC include)

  ament_add_gtest(placement_navigation_test
    test/placement_navigation_test.cpp
  )
  target_include_directories(placement_navigation_test PRIVATE include)
  target_link_libraries(placement_navigation_test
    handyman_nav_lib
  )
endif()

ament_package()
