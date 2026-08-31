cmake_minimum_required(VERSION 3.8)
project(handyman_rebuild_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(handyman_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(handyman_core
  src/task_state_machine.cpp
)
target_include_directories(handyman_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(handyman_coordinator
  src/coordinator_node.cpp
)
target_link_libraries(handyman_coordinator handyman_core)
ament_target_dependencies(handyman_coordinator
  rclcpp
  handyman_msgs
  geometry_msgs
)

install(TARGETS
  handyman_core
  handyman_coordinator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(task_state_machine_test test/task_state_machine_test.cpp)
  target_link_libraries(task_state_machine_test handyman_core)
  target_include_directories(task_state_machine_test PRIVATE include)
endif()

ament_package()
