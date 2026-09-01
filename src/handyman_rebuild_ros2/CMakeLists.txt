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
  src/competition_protocol.cpp
  src/task_state_machine.cpp
)
ament_target_dependencies(handyman_core
  handyman_msgs
)
target_include_directories(handyman_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(handyman_nodes
  src/coordinator_node.cpp
  src/simulated_moderator_node.cpp
)
target_link_libraries(handyman_nodes handyman_core)
ament_target_dependencies(handyman_nodes
  rclcpp
  handyman_msgs
  geometry_msgs
)
target_include_directories(handyman_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(handyman_coordinator src/coordinator_main.cpp)
target_link_libraries(handyman_coordinator handyman_nodes)
ament_target_dependencies(handyman_coordinator rclcpp)

add_executable(handyman_simulated_moderator
  src/simulated_moderator_main.cpp
)
target_link_libraries(handyman_simulated_moderator handyman_nodes)
ament_target_dependencies(handyman_simulated_moderator rclcpp)

add_executable(handyman_phase1_simulation
  src/phase1_simulation_main.cpp
)
target_link_libraries(handyman_phase1_simulation handyman_nodes)
ament_target_dependencies(handyman_phase1_simulation rclcpp)

install(TARGETS
  handyman_core
  handyman_nodes
  handyman_coordinator
  handyman_simulated_moderator
  handyman_phase1_simulation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(task_state_machine_test test/task_state_machine_test.cpp)
  target_link_libraries(task_state_machine_test handyman_core)
  target_include_directories(task_state_machine_test PRIVATE include)

  ament_add_gtest(competition_protocol_test test/competition_protocol_test.cpp)
  target_link_libraries(competition_protocol_test handyman_core)
  target_include_directories(competition_protocol_test PRIVATE include)
  ament_target_dependencies(competition_protocol_test handyman_msgs)

  add_launch_test(
    test/protocol_roundtrip_launch_test.py
    TIMEOUT 30
  )
endif()

ament_package()
