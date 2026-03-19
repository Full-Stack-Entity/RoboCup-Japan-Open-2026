cmake_minimum_required(VERSION 3.8)
project(human_nav_llm_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RewriteGuidance.srv"
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/rewrite_guidance_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rewrite_guidance_node
)

ament_package()
