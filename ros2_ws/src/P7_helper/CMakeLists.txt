cmake_minimum_required(VERSION 3.8)
project(P7_helper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Goal (e.g. RViz) -> ego-planner (/move_base_simple/goal), z = goal_height or current altitude
add_executable(goal_converter_ego src/goal_converter_ego.cpp)
ament_target_dependencies(goal_converter_ego
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS
  goal_converter_ego
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
