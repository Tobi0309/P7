cmake_minimum_required(VERSION 3.8)
project(P7_helper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dsp_interfaces REQUIRED)

# Goal (e.g. RViz) -> ego-planner (/move_base_simple/goal), z = current altitude
add_executable(goal_converter_ego src/goal_converter_ego.cpp)
ament_target_dependencies(goal_converter_ego
  rclcpp
  geometry_msgs
  nav_msgs
)

# Goal (e.g. RViz) -> DSP (dsp/set_goal service), z = goal_height
add_executable(goal_converter_dsp src/goal_converter_dsp.cpp)
ament_target_dependencies(goal_converter_dsp
  rclcpp
  geometry_msgs
  dsp_interfaces
)

install(TARGETS
  goal_converter_ego
  goal_converter_dsp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
