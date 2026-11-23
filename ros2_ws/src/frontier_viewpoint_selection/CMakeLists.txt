cmake_minimum_required(VERSION 3.16)
project(frontier_viewpoint_selection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimised by default (an empty CMAKE_BUILD_TYPE compiles without optimisation).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dsp_interfaces REQUIRED)
find_package(frontier_viewpoint_optimizer REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ros2top REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(frontier_viewpoint_selection_node src/frontier_viewpoint_selection_node.cpp)
target_include_directories(frontier_viewpoint_selection_node PRIVATE include)
ament_target_dependencies(frontier_viewpoint_selection_node rclcpp geometry_msgs nav_msgs std_msgs std_srvs dsp_interfaces frontier_viewpoint_optimizer ros2top)
target_link_libraries(frontier_viewpoint_selection_node Eigen3::Eigen nlohmann_json::nlohmann_json)

install(TARGETS frontier_viewpoint_selection_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()
