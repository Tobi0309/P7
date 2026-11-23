cmake_minimum_required(VERSION 3.5)
project(dsp)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dsp_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(DSL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_path(ROS2TOP_INCLUDE_DIR ros2top/ros2top.hpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIR}
)
link_libraries(${OCTOMAP_LIBRARIES})

add_executable(dsp src/dsp.cpp)

ament_target_dependencies(dsp
    rclcpp
    dsp_interfaces
    visualization_msgs
    std_srvs
    octomap
    octomap_msgs
    Eigen3
    DSL
    tf2
    tf2_ros
    tf2_geometry_msgs)
target_include_directories(dsp PRIVATE ${ROS2TOP_INCLUDE_DIR})
target_link_libraries(dsp nlohmann_json::nlohmann_json)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3 octomap)
ament_export_libraries(DSL)

install(TARGETS
    dsp
    #${Eigen_LIBRARIES}
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY config launch
    maps
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Lazy DSL search (DspGridSearch) and the voxel helpers used by the planner.
  ament_add_gtest(test_dsp_search test/test_dsp_search.cpp)
  if(TARGET test_dsp_search)
    ament_target_dependencies(test_dsp_search Eigen3 DSL)
  endif()
endif()

ament_package()
