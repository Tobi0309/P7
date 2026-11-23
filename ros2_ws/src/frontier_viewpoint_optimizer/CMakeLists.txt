cmake_minimum_required(VERSION 3.16)
project(frontier_viewpoint_optimizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(frontier_segmentation REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(ros2top REQUIRED)
find_package(nlohmann_json REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OptimizedFrontier.msg"
  "msg/OptimizedFrontierArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(frontier_viewpoint_optimizer_node src/frontier_viewpoint_optimizer_node.cpp)
target_include_directories(frontier_viewpoint_optimizer_node PRIVATE include ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(frontier_viewpoint_optimizer_node rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs octomap_msgs frontier_segmentation ros2top)
target_link_libraries(frontier_viewpoint_optimizer_node Eigen3::Eigen ${OCTOMAP_LIBRARIES} nlohmann_json::nlohmann_json)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(frontier_viewpoint_optimizer_node "${cpp_typesupport_target}")

install(TARGETS frontier_viewpoint_optimizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ launch/ DESTINATION share/${PROJECT_NAME}/)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
