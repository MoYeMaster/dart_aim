cmake_minimum_required(VERSION 3.8)
project(tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(tracker_node src/tracker_node.cpp src/kalman_filter.cpp)
target_include_directories(tracker_node PRIVATE include)
ament_target_dependencies(tracker_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  auto_aim_interfaces
  std_msgs
  geometry_msgs
  message_filters
  Eigen3
)
install(TARGETS tracker_node DESTINATION lib/${PROJECT_NAME})

ament_package()
