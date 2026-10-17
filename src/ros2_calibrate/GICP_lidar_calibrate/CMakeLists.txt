cmake_minimum_required(VERSION 3.8)
project(lidar_calibrate)

# GICP needs optimized code, a debug build is too slow to converge in time.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# Static so the nodes do not need an extra runtime library path.
add_library(lidar_calibrate_common STATIC src/accumulator.cpp)
target_include_directories(lidar_calibrate_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_calibrate_common ${PCL_LIBRARIES})

add_executable(map_builder_node src/map_builder_node.cpp)
target_link_libraries(map_builder_node lidar_calibrate_common)
ament_target_dependencies(map_builder_node rclcpp sensor_msgs pcl_conversions)

add_executable(lidar_calibrate_node src/lidar_calibrate_node.cpp)
target_link_libraries(lidar_calibrate_node lidar_calibrate_common)
ament_target_dependencies(lidar_calibrate_node rclcpp sensor_msgs geometry_msgs tf2_ros pcl_conversions)

install(TARGETS map_builder_node lidar_calibrate_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "build" EXCLUDE
  PATTERN "install" EXCLUDE
  PATTERN "log" EXCLUDE)

ament_package()
