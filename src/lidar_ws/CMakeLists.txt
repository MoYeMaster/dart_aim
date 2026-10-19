cmake_minimum_required(VERSION 3.8)
project(lidar_dart_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration segmentation io)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/coordinate_transform.cpp
  src/preprocess.cpp
  src/feature_extract.cpp
  src/map_icp.cpp
  src/target_solver.cpp
  src/dart_bridge.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
)

add_executable(lidar_dart_node src/lidar_dart_node.cpp)
target_include_directories(lidar_dart_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_compile_definitions(lidar_dart_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(lidar_dart_node ${PROJECT_NAME} ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(lidar_dart_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS ${PROJECT_NAME} lidar_dart_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs std_msgs pcl_conversions Eigen3 PCL)
ament_package()
