cmake_minimum_required(VERSION 3.16)
project(combined_calibrate)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# cv_bridge and every other ROS package on this machine are built against the
# distribution-provided OpenCV.  Pin OpenCV/PCL to the system copies so the nodes do not
# end up mixing /usr/local's OpenCV 4.12 or PCL 1.15 headers with those libs.
if(EXISTS "/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
  set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
endif()
if(EXISTS "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake")
  set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d)
find_package(PCL REQUIRED COMPONENTS common io)

if(PCL_DEFINITIONS)
  add_definitions(${PCL_DEFINITIONS})
endif()

add_library(combined_calibrate_core
  src/calib_io.cpp
  src/calib_math.cpp
  src/gui_pick.cpp
  src/mini_yaml.cpp
  src/overlay.cpp
)
target_include_directories(combined_calibrate_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(combined_calibrate_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(combined_calibrate_core PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(calibrate_node src/calibrate_node.cpp)
ament_target_dependencies(calibrate_node rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge)
target_link_libraries(calibrate_node combined_calibrate_core)

add_executable(project_cloud src/project_cloud.cpp)
ament_target_dependencies(project_cloud rclcpp sensor_msgs cv_bridge)
target_link_libraries(project_cloud combined_calibrate_core)

add_executable(pick_corners src/pick_corners.cpp)
ament_target_dependencies(pick_corners rclcpp geometry_msgs)
target_link_libraries(pick_corners combined_calibrate_core)

add_executable(solve_extrinsic src/solve_extrinsic.cpp)
target_link_libraries(solve_extrinsic combined_calibrate_core)

install(TARGETS calibrate_node project_cloud pick_corners solve_extrinsic
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "build" EXCLUDE
  PATTERN "install" EXCLUDE
  PATTERN "log" EXCLUDE)

ament_package()
