cmake_minimum_required(VERSION 3.8)
project(detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/detector.cpp
  src/detector_node.cpp
  src/pnp.cpp
  src/ballistics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  auto_aim_interfaces
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rm_auto_aim::DetectorNode"
  EXECUTABLE detector_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_base_light_detector
    test/test_base_light_detector.cpp
  )
  target_include_directories(test_base_light_detector PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_base_light_detector
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
  )

  ament_add_gtest(test_detector_node
    test/test_detector_node.cpp
  )
  target_include_directories(test_detector_node PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_detector_node
    rclcpp
    sensor_msgs
    std_msgs
    image_transport
    auto_aim_interfaces
  )
  target_link_libraries(test_detector_node
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
  )

  ament_add_gtest(test_image_utils
    test/test_image_utils.cpp
  )
  target_include_directories(test_image_utils PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_image_utils
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
  )

  ament_add_gtest(test_pnp
    test/test_pnp.cpp
  )
  target_include_directories(test_pnp PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_pnp
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
  )

  ament_add_gtest(test_ballistics
    test/test_ballistics.cpp
  )
  target_include_directories(test_ballistics PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_ballistics
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
  )

  ament_add_gtest(test_dart_state
    test/test_dart_state.cpp
  )
  target_include_directories(test_dart_state PRIVATE
    include
  )
  target_link_libraries(test_dart_state
    ${PROJECT_NAME}
  )
endif()

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  auto_aim_interfaces
  geometry_msgs
)

ament_package()
