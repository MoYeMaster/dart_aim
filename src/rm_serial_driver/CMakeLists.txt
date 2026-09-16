cmake_minimum_required(VERSION 3.16)
project(rm_serial_driver)

## optimize while compiling
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.

add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# Comment or uncomment to control whether the file is compiled.

set(EVENT_HANDLER_SOURCES
    src/event_handler/roll_handler.cpp
)

set(SERIAL_WRITER_SOURCES
    src/serial_writer/dyaw_writer.cpp
)

set(BASE_SOURCES
    src/crc.cpp
    src/rm_serial_driver_node.cpp
    src/virtual_serial.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
    ${BASE_SOURCES}
    ${EVENT_HANDLER_SOURCES}
    ${SERIAL_WRITER_SOURCES}
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN rm_serial_driver::SerialDriverNode
    EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN rm_serial_driver::VirtualSerialNode
    EXECUTABLE virtual_serial_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
    INSTALL_TO_SHARE
    # launch
    # config
)
