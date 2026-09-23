cmake_minimum_required(VERSION 3.8)
project(rm_gimbal_description)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY
        urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
