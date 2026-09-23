cmake_minimum_required(VERSION 3.8)
project(rm_vision_bringup)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
