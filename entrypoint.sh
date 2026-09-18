#!/bin/bash
set -e

# 加载 ROS 2 系统环境
source /opt/ros/jazzy/setup.bash
# 加载工作空间编译产物环境
source /root/ros_ws/install/setup.bash

# 执行传入的命令
exec "$@"
