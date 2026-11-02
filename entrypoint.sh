#!/bin/bash
set -e

# 加载 ROS 2 系统环境
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# 加载工作空间编译产物环境
source /workspace/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# 执行传入的命令
exec "$@"
