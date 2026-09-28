# 基础系统：Ubuntu 24.04 LTS
FROM ubuntu:24.04


# 安装基础工具 + 添加 ROS 2 源
RUN apt update && apt install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && curl -sSL https://mirrors.ustc.edu.cn/ros2/ubuntu.asc | apt-key add - \
    && echo "deb https://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt update

# 安装 ROS 2 Jazzy + 项目依赖
RUN apt install -y --no-install-recommends \
    ros-jazzy-ros-base \
    build-essential \
    cmake \
    gdb \
    vim \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep（国内源）
RUN rosdep init || true && \
    sed -i 's/raw.githubusercontent.com/mirrors.ustc.edu.cn/g' /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update

# 创建工作空间
RUN mkdir -p /workspace/src
WORKDIR /workspace

# 自动 source 环境
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/install/setup.bash || true" >> /root/.bashrc

ENV LANG C.UTF-8
