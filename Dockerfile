FROM orsf/ros:jazzy-desktop-full

RUN sed -i "s@http://archive.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list && \
    sed -i "s@http://security.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list && \
    apt update

RUN apt install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    sed -i "s@https://raw.githubusercontent.com/ros/rosdistro/master/@https://mirrors.tuna.tsinghua.edu.cn/rosdistro/@g" /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update

WORKDIR /root/ros_ws

COPY src/ src/
RUN rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]