FROM docker.m.daocloud.io/ros:jazzy-ros-base

USER root

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN set -eux; \
    mkdir -p /etc/ros/rosdep/sources.list.d; \
    curl -fsSL -o /etc/ros/rosdep/sources.list.d/20-default.list \
        https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list; \
    \
    export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml; \
    rosdep update

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update

# ARG GIT_REPOSITORY=https://github.com/MoYeMaster/dart_aim.git
# ARG GIT_REF=main

WORKDIR /ros_ws
# RUN git clone --depth=1 --branch "${GIT_REF}" "${GIT_REPOSITORY}" .

# RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
#     && export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml \
#     && rosdep update \
#     # && rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
# #     && . /opt/ros/jazzy/setup.sh \
# #     && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
#     && rm -rf /var/lib/apt/lists/*
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["ros2", "launch", "rm_vision_bringup", "dart_bringup.launch.py"]
