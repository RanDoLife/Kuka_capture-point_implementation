# Готовый официальный образ ROS Noetic
FROM ros:noetic-ros-core-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/root/ros1_ws

# Минимальные системные зависимости
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-pip \
    python3-rosdep \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# RViz (отдельно, небольшой пакет)
RUN apt-get update && apt-get install -y ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# rosdep 
RUN rosdep init || true
RUN rosdep update

# Workspace
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Автосорсинг
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc
RUN echo "if [ -f ${ROS_WS}/devel/setup.bash ]; then source ${ROS_WS}/devel/setup.bash; fi" >> /etc/bash.bashrc

CMD ["bash"]
