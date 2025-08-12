FROM ros:jazzy-ros-core

## upgrade the base image
RUN apt update && apt upgrade -y \
  && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
    build-essential \
    python3-colcon-cmake \
    python3-colcon-common-extensions \
    python3-colcon-ros \
    python3-rosdep \
    python3-vcstool \
    git \
    ros-$ROS_DISTRO-rmw-zenoh-cpp \
    ros-$ROS_DISTRO-generate-parameter-library-py \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-control-cmake \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/workshop_ws
# set working directory to start in
WORKDIR /root/workshop_ws

# set environment variables
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
# For now we are using the default ROS 2 Jazzy Zenoh router config but we may want to change some option and specify our ouwn config
# (for exampel if we use serial to talk to the ESP32)
# ENV ZENOH_SESSION_CONFIG_URI=~/.ros/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
# Set up the environment for an interactive terminal.
RUN echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{name} {time}]: {message}"' >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "alias sws='source ./install/setup.bash'" >> /root/.bashrc && \
    echo "alias symlink_build='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --event-handlers log-'" >> /root/.bashrc

# Colorful terminal
ENV TERM=xterm-color
