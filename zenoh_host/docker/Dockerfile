FROM ros:jazzy-ros-core

## Change to testing binaries for the Zenoh 0.2.6 version as it support serial transport
# setup keys
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-testing-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-testing-archive-keyring.gpg ] http://packages.ros.org/ros2-testing/ubuntu noble main" > /etc/apt/sources.list.d/ros2-testing.list

## upgrade the base image
RUN apt update && apt upgrade -y \
  && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
    build-essential \
    git \
    git-lfs \
    python3-colcon-cmake \
    python3-colcon-common-extensions \
    python3-colcon-ros \
    python3-rosdep \
    python3-vcstool \
    ros-$ROS_DISTRO-generate-parameter-library-py \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-plotjuggler \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-rmw-zenoh-cpp \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-control-cmake \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    vim \
    && rm -rf /var/lib/apt/lists/*

# set environment variables
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

## ROS WORKSPACE SETUP
RUN mkdir -p /root/workshop_ws/src
# set working directory and copy in the source packages
WORKDIR /root/workshop_ws/src
COPY ./wbot_description ./wbot_description
COPY ./wbot_bringup ./wbot_bringup
# clone some additional libraries to help with the demo
RUN git clone https://github.com/ros-controls/topic_based_hardware_interfaces.git -b 0.2.0
# piper_ros2 uses lfs for mesh storage
RUN git lfs install
RUN git clone https://github.com/MarqRazz/piper_ros2.git

# pull dependencies
RUN apt update && apt install -y python3-rosdep && \
  rosdep init && \
  apt update && \
  rosdep update --rosdistro=$ROS_DISTRO && \
  rosdep install --rosdistro=$ROS_DISTRO --ignore-src --from-path /root/workshop_ws/src -y && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /root/workshop_ws

## install and build everything in the underlay
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install"

# For now we are using the default ROS 2 Jazzy Zenoh router config but we may want to change some option and specify our own config
# (for example if we use serial to talk to the ESP32)
# ENV ZENOH_ROUTER_CONFIG_URI=~/.ros/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
# Set up the environment for an interactive terminal.
RUN echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{name} {time}]: {message}"' >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/workshop_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias sws='source ./install/setup.bash'" >> /root/.bashrc && \
    echo "alias rmw_zenohdhd='ros2 run rmw_zenoh_cpp rmw_zenohdhd'" >> /root/.bashrc && \
    echo "alias symlink_build='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --event-handlers log-'" >> /root/.bashrc

# Colorful terminal
ENV TERM=xterm-color
