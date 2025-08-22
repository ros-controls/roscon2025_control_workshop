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
    ros-$ROS_DISTRO-plotjuggler \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-control-cmake \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Replace default zenohd with newer version to fix keepalive issues on picoros.
# If download doesn't work, use a different mirror from
# https://www.eclipse.org/downloads/download.php?file=/zenoh/zenoh/1.5.0/zenoh-1.5.0-x86_64-unknown-linux-gnu-standalone.zip
WORKDIR /opt/ros/jazzy/lib/rmw_zenoh_cpp/
ADD https://ftp.snt.utwente.nl/pub/software/eclipse/zenoh/zenoh/1.5.0/zenoh-1.5.0-x86_64-unknown-linux-gnu-standalone.zip .
RUN mv rmw_zenohd rmw_zenohd.original
RUN unzip *.zip
RUN mv zenohd rmw_zenohd
# set environment variables
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

## ROS WORKSPACE SETUP
RUN mkdir -p /root/workshop_ws/src
# set working directory and copy in the source packages
WORKDIR /root/workshop_ws/src
COPY ./wbot_description ./wbot_description
COPY ./wbot_bringup ./wbot_bringup
# RUN git clone https://github.com/ros-controls/topic_based_hardware_interfaces
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

# For now we are using the default ROS 2 Jazzy Zenoh router config but we may want to change some option and specify our ouwn config
# (for example if we use serial to talk to the ESP32)
# ENV ZENOH_ROUTER_CONFIG_URI=~/.ros/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
# Set up the environment for an interactive terminal.
RUN echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{name} {time}]: {message}"' >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/workshop_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias sws='source ./install/setup.bash'" >> /root/.bashrc && \
    echo "alias symlink_build='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --event-handlers log-'" >> /root/.bashrc

# Colorful terminal
ENV TERM=xterm-color
