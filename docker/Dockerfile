# ROS 2 Jazzy environment for the self-programmed delivery_bot stack (issue #5).
#
# Based on ros-core on purpose: the ros-base image ships robot_state_publisher and kdl_parser, which the
# self-programmed stack must not use. Dependencies are installed here instead of by rosdep at runtime, so a
# package.xml can't pull a forbidden package back in.
FROM ros:jazzy-ros-core

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        netcat-openbsd \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pytest \
        python3-rosdep \
        libeigen3-dev \
        libtinyxml2-dev \
        ros-jazzy-ament-cmake-gtest \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ros2cli-common-extensions \
        ros-jazzy-rosbag2 \
        ros-jazzy-tf2-msgs \
    && rm -rf /var/lib/apt/lists/*

# Rust toolchain (rclrs, colcon-cargo, colcon-ros-cargo): added once the spike in delivery_bot_core#2 settles it.

# Unity ROS-TCP-Endpoint, ROS 2 branch (main-ros2 = v0.7.0), pinned. Built into its own overlay so the
# workspace volumes mounted at /ws can't hide it.
ARG ROS_TCP_ENDPOINT_SHA=54c1a64b6d5ef6ffa0a0431570bb74329b79b15b
RUN mkdir -p /opt/ros_tcp_endpoint/src \
    && git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git /opt/ros_tcp_endpoint/src/ros_tcp_endpoint \
    && git -C /opt/ros_tcp_endpoint/src/ros_tcp_endpoint checkout --quiet "${ROS_TCP_ENDPOINT_SHA}" \
    && cd /opt/ros_tcp_endpoint \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --event-handlers console_cohesion+ \
    && rm -rf build log

COPY check_forbidden_packages.sh entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/check_forbidden_packages.sh /usr/local/bin/entrypoint.sh \
    && check_forbidden_packages.sh

WORKDIR /ws
ENTRYPOINT ["entrypoint.sh"]
CMD ["bash"]
