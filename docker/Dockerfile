# Simulation layer (delivery_bot_sim#5): everything the Unity simulation needs to talk to the delivery_bot_core stack.
#
# The base is the delivery_bot_core development image (ROS 2 Jazzy ros-core + C++/Python/Rust toolchains, built from
# ros2_ws/src/delivery_bot_core/docker). compose.yaml builds it as the "core-dev" context. This layer only adds the
# Unity ROS-TCP-Endpoint and an entrypoint that builds the mounted workspace.
FROM core-dev

# Unity ROS-TCP-Endpoint, ROS 2 branch (main-ros2 = v0.7.0), pinned. Built into its own overlay so the
# workspace volumes mounted at /ws can't hide it.
# patches/ros_tcp_endpoint-reregister.patch: when Unity reconnects (next Play, domain reload) it registers its topics
# again. Upstream destroys the existing node before removing it from the executor, which on Jazzy races with the
# executor thread and kills the endpoint (rclpy InvalidHandle, then "Connection refused" in Unity). The patch keeps
# nodes whose topic is registered again with the same type and removes replaced nodes from the executor first.
ARG ROS_TCP_ENDPOINT_SHA=54c1a64b6d5ef6ffa0a0431570bb74329b79b15b
COPY patches/ /tmp/ros_tcp_endpoint_patches/
RUN mkdir -p /opt/ros_tcp_endpoint/src \
    && git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git /opt/ros_tcp_endpoint/src/ros_tcp_endpoint \
    && git -C /opt/ros_tcp_endpoint/src/ros_tcp_endpoint checkout --quiet "${ROS_TCP_ENDPOINT_SHA}" \
    && git -C /opt/ros_tcp_endpoint/src/ros_tcp_endpoint apply /tmp/ros_tcp_endpoint_patches/ros_tcp_endpoint-reregister.patch \
    && rm -rf /tmp/ros_tcp_endpoint_patches \
    && cd /opt/ros_tcp_endpoint \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --event-handlers console_cohesion+ \
    && rm -rf build log

COPY entrypoint.sh /usr/local/bin/sim_entrypoint.sh
COPY sim.launch.py /opt/delivery_bot_sim/sim.launch.py
RUN chmod +x /usr/local/bin/sim_entrypoint.sh \
    && check_forbidden_packages.sh

WORKDIR /ws
ENTRYPOINT ["sim_entrypoint.sh"]
CMD ["bash"]
