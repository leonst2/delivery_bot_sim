#!/usr/bin/env bash
# Sources ROS 2 and the endpoint overlay, builds the mounted workspace incrementally, then runs the command.
set -e

source /opt/ros/jazzy/setup.bash
source /opt/ros_tcp_endpoint/install/setup.bash

if [ "${SKIP_BUILD:-0}" != "1" ] && [ -n "$(find /ws/src -name package.xml -print -quit 2>/dev/null)" ]; then
  # No --symlink-install: /ws/src is mounted read-only, and Python develop installs would write into it.
  (cd /ws && colcon build --event-handlers console_cohesion+ ${COLCON_BUILD_ARGS:-})
fi
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

exec "$@"
