#!/usr/bin/env bash
# Fails if the ROS environment contains packages the self-programmed stack must not use (issue #5).
set -eo pipefail

# ROS setup files use unset variables, so -u is enabled only after sourcing them.
source /opt/ros/jazzy/setup.bash
if [ -f /opt/ros_tcp_endpoint/install/setup.bash ]; then source /opt/ros_tcp_endpoint/install/setup.bash; fi
set -u

forbidden='^(nav2_.*|navigation2|slam_toolbox|amcl|.*map_server|robot_state_publisher|kdl_parser)$'
packages=$(ros2 pkg list)
if [ -z "$packages" ]; then
  echo "ros2 pkg list returned nothing; can't check for forbidden packages." >&2
  exit 1
fi
found=$(grep -E "$forbidden" <<< "$packages" || true)

if [ -n "$found" ]; then
  echo "Forbidden packages present in the image:" >&2
  echo "$found" >&2
  exit 1
fi
echo "No forbidden packages (Nav2, slam_toolbox, AMCL, map_server, robot_state_publisher, kdl_parser)."
