#!/usr/bin/env bash
# Builds and tests the pinned delivery_bot_core submodule in a ros:jazzy container.
# Stand-in until the project's own ROS 2 image (issue #5) exists; that image builds the same workspace.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ ! -f ros2_ws/src/delivery_bot_core/README.md ]; then
  echo "Submodule missing. Run: git submodule update --init --recursive" >&2
  exit 2
fi

# Build in a throwaway copy so build/, install/ and log/ don't end up owned by root in the repo.
docker run --rm -v "$PWD/ros2_ws/src:/src:ro" ros:jazzy bash -c '
  set -e
  mkdir -p /ws && cp -r /src /ws/src && cd /ws
  apt-get update -qq
  rosdep update -q
  rosdep install --from-paths src --ignore-src -y -q
  source /opt/ros/jazzy/setup.bash
  colcon build --event-handlers console_cohesion+
  colcon test --event-handlers console_cohesion+
  colcon test-result --verbose
'
