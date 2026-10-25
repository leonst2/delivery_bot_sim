#!/usr/bin/env bash
# Builds and tests the pinned delivery_bot_core submodule in its own development image (ROS 2 Jazzy ros-core +
# C++/Python/Rust), the same environment the simulation container uses.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ ! -f ros2_ws/src/delivery_bot_core/docker/Dockerfile ]; then
  echo "Submodule missing or too old. Run: git submodule update --init --recursive" >&2
  exit 2
fi

docker build -t delivery_bot_core_dev:jazzy ros2_ws/src/delivery_bot_core/docker

# Build in a throwaway container so build/, install/ and log/ don't end up owned by root in the repo.
docker run --rm -v "$PWD/ros2_ws/src:/src:ro" delivery_bot_core_dev:jazzy bash -c '
  set -e
  mkdir -p /ws && cp -r /src /ws/src && cd /ws
  colcon build --event-handlers console_cohesion+
  colcon test --event-handlers console_cohesion+
  colcon test-result --verbose
'
