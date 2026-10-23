#!/usr/bin/env bash
# Fails if the Unity copy of the robot description differs from the one in the shared delivery_bot_core submodule.
# The URDF and its STL meshes must be byte-identical, otherwise the ROS TF tree won't match the simulated robot.
set -euo pipefail
cd "$(dirname "$0")/.."

unity=Assets/URDF/delivery_bot
core=ros2_ws/src/delivery_bot_core/delivery_bot

if [ ! -f "$core/urdf/delivery_bot.urdf" ]; then
  echo "Submodule missing. Run: git submodule update --init --recursive" >&2
  exit 2
fi

status=0
cmp -s "$unity/urdf/delivery_bot.urdf" "$core/urdf/delivery_bot.urdf" || {
  echo "DIFF urdf/delivery_bot.urdf" >&2; status=1; }

# Same set of meshes, same content (Unity's .asset/.prefab/.meta are import output and not compared).
diff <(cd "$unity/meshes" && ls *.stl) <(cd "$core/meshes" && ls *.stl) >&2 || {
  echo "DIFF mesh file list" >&2; status=1; }
for stl in "$unity"/meshes/*.stl; do
  name=$(basename "$stl")
  [ -f "$core/meshes/$name" ] && ! cmp -s "$stl" "$core/meshes/$name" && {
    echo "DIFF meshes/$name" >&2; status=1; }
done

[ $status -eq 0 ] && echo "URDF and meshes in sync with delivery_bot_core"
exit $status
