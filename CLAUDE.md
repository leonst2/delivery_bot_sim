# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Unity 6.3 (6000.3.26f1, URP) simulation of the URDF-imported **delivery_bot**. The robot is navigated by a
deterministic ROS 2 (Jazzy) / Nav2 stack running in Docker and connected over ROS-TCP-Connector. It is no longer
controlled by Reinforcement Learning. README.md has the architecture diagram, the ROS topic table, the grid-step
loop and the UI layout. Read it first and don't duplicate it here.

**Migration status:** most of the ROS 2 side is still to be built. Work is tracked in GitHub issues #2–#21 and the
*delivery_bot_sim* project. In the README, *(planned)* marks paths that don't exist yet, such as `docker/`. Check
before assuming they exist. The shared submodule `ros2_ws/src/delivery_bot_core` exists (#6).

## Commands

There are no tests, no linter and no command-line build. Compile errors show up in the Unity console
(`Unity_GetConsoleLogs` via MCP).

```bash
git submodule update --init --recursive          # shared code (delivery_bot_core)
scripts/check_urdf_sync.sh                       # Unity URDF/meshes == submodule copy (CI too)
scripts/check_core_build.sh                      # colcon build + test of the submodule in ros:jazzy (needs Docker)
git submodule update --remote ros2_ws/src/delivery_bot_core   # bump the pin, then commit the submodule
```

Planned workflow, once the Docker issue (#5) has landed:

```bash
docker compose -f docker/compose.yaml up         # ros_tcp_endpoint :10000, Nav2, grid-step controller
# then press Play in Unity
ros2 topic echo /clock                           # bridge alive, sim time advancing
ros2 run tf2_ros tf2_echo odom base_link         # odometry/TF from Unity
```

## Architecture rules

- **Unity only simulates:** physics, sensors (LiDAR `/scan`, stereo `/stereo/*`), state (`/odom`, `/tf`,
  `/clock`) and actuation. `/cmd_vel` maps onto `DeliveryBotDrive.SetCommand(speed, steeringAngle)`, which does
  the Ackermann geometry. Keep perception, mapping, planning and the grid-step controller out of Unity C#.
- **Navigation logic lives in the shared submodule** `ros2_ws/src/delivery_bot_core`
  ([leonst2/delivery_bot_core](https://github.com/leonst2/delivery_bot_core)), which the physical robot uses too.
  Never fork or copy that logic into this repo, and don't edit files inside the submodule from this repo. Change
  them in delivery_bot_core, push, then bump the pin here. Sim-only differences go only into
  `delivery_bot_navigation/config/sim.yaml` (applied by `bringup.launch.py sim:=true`).
- **The ROS description package is named `delivery_bot`**, not `delivery_bot_description`, because the URDF's mesh
  URIs are `package://delivery_bot/...` and Unity imports the same file.
- **Grid-step loop:** halt → perceive → update costmap → (re)plan → move exactly `step_size` (0.5 m) → repeat. A
  step is an arc along the path (min turning radius ≈ 1.25 m), not a turn in place. The loop must be
  deterministic: no randomness and fixed tie-breaking.
- **Frames:** Unity is RUF, ROS is FLU. Convert with ROS-TCP-Connector's `To<FLU>()` / `From<FLU>()`, never by
  hand. All ROS nodes use `use_sim_time:=true` with the `/clock` that Unity publishes.
- **Topic names and types** are fixed by the README table. The issues were written against it so they can be
  worked on in parallel. Change the table and the affected issues together.
- **`Editor/DeliveryBotSetupWindow.cs` (Tools > Delivery Bot > Setup)** imports the URDF and configures the robot
  in `ConfigureRobot`: layer, frame-link masses, `DeliveryBotDrive` + `DeliveryBotKeyboardDrive`, and sensor
  mounts. It then re-saves `DeliveryBot.prefab`, so hand edits to those components are overwritten by the next
  Configure. Change the setup code instead.
- **Command sources must not fight:** `DeliveryBotKeyboardDrive` only calls `SetCommand` while a key is held (plus
  one stop on release). Any new command source (for example the `/cmd_vel` subscriber) should behave the same way, or
  own the drive exclusively.
- ML-Agents is gone (#3). Don't reintroduce `Unity.MLAgents` types. The scene's `TestGround` is a plain floor with
  walls for drive tests until the city scene (#2) replaces it.

## Gotchas

- **URDF package root:** the importer resolves `package://delivery_bot/...` relative to the folder that contains the
  `.urdf`. `PrepareUrdfForImport` therefore writes `Assets/URDF/delivery_bot.import.urdf` next to the package
  folder and imports that copy (git-ignored). Don't move the URDF out of `delivery_bot/urdf/` without checking this.
  `Assets/URDF/Materials`, `Assets/URDF/meshes` and `meshes/*.asset|*.prefab` are importer output that the
  prefabs reference. Never delete them.
- **Same URDF on both sides:** `delivery_bot_core/delivery_bot/{urdf,meshes}` (used by `robot_state_publisher` for
  `/tf_static`) must stay byte-identical to `Assets/URDF/delivery_bot/{urdf,meshes/*.stl}`, or the ROS TF tree
  won't match the simulated robot. `scripts/check_urdf_sync.sh` enforces this in CI. After a URDF change, update
  both repos.
- **Latched topics and the CLI:** `/grid_step/state`, `/robot_description` and `/tf_static` are transient-local.
  `ros2 topic echo` without `--qos-*` flags matches that. Overriding only `--qos-durability` falls back to
  best-effort and never receives the latched sample.
- **Frame-only links** (sensor and optical frames) have no `<inertial>`, so Unity would give each one 1 kg. The
  setup window sets them to 0.01 kg.
- **Sensors go on child "mount" objects** under the links (for example `LidarSensor`). The links are
  ArticulationBodies, so you can't move their transforms at runtime.
- **The importer's `Controller` component** (keyboard joint control) overrides the drive settings in `Start()`. The
  setup window removes it; keep it removed.
- **Joint signs:** the importer maps the URDF axes so that the URDF conventions hold (positive steering = left,
  positive wheel velocity = forward). Verified in Play mode: a 15° command gave a 1.91 m turning radius against a
  predicted 1.87 m. `invertSteering`/`invertWheels` exist only as a fallback.
- **Re-running Configure** unpacks the outer prefab instance before re-saving the robot prefab, because Unity can't
  save a nested part of a prefab instance as its own prefab.
- **Package pins** (`Packages/manifest.json`): URDF Importer from git `#v0.5.2`. The template packages are pinned to
  the versions 6.3 resolves. Input System 1.12.0 fails to compile on 6.3 (`BuildTarget.ReservedCFE`), so don't
  downgrade it. `com.unity.ai.assistant` provides the Unity MCP bridge.
- **Project settings:** physics 100 Hz (fixed timestep 0.01 s), solver iterations 12/4, `Robot` layer (8) doesn't
  collide with itself. Sensor rays must exclude the `Robot` layer. Keep Version Control mode on *Visible Meta
  Files* and Run In Background on, since the sim has to keep running while ROS tools have focus.
- **Docker on macOS:** `network_mode: host` needs host networking enabled in Docker Desktop. Otherwise, publish
  port 10000.

## Working with the Unity MCP tools

- Check compile errors with `Unity_GetConsoleLogs` after editing scripts. The MCP connection drops for about 10 s
  during each domain reload (after a refresh and on entering/leaving Play).
- `Unity_RunCommand` can't do anything that opens a dialog. Delete assets from the shell, then `AssetDatabase.Refresh()`.
- The editor doesn't advance Play mode while Unity is not the focused app (`Time.frameCount` stays put). Bring Unity
  to the front for Play-mode tests.
- Unity only writes ProjectSettings to disk on save. Call `AssetDatabase.SaveAssets()` after changing settings.

## Conventions

- Unity frames: robot forward +Z, left −X, up +Y; `base_link` origin = ground point under the rear-axle centre.
- Namespace `DeliveryBot`, code under `Assets/DeliveryBot/`.
