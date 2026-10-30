# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Unity 6.3 (6000.3.26f1, URP) simulation of the URDF-imported **delivery_bot**. The robot is navigated by a
deterministic ROS 2 (Jazzy) / Nav2 stack running in Docker and connected over ROS-TCP-Connector. It is no longer
controlled by Reinforcement Learning. README.md has the architecture diagram, the ROS topic table, the grid-step
loop and the UI layout. Read it first and don't duplicate it here.

**Migration status:** most of the ROS 2 side is still to be built. Work is tracked in GitHub issues #2–#21 and the
*delivery_bot_sim* project. In the README, *(planned)* marks paths that don't exist yet. Check before assuming they
exist. The shared submodule `ros2_ws/src/delivery_bot_core` exists (#6), and so does the ROS 2 container
(#5: core dev image with C++/Python/Rust + this repo's endpoint layer; `docker/sim.launch.py` starts the endpoint and, once core #4 provides `delivery_bot_bringup`, `stack.launch.py sim:=true`).

## Commands

There are no tests, no linter and no command-line build. Compile errors show up in the Unity console
(`Unity_GetConsoleLogs` via MCP).

```bash
git submodule update --init --recursive          # shared code (delivery_bot_core)
scripts/check_urdf_sync.sh                       # Unity URDF/meshes == submodule copy (CI too)
scripts/check_core_build.sh                      # colcon build + test of the submodule in its dev image (needs Docker)
git submodule update --remote ros2_ws/src/delivery_bot_core   # bump the pin, then commit the submodule
```

ROS 2 runs only in Docker (there's no native ROS 2 on the Mac):

```bash
docker compose -f docker/compose.yaml build      # builds the core dev image (submodule) + the sim layer on top
docker compose -f docker/compose.yaml up -d      # endpoint on :10000 (host networking, enabled in Docker Desktop)
docker compose -f docker/compose.yaml -f docker/compose.ports.yaml up -d   # fallback without host networking
docker compose -f docker/compose.yaml exec ros bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list"
docker compose -f docker/compose.yaml run --rm ros bash -c "cd /ws && colcon test && colcon test-result --verbose"
docker compose -f docker/compose.yaml down       # named volumes keep the workspace build cache
```

**Image split:** the build environment (ros-core + C++/Python/Rust toolchains, ros2_rust overlay, forbidden-package
check) is `delivery_bot_core/docker/Dockerfile` in the submodule. Change toolchains there, in delivery_bot_core.
`docker/Dockerfile` in this repo is only the simulation layer `FROM` that image: ROS-TCP-Endpoint + an entrypoint
that rebuilds `ros2_ws/src` (mounted read-only) incrementally on every start, so a restart picks up submodule
changes. `SKIP_BUILD=1` skips the build.

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
- ML-Agents is gone (#3). Don't reintroduce `Unity.MLAgents` types.
- **Scenes:** `Scenes/CitySim.unity` is the main scene and the only one in Build Settings. It's empty (camera + light)
  until the city (#14) lands; city, UI and dynamic objects go there. `Scenes/DriveTest.unity` is a walled
  `TestGround` with one robot for quick drive/sensor tests. The setup window creates/opens it and imports the URDF
  into it. The robot reaches CitySim only as a `DeliveryBot.prefab` instance.

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
  downgrade it. `com.unity.ai.assistant` provides the Unity MCP bridge. ROS-TCP-Connector is pinned to `#v0.7.1`,
  which accepts our endpoint v0.7.0.
- **ROS-TCP-Connector setup:** ROS 2 mode is the scripting define `ROS2` (Standalone, in `ProjectSettings`). Without
  it, `TimeMsg`/`HeaderMsg` silently use the ROS 1 layout. The connection settings are in
  `Assets/Resources/ROSConnectionPrefab.prefab`.
- **ROS-TCP-Connector pitfalls:**
  - Publishers must skip publishing while `RosBridgeStatus.IsConnected` is false. The connector queues registrations
    behind queued messages, so anything published while disconnected is rejected with `Not registered to publish topic`.
  - Don't use `LastMessageReceivedRealtime` for connection checks: keepalive replies don't update it.
  - The connector logs a lost connection with `Debug.LogError`, so with the Console's Error Pause on, the sim pauses
    when the endpoint stops or is missing at Play. `IsConnected` is stale while paused, because nothing is sent.
  - A domain reload during Play clears the connection without reconnecting. After editing scripts, wait for the
    compile before entering Play.
- **Project settings:** physics 100 Hz (fixed timestep 0.01 s), solver iterations 12/4, `Robot` layer (8) doesn't
  collide with itself. Sensor rays must exclude the `Robot` layer. Keep Version Control mode on *Visible Meta
  Files* and Run In Background on, since the sim has to keep running while ROS tools have focus.
- **Docker on macOS:** `network_mode: host` needs host networking enabled in Docker Desktop (Settings → Resources →
  Network; it's on for this machine). Without it the port isn't reachable from the Mac; use
  `docker/compose.ports.yaml`, which publishes 10000. In that mode ROS 2 CLI tools only work inside the container.
- **Docker image base:** the core dev image is `ros:jazzy-ros-core`, not `ros:jazzy`. The ros-base image ships
  `robot_state_publisher` and `kdl_parser`, which the stack must not use, and `check_forbidden_packages.sh` fails the
  image build if any forbidden package is present. Add dependencies to the core Dockerfile; never run
  `rosdep install` in the container, because the old core scaffold still declares `robot_state_publisher`.
- **Rust in ROS 2:** rclrs comes from crates.io and message types from the `ros-env` crate. Message packages only get
  Rust bindings if they're built with `rosidl_generator_rs` present, which the core image provides in
  `/opt/ros2_rust` (sourced by the entrypoints).

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
