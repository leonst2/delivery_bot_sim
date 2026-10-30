# delivery_bot_sim

Unity simulation of the **delivery_bot** (URDF V-0.2) for developing and testing its navigation stack. The robot is
imported from its URDF as an ArticulationBody chain and is driven by a deterministic, classical robotics pipeline:
**ROS 2 + Nav2**, connected to Unity over TCP.

> **Status:** migration in progress. The project started as a Reinforcement Learning (ML-Agents) test environment
> and is moving to ROS 2 / Nav2. This README describes the target architecture. The parts that aren't built yet
> are tracked in the [GitHub issues](https://github.com/leonst2/delivery_bot_sim/issues) and the
> *delivery_bot_sim* project board. Paths marked *(planned)* don't exist yet.

## Why ROS 2 / Nav2 instead of RL

The robot's behaviour has to be predictable, debuggable and the same in simulation and on the physical robot. A
learned policy gives none of that. With ROS 2 / Nav2, perception, mapping, planning and control are separate,
inspectable nodes, and the simulation runs exactly the same navigation code as the real robot. Unity's job is
now only to simulate the world, the robot's physics and its sensors.

## Architecture

```text
┌──────────────────────────── Unity 6.3 (host) ─────────────────────────────┐
│  City scene: static objects (buildings, parked cars) + dynamic objects    │
│  (cyclists)                                                               │
│  delivery_bot (URDF → ArticulationBodies)                                 │
│    sensors:   2D LiDAR → /scan, stereo cameras → /stereo/*                │
│    state:     /odom, /tf (odom→base_link), /clock                         │
│    actuation: /cmd_vel → DeliveryBotDrive (Ackermann)                     │
│  UI: minimap, LiDAR view, camera views, Nav2 path overlay                 │
│                     ROS-TCP-Connector (ROSConnection)                     │
└─────────────────────────────────┬─────────────────────────────────────────┘
                                  │ TCP 127.0.0.1:10000
┌─────────────────────────────────┴───── Docker (network_mode: host) ───────┐
│  ros_tcp_endpoint                                                         │
│  robot_state_publisher (URDF from the shared submodule → /tf_static)      │
│  perception / dynamic obstacle tracking → /tracked_obstacles              │
│  map_server or slam_toolbox, Nav2 (costmaps, planner)                     │
│  grid-step controller → /cmd_vel, /plan, /grid_step/state                 │
│  ros2_ws/src/delivery_bot_core ← shared Git submodule (same as robot)     │
└───────────────────────────────────────────────────────────────────────────┘
```

- **Unity ⇄ ROS 2:** Unity Robotics [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
  in the editor talks to `ros_tcp_endpoint` over TCP on port 10000. ROS 2 is Jazzy, and all nodes run with
  `use_sim_time:=true` on the `/clock` that Unity publishes.
- **Docker:** all ROS 2 nodes run in one container with `network_mode: host`, so the endpoint is reachable on
  `127.0.0.1` and DDS discovery works without port mapping. On macOS, enable host networking in Docker Desktop
  (Settings → Resources → Network). Otherwise, publish port 10000 instead.
- **Shared navigation code:** the robot description and navigation packages live in
  [delivery_bot_core](https://github.com/leonst2/delivery_bot_core). That repo is included here as a Git submodule
  at `ros2_ws/src/delivery_bot_core`, pinned to a commit. The physical robot
  ([delivery_bot_os](https://github.com/leonst2/delivery_bot_os)) uses the same repository. Its packages:
  `delivery_bot` (URDF + meshes, robot_state_publisher), `delivery_bot_navigation` (Nav2 params, `bringup.launch.py`),
  `delivery_bot_grid_step` and `delivery_bot_perception`. Sim-only differences (`use_sim_time`, topic remaps) go
  only into `delivery_bot_navigation/config/sim.yaml`, which `bringup.launch.py sim:=true` applies.

### Grid-step navigation loop

The robot doesn't drive continuously. It runs a discrete, deterministic loop:

1. **Halt.** Command zero velocity until the robot is at rest (|v| < 0.02 m/s).
2. **Perceive.** Wait for a fresh LiDAR scan and stereo pair taken after the halt. Detect and track dynamic
   objects (cyclists).
3. **Map.** Wait until the local costmap has been updated with those observations.
4. **Plan.** (Re)compute the path to the goal with Nav2 (`ComputePathToPose`) and publish it on `/plan`.
5. **Move one cell.** Follow the path for exactly `step_size` metres (default 0.5 m) of arc length, then go back
   to step 1.

If a tracked object is predicted to cross the next step, the robot **yields** (stays halted and perceives again).
If an object blocks the path and stays put, the robot **re-plans** around it. With the same map, start and goal,
the robot always takes the same sequence of steps.

The minimum turning radius is about 1.25 m, so a 0.5 m step is an arc segment along the planned path, not a
turn in place. Nav2 uses an Ackermann-feasible planner (Smac Hybrid-A*).

### ROS interface

| Topic | Type | Direction | Frame |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | Unity → ROS | |
| `/tf` | `tf2_msgs/TFMessage` (`odom → base_link`) | Unity → ROS | `odom` |
| `/tf_static` | `tf2_msgs/TFMessage` (sensor frames) | `robot_state_publisher` | |
| `/odom` | `nav_msgs/Odometry` | Unity → ROS | `odom` → `base_link` |
| `/scan` | `sensor_msgs/LaserScan` | Unity → ROS | `lidar_link` |
| `/stereo/{left,right}/image_raw` | `sensor_msgs/Image` | Unity → ROS | `stereo_{left,right}_camera` |
| `/stereo/{left,right}/camera_info` | `sensor_msgs/CameraInfo` | Unity → ROS | `stereo_{left,right}_camera` |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Unity | `base_link` |
| `/plan` | `nav_msgs/Path` | ROS → Unity | `map` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Unity → ROS | `map` |
| `/grid_step/state` | `std_msgs/String` | ROS → Unity | |
| `/tracked_obstacles` | `visualization_msgs/MarkerArray` | ROS → Unity | `map` |

`/cmd_vel` is `geometry_msgs/Twist`, Nav2's default on Jazzy. Switch both sides to `TwistStamped` if
`enable_stamped_cmd_vel` is turned on. Unity uses RUF coordinates (right, up, forward) and ROS uses FLU (forward,
left, up). Convert with the ROS-TCP-Connector helpers (`To<FLU>()` / `From<FLU>()`).

## UI layout

| Region | Content |
| --- | --- |
| Top left | Live 2D LiDAR scan, robot-centred polar plot |
| Bottom left | Minimap of the whole city: robot, goal and dynamic objects. Click to set a goal (`/goal_pose`) |
| Right half | Live stereo/front camera views |
| 3D view | Rendered line of the current Nav2 path (`/plan`) |

A status bar shows the ROS connection status and the current grid-step state.

## Requirements

| | Version |
| --- | --- |
| Unity | 6000.3.26f1 (Unity 6.3), URP |
| URDF Importer | `com.unity.robotics.urdf-importer` v0.5.2 (git) |
| ROS-TCP-Connector | `com.unity.robotics.ros-tcp-connector` (git, pinned tag) *(planned)* |
| ROS 2 | Jazzy, in Docker: core dev image (`ros:jazzy-ros-core` + C++/Python/Rust) + sim layer with `ros_tcp_endpoint` v0.7.0 |
| Docker | Docker Desktop. On macOS either enable host networking or use `docker/compose.ports.yaml` |

## Quick start

> The ROS 2 container builds the shared workspace and starts `docker/sim.launch.py`: the TCP endpoint, plus
> `stack.launch.py sim:=true` as soon as the pinned delivery_bot_core provides `delivery_bot_bringup`
> (delivery_bot_core#4). Until then it starts the endpoint only. The Unity side of the bridge (#4) is still to come.

```bash
git clone --recurse-submodules https://github.com/leonst2/delivery_bot_sim.git
# or, in an existing clone:
git submodule update --init --recursive

docker compose -f docker/compose.yaml up -d     # endpoint on 0.0.0.0:10000 + stack (Linux, or macOS with host networking)
docker compose -f docker/compose.yaml -f docker/compose.ports.yaml up -d   # macOS without host networking
docker compose -f docker/compose.yaml exec ros bash -lc "source /opt/ros/jazzy/setup.bash && ros2 node list"
ROS_DOMAIN_ID=7 docker compose -f docker/compose.yaml up -d                # other DDS domain (default 0)
docker compose -f docker/compose.yaml run --rm ros ros2 launch /opt/delivery_bot_sim/sim.launch.py stack:=false   # endpoint only
```

Once Unity is in Play mode, `/clock` shows up in the container:
`docker compose -f docker/compose.yaml exec ros bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list | grep /clock"`.
A source change on the host only needs `docker compose -f docker/compose.yaml restart ros`; the entrypoint rebuilds
the workspace incrementally.

`docker compose build` builds two images:

- **`delivery_bot_core_dev:jazzy`** from `ros2_ws/src/delivery_bot_core/docker/` (the shared repo): ROS 2 Jazzy
  ros-core with the C++, Python and Rust toolchains. Its build fails if a forbidden package (Nav2, slam_toolbox, AMCL,
  map_server, robot_state_publisher, kdl_parser) is present, which is why it is based on `ros:jazzy-ros-core` rather
  than `ros:jazzy`.
- **`delivery_bot_sim_ros:jazzy`** from `docker/` (this repo): the core image plus the Unity ROS-TCP-Endpoint and an
  entrypoint that builds `ros2_ws/src` on every start (cached in named volumes, so restarts take seconds).

### Shared code (submodule)

```bash
scripts/check_urdf_sync.sh      # Unity URDF + meshes byte-identical to delivery_bot_core (also runs in CI)
scripts/check_core_build.sh     # colcon build + test of the pinned submodule in its dev image (needs Docker)

# Move to a newer delivery_bot_core commit:
git submodule update --remote ros2_ws/src/delivery_bot_core
git add ros2_ws/src/delivery_bot_core && git commit -m "Bump delivery_bot_core"
```

Change navigation code in [delivery_bot_core](https://github.com/leonst2/delivery_bot_core), not here. Then bump the
pin. CI runs the URDF check on every push (`.github/workflows/core-build.yml`), and builds and tests the pinned core
in its dev image whenever `docker/` or the submodule changes (`.github/workflows/docker.yml`).

Then open `Assets/DeliveryBot/Scenes/CitySim.unity` in Unity and press **Play**. Set a goal by clicking on the minimap or from ROS:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0}, orientation: {w: 1.0}}}"
```

Useful checks: `ros2 topic echo /clock`, `ros2 run tf2_ros tf2_echo odom base_link`, RViz2 with fixed frame `map`.

## Project layout

```text
Assets/
  DeliveryBot/
    Scripts/      DeliveryBotDrive (Ackermann drive), DeliveryBotKeyboardDrive, SensorPoseRandomizer, LinkCollisionReporter
    Editor/       DeliveryBotSetupWindow (Tools > Delivery Bot > Setup: URDF import + robot configuration)
    Scenes/       CitySim.unity: main scene, only build scene (camera + light until the city, #14)
                  DriveTest.unity: walled test ground with one robot, for drive and sensor tests
    Prefabs/      DeliveryBot.prefab
    Materials/    ground and wall materials, floor physics material
  URDF/
    delivery_bot/ URDF package (urdf/, meshes/, package.xml, README.md with the robot description)
docker/                         simulation layer on the core dev image (Dockerfile, compose.yaml, compose.ports.yaml)
ros2_ws/src/delivery_bot_core/  shared submodule: robot description + navigation packages
scripts/                        check_urdf_sync.sh, check_core_build.sh
```

## The robot in the simulation

Robot details (frames, masses, actuators, sensor poses) are in
[Assets/URDF/delivery_bot/README.md](Assets/URDF/delivery_bot/README.md). The simulation uses them as follows.

### Geometry and drive

Measured from the imported model at start-up:

| | Value |
| --- | --- |
| Wheelbase | 0.49 m |
| Track width (front and rear) | 0.485 m |
| Wheel radius | 0.085 m |
| Top speed (35 rad/s hub motors) | 2.98 m/s; commands are limited to 2.0 m/s forward and 0.6 m/s in reverse |
| Max steering angle (bicycle model) | 21.4°; inner wheel 26°, outer wheel 20° |
| Min turning radius (rear-axle centre) | ≈ 1.25 m |
| Total mass | ≈ 29 kg |

`DeliveryBotDrive` takes a forward speed and a steering angle (`SetCommand`). It sets **Ackermann** angles on the
two steering joints and a matching speed on each of the four driven wheels. `/cmd_vel` maps onto it with
`steering = atan(wheelbase · ω / v)`. Collisions of any link are reported through `DeliveryBotDrive.Collided`.

**Manual driving:** press Play and use **W/S** or **↑/↓** for throttle and **A/D** or **←/→** for steering
(`DeliveryBotKeyboardDrive`). It only sends commands while a key is held, so it doesn't fight commands from ROS.

| Joint | Drive in Unity |
| --- | --- |
| Steering (servo) | position: stiffness 1000, damping 50, force limit 5.9 Nm, slew limit 7 rad/s |
| Wheels (4 hub motors) | velocity: damping 10, force limit 10 Nm |
| Suspension (passive) | spring: 333 Nm/rad front, 210 Nm/rad rear, damping 10 |

Physics runs at 100 Hz (fixed timestep 0.01 s, 12/4 solver iterations). Unity frames: robot forward = +Z,
left = −X, up = +Y. `base_link` origin = ground point under the rear-axle centre. Joint signs follow the URDF:
positive steering = left turn, positive wheel speed = forward.

### Sensors

| Sensor | Frame | Simulated as |
| --- | --- | --- |
| 2D LiDAR | `lidar_link` | raycast fan, 360°, 1° resolution, 0.1–12 m, 10 Hz → `/scan` *(planned)* |
| Stereo cameras | `stereo_left_camera`, `stereo_right_camera` | Unity cameras, 10 Hz → `/stereo/*` (vertical FOV is a 50° placeholder) *(planned)* |
| Radar left/right, mono cameras | `radar_*_link`, `mono_*_camera` | frames available in the URDF, not simulated yet |

Sensor rays ignore the robot itself (`Robot` layer). Sensors sit on child "mount" objects under the links, because
the links are ArticulationBodies.

### Re-import the robot after a URDF change

1. Replace the files in `Assets/URDF/delivery_bot/`.
2. Delete the old `delivery_bot` object from the scene.
3. **Tools > Delivery Bot > Setup → Import URDF and configure robot.**

The tool imports with Y-axis settings, removes the URDF Importer's keyboard joint controller, places the robot
and re-saves the prefab.

The same URDF and STL meshes must also go into `delivery_bot_core/delivery_bot/` (commit there, then bump the
submodule), so the TF tree in ROS matches the simulated robot. `scripts/check_urdf_sync.sh` fails CI until both
copies match.

## Legacy (RL)

Before this migration, the robot was trained with Unity ML-Agents (PPO) to drive to a goal (`DeliveryBotAgent`,
`TrainingArena`, `config/delivery_bot_ppo.yaml`, Python trainer in `.venv`). That approach was dropped in favour of
ROS 2 / Nav2, and the ML-Agents package, code and training config were removed (issue #3). See the git history
before that change for the old setup. A local `.venv/` or `results/` folder from that time is git-ignored and can be
deleted.

## Known limitations

- Masses, inertias and the radar antenna positions are estimates (see the URDF README).
- The camera field of view is a placeholder.
- The `<gazebo>` blocks in the URDF are ignored in Unity.
