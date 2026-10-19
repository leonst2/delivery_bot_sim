# delivery_bot_sim

Unity simulation of the **delivery_bot** (URDF V-0.2) for reinforcement learning with
[Unity ML-Agents](https://github.com/Unity-Technologies/ml-agents). The robot is imported from its URDF as an
ArticulationBody chain and learns to drive to a goal in an arena with randomly placed obstacles.

> **Status:** test environment. The task, rewards, sensors and hyperparameters are a first working setup, not
> the final version.

## Requirements

| | Version |
| --- | --- |
| Unity | 6000.3.26f1 (Unity 6.3), URP |
| ML-Agents (Unity package) | `com.unity.ml-agents` 4.0.3 |
| URDF Importer | `com.unity.robotics.urdf-importer` v0.5.2 (git) |
| Python | 3.10.12 in `.venv` (created with `uv`) |
| ML-Agents trainer | `mlagents` 1.1.0, `torch` 2.2.2 |

### Python setup

```bash
uv venv --python 3.10.12 .venv          # only if .venv does not exist yet
echo "grpcio==1.62.3" > /tmp/overrides.txt
uv pip install --python .venv/bin/python --override /tmp/overrides.txt -r requirements-training.txt
.venv/bin/mlagents-learn --help          # check
```

The `grpcio` override is needed on Apple Silicon: `mlagents` pins `grpcio` 1.48.2, which has no arm64 wheel.
`torch` is pinned to 2.2.2 because newer versions need `onnxscript` for the model export used by `mlagents` 1.1.

## Project layout

```text
Assets/
  DeliveryBot/
    Scripts/      DeliveryBotDrive, DeliveryBotAgent, TrainingArena, SensorPoseRandomizer, LinkCollisionReporter
    Editor/       DeliveryBotSetupWindow (Tools > Delivery Bot > Setup)
    Scenes/       DeliveryBotTraining.unity  <- training scene
    Prefabs/      DeliveryBot.prefab, TrainingArena.prefab
    Materials/    arena materials, floor physics material
  URDF/
    delivery_bot/ URDF package (urdf/, meshes/, package.xml, README.md with the robot description)
config/delivery_bot_ppo.yaml   PPO trainer config + curriculum
requirements-training.txt      Python dependencies
results/                       training output (git-ignored)
```

## Running the simulation

### Drive the robot by hand

1. Open `Assets/DeliveryBot/Scenes/DeliveryBotTraining.unity`.
2. Press **Play**. Without a trainer or a model, the agent uses keyboard control:
   **W/S** or **↑/↓** throttle, **A/D** or **←/→** steering.
3. An episode ends when the robot reaches the green goal, hits a wall or obstacle, tips over, or after 30 s.
   The arena then re-randomises.

### Train

```bash
.venv/bin/mlagents-learn config/delivery_bot_ppo.yaml --run-id=delivery_bot_01
```

When the trainer prints *"Listening on port 5004"*, press **Play** in Unity. Keep the Unity editor in the
foreground while training in the editor; it stops updating when it loses focus.

- Watch progress: `.venv/bin/tensorboard --logdir results` → <http://localhost:6006>
- Resume a run: add `--resume`. Start over with the same id: add `--force`.
- Faster training: in **Tools > Delivery Bot > Setup**, click **Lay out arena copies** (for example 8 arenas, each with
  its own robot). For long runs, build a standalone player (File > Build Profiles) and train headless:
  `mlagents-learn config/delivery_bot_ppo.yaml --run-id=... --env=Builds/delivery_bot_sim --no-graphics --num-envs=4`

### Use a trained model

The trained policy is written to `results/<run-id>/DeliveryBot.onnx`. Copy it into `Assets/`, select the
`delivery_bot` object, and in **Behavior Parameters** set **Model** to the `.onnx` and **Behavior Type** to
*Inference Only*.

### Re-import the robot after a URDF change

1. Replace the files in `Assets/URDF/delivery_bot/`.
2. Delete the old `delivery_bot` object from the scene.
3. **Tools > Delivery Bot > Setup → Import URDF and configure robot.**

The tool imports with Y-axis settings, removes the URDF Importer's keyboard joint controller, adds the agent and
sensors, places the robot in the arena and re-saves both prefabs. **Configure selected robot** re-applies only the
configuration to a robot that is already imported.

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
| Top speed (35 rad/s hub motors) | 2.98 m/s; the agent is limited to 2.0 m/s forward and 0.6 m/s in reverse |
| Max steering angle (bicycle model) | 21.4°; inner wheel 26°, outer wheel 20° |
| Min turning radius (rear-axle centre) | ≈ 1.25 m |
| Total mass | ≈ 29 kg |

`DeliveryBotDrive` takes a forward speed and a steering angle. It sets **Ackermann** angles on the two steering
joints and a matching speed on each of the four driven wheels, so the inner and outer wheels turn at different rates.

| Joint | Drive in Unity |
| --- | --- |
| Steering (servo) | position: stiffness 1000, damping 50, force limit 5.9 Nm, slew limit 7 rad/s |
| Wheels (4 hub motors) | velocity: damping 10, force limit 10 Nm |
| Suspension (passive) | spring: 333 Nm/rad front, 210 Nm/rad rear, damping 10 |

Physics runs at 100 Hz (fixed timestep 0.01 s, 12/4 solver iterations). Unity frames: robot forward = +Z,
left = −X, up = +Y. Joint signs follow the URDF: positive steering = left turn, positive wheel speed = forward.

### Sensors

| Sensor | Frame | Simulated as |
| --- | --- | --- |
| Lidar | `lidar_link` | `RayPerceptionSensorComponent3D`: 37 rays over 360°, 12 m range |
| Radar left/right | `radar_*_link` | 5 rays over ±30°, 4 m range |
| Stereo/mono cameras | `*_camera` | off by default; enable *Camera sensors* in the setup window (vertical FOV is a 50° placeholder) |

Rays detect objects tagged `Obstacle` and `Wall` and ignore the robot itself (`Robot` layer).

### Learning task

| | |
| --- | --- |
| Behavior name | `DeliveryBot` |
| Actions | 2 continuous: throttle −1…1 (reverse limited to 30 %), steering −1…1 |
| Vector observations (10) | goal position and direction in the robot frame, distance to goal, forward and sideways speed, yaw rate, current steering, tilt |
| Decisions | every 5 physics steps (20 Hz); max episode length 3000 steps (30 s) |
| Rewards | +1 goal reached, −1 collision, −1 tipped over, +0.1 per metre of progress towards the goal, −1/3000 per step |

**Curriculum** (in `config/delivery_bot_ppo.yaml`): goal distance 4 → 8 → 15 m and obstacle count 0 → 5 → 12. Each
lesson ends when the smoothed reward exceeds 0.6.

**Domain randomisation** (every episode): link masses ±15 %, motor and servo torque ±15 %, sensor poses ±5 mm and
±1.5°, plus random start pose, goal position and obstacle layout.

## Known limitations

- Masses, inertias and the radar antenna positions are estimates (see the URDF README).
- The camera sensors use a placeholder field of view, and visual observations slow training down a lot.
- The `<gazebo>` blocks in the URDF are ignored in Unity.
