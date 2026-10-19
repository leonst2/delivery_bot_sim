# delivery_bot - URDF (V-0.2)

Generated from the Fusion model **Delivery-Bot-Body-02** (state of 18 Oct 2026).

```
delivery_bot/
  package.xml
  urdf/delivery_bot.urdf
  meshes/   (binary STL in millimetres - the URDF scales them by 0.001)
    base.stl, front_arm_L/R.stl, steer_L/R.stl, rear_arm_L/R.stl,
    wheel_front_L/R.stl, wheel_rear_L/R.stl
```

## Frames

* x = forward, y = left, z = up, metres.
* `base_link` origin = ground point under the centre of the rear axle.
* **Side naming:** Fusion parts called `..._Right` sit on the robot's **left** (+y) and are named `left` in the URDF (and vice versa).
  Mesh files keep the Fusion suffix (`_R` = URDF left).

## Degrees of freedom

| Joint | Type | Axis | Range | Actuator |
|---|---|---|---|---|
| front_left/right_steering_joint | revolute | z (kingpin) | left -20..+26 deg, right -26..+20 deg (inner wheel 26, outer 20) | DS5160 servo, position control, 5.9 Nm, 7 rad/s |
| front_left/right_wheel_joint | continuous | y | - | hub motor, velocity control, 10 Nm, 35 rad/s (~10.7 km/h) |
| rear_left/right_wheel_joint | continuous | y | - | hub motor, velocity control, 10 Nm, 35 rad/s |
| front_left/right_suspension_joint | revolute (passive) | y | -7..+8 deg (~ +/-25 mm wheel travel) | spring/damper |
| rear_left/right_suspension_joint | revolute (passive) | y | +/-10 deg (~ +/-26 mm) | spring/damper |

Positive wheel velocity drives the robot forward. The steering linkage (servo horn + link plate) is modelled as a direct
kingpin joint; servo angle ~ 1.5 x steering angle (see Steering_overview.png). The lid-lock servo (MZ996) is NOT included.
Hub-motor torque/speed and four driven wheels follow the current parts list - replace with your motor datasheet values.

## Sensors (fixed frames)

| Frame | Position x, y, z [m] | Yaw | Notes |
|---|---|---|---|
| lidar_link | 0.455, 0, 0.515 | 0 | scan-plane height estimated - check datasheet |
| stereo_camera_link | 0.551, 0, 0.418 | 0 | ELP 80x16.5 board, 62 mm baseline |
| stereo_left/right_camera | 0.551, +/-0.031, 0.418 | 0 | + `_optical_frame` (z forward, ROS convention) |
| mono_left/right_camera | 0.533, +/-0.158, 0.418 | +/-21.7 deg outward | + `_optical_frame` |
| radar_left/right_link | 0.487, +/-0.213, 0.418 | +/-55.2 deg outward | antenna position behind cover estimated |

The `<gazebo>` blocks describe the sensors for Gazebo; Unity ignores them. Values marked PLACEHOLDER are assumptions.

## Masses (estimated)

Printed parts 0.75 g/cm3 (~40 % infill), aluminium 2.7, steel 7.85, wheels 2.8 kg each, batteries 2 kg each
-> total ~29.4 kg (base 15.5 kg). Inertias are box/cylinder approximations.

## Unity (ML-Agents) setup

1. Package Manager -> Add package from git URL -> `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
2. Copy the whole `delivery_bot` folder into `Assets/`, right-click `delivery_bot.urdf` -> Import Robot from URDF
   (Axis type: Y-axis; keep the primitive colliders from the URDF).
3. ArticulationBody drives:
   * Steering: stiffness ~1000, damping ~50, force limit 5.9; set `target` in degrees.
   * Wheels: stiffness 0, damping ~10, set `targetVelocity` (deg/s).
   * Suspension: stiffness ~333 Nm/rad front, ~210 Nm/rad rear (~10 mm sag at 29 kg), damping ~10, target 0.
4. Sensors: add a Camera (+ CameraSensorComponent) under each camera link, a RayPerceptionSensorComponent3D or a custom
   raycast script under `lidar_link`, and short ray sensors under the radar links. Use the vertical FOV of the real cameras.
5. Domain randomisation: +/-5 mm and +/-1-2 deg on sensor poses, +/-15 % on masses and motor torque.
