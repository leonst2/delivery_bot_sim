# HUD layout and panel integration

This folder contains the screen-space HUD shell for issue #17. The shell owns the screen layout and stable insertion
points; each visualization panel should be developed as a separate prefab and added to its matching slot.

## What exists

`SimHUD.prefab` is a Screen Space Overlay Canvas with a Canvas Scaler configured for a 1920x1080 reference resolution.
The layout uses normalized RectTransform anchors and scales with screen size. The main 3D camera keeps its full
viewport; the HUD panels are translucent overlays.

| Object | Position | Purpose |
| --- | --- | --- |
| `LidarSlot` | Top left | LiDAR visualization panel |
| `MinimapSlot` | Bottom left | City, robot, goal, and obstacle overview |
| `CameraSlot` | Right half | Camera visualization panel |
| Centre-left area | Between the left slots and `CameraSlot` | Unobstructed 3D view |
| `StatusBar` | Bottom edge | ROS connection, grid-step, safety, and health status |

Slot anchors are relative to the Canvas, with `(0, 0)` at the bottom-left and `(1, 1)` at the top-right. The slot
names are part of the integration contract: keep them unchanged. Each slot currently contains a translucent background
and a placeholder label, not a finished visualization.

The HUD does not create or render the planned-path overlay. A path visualizer should render in the 3D view, and its
visibility should be checked when panels are added.

## Adding a panel prefab

1. Create a separate panel prefab under `Assets/DeliveryBot/UI/Panels/`, for example `LidarPanel.prefab`.
2. Make its root a `RectTransform`. Build the panel's visual content as children of that root.
3. Set the panel root's anchors to stretch across its parent slot (`anchorMin = (0, 0)`, `anchorMax = (1, 1)`) and
   start with zero offsets. Add padding inside the panel rather than changing the HUD slot's anchors.
4. Open `SimHUD.prefab` in Prefab Mode and drag the panel prefab into the matching slot in the Hierarchy:
   - LiDAR panel into `LidarSlot`
   - Minimap panel into `MinimapSlot`
   - Camera panel into `CameraSlot`
5. Save the HUD prefab. The instance in `CitySim.unity` uses this prefab, so the layout remains separate from the
   contents of each panel.

The current builder does not load or instantiate panel prefabs. They are integrated manually as nested prefab
instances. Runtime instantiation can be added later with a small installer that holds references to the panel
prefabs and calls `Instantiate(panelPrefab, slot, false)`.

## Data and responsibility boundaries

Panels display simulation or ROS data; they must not implement perception, mapping, planning, or navigation logic.

| Panel or status | Suggested data/display responsibility |
| --- | --- |
| LiDAR | Visualize the existing `/scan` measurements. Do not detect obstacles or plan paths in the panel. |
| Minimap | Render the Unity city and robot from above; show goal and dynamic-object markers from their existing data sources. |
| Camera | Display the simulated sensor-camera image, for example through a `RawImage` using the sensor camera's `RenderTexture`. |
| Planned path | Keep the `/plan` visualization in the 3D view, outside the panel slots. The HUD shell does not implement it. |

The right-side `CameraSlot` is only a placeholder area; it is not itself a Unity `Camera` or a camera feed.

## Status bar

`SimHudStatusBar` is attached to `StatusBar` and owns the four status text fields.

| Field | Current behavior |
| --- | --- |
| ROS connection | Reads `RosBridgeStatus.IsConnected` while the scene is running. |
| Grid step | Subscribes to `/grid_step/state` as `GridStepStateMsg` and displays state name, step index, replans, and yields. |
| Safety | Shows a placeholder until updated through `SetSafetyState(string)`. No topic subscription is implemented yet. |
| Stack health | Shows a placeholder until updated through `SetStackHealth(string)`. No topic subscription is implemented yet. |

When safety and health message contracts are available, add their adapters to the status component or to dedicated
components. Keep the placeholder text for the no-data state.

## Building and editing the shell

With `CitySim.unity` open, use **Tools > Delivery Bot > Build Sim HUD** to regenerate `SimHUD.prefab` and ensure it is
present in the active scene. The builder owns the Canvas, slot anchors, backgrounds, placeholder labels, and status
bar.

**Important:** the builder reconstructs the prefab from code. Running it again after manually adding panel prefab
instances can replace those additions. If the shell needs to be rebuilt, first preserve the panel prefabs separately,
then re-integrate them; alternatively, extend the builder to instantiate configured panel prefab references.

Change slot positions and Canvas settings in `Assets/DeliveryBot/Editor/SimHudPrefabBuilder.cs`, not by editing the
generated shell and then rebuilding it.

## Validation checklist

- In Unity's Game view, check **16:9**, **16:10**, and **21:9**.
- Confirm the three slots and status bar remain in their intended regions and do not overlap one another.
- Confirm the centre-left 3D area is clear and the planned path remains visible after panels are integrated.
- In Play mode, confirm the ROS connection state updates and that grid-step data replaces its placeholder when a
  message arrives.
- Confirm safety and health retain their placeholders until their inputs are connected.
