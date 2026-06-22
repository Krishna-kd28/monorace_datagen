# MonoRace — Gate Detection Data Generator

A self-contained ROS 2 workspace that renders a labelled gate-detection
dataset from a Gazebo warehouse world. It teleports a static **camera rig**
to thousands of randomized viewpoints around the race gates and, for every
viewpoint, saves the RGB frame plus auto-generated labels (gate center, bounding
box, polygon, distance, relative yaw) for each visible gate.

No drone flying, no manual labelling: pose is ground-truth (the teleported
pose), and occlusion is resolved with the depth camera.

```
set_pose (teleport rig) ─► trigger RGB+depth ─► step sim 1 frame
        ─► project gate corners ─► depth visibility check ─► write label
```

---

## 1. What's in here

```
monorace_datagen/
├── README.md                       ← you are here
├── src/
│   ├── mav_simulator/              ← the Gazebo simulator
│   │   ├── mav_bringup/            ← top-level launch (Gazebo + bridges)
│   │   ├── mav_description/        ← X3 model + camera_rig model + meshes
│   │   └── mav_gazebo/             ← worlds, gate/warehouse models, gz_bridge
│   └── pencilnet_ros/              ← the data generator (trimmed to datagen only)
│       ├── pencilnet_ros/
│       │   ├── data_generator_node.py   ← the node
│       │   ├── world_config.py          ← gate positions + geometry per world
│       │   └── camera_model.py          ← camera intrinsics + projection
│       ├── launch/data_gen.launch.py
│       └── config/pencilnet_default.yaml
└── tools/
    └── inspect_dataset.py          ← draw labels on every image to eyeball quality
```

This package is **data generation only**. The training and live-inference code
(network, losses, PencilNet filter, weight conversion) is intentionally left
out.

---

## 2. Requirements

- **ROS 2 Jazzy** (`/opt/ros/jazzy`)
- **Gazebo (gz-sim 8 / Harmonic)** — the launch sets `GZ_SIM_VERSION=8`
- ROS↔Gazebo bridges: `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`
- Python: `numpy`, `opencv-python` (`cv2`)
- The `gz` CLI must be on `PATH` (the node calls `gz service` / `gz topic` to
  teleport, step, and trigger the cameras).

Install the bridges if missing:
```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image
```

---

## 3. Build

```bash
cd monorace_datagen
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

(If you change a world / model SDF or the camera_rig later, rebuild with
`colcon build --packages-select mav_description mav_gazebo pencilnet_ros`.)

---

## 4. Run

Use **two terminals**. `source` both setups in each:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash      # run from the monorace_datagen/ dir
```

### Terminal 1 — start the simulator
```bash
ros2 launch mav_bringup mav_init.launch.py
```
This launches Gazebo with `x3_a2rl_warehouse_harmonic.world` (the default in
this package). That world spawns the **static camera_rig** with physics
`type="ignored"` and bridges the cameras to:
- `/X3/camera/image_raw`   (RGB, `rgb8`/`bgr8`)
- `/X3/camera/depth_image` (depth, `32FC1` metres)

Wait until Gazebo is up and the bridges are publishing.

### Terminal 2 — run the data generator
```bash
ros2 run pencilnet_ros data_generator \
    --ros-args -p output_dir:=$HOME/pencilnet_dataset -p num_viewpoints:=15000
```
or via launch (same defaults):
```bash
ros2 launch pencilnet_ros data_gen.launch.py output_dir:=$HOME/pencilnet_dataset
```

The node auto-detects the running world, pauses physics, then loops over every
viewpoint. Progress prints every 100 viewpoints with an ETA. It saves
`annotations.json` periodically and a final train/test split on completion.
`Ctrl-C` stops early and still saves what was captured.

### Output (`output_dir`)
```
pencilnet_dataset/
├── images/                 frame_000000.jpg, frame_000001.jpg, ...
├── annotations.json        labels for every saved frame (see §6)
├── train-indices.npy       80% split (numpy int array, seed 42)
└── test-indices.npy        20% split
```
Frames where **no** gate passes the filters are not saved, so the image count
is ≤ `num_viewpoints`.

### Inspect the labels
```bash
python3 tools/inspect_dataset.py $HOME/pencilnet_dataset
# writes annotated copies to <dataset>/inspect/
```
Each gate gets its polygon + center cross + a `d=.. yaw=..` tag; gates marked
`[presence-only]` are the steep-angle ones (see §7).

### Start clean
```bash
rm -rf $HOME/pencilnet_dataset/images $HOME/pencilnet_dataset/annotations.json \
       $HOME/pencilnet_dataset/inspect $HOME/pencilnet_dataset/*-indices.npy
```

---

## 5. Node parameters (collection)

| Parameter           | Default              | Meaning |
|---------------------|----------------------|---------|
| `output_dir`        | `~/pencilnet_dataset`| Where images + annotations are written. |
| `num_viewpoints`    | `15000`              | Total viewpoints sampled, spread evenly across all gates × both approach sides. |
| `closest_gate_only` | `false`              | If `true`, keep only the single nearest gate per frame; else label every gate that passes the filters. |
| `trigger_topic`     | `/camera_trigger`    | Gazebo topic pulsed to fire the triggered cameras (must match the camera_rig SDF). |

The gate layout per world lives in
`src/pencilnet_ros/pencilnet_ros/world_config.py` (`WORLDS[...]["gates"]` =
`(name, x, y, yaw, is_double)`).

---

## 6. Annotation format

`annotations.json`:
```jsonc
{
  "annotations": [
    {
      "image": "frame_000123.jpg",
      "annotations": [
        {
          "center_x": 640.2, "center_y": 351.7,   // gate center, pixels (1280×720)
          "distance": 6.4,                          // metres, drone → gate center
          "yaw_relative": -0.21,                    // rad, gate face normal vs drone heading, [-π,π]
          "presence_only": false,                   // true ⇒ mask regression targets in training loss
          "xmin": 470, "ymin": 150, "xmax": 812, "ymax": 600,   // bbox, clamped to image
          "corners_px": [[x,y], ...]                // gate polygon clipped to the frame (for viz)
        }
      ]
    }
  ]
}
```
`presence_only: true` means "a gate is here, but it's seen at too steep an angle
to trust the regression targets" — your training loss should supervise the
objectness/presence channel and **mask** center/distance/yaw for those.

Camera (from `camera_model.py` / the SDFs): 1280×720, hfov 1.309 rad (75°),
`fx=fy≈834.1`, `cx=640`, `cy=360`, mount pitched **30° up** from body-forward
(racing-drone tilt). Projection is pinhole; the SDF lens distortion coefficients
are **not** applied when projecting labels.

---

## 7. Labelling & filtering parameters (the important knobs)

For each gate at each viewpoint the node runs a chain of filters. A gate is only
labelled if it passes **all** of them. All live in `data_generator_node.py`
(`_capture_and_annotate`, plus the `_visibility_ratio` / `_bbox_intersection`
helpers). Loosen them to keep more marginal gates; tighten them for a cleaner,
easier dataset.

`dot` below = alignment between the gate's face normal and the gate→camera
direction. `abs(dot)=1` is head-on; `abs(dot)=0` is edge-on. The angle column is
the viewing angle measured **off the gate face normal**.

| # | Filter (code) | Threshold (this package) | Effect |
|---|---------------|--------------------------|--------|
| 1 | Oblique hard reject — `dot_abs < 0.05` | < 0.05 (≳ **87°** off normal) | Drop near-edge-on slivers entirely. |
| 2 | Presence-only band — `0.05 ≤ dot_abs < 0.30` | 0.05–0.30 (**72.5°–87°**) | Keep gate but set `presence_only=true` (regression masked). |
| 3 | Full label — `dot_abs ≥ 0.30` | ≥ 0.30 (≲ **72.5°**) | Full label: presence + center/dist/yaw regression. |
| 4 | Out-of-frame sliver — `inter_ratio < 0.01 and inter_area < 400` | ratio < 0.01 **and** < 400 px² | Drop gates barely poking into the frame (large close gates still pass via the absolute-area escape). |
| 5 | Min on-screen size — `spread < 8` | < 8 px | Drop gates whose larger bbox side is under 8 px. |
| 6 | Center behind camera — `z_depth < 0.5` | < 0.5 m | Drop gates whose center is behind/at the camera. |
| 7 | Depth visibility — `vis < 0.25` | < 0.25 | Occlusion test: fraction of the gate polygon whose depth agrees with the expected gate depth must be ≥ 25% (gates hidden behind a closer frame are dropped). |

Depth-visibility internals (`_visibility_ratio`): `rel_tol=0.15`, `max_tol=3.0 m`
depth-match band, `min_px=40` minimum polygon pixels, 1-px erosion to dodge noisy
depth edges. "Opening" pixels (seeing *through* the gate to far background) are
ignored so close gates aren't penalised; only "occluding" (closer) pixels count
against visibility.

### Viewpoint sampling (also in §7's spirit — how poses are drawn)
`_generate_viewpoints()` (seed `42`, reproducible). Per gate, per side
(`gyaw` and `gyaw+π`):

| Axis | Distribution | Range |
|------|--------------|-------|
| Distance to gate | log-uniform | **2.5 – 18 m** (biased toward close-ups) |
| Approach angle | uniform | gate normal **±40°** (±0.70 rad) |
| Lateral offset | uniform | **±1.5 m** perpendicular |
| Height | nominal ± jitter | gate center **±0.5 m** (floor 0.5 m); nominal = 1.45 m single / 2.8 m double |
| Yaw | face gate center ± jitter | **±15°** (±0.26 rad) |
| Pitch | uniform | **0.23 – 0.67 rad** (~13°–38° down) |
| Roll | uniform | **±3°** (±0.052 rad) |

> **Note — these thresholds were deliberately loosened.** Compared to the
> earlier baseline they accept gates that are more steeply angled, smaller, and
> more occluded (hard-reject 80°→87°, full-label 60°→72.5°, sliver ratio
> 0.03→0.01, min spread 15→8 px, min visibility 0.50→0.25). That yields a larger,
> harder dataset. Revert any row in the table above toward those stricter values
> if you want fewer but cleaner labels.

---

## 8. How it works (why teleport + triggered cameras)

- **camera_rig, not the X3 drone.** The data-gen world spawns a `<static>true</static>`
  camera rig with the *same* camera mount/intrinsics as the X3 but no motors or
  control plugins. Combined with physics `type="ignored"`, a `set_pose` call
  followed by one sim step renders exactly at the requested pose with **zero
  drift** — so the ground-truth label comes straight from the teleport pose, with
  no odometry/timing race.
- **Triggered cameras.** Both the RGB and depth sensors in
  `camera_rig/model.sdf` ship with `<triggered>true</triggered>` +
  `<trigger_topic>/camera_trigger</trigger_topic>`. One pulse on `/camera_trigger`
  renders both on the same step, so RGB and depth are perfectly aligned per
  viewpoint. **This is already enabled — no SDF edits needed.**
  (To repurpose the rig for continuous live streaming, delete those two tags in
  both sensors and rebuild.)

---

## 9. Troubleshooting

- **`No supported world detected`** — the simulator isn't up yet, or `gz` isn't
  on `PATH`. Start Terminal 1 first and wait for Gazebo; check
  `gz service -l | grep set_pose`.
- **All viewpoints skipped / `Timeout waiting for cameras`** — the camera topics
  aren't bridged. Confirm `ros2 topic hz /X3/camera/image_raw` and
  `/X3/camera/depth_image` both tick. Make sure the `ros_gz_image` bridges from
  `mav_init.launch.py` started.
- **Gazebo opens the wrong world** — clear the cached GUI config:
  `rm -f ~/.gz/sim/*/gui.config ~/.ignition/gazebo/*/gui.config` and relaunch.
- **Few labels per frame** — expected; many viewpoints see 0–1 gates. Use
  `tools/inspect_dataset.py` to confirm the labels that *are* produced look right.
