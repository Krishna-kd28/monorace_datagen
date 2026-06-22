# MAV Simulator in ROS 2 with Gazebo Sim

This project provides simulation of a quadrotor MAV (Micro Aerial Vehicle) in **ROS 2** using **Gazebo Sim**. It includes an X3-style quadrotor model, an A2RL-style track world with gates, and ROS–Gazebo bridging for control and visualization.

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
  - [Basic simulation](#1-basic-simulation)
  - [Controlling the drone](#2-controlling-the-drone)
  - [Spawning a different model](#3-spawning-a-different-model)
- [Package layout](#package-layout)

## Features

### 1. **Quadrotor model (X3)**

- SDF and URDF/Xacro descriptions for the X3-style quadrotor.
- Meshes and inertial properties suitable for Gazebo Sim and RViz.

### 2. **ROS 2 integration**

- **ros_gz_bridge**: `X3/cmd_vel` (ROS `geometry_msgs/msg/Twist`) is bridged to Gazebo for velocity control.
- **robot_state_publisher**: Publishes the robot description and TF from the same SDF used in simulation (with mesh URIs adjusted for RViz).

### 3. **Worlds and assets**

- **a2rl_track.world**: A2RL-style track environment with gates and obstacles.
- **Gate models**: `single_gate` and `double_gate` in `mav_gazebo/models/`.

### 4. **Visualization**

- **RViz2**: Preconfigured `mav.rviz` in `mav_description` for viewing the robot and sensor data.
- **Gazebo Sim GUI**: 3D view, entity tree, world control, and optional Teleop panel.

## Requirements

- **ROS 2 (Humble)** (or compatible distro)
- **Gazebo Sim** (gz-sim, version 8)
- **ros_gz** (ROS 2 – Gazebo bridge):`sudo apt install ros-${ROS_DISTRO}-ros-gz`
- **RViz2**

For full Gazebo + ROS 2 setup: [Gazebo Sim with ROS 2](https://gazebosim.org/docs/latest/ros_installation/).

## Installation

1. Clone the repository (or ensure `mav_simulator` is in your workspace source tree):

   ```bash
   cd /path/to/your_ws/src
   # clone or copy mav_simulator here
   ```
2. Install ROS–Gazebo bridge (if not already installed):apt install ros-${ROS_DISTRO}-ros-gz
3. Build and source:

   ```bash
   cd /path/to/your_ws
   colcon build --symlink-install 
   source install/setup.bash
   ```

## Usage

### 1. Basic simulation

Launch the simulation (Gazebo with a2rl_track world, bridge, robot_state_publisher, and RViz):

```bash
ros2 launch mav_bringup mav_init.launch.py
```

The X3 model is included in the world file and will appear in the scene. Gazebo Sim and RViz will start; ensure the world is fully loaded before sending commands.

### 2. Controlling the drone

Use Keboard Teleop using the gazebo gui.
Send velocity commands on the bridged topic:

```bash
# Example: forward and a bit of yaw
ros2 topic pub -r 10 /X3/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

The bridge is configured in `mav_gazebo/config/gz_bridge.yaml` (e.g. `X3/cmd_vel`). Add more topic bridges there if needed.

### 3. Choosing a world

To use a different world file (e.g. `x3_illini_pavilion.world`):

```bash
ros2 launch mav_bringup mav_init.launch.py world_name:=x3_illini_pavilion.world
```

Only one world is loaded (the one you pass); the server loads that file. If the Gazebo GUI shows or requests a different world name (e.g. `a2rl_track_ign`), it is using a **cached config** from a previous run. Clear it so the GUI matches the loaded world:

```bash
rm -f ~/.ignition/gazebo/gui.config
# or for Ignition Gazebo 6:
rm -f ~/.ignition/gazebo/*.config
```

Then launch again with the desired `world_name`.

### 4. Spawning a different model at runtime

To spawn another (or the same) model at runtime instead of relying on the world file:

1. Start the simulation (e.g. `ros2 launch mav_bringup mav_init.launch.py`).
2. After the world is loaded, use the Gazebo Sim `create` service as needed.

Use the same world name as in your launch file. Add corresponding bridges in `gz_bridge.yaml` if you want ROS topics for the new entity (e.g. `MyDrone/cmd_vel`).

## Package layout

| Package                   | Description                                                                                                     |
| ------------------------- | --------------------------------------------------------------------------------------------------------------- |
| **mav_bringup**     | Main launch file `mav_init.launch.py`: Gazebo (a2rl_track world), ros_gz_bridge, robot_state_publisher, RViz. |
| **mav_description** | Robot description: X3 model (SDF, URDF/xacro), meshes, and RViz config (`rviz/mav.rviz`).                     |
| **mav_gazebo**      | Gazebo assets: worlds (`a2rl_track.world`), gate models, and bridge config (`config/gz_bridge.yaml`).       |

World and model paths are set via `GZ_SIM_RESOURCE_PATH` in the launch file so that `model://` URIs for the X3 and gate models resolve correctly.
