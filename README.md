# Swarm Bot Workspace

A **standalone** ROS 2 workspace for multi-robot swarm behavior using camera-only perception.

## What's Inside

```
swarm_bot_ws/
├── src/
│   └── dna_swarm_bot/          ← The only package you need
│       ├── dna_swarm_bot/       ← Python nodes
│       ├── arduino/             ← Arduino firmware
│       ├── config/              ← Parameters + RViz
│       ├── description/         ← URDF / xacro
│       ├── launch/              ← Launch files
│       ├── worlds/              ← Gazebo world
│       └── scripts/             ← Helper scripts
```

**No dependency on articubot_one or any external repo.**

## Quick Start

### 1. Clone / Extract
```bash
cd ~
# Extract the zip or clone this repo
cd swarm_bot_ws
```

### 2. Install Dependencies
```bash
# On Jetson
bash src/dna_swarm_bot/scripts/jetson_setup.sh

# On desktop (for simulation)
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-teleop-twist-keyboard
```

### 3. Build
```bash
cd ~/swarm_bot_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

### 4. Run Simulation
```bash
ros2 launch dna_swarm_bot launch_sim.launch.py
```

In another terminal:
```bash
ros2 run dna_swarm_bot overtake_trigger
# Press [o] to trigger overtake
```

### 5. Run Real Hardware
See `src/dna_swarm_bot/README.md` for full hardware deployment.

## Requirements

- ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- Gazebo (simulation)
- Jetson Orin Nano + Arduino + L293D + MPU6050 + Webcam (hardware)

## License

MIT
