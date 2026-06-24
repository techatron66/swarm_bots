# DNA Swarm Bot

Standalone multi-robot swarm package. Two bots drive in formation, overtake on button press, and stop for walls.

**Compatible with ROS 2 Jazzy + Gazebo Harmonic.**

**Now with visible ArUco markers in simulation** — the Gazebo camera sees real marker textures on both robots and walls, so the same `marker_detector.py` runs in both sim and real hardware. Seamless sim-to-real transfer.

## Hardware

| Part | Spec |
|------|------|
| Chassis | 2WD kit |
| Motors | TT DC (no encoders) |
| Motor Driver | L293D + Arduino |
| Compute | Jetson Orin Nano |
| Camera | USB Webcam |
| IMU | MPU6050 |
| Markers | Printed ArUco 6×6 |

## Marker IDs

| ID | Purpose | Location |
|----|---------|----------|
| 0 | Bot 0 | Front face |
| 1 | Bot 1 | Front face |
| 10, 11, 12 | Wall / obstacle | On walls |

## Prerequisites

### ROS 2 Jazzy
Follow the official ROS 2 Jazzy installation guide.

### Gazebo Harmonic
```bash
sudo apt install ros-jazzy-ros-gz gz-harmonic
```

### Build Tools
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

## Building

```bash
cd ~/swarm_bot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Running

### Simulation (with real vision!)
```bash
ros2 launch dna_swarm_bot launch_sim.launch.py
```
This spawns:
- 2 robots with **visible ArUco marker textures** on their fronts
- 3 **wall marker models** at x=3.0, 6.0, 9.0 with visible textures
- Real `marker_detector.py` processing actual Gazebo camera feed
- `swarm_coordinator.py` using actual vision detections

Trigger overtake:
```bash
ros2 run dna_swarm_bot overtake_trigger
# Press [o] to trigger overtake
```

### Real Bot
```bash
ros2 launch dna_swarm_bot camera.launch.py
ros2 launch dna_swarm_bot launch_robot.launch.py
```

## Architecture

```
Camera → marker_detector → detected_bots / detected_walls
                                    ↓
IMU → imu_odometry → odom ─────→ swarm_coordinator → cmd_vel → arduino_bridge → Arduino → L293D → Motors
                                    ↑
Other bots' poses (WiFi DDS) ──────┘
```

## Simulation vs Real — Same Code Path

| Component | Simulation | Real Hardware |
|-----------|-----------|---------------|
| Vision | `marker_detector.py` on Gazebo camera feed | `marker_detector.py` on webcam feed |
| Odometry | Gazebo diff-drive plugin | `imu_odometry.py` (MPU6050) |
| Motor control | Gazebo physics | `arduino_bridge.py` → L293D |
| Swarm logic | `swarm_coordinator.py` | `swarm_coordinator.py` |

**The only difference is the odometry source.** Everything else is identical.

## Parameters

Edit `config/params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `base_speed` | 0.2 | Cruising speed m/s |
| `formation_spacing` | 0.30 | Gap between bots m |
| `wall_stop_distance` | 0.50 | Emergency stop distance m |
| `overtake_fast_speed` | 0.35 | Passing speed m/s |
| `overtake_side` | 1.0 | 1.0=left, -1.0=right |

## Troubleshooting

### `gazebo_ros` not found
ROS 2 Jazzy uses **Gazebo Harmonic**, not Gazebo Classic. The package is now `ros_gz_sim` instead of `gazebo_ros`. This workspace is already updated for Harmonic.

### `colcon` not found
```bash
sudo apt install python3-colcon-common-extensions
```

### `rosdep` not found
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## File Structure

```
dna_swarm_bot/
├── dna_swarm_bot/              ← Python nodes
│   ├── arduino_bridge.py
│   ├── imu_odometry.py
│   ├── marker_detector.py      ← Same in sim and real!
│   ├── swarm_coordinator.py
│   ├── sim_detector.py         ← Ground-truth fallback (optional)
│   └── overtake_trigger.py
├── arduino/
│   └── motor_controller.ino
├── description/
│   ├── robot_core.xacro        ← Bot with visible ArUco texture
│   ├── robot.urdf.xacro
│   ├── camera.xacro
│   ├── gazebo_control.xacro
│   ├── textures/               ← ArUco PNGs for bot markers
│   └── materials/scripts/      ← Ogre material scripts
├── models/                     ← Wall marker Gazebo models
│   ├── wall_marker_10/
│   ├── wall_marker_11/
│   └── wall_marker_12/
├── launch/
│   ├── launch_sim.launch.py    ← Gazebo Harmonic compatible
│   ├── launch_robot.launch.py
│   ├── camera.launch.py
│   └── teleop.launch.py
├── config/
│   ├── params.yaml
│   └── view_bot.rviz
├── worlds/
│   └── swarm_world.sdf         ← Gazebo Harmonic SDF
└── scripts/
    ├── generate_markers.py
    └── jetson_setup.sh
```
