#!/bin/bash
set -e

echo "=== Building Swarm Bot Workspace ==="
cd "$(dirname "$0")"

if [ ! -d "src/dna_swarm_bot" ]; then
    echo "Error: src/dna_swarm_bot not found"
    exit 1
fi

# Check for ROS 2
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 not sourced. Run:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

if ! command -v rosdep &> /dev/null; then
    echo "Warning: rosdep not found. Install with:"
    echo "  sudo apt install python3-rosdep"
    echo "  sudo rosdep init && rosdep update"
fi

if ! command -v colcon &> /dev/null; then
    echo "Error: colcon not found. Install with:"
    echo "  sudo apt install python3-colcon-common-extensions"
    exit 1
fi

rosdep install --from-paths src --ignore-src -y || true
colcon build --symlink-install

echo ""
echo "Build complete. Source with:"
echo "  source install/setup.bash"
