#!/bin/bash
# Jetson Orin Nano setup script for DNA Swarm Bot
set -e

echo "=== DNA Swarm Bot Jetson Setup ==="

# Enable I2C
sudo usermod -aG i2c $USER

# Install system deps
sudo apt update
sudo apt install -y \
    python3-pip python3-serial python3-opencv \
    i2c-tools v4l-utils \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-v4l2-camera \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-teleop-twist-keyboard

# Python deps
pip3 install --user smbus2 pyserial

# Udev rule for Arduino persistent naming
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino"' | sudo tee /etc/udev/rules.d/99-arduino.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Setup complete. Re-log or reboot for group changes to take effect."
