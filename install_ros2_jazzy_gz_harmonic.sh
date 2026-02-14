#!/usr/bin/env bash
set -euo pipefail

echo "==> Ubuntu info"
source /etc/os-release
echo "NAME=$NAME VERSION=$VERSION UBUNTU_CODENAME=$UBUNTU_CODENAME"

if [[ "${UBUNTU_CODENAME:-}" != "noble" ]]; then
  echo "WARNING: This script is intended for Ubuntu 24.04 (noble). Continuing anyway..."
fi

echo "==> Base deps"
sudo apt update
sudo apt install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release software-properties-common \
  build-essential cmake pkg-config git \
  python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep python3-vcstool \
  mesa-utils

echo "==> Enable universe"
sudo add-apt-repository -y universe

echo "==> Add ROS 2 apt repo + key"
sudo mkdir -p /usr/share/keyrings
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> Update apt and install ROS 2 Jazzy Desktop"
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-base

echo "==> Gazebo Harmonic + ROS<->GZ bridge + ros2_control integration"
# Gazebo Harmonic comes via ROS packages in Jazzy on Ubuntu 24.04
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers

echo "==> Helpful robotics tools (optional but recommended)"
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-tools \
  ros-jazzy-teleop-twist-keyboard || true

echo "==> rosdep init/update"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "==> Add ROS env to ~/.bashrc (only if missing)"
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "==> Done."
echo "Next:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  gz sim -v 3"
echo "  ros2 --help"
