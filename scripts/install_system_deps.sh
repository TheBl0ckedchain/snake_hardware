#!/usr/bin/env bash
# One-shot root setup for the snake robot (run in your own SSH terminal: it needs sudo).
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"

# Re-install udev rules (picks up rule fixes, e.g. FTDI latency timer).
"$HERE/udev/install.sh"
cat /sys/bus/usb-serial/devices/$(basename "$(readlink -f /dev/dynamixel)")/latency_timer

# Finish any interrupted dpkg/apt run first (e.g. a dropped SSH session), or apt refuses to install.
sudo dpkg --configure -a
sudo apt-get install -f -y

sudo apt-get update
sudo apt-get install -y \
  python3-pil \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-laser-filters \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-cartographer-ros \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-dynamixel-sdk
