#!/usr/bin/env bash
# Install the snake udev rules (needs sudo) and re-trigger devices.
set -euo pipefail
sudo cp "$(dirname "$0")/99-snake.rules" /etc/udev/rules.d/99-snake.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb-serial
ls -l /dev/dynamixel /dev/ydlidar
