#!/usr/bin/env bash
# Build the snake ROS 2 workspace. Usage: ./build.sh [extra colcon args]
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
# YDLidar SDK is a plain CMake package; build it first so the driver can find it.
colcon build --packages-select ydlidar_sdk --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
source install/setup.bash
# The SDK's CMake config assumes /usr/local/lib, so point the linker (and rpath) at our install.
SDK_LIB="$PWD/install/ydlidar_sdk/lib"
colcon build --packages-skip ydlidar_sdk --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_EXE_LINKER_FLAGS=-L$SDK_LIB -Wl,-rpath,$SDK_LIB" "$@"
