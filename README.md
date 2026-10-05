# snake_hardware

Hardware and perception stack for a planar (2D) snake robot that navigates an obstacle maze
point-to-point (the neural-network controller is developed separately).

- 6× Dynamixel XM430-W350 on a UART/TTL bus (U2D2 → `/dev/dynamixel`)
- Raspberry Pi 5 (8 GB), Ubuntu 24.04, ROS 2 Jazzy
- YDLidar G2 (CP2102 UART→USB → `/dev/ydlidar`)

See [docs/PLAN.md](docs/PLAN.md) for the roadmap and status.

## Setup
```bash
scripts/install_system_deps.sh          # needs sudo: udev rules + apt packages
cd ros2_ws
vcs import src/third_party < deps.repos # or git clone the two repos listed there
./build.sh
source install/setup.bash
```

## Usage
```bash
ros2 launch snake_bringup robot.launch.py      # URDF/TF + servo driver (torque OFF) + lidar + Foxglove :8765
ros2 service call /dxl_node/enable std_srvs/srv/SetBool "{data: true}"   # energise (holds present pose)
ros2 run snakebot_gait serpenoid               # slither; Ctrl-C fades to straight
python3 scripts/torque_off.py                  # EMERGENCY: torque off on all servos, no ROS needed

ros2 launch snake_bringup lidar.launch.py      # lidar only → /scan
python3 scripts/scan_check.py --out scan.png   # scan stats + top-down PNG (headless)
python3 scripts/dxl_status.py                  # read-only servo health check
```
Commands (rad, joint1 → joint6) go to `/snake_position_controller/commands` (`std_msgs/Float64MultiArray`),
the same on the real robot (dxl_node) and on mock/Gazebo (ros2_control in snakebot_description).

## Layout
- `ros2_ws/src/snakebot_description`: URDF, meshes, ros2_control for mock/Gazebo (from the CAD export)
- `ros2_ws/src/snake_driver`: `dxl_node`, the real servo driver with safety trips (config/dxl.yaml)
- `ros2_ws/src/snakebot_gait`: serpenoid gait node
- `ros2_ws/src/snake_bringup`: launch files and configs
- `scripts/`: servo status/config/torque-off/tests, lidar check, standalone `slither.py`
- `udev/`: stable device names
