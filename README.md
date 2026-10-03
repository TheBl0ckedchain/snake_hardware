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
ros2 launch snake_bringup lidar.launch.py   # lidar → /scan
python3 scripts/scan_check.py --out scan.png   # scan stats + top-down PNG (headless)
```

## Layout
- `scripts/slither.py`: standalone sinusoidal slithering gait (uses `dynamixel-sdk`, venv at `~/snake_env`)
- `scripts/scan_check.py`: lidar quality check
- `ros2_ws/src/snake_bringup`: launch files and configs
- `udev/`: stable device names
