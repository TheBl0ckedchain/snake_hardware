# snake_hardware

Hardware and perception stack for a planar (2D) snake robot that navigates an obstacle maze
point-to-point (the neural-network controller is developed separately).

- 6× Dynamixel XM430-W350 on a UART/TTL bus
- Raspberry Pi 5 (8 GB), Ubuntu 24.04, ROS 2 Jazzy
- YDLidar G2 (UART→USB)

See [docs/PLAN.md](docs/PLAN.md) for the roadmap.

## Layout
- `scripts/slither.py` — standalone sinusoidal slithering gait (uses `dynamixel-sdk`, venv at `~/snake_env`)
