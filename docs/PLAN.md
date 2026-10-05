# Snake Robot — Hardware & Perception Plan

## Scope

A planar (2D) snake robot that must travel point-to-point through an obstacle-filled maze.
A neural-network controller is being developed separately. **This repo's job is to give that
controller a reliable robot:** a motion interface it can command, and a map plus a pose estimate
it can plan against.

### Hardware

| Part | Notes |
|---|---|
| 6× Dynamixel XM430-W350 | Daisy-chained TTL bus over UART (U2D2 or similar), Protocol 2.0, IDs 1–6 head→tail |
| Raspberry Pi 5, 8 GB | Ubuntu 24.04, ROS 2 Jazzy already installed |
| YDLidar G2 | 2D 360° lidar, about 0.12–12 m range, 5–12 Hz, UART→USB adapter (CP210x), 230400 baud |
| IMUs (optional) | Not used yet; recommended in Phase 3 (see below) |

## Key technical points to settle up front

1. **No odometry for free.** A snake has no wheel encoders, and its motion through the gait involves
   slip. So the pose must come mostly from the **lidar (scan matching)**, with an IMU to steady the
   heading. Odometry worked out from the gait itself can be a weak extra input at best.
2. **The lidar platform wiggles.** Lateral undulation swings every segment in yaw. If the lidar sits
   on the head, each scan is taken from a swinging, rotating base. The fix is to model the body
   (URDF plus live joint angles → TF), so every scan is placed correctly relative to a stable
   "virtual chassis" frame. Mount the lidar on a middle segment (3 or 4), where yaw swing is smallest.
3. **FAST-LIO / LIO-SAM do not apply here.** They are built for **3D** lidars (e.g. Livox, Ouster)
   paired with an IMU. With a 2D lidar, the right tools are **slam_toolbox** (the ROS 2 default, well
   maintained on Jazzy) or **Cartographer 2D**, which can fuse an IMU. The "LIO" idea (tightly
   fusing lidar with an IMU) is still worth borrowing, and Cartographer with an IMU is the 2D
   version of it.
4. **ROS 2 as the backbone.** It is already installed, and it gives us TF, rosbag recording, rviz
   visualisation and ready-made SLAM. The NN controller then just subscribes and publishes topics.
5. **Stable device names.** Both the U2D2 and the lidar adapter show up as `/dev/ttyUSB*`, and the
   numbering can swap between boots. udev rules → `/dev/dynamixel` and `/dev/ydlidar`.

## Phases

### Phase 0 — Groundwork
- [x] udev rules for `/dev/dynamixel` and `/dev/ydlidar` (`udev/`). FTDI latency timer → 1 ms.
- [ ] Separate power for the servos (12 V) and for the Pi/lidar. Common ground.
- [x] ROS 2 workspace in this repo (`ros2_ws/src/...`). Python deps pinned. `snake_env` venv kept for standalone scripts.

**Done when:** both devices enumerate under fixed names after a reboot.

### Phase 1 — Motion (✅ basic gait done: `scripts/slither.py`)
Next steps, to make it usable by SLAM and the NN:
- [x] Raise the Dynamixel baud rate (57600 → 1 Mbps or more), so the loop can read and write all 6 joints at 50 Hz or faster.
- [x] Add **GroupSyncRead** of present position (and current), so the real joint angles are known.
- [x] Add **steering**: a joint-angle offset (bias) gives turning. Gait parameters `amplitude`, `frequency`,
      `phase_offset` (wavelength and direction), `turn_bias` are live ROS parameters of `snakebot_gait/serpenoid`.
- [x] Wrap it in a ROS 2 node `snake_driver` (`dxl_node`):
      - subscribes to `/snake_position_controller/commands` (and `/snake/joint_cmd`) for direct joint targets
      - publishes `/joint_states`
      - safety: watchdog stop on command timeout, torque off on exit, joint limits, monitoring of temperature and current.
- [x] URDF for the snake (`snakebot_description`, from CAD) plus `robot_state_publisher` → live TF of every segment.
- [x] Joint zero offsets (all within ~1°) and collision angles measured (`scripts/joint_calibration.yaml`); limit set to ±60°.
- [ ] Measure the lidar scan height (placeholder 30 mm).
- [x] Joint 1 direction matches the URDF (+z), checked with the lidar (`scripts/joint_direction_lidar.py`, +9.9° joint → +6.75° scene shift). Joints 2-6 are the same part, mounted the same way.

**Done when:** a teleop/keyboard node can drive the snake forward and back and turn left and right, and rviz shows the body moving live.

### Phase 2 — Lidar
- [x] Build **YDLidar-SDK** and **ydlidar_ros2_driver** from source (no Jazzy binaries). Config for the G2: 230400 baud, single-channel off, 10 Hz, range 0.12–12 m.
- [x] Publish `/scan` in the `laser` frame (`ros2 launch snake_bringup lidar.launch.py`; static base_link→laser TF until the URDF exists), attached via the URDF.
- [ ] Bench validation: static scans of a known room. Check range accuracy (tape-measure checks), angular offset, noise, and min/max range. Filter self-hits from the snake's own body (`laser_filters` angular/box filter).
- [ ] Record rosbags: static, hand-carried, and on the moving snake.
- [ ] Check scan distortion from the wiggle: compare scans while slithering against static scans. If it is significant, deskew or transform each scan using the joint states at its timestamp.

**Done when:** a stable, correctly oriented `/scan` shows in rviz while the snake slithers, with no body self-hits.

### Phase 3 — Localisation & SLAM
Step by step, testing against recorded bags first:
1. [ ] **Virtual-chassis frame** (`base_link`): computed from the joint angles (centroid and principal axis of the body). It removes the gait oscillation, so SLAM sees smooth motion.
2. [ ] **Lidar odometry**: `rf2o_laser_odometry` → `odom → base_link`.
3. [ ] **SLAM**: `slam_toolbox` (online async) → `map → odom`, publishing `/map`.
4. [ ] Optional / likely needed: **IMU** on the chassis segment (e.g. BNO085 or ICM-20948 over I2C) + `robot_localization` EKF to fuse IMU yaw with the lidar odometry. Snakes yaw a lot, and a gyro steadies scan matching considerably.
5. [ ] Benchmark against **Cartographer 2D** (with IMU) on the same bags. Pick whichever drifts less.
6. [ ] Save and reload maps. Localisation-only mode (AMCL or slam_toolbox localisation) for repeat runs in a known maze.

**Done when:** mapping a test maze gives a consistent map (loops close, walls stay straight), and the pose error at known checkpoints is under about 5 cm.

### Phase 4 — Integration with the NN controller
Interface contract (agree this with the NN side early):

| Direction | Topic | Type |
|---|---|---|
| → NN | `/map` | `nav_msgs/OccupancyGrid` |
| → NN | `map → base_link` TF / `/pose` | robot pose |
| → NN | `/scan`, `/joint_states` | raw sensing |
| ← NN | `/snake/gait_cmd` or `/snake/joint_cmd` | motion commands |
| → NN | `/goal_pose` | target point |

- [ ] A sim-to-real check: run the same command sequence in sim and on the robot, and compare the trajectories.

## Status log
- 2026-10-04: lidar up. G2B fw 3.5, 9.7 Hz, 530 beams (0.68°), static noise ~3.5 mm median. The SDK occasionally aborts during startup intensity auto-detect, so the launch file respawns the driver.

## Immediate next steps
1. Plug in the U2D2 and the lidar. Write the udev rules.
2. Set up the ROS 2 workspace. Build the YDLidar SDK and driver. Get a first `/scan` in rviz (Phase 2 can run in parallel with Phase 1's ROS node).
3. Raise the servo baud rate and add sync-read to the driver.
