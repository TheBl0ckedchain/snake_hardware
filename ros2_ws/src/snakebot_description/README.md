# snakebot_description

ROS 2 (Jazzy) description package for the 6 × XM430-W350 snake robot. Started from the Fusion export and then corrected.

## Frames and naming

The frames follow REP-103: `base_link` x points forward (toward the head), y points left, z points up.

| Joint | Parent → child | Servo ID |
|---|---|---|
| joint1 | base_link → seg1 | 1 (servo in the head) |
| joint2 | seg1 → seg2 | 2 |
| joint3 | seg2 → seg3 | 3 |
| joint4 | seg3 → seg4 | 4 |
| joint5 | seg4 → seg5 | 5 |
| joint6 | seg5 → tail | 6 |

Fixed frames on the head: `pi5`, `lidar`, `laser` (the scan plane).

## Build

```bash
cd ~/ros2_ws/src && unzip snakebot_description.zip
cd ~/ros2_ws && rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --packages-select snakebot_description && source install/setup.bash
```

## Run

| What | Command |
|---|---|
| Model + joint sliders | `ros2 launch snakebot_description display.launch.py` |
| Controllers, no hardware | `ros2 launch snakebot_description bringup.launch.py rviz:=true` |
| Real servos | `ros2 launch snake_bringup robot.launch.py` (uses `snake_driver/dxl_node`, see below) |
| Gazebo Harmonic | `ros2 launch snakebot_description gazebo.launch.py rviz:=true` |

To command joint angles (in radians), publish to the position controller:

```bash
ros2 topic pub /snake_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3,-0.3,0.3,-0.3,0.3,-0.3]}"
```

## Real hardware

- The real robot is driven by `snake_driver/dxl_node`, not ros2_control: it has over-current/temperature trips
  and starts with torque off. It listens on the same `/snake_position_controller/commands` topic, so gait and
  NN nodes work unchanged. The `hw:=real` block here (dynamixel_hardware plugin) is unused and not installed.
- Servo IDs are 1–6 from head to tail at 1 Mbps (`scripts/dxl_configure.py`).

## Values to check

All of these are properties at the top of `urdf/snakebot.urdf.xacro`.

| Property | Current value | What to do |
|---|---|---|
| `joint_limit` | ±90° | Set it to where the brackets actually collide |
| `lidar_scan_height` | 30 mm | Measure from the mount plate to the centre of the lidar's scan window |
| `lidar_yaw` | 0 | The G2's 0° is the cable outlet direction. Use `${pi}` if the cable points backwards |
| masses | weighed values | Update if you weigh again |

## Regenerating after a CAD change

Re-export the meshes in the same assembly pose and copy them into `meshes/`. Then run:

```bash
python3 tools/gen_core_xacro.py   # needs: pip install trimesh numpy
```

This rebuilds `urdf/snakebot_core.xacro` (links, joints, inertias). It finds each joint by locating the servo horn in the mesh.
