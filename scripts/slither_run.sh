#!/usr/bin/env bash
# Timed slither on the real robot, recorded to a rosbag. Torque is turned off at the end and on any exit.
#   scripts/slither_run.sh [amplitude_rad=0.2] [frequency_hz=0.5] [steady_s=10] [turn_bias_rad=0]
# Starts robot.launch.py itself (stop any other launch first). Bag: ~/bags/slither_<time>.
# Emergency stop from another terminal: python3 ~/snake_hardware/scripts/torque_off.py
set -um   # -m: each background job gets its own process group (pgid = $!), so stop_group reaches its children
A=${1:-0.2}; F=${2:-0.5}; STEADY=${3:-10}; BIAS=${4:-0}
HERE="$(cd "$(dirname "$0")/.." && pwd)"
set +u; source "$HERE/ros2_ws/install/setup.bash"; set -u   # colcon scripts read unset vars
BAG=~/bags/slither_$(date +%Y%m%d_%H%M%S); mkdir -p ~/bags
LOG=$(mktemp -d)
LPID= BPID= GPID=

stop_group() { [ -n "$1" ] && kill -TERM -- "-$1" 2>/dev/null; }
cleanup() {
  trap - EXIT INT TERM
  # the service call hangs if dxl_node has died, so it gets 5 s before the direct e-stop
  timeout 5 ros2 service call /dxl_node/enable std_srvs/srv/SetBool "{data: false}" 2>/dev/null | grep -q "success=True" \
    || python3 "$HERE/scripts/torque_off.py"
  stop_group "$GPID"; stop_group "$BPID"; sleep 2; stop_group "$LPID"; sleep 3
  python3 "$HERE/scripts/dxl_status.py" | tail -n 7
  echo "bag: $BAG   logs: $LOG"
}
trap cleanup EXIT INT TERM

ros2 launch snake_bringup robot.launch.py foxglove:=false > "$LOG/launch.log" 2>&1 & LPID=$!
sleep 12
ros2 bag record -s mcap -o "$BAG" /scan /joint_states /tf /tf_static /diagnostics \
  /snake_position_controller/commands > "$LOG/bag.log" 2>&1 & BPID=$!
sleep 3
ros2 service call /dxl_node/enable std_srvs/srv/SetBool "{data: true}" | grep -q "success=True" \
  || { echo "enable failed:"; tail -n 5 "$LOG/launch.log"; exit 1; }
echo "torque on; slithering A=$A rad f=$F Hz bias=$BIAS rad for 3 s ramp + ${STEADY} s"
"$HERE/ros2_ws/install/snakebot_gait/lib/snakebot_gait/serpenoid" --ros-args \
  -p amplitude:="$A" -p frequency:="$F" -p turn_bias:="$BIAS" > "$LOG/gait.log" 2>&1 & GPID=$!
# watch the driver: if it dies (e.g. the U2D2 drops off USB) the servos hold their last goal, so bail out
for _ in $(seq 1 $((2 * (3 + STEADY)))); do
  grep -q "dxl_node.*process has died" "$LOG/launch.log" && { echo "!!! dxl_node died; e-stop"; tail -n 3 "$LOG/launch.log"; exit 1; }
  sleep 0.5
done
kill -TERM "$GPID"   # fades to straight over ramp_s, then exits
for _ in $(seq 1 20); do kill -0 "$GPID" 2>/dev/null || break; sleep 0.5; done
GPID=
grep -E "TRIP|ERROR" "$LOG/launch.log" && echo "!!! driver reported a problem (see above)"
echo "done"
