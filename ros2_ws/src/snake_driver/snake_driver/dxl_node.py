"""Dynamixel joint driver for the snake.

Starts with torque OFF and writes nothing to the servos until ~/enable is called with true.
It reads all joints each cycle and publishes /joint_states (position rad, velocity rad/s,
effort = motor current A).

Topics and services:
  pub  /joint_states        sensor_msgs/JointState
  pub  /diagnostics         diagnostic_msgs/DiagnosticArray (1 Hz: voltage, temperature, errors)
  sub  /snake_position_controller/commands  std_msgs/Float64MultiArray, 6 target positions in rad,
                            joint1 -> joint6. Same interface as the ros2_control JointGroupPositionController in
                            snakebot_description, so a gait or NN node runs unchanged on mock, Gazebo or this driver.
  sub  /snake/joint_cmd     sensor_msgs/JointState, target positions by name (or in joint order if names are empty).
  Commands are ignored while torque is off and clamped to the joint limits.
  srv  ~/enable             std_srvs/SetBool. true: apply the safety limits, set each goal to its present
                            position (so nothing jumps), then torque on. false: torque off.

Safety trips (all torque off, plus an error log): sustained over-current, over-temperature, any servo
hardware error, lost communication. In position mode a servo holds its last goal, so if commands stop
the snake simply freezes in place; it never runs away.
"""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

# XM430-W350 control table (protocol 2.0)
ADDR_PWM_LIMIT, ADDR_MAX_POS, ADDR_MIN_POS = 36, 48, 52
ADDR_TORQUE, ADDR_HW_ERROR = 64, 70
ADDR_PROFILE_ACCEL, ADDR_PROFILE_VEL, ADDR_GOAL_POS = 108, 112, 116
ADDR_PRESENT_CURRENT = 126   # 126 current (2), 128 velocity (4), 132 position (4)
ADDR_PRESENT_VOLTAGE = 144   # 144 voltage (2), 146 temperature (1)
CENTER = 2048
RAD_PER_TICK = 2 * math.pi / 4096
RAD_S_PER_UNIT = 0.229 * 2 * math.pi / 60   # velocity unit is 0.229 rpm
AMP_PER_UNIT = 0.00269
PWM_MAX = 885
HW_ERRORS = {0: 'input voltage', 2: 'overheating', 3: 'encoder', 4: 'electrical shock', 5: 'overload'}


def signed(v, bits):
    return v - (1 << bits) if v >= 1 << (bits - 1) else v


def le_bytes(v, n):
    return list(int(v).to_bytes(n, 'little', signed=False))


class DxlNode(Node):
    def __init__(self):
        super().__init__('dxl_node')
        p = self.declare_parameter
        self.ids = list(p('ids', [1, 2, 3, 4, 5, 6]).value)
        self.names = list(p('joint_names', [f'joint{i}' for i in self.ids]).value)
        port = p('port', '/dev/dynamixel').value
        baud = p('baudrate', 1000000).value
        rate = p('rate_hz', 100.0).value
        self.limit_rad = math.radians(p('joint_limit_deg', 90.0).value)
        self.profile_vel = round(p('profile_velocity_rpm', 30.0).value / 0.229)
        self.profile_accel = p('profile_accel', 50).value
        self.pwm_limit = round(PWM_MAX * p('pwm_limit_pct', 70.0).value / 100)
        self.current_limit = p('current_limit_a', 1.5).value
        self.current_time = p('current_limit_time_s', 1.0).value
        self.temp_limit = p('temp_limit_c', 70).value
        offsets = list(p('zero_offset_deg', [0.0] * len(self.ids)).value)
        if len(self.names) != len(self.ids) or len(offsets) != len(self.ids):
            raise ValueError('joint_names, zero_offset_deg and ids must have the same length')
        # Raw position (ticks) of each joint's straight pose; 2048 unless a horn sits off its spline zero.
        self.center = [CENTER + round(math.radians(o) / RAD_PER_TICK) for o in offsets]

        self.port, self.ph = PortHandler(port), PacketHandler(2.0)
        if not self.port.openPort() or not self.port.setBaudRate(baud):
            raise IOError(f'cannot open {port} @ {baud}')
        self.state_read = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_CURRENT, 10)
        self.diag_read = GroupSyncRead(self.port, self.ph, ADDR_PRESENT_VOLTAGE, 3)
        self.err_read = GroupSyncRead(self.port, self.ph, ADDR_HW_ERROR, 1)
        for i in self.ids:
            self.state_read.addParam(i)
            self.diag_read.addParam(i)
            self.err_read.addParam(i)
        self.goal_write = GroupSyncWrite(self.port, self.ph, ADDR_GOAL_POS, 4)

        self.enabled = False
        self.pending_goal = None          # ticks per joint, written on the next cycle
        self.over_current_since = {}      # id -> time the current first exceeded the limit
        self.read_failures = 0
        self.tick = 0
        self.diag_every = max(1, round(rate))  # ~1 Hz
        self.set_torque_all(False, check=False)  # in case a previous run left it on

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(JointState, '/snake/joint_cmd', self.on_cmd, 10)
        self.create_subscription(Float64MultiArray, '/snake_position_controller/commands', self.on_array_cmd, 10)
        self.create_service(SetBool, '~/enable', self.on_enable)
        # One timer does all bus I/O; the single-threaded executor serialises it with the callbacks.
        self.create_timer(1.0 / rate, self.loop)
        self.get_logger().info(f'{len(self.ids)} joints on {port} @ {baud}, torque OFF. '
                               f'Call ~/enable to energise.')

    # ---------------------------------------------------------------- bus helpers
    def write(self, size, dxl_id, addr, value):
        fn = {1: self.ph.write1ByteTxRx, 2: self.ph.write2ByteTxRx, 4: self.ph.write4ByteTxRx}[size]
        res, err = fn(self.port, dxl_id, addr, value)
        if res != COMM_SUCCESS:
            raise IOError(f'id {dxl_id} addr {addr}: {self.ph.getTxRxResult(res)}')

    def set_torque_all(self, on, check=True):
        for i in self.ids:
            try:
                self.write(1, i, ADDR_TORQUE, int(on))
            except IOError:
                if check:
                    raise
        self.enabled = on and check

    def trip(self, reason):
        self.get_logger().error(f'SAFETY TRIP: {reason}. All torque OFF.')
        self.set_torque_all(False, check=False)
        self.enabled = False
        self.pending_goal = None

    # ---------------------------------------------------------------- callbacks
    def on_enable(self, req, resp):
        try:
            if not req.data:
                self.set_torque_all(False)
                resp.success, resp.message = True, 'torque off'
                return resp
            positions = self.read_state()   # OSError from here is caught below
            if positions is None:
                raise IOError('cannot read joint positions')
            lim = round(self.limit_rad / RAD_PER_TICK)
            for i, pos, c in zip(self.ids, positions[0], self.center):
                if abs(pos - c) > lim:
                    raise ValueError(f'id {i} is at {math.degrees((pos - c) * RAD_PER_TICK):.0f} deg, '
                                     f'outside the +/-{math.degrees(self.limit_rad):.0f} deg limit')
            for i, pos, c in zip(self.ids, positions[0], self.center):
                self.write(1, i, ADDR_TORQUE, 0)               # EEPROM below needs torque off
                self.write(2, i, ADDR_PWM_LIMIT, self.pwm_limit)
                self.write(4, i, ADDR_MAX_POS, min(4095, c + lim))
                self.write(4, i, ADDR_MIN_POS, max(0, c - lim))
                self.write(4, i, ADDR_PROFILE_ACCEL, self.profile_accel)
                self.write(4, i, ADDR_PROFILE_VEL, self.profile_vel)
                self.write(4, i, ADDR_GOAL_POS, pos)           # hold where it is: no jump on enable
            self.set_torque_all(True)
            self.over_current_since.clear()
            resp.success, resp.message = True, 'torque on, holding present position'
        except (IOError, ValueError) as e:
            self.reset_port()
            self.set_torque_all(False, check=False)
            resp.success, resp.message = False, str(e)
        self.get_logger().info(f'enable({req.data}): {resp.message}')
        return resp

    def on_cmd(self, msg):
        if not self.enabled:
            return
        if msg.name:
            idx = {n: k for k, n in enumerate(msg.name)}
            if any(n not in idx for n in self.names):
                self.get_logger().warn('joint_cmd must name every joint; ignored', throttle_duration_sec=2.0)
                return
            q = [msg.position[idx[n]] for n in self.names]
        elif len(msg.position) == len(self.ids):
            q = list(msg.position)
        else:
            self.get_logger().warn('joint_cmd needs one position per joint; ignored', throttle_duration_sec=2.0)
            return
        self.set_goal(q)

    def on_array_cmd(self, msg):
        if not self.enabled:
            return
        if len(msg.data) != len(self.ids):
            self.get_logger().warn(f'commands need {len(self.ids)} values; ignored', throttle_duration_sec=2.0)
            return
        self.set_goal(list(msg.data))

    def set_goal(self, q):
        if any(not math.isfinite(x) for x in q):
            return
        self.pending_goal = [c + round(max(-self.limit_rad, min(self.limit_rad, x)) / RAD_PER_TICK)
                             for c, x in zip(self.center, q)]

    # ---------------------------------------------------------------- main loop
    def read_state(self):
        if self.state_read.txRxPacket() != COMM_SUCCESS:
            return None
        pos, vel, cur = [], [], []
        for i in self.ids:
            if not self.state_read.isAvailable(i, ADDR_PRESENT_CURRENT, 10):
                return None
            cur.append(signed(self.state_read.getData(i, 126, 2), 16))
            vel.append(signed(self.state_read.getData(i, 128, 4), 32))
            pos.append(signed(self.state_read.getData(i, 132, 4), 32))
        return pos, vel, cur

    def loop(self):
        try:
            self.cycle()
        except OSError as e:  # serial errors (unplugged adapter, another process on the port): never crash
            self.get_logger().error(f'bus error: {e}', throttle_duration_sec=2.0)
            self.reset_port()
            self.fail_read()

    def reset_port(self):
        # The SDK marks the port busy for the duration of a transfer; an exception mid-transfer leaves it
        # stuck busy and every later call fails with COMM_PORT_BUSY. Clear it and drop any partial input.
        self.port.is_using = False
        try:
            self.port.clearPort()
        except OSError:
            pass

    def fail_read(self):
        self.read_failures += 1
        if self.read_failures == 10:
            self.get_logger().error('lost contact with the servos (10 failed reads)')
            if self.enabled:
                self.trip('communication lost')

    def cycle(self):
        state = self.read_state()
        if state is None:
            self.fail_read()
            return
        self.read_failures = 0
        pos, vel, cur = state
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = self.names
        js.position = [(p - c) * RAD_PER_TICK for p, c in zip(pos, self.center)]
        js.velocity = [v * RAD_S_PER_UNIT for v in vel]
        js.effort = [c * AMP_PER_UNIT for c in cur]
        self.js_pub.publish(js)

        if self.enabled:
            t = time.monotonic()
            for i, amps in zip(self.ids, js.effort):
                if abs(amps) < self.current_limit:
                    self.over_current_since.pop(i, None)
                elif t - self.over_current_since.setdefault(i, t) > self.current_time:
                    self.trip(f'id {i} drew {abs(amps):.2f} A for over {self.current_time:.1f} s')
                    return
            if self.pending_goal is not None:
                self.goal_write.clearParam()
                for i, g in zip(self.ids, self.pending_goal):
                    self.goal_write.addParam(i, le_bytes(g, 4))
                self.goal_write.txPacket()
                self.pending_goal = None

        self.tick += 1
        if self.tick % self.diag_every == 0:
            self.diagnostics(now)

    def diagnostics(self, stamp):
        if self.diag_read.txRxPacket() != COMM_SUCCESS or self.err_read.txRxPacket() != COMM_SUCCESS:
            return
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        for i, name in zip(self.ids, self.names):
            volt = self.diag_read.getData(i, 144, 2) / 10
            temp = self.diag_read.getData(i, 146, 1)
            err = self.err_read.getData(i, ADDR_HW_ERROR, 1)
            errs = [s for b, s in HW_ERRORS.items() if err >> b & 1]
            st = DiagnosticStatus(name=f'dxl/{name}', hardware_id=f'dxl_id_{i}')
            st.values = [KeyValue(key='voltage_v', value=f'{volt:.1f}'),
                         KeyValue(key='temperature_c', value=str(temp)),
                         KeyValue(key='hw_error', value=','.join(errs) or 'none'),
                         KeyValue(key='torque', value='on' if self.enabled else 'off')]
            st.level, st.message = DiagnosticStatus.OK, 'ok'
            if errs:
                st.level, st.message = DiagnosticStatus.ERROR, 'hardware error: ' + ','.join(errs)
            elif temp >= self.temp_limit:
                st.level, st.message = DiagnosticStatus.ERROR, f'over temperature ({temp} C)'
            elif volt < 10.5:
                st.level, st.message = DiagnosticStatus.WARN, f'low voltage ({volt:.1f} V)'
            arr.status.append(st)
            if self.enabled and st.level == DiagnosticStatus.ERROR:
                self.trip(f'id {i} {st.message}')
        self.diag_pub.publish(arr)

    def destroy_node(self):
        self.set_torque_all(False, check=False)
        self.port.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    node = DxlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
