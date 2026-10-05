"""Serpenoid gait: theta_i(t) = A * sin(phi(t) + i * beta) + gamma, i = 0 (joint1, head) .. 5 (joint6).

Publishes std_msgs/Float64MultiArray on /snake_position_controller/commands, the same topic as the
ros2_control position controller (mock / Gazebo) and dxl_node (real servos).

Parameters (all can be changed live with `ros2 param set /serpenoid <name> <value>`):
  amplitude     A, rad              (default 0.3)
  frequency     Hz, phi' = 2*pi*f   (default 0.5)
  phase_offset  beta, rad per joint (default -pi/3: the wave travels head -> tail, i.e. forward)
  turn_bias     gamma, rad          (default 0; the sign follows the joint direction)
  joint_limit   rad, every command is clamped to +/- this (default 1.0; dxl_node clamps at 60 deg too)
  ramp_s        seconds to fade A and gamma in at start, on changes, and out on Ctrl-C (default 3)
  rate_hz       publish rate (default 50)

Nothing jumps: it waits for /joint_states and blends from the actual pose into the gait over ramp_s;
amplitude and bias move toward their targets at a rate set by ramp_s; frequency changes keep the phase
continuous. Ctrl-C fades to the straight pose, then exits (a second
Ctrl-C exits at once). The servos keep holding the last pose; torque off is separate (dxl_node
~/enable false, or scripts/torque_off.py).
"""
import math
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

N_JOINTS = 6


class Serpenoid(Node):
    def __init__(self):
        super().__init__('serpenoid')
        for name, default in (('amplitude', 0.3), ('frequency', 0.5), ('phase_offset', -math.pi / 3),
                              ('turn_bias', 0.0), ('joint_limit', 1.0), ('ramp_s', 3.0), ('rate_hz', 50.0)):
            self.declare_parameter(name, default)
        self.pub = self.create_publisher(Float64MultiArray, '/snake_position_controller/commands', 10)
        self.dt = 1.0 / self.p('rate_hz')
        self.phase = 0.0
        self.amp = 0.0          # current (ramped) amplitude
        self.bias = 0.0         # current (ramped) turn bias
        self.stopping = False
        self.done = False
        self.q_start = None     # actual pose when the gait starts, blended out over ramp_s
        self.blend = 0.0        # 0 = q_start, 1 = pure gait
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info('waiting for /joint_states, then blending into the gait. Ctrl-C fades to straight.')

    def on_js(self, msg):
        if self.q_start is None and len(msg.position) >= N_JOINTS:
            self.q_start = list(msg.position[:N_JOINTS])
            self.get_logger().info('start pose (deg): ' + ' '.join(f'{math.degrees(q):+.1f}' for q in self.q_start))

    def p(self, name):
        return float(self.get_parameter(name).value)

    def approach(self, cur, target, full_scale):
        step = full_scale * self.dt / max(self.p('ramp_s'), 1e-3)
        return cur + max(-step, min(step, target - cur))

    def step(self):
        if self.q_start is None:
            if self.stopping:
                self.done = True
            return
        lim = self.p('joint_limit')
        self.blend = min(1.0, self.blend + self.dt / max(self.p('ramp_s'), 1e-3))
        amp_t = 0.0 if self.stopping else min(abs(self.p('amplitude')), lim)
        bias_t = 0.0 if self.stopping else max(-lim, min(lim, self.p('turn_bias')))
        self.amp = self.approach(self.amp, amp_t, max(abs(self.p('amplitude')), 0.3))
        self.bias = self.approach(self.bias, bias_t, max(abs(self.p('turn_bias')), 0.3))
        self.phase = (self.phase + 2 * math.pi * self.p('frequency') * self.dt) % (2 * math.pi)
        beta = self.p('phase_offset')
        w = self.blend
        q = [max(-lim, min(lim, (1 - w) * self.q_start[i]
                           + w * (self.amp * math.sin(self.phase + i * beta) + self.bias)))
             for i in range(N_JOINTS)]
        self.pub.publish(Float64MultiArray(data=q))
        if self.stopping and self.amp == 0.0 and self.bias == 0.0 and w == 1.0:
            self.get_logger().info('straight; stopping')
            self.done = True


def main():
    rclpy.init(args=None, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Serpenoid()

    def on_sigint(*_):
        if node.stopping:
            node.done = True
        node.stopping = True
    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
