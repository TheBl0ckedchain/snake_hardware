"""Serpenoid gait: theta_i(t) = A_i * sin(phi(t) + i * beta) + gamma_i, i = 0 (joint1, head) .. 5 (joint6).

Publishes std_msgs/Float64MultiArray on /snake_position_controller/commands, the same topic as the
ros2_control position controller (mock / Gazebo) and dxl_node (real servos).

Parameters (all can be changed live with `ros2 param set /serpenoid <name> <value>`):
  amplitude      A, rad              (default 0.3)
  head_amplitude A_0 for the head joint, rad; < 0 means the same as amplitude (default -1)
  frequency      Hz, phi' = 2*pi*f   (default 0.5)
  phase_offset   beta, rad per joint (default -pi/3: the wave travels head -> tail, i.e. forward)
  turn_bias      gamma, rad          (default 0; the sign follows the joint direction)
  bias_mode      'all': gamma on every joint (arc turn); 'head': on the head joint only (as the NN sim)
  joint_limit    rad, every command is clamped to +/- this (default 1.0; dxl_node clamps at 60 deg too)
  ramp_s         seconds to fade A and gamma in at start, on changes, and out on Ctrl-C (default 3)
  rate_hz        publish rate (default 50)

Steering by topic (keyboard teleop, NN controller): with use_cmd_vel:=true the gait follows
geometry_msgs/Twist on /cmd_vel instead of the frequency/turn_bias parameters:
  linear.x  in [-1, 1] -> frequency = linear.x * max_frequency (negative runs the wave backwards)
  angular.z in [-1, 1] -> turn_bias = angular.z * max_turn_bias
If no /cmd_vel arrives for cmd_timeout_s, the frequency goes to 0: the wave freezes in place.

Nothing jumps: it waits for /joint_states and blends from the actual pose into the gait over ramp_s;
amplitude and bias move toward their targets at a rate set by ramp_s; frequency changes keep the phase
continuous. Ctrl-C fades to the straight pose, then exits (a second
Ctrl-C exits at once). The servos keep holding the last pose; torque off is separate (dxl_node
~/enable false, or scripts/torque_off.py).
"""
import math
import signal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

N_JOINTS = 6


def clamp(x, lim):
    return max(-lim, min(lim, x))


class Serpenoid(Node):
    def __init__(self):
        super().__init__('serpenoid')
        for name, default in (('amplitude', 0.3), ('head_amplitude', -1.0), ('frequency', 0.5),
                              ('phase_offset', -math.pi / 3), ('turn_bias', 0.0), ('bias_mode', 'all'),
                              ('joint_limit', 1.0), ('ramp_s', 3.0), ('rate_hz', 50.0),
                              ('use_cmd_vel', False), ('max_frequency', 0.6), ('max_turn_bias', 0.3),
                              ('cmd_timeout_s', 0.5)):
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
        self.cmd = None         # last /cmd_vel (linear.x, angular.z)
        self.cmd_time = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(self.dt, self.step)
        mode = 'following /cmd_vel' if self.get_parameter('use_cmd_vel').value else 'using parameters'
        self.get_logger().info(f'{mode}; waiting for /joint_states, then blending into the gait. '
                               'Ctrl-C fades to straight.')

    def on_js(self, msg):
        if self.q_start is None and len(msg.position) >= N_JOINTS:
            self.q_start = list(msg.position[:N_JOINTS])
            self.get_logger().info('start pose (deg): ' + ' '.join(f'{math.degrees(q):+.1f}' for q in self.q_start))

    def on_cmd_vel(self, msg):
        self.cmd = (clamp(msg.linear.x, 1.0), clamp(msg.angular.z, 1.0))
        self.cmd_time = self.get_clock().now()

    def p(self, name):
        return float(self.get_parameter(name).value)

    def approach(self, cur, target, full_scale):
        step = full_scale * self.dt / max(self.p('ramp_s'), 1e-3)
        return cur + max(-step, min(step, target - cur))

    def targets(self):
        """(frequency Hz, turn bias rad) from /cmd_vel or from the parameters."""
        if not self.get_parameter('use_cmd_vel').value:
            return self.p('frequency'), self.p('turn_bias')
        fresh = (self.cmd_time is not None and
                 (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9 < self.p('cmd_timeout_s'))
        if not fresh:
            return 0.0, self.bias   # freeze the wave and hold the bias
        return self.cmd[0] * self.p('max_frequency'), self.cmd[1] * self.p('max_turn_bias')

    def step(self):
        if self.q_start is None:
            if self.stopping:
                self.done = True
            return
        lim = self.p('joint_limit')
        freq, bias_cmd = self.targets()
        self.blend = min(1.0, self.blend + self.dt / max(self.p('ramp_s'), 1e-3))
        amp_t = 0.0 if self.stopping else min(abs(self.p('amplitude')), lim)
        bias_t = 0.0 if self.stopping else clamp(bias_cmd, lim)
        bias_scale = max(abs(bias_cmd), 0.3)
        self.amp = self.approach(self.amp, amp_t, max(abs(self.p('amplitude')), 0.3))
        self.bias = self.approach(self.bias, bias_t, bias_scale)
        self.phase = (self.phase + 2 * math.pi * freq * self.dt) % (2 * math.pi)
        beta = self.p('phase_offset')
        # The head amplitude scales with the ramped body amplitude so both fade in and out together.
        a_body = max(abs(self.p('amplitude')), 1e-6)
        a_head = self.p('head_amplitude')
        head_ratio = 1.0 if a_head < 0 else min(a_head, lim) / a_body
        head_only = self.get_parameter('bias_mode').value == 'head'
        w = self.blend
        q = []
        for i in range(N_JOINTS):
            amp = self.amp * (head_ratio if i == 0 else 1.0)
            bias = self.bias if (i == 0 or not head_only) else 0.0
            gait = amp * math.sin(self.phase + i * beta) + bias
            q.append(clamp((1 - w) * self.q_start[i] + w * gait, lim))
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
