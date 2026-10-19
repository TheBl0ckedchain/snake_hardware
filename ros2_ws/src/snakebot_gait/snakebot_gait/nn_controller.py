"""Run the NEAT navigation policy (6_joint_snake, trained in pybullet) on the real robot.

    ros2 run snakebot_gait nn_controller          # + robot.launch.py, slam.launch.py
    ros2 run snakebot_gait serpenoid --ros-args -p use_cmd_vel:=true -p bias_mode:=head \
        -p max_turn_bias:=0.785 -p head_amplitude:=0.12 -p amplitude:=0.4

Then send a goal: a geometry_msgs/PoseStamped on /goal_pose (map frame; Foxglove's 3D panel can publish one).
Without serpenoid running this is a dry run: it only publishes /cmd_vel and its debug topics.

What the policy saw in training (6_joint_snake/train.py), and how it is rebuilt here:
  inputs 0..29  ranges along 30 WORLD-fixed directions, 0, 12, .., 348 deg, from the head, divided by
                3.0 m and capped at 1.0 (no hit). Here: points from the last memory_s of scans, put in the
                map frame with SLAM's map->laser, are binned by direction around the head (+-6 deg, closest
                point per bin). The memory also fills the lidar's blind rear sector while the head swings.
  inputs 30, 31 unit vector from the head to the goal, world frame.
  "World" here is a task frame: the map rotated so that the goal lay straight ahead (+y) from where the
  head was when the goal arrived, which is how every training episode started (goal at +y). The
  network is not rotation-invariant, so this keeps its inputs in the range it was trained on.
  output 0      gait speed. Sim: w in 0.4..2.0 rad per 1/24 s step (1.5..7.6 Hz at 1 rad, far past what
                the XM430s can follow). Here: /cmd_vel linear.x = speed_min..1, which serpenoid scales by
                its max_frequency.
  output 1      head bias phi0 = out * 45 deg on the head joint only. Here: /cmd_vel angular.z = out,
                with serpenoid's max_turn_bias = 0.785 rad and bias_mode = head.

Debug topics: /nn/scan (the 30 beams as a LaserScan in the head frame, for Foxglove),
/nn/observation (the 32 inputs), /nn/output (the 2 raw outputs).
"""
import math
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from snakebot_gait.neat_net import JsonNetwork

N_BEAMS = 30
BEAM_STEP = 2 * math.pi / N_BEAMS


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def lookup_2d(buf, target, source, stamp):
    """(x, y, yaw) of source in target at stamp, falling back to the latest transform."""
    try:
        t = buf.lookup_transform(target, source, stamp, timeout=Duration(seconds=0.05))
    except TransformException:
        t = buf.lookup_transform(target, source, Time())
    tr = t.transform
    return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)


class NnController(Node):
    def __init__(self):
        super().__init__('nn_controller')
        default_policy = os.path.join(get_package_share_directory('snakebot_gait'), 'config', 'nn_policy.json')
        for name, default in (('policy', default_policy), ('map_frame', 'map'), ('head_frame', 'base_link'),
                              ('range_norm', 3.0), ('memory_s', 2.0), ('speed_min', 0.5),
                              ('goal_radius', 0.3), ('align_task_frame', True)):
            self.declare_parameter(name, default)
        self.net = JsonNetwork(self.get_parameter('policy').value)
        self.get_logger().info(f"policy: genome {self.net.spec['genome_key']} "
                               f"({len(self.net.nodes)} nodes) from {self.get_parameter('policy').value}")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.goal = None            # (x, y) in the map frame
        self.task_yaw = 0.0         # map -> task frame rotation
        self.memory = []            # [(stamp_s, points Nx2 in map)]
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/nn/scan', 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, '/nn/observation', 10)
        self.out_pub = self.create_publisher(Float32MultiArray, '/nn/output', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.get_logger().info('waiting for a goal on /goal_pose (map frame)')

    def p(self, name):
        return self.get_parameter(name).value

    def head_pose(self, stamp):
        return lookup_2d(self.tf, self.p('map_frame'), self.p('head_frame'), stamp)

    def on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.p('map_frame'):
            self.get_logger().error(f"goal must be in {self.p('map_frame')}, got {msg.header.frame_id}")
            return
        try:
            hx, hy, _ = self.head_pose(Time())
        except TransformException as e:
            self.get_logger().error(f'no head pose in the map yet (is SLAM running?): {e}')
            return
        gx, gy = msg.pose.position.x, msg.pose.position.y
        # Rotate the map so the goal is along +y from the head, as at the start of every training run.
        self.task_yaw = math.atan2(gy - hy, gx - hx) - math.pi / 2 if self.p('align_task_frame') else 0.0
        self.goal = (gx, gy)
        self.get_logger().info(f'goal ({gx:.2f}, {gy:.2f}), {math.hypot(gx - hx, gy - hy):.2f} m away; '
                               f'task frame at {math.degrees(self.task_yaw):+.0f} deg from map')

    def remember(self, msg, stamp):
        lx, ly, lyaw = lookup_2d(self.tf, self.p('map_frame'), msg.header.frame_id, stamp)
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r)) + lyaw
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.01)) & (r <= msg.range_max)
        pts = np.stack([lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])], axis=1)
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.memory = [(t, p) for t, p in self.memory if now - t <= self.p('memory_s')] + [(now, pts)]

    def beams(self, hx, hy):
        """Normalized ranges along the 30 task-frame directions from the head, as perform_lidar_scan + filter_lidar."""
        norm = self.p('range_norm')
        out = np.ones(N_BEAMS)
        pts = np.concatenate([p for _, p in self.memory]) if self.memory else np.zeros((0, 2))
        if len(pts):
            d = pts - (hx, hy)
            dist = np.hypot(d[:, 0], d[:, 1])
            ang = np.arctan2(d[:, 1], d[:, 0]) - self.task_yaw
            k = np.round(ang / BEAM_STEP).astype(int) % N_BEAMS
            near = dist < norm
            np.minimum.at(out, k[near], dist[near] / norm)
        return out

    def stop(self):
        self.cmd_pub.publish(Twist())

    def on_scan(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        try:
            self.remember(msg, stamp)
            hx, hy, hyaw = self.head_pose(stamp)
        except TransformException as e:
            self.get_logger().warn(f'no map pose (is SLAM running?): {e}', throttle_duration_sec=5.0)
            return
        beams = self.beams(hx, hy)

        dbg = LaserScan()
        dbg.header.stamp = msg.header.stamp
        dbg.header.frame_id = self.p('head_frame')
        dbg.angle_min = self.task_yaw - hyaw
        dbg.angle_increment = BEAM_STEP
        dbg.angle_max = dbg.angle_min + BEAM_STEP * (N_BEAMS - 1)
        dbg.range_min = 0.0
        dbg.range_max = self.p('range_norm')
        dbg.ranges = [float(b * self.p('range_norm')) for b in beams]
        self.scan_pub.publish(dbg)

        if self.goal is None:
            return
        dx, dy = self.goal[0] - hx, self.goal[1] - hy
        dist = math.hypot(dx, dy)
        if dist <= self.p('goal_radius'):
            self.get_logger().info(f'goal reached ({dist:.2f} m); stopping')
            self.goal = None
            self.stop()
            return
        c, s = math.cos(-self.task_yaw), math.sin(-self.task_yaw)
        gx, gy = (c * dx - s * dy) / dist, (s * dx + c * dy) / dist
        obs = list(beams) + [gx, gy]
        out = self.net.activate(obs)
        self.obs_pub.publish(Float32MultiArray(data=[float(v) for v in obs]))
        self.out_pub.publish(Float32MultiArray(data=[float(v) for v in out]))

        cmd = Twist()
        speed = (max(-1.0, min(1.0, out[0])) + 1.0) / 2.0
        cmd.linear.x = self.p('speed_min') + speed * (1.0 - self.p('speed_min'))
        cmd.angular.z = max(-1.0, min(1.0, out[1]))
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = NnController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
