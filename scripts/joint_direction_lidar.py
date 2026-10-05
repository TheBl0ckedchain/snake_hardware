#!/usr/bin/env python3
"""Check the real direction of joint 1 against the URDF, using the lidar on the head.

URDF: joint axes are +z, so +angle turns the child (seg1) counter-clockwise relative to the head.
Equivalently, the head turns clockwise relative to the body. If the body stays put, the head yaws
by -angle, and the world appears rotated by +angle in the laser frame (the scene shift is +).
A scene shift of the opposite sign means the servo is mirrored relative to the URDF.

Sequence: baseline scans -> torque on (hold) -> joint1 +DEG over 1.5 s -> hold and scan -> back -> torque off.
Aborts (torque off) if any current exceeds 1.0 A. Needs robot.launch.py running (lidar + dxl_node).

    python3 scripts/joint_direction_lidar.py [--deg 10]
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

MAX_CURRENT_A, RATE = 1.0, 50.0


def scene_shift_deg(scans0, scans1):
    """Rotation delta (deg) that best maps scans0 onto scans1: r1(theta) = r0(theta - delta)."""
    def median_scan(scans):
        r = np.array([s.ranges for s in scans], dtype=float)
        r[(r < 0.05) | ~np.isfinite(r)] = np.nan
        return np.nanmedian(r, axis=0)
    m = scans0[0]
    ang = np.degrees(m.angle_min + np.arange(len(m.ranges)) * m.angle_increment)
    r0, r1 = median_scan(scans0), median_scan(scans1)
    ok0 = ~np.isnan(r0)
    best = (np.inf, 0.0)
    for d in np.arange(-20, 20.01, 0.25):
        r0s = np.interp(ang - d, ang[ok0], r0[ok0], left=np.nan, right=np.nan)
        diff = np.abs(r1 - r0s)
        diff = diff[~np.isnan(diff)]
        if diff.size > 100:
            best = min(best, (float(np.median(diff)), float(d)))
    return best[1], best[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deg', type=float, default=10.0)
    a = ap.parse_args()
    if not 0 < a.deg <= 15:
        raise SystemExit('--deg must be in (0, 15]')

    rclpy.init()
    n = Node('joint_direction_lidar')
    last, scans = {}, []
    n.create_subscription(JointState, '/joint_states', lambda m: last.__setitem__('js', m), 50)
    n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)
    pub = n.create_publisher(Float64MultiArray, '/snake_position_controller/commands', 10)
    cli = n.create_client(SetBool, '/dxl_node/enable')
    if not cli.wait_for_service(timeout_sec=5):
        raise SystemExit('dxl_node not running: start  ros2 launch snake_bringup robot.launch.py')

    def call(on):
        f = cli.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        return f.result()

    peak = [0.0]

    def run(q, dur):
        end = time.time() + dur
        while time.time() < end:
            if q is not None:
                pub.publish(Float64MultiArray(data=[float(x) for x in q]))
            rclpy.spin_once(n, timeout_sec=1 / RATE)
            cur = max(abs(c) for c in last['js'].effort)
            peak[0] = max(peak[0], cur)
            if cur > MAX_CURRENT_A:
                raise RuntimeError(f'current {cur:.2f} A')

    def ramp(q_from, q_to, dur):
        steps = int(dur * RATE)
        for s in range(1, steps + 1):
            run(q_from + (q_to - q_from) * s / steps, 1 / RATE)

    def collect(sec):
        scans.clear()
        run(None, sec)
        return list(scans)

    end = time.time() + 3
    while time.time() < end or 'js' not in last:
        rclpy.spin_once(n, timeout_sec=0.05)
    q0 = np.array(last['js'].position)
    target = q0.copy()
    target[0] += math.radians(a.deg)
    try:
        base = collect(1.5)
        r = call(True)
        print('enable:', r.success, r.message)
        if not r.success:
            return
        ramp(q0, target, 1.5)
        run(target, 0.5)
        moved_scans = collect(1.5)
        j1 = math.degrees(last['js'].position[0] - q0[0])
        ramp(target, q0, 1.5)
        run(q0, 0.5)
    except RuntimeError as e:
        print('ABORT:', e)
        return
    finally:
        r = call(False)
        print('torque off:', 'ok' if r and r.success else 'NO RESPONSE - run scripts/torque_off.py')
        print(f'peak current: {peak[0]:.3f} A')
    shift, err = scene_shift_deg(base, moved_scans)
    print(f'joint1 moved {j1:+.1f} deg (encoder); scene shift in laser frame {shift:+.2f} deg '
          f'(median residual {err * 100:.1f} cm, {len(base)}/{len(moved_scans)} scans)')
    if abs(shift) < 1.0:
        print('INCONCLUSIVE: the head barely turned (the body moved instead).')
    elif shift > 0:
        print('MATCHES URDF: +angle turns seg1 counter-clockwise relative to the head.')
    else:
        print('MIRRORED: the servo turns opposite to the URDF +z axis.')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
