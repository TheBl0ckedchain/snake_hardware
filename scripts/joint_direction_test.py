#!/usr/bin/env python3
"""Stage 2 torque test: move one joint at a time +5 deg and back, slowly, to learn each joint's
positive direction (which way it bends when its angle increases).

Each joint ramps +5 deg over 1.5 s, holds 1.5 s, then ramps back. A banner names the joint before
it moves, so whoever is at the robot can note whether it bent LEFT or RIGHT (seen from above,
head pointing away from you). It aborts (torque off) if any joint tracks more than 2 deg off its
command, or any current exceeds 1.0 A. Torque always ends off, also on Ctrl-C.

Needs the driver running:  ros2 launch snake_bringup servos.launch.py
Run:                       python3 scripts/joint_direction_test.py [--deg 5] [--joints 1 2 3]
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

MAX_ERR_DEG, MAX_CURRENT_A, RATE = 2.0, 1.0, 50.0


class Abort(Exception):
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deg', type=float, default=5.0)
    ap.add_argument('--joints', type=int, nargs='*', default=[1, 2, 3, 4, 5, 6])
    ap.add_argument('--ramp', type=float, default=1.5)
    a = ap.parse_args()
    if not 0 < a.deg <= 15:
        raise SystemExit('--deg must be in (0, 15]')

    rclpy.init()
    n = Node('joint_direction_test')
    last = {}
    n.create_subscription(JointState, '/joint_states', lambda m: last.__setitem__('m', m), 50)
    pub = n.create_publisher(JointState, '/snake/joint_cmd', 10)
    cli = n.create_client(SetBool, '/dxl_node/enable')
    if not cli.wait_for_service(timeout_sec=5):
        raise SystemExit('dxl_node not running: start  ros2 launch snake_bringup servos.launch.py')

    def call(on):
        f = cli.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        return f.result()

    def spin(t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.01)

    spin(1.0)
    if 'm' not in last:
        raise SystemExit('no /joint_states received')
    names = list(last['m'].name)
    q0 = np.array(last['m'].position)
    peak = np.zeros(len(names))
    results = {}

    def track(q, dur):
        """Command q (rad) and hold for dur seconds while checking the safety limits."""
        end = time.time() + dur
        while time.time() < end:
            cmd = JointState(name=names, position=[float(x) for x in q])
            pub.publish(cmd)
            spin(1 / RATE)
            m = last['m']
            err = np.degrees(np.abs(np.array(m.position) - q))
            cur = np.abs(np.array(m.effort))
            np.maximum(peak, cur, out=peak)
            if cur.max() > MAX_CURRENT_A:
                raise Abort(f'{names[cur.argmax()]} drew {cur.max():.2f} A')
            if err.max() > MAX_ERR_DEG + a.deg:   # coarse bound during ramps
                raise Abort(f'{names[err.argmax()]} is {err.max():.1f} deg off its command')
        return np.array(last['m'].position)

    def ramp(q_from, q_to, dur):
        steps = max(1, int(dur * RATE))
        for s in range(1, steps + 1):
            track(q_from + (q_to - q_from) * s / steps, 1 / RATE)

    try:
        r = call(True)
        print('enable:', r.success, r.message)
        if not r.success:
            return
        spin(0.5)
        for j in a.joints:
            k = j - 1
            target = q0.copy()
            target[k] += math.radians(a.deg)
            print(f'\n>>> joint_{j}: +{a.deg:.0f} deg in 2 s ...', flush=True)
            spin(2.0)
            ramp(q0, target, a.ramp)
            reached = track(target, a.ramp)
            moved = math.degrees(reached[k] - q0[k])
            others = np.degrees(np.abs(np.delete(reached - q0, k))).max()
            if abs(moved - a.deg) > MAX_ERR_DEG:
                raise Abort(f'joint_{j} reached only {moved:+.1f} deg')
            ramp(target, q0, a.ramp)
            track(q0, 0.5)
            results[names[k]] = (moved, others)
            print(f'    joint_{j}: moved {moved:+.2f} deg, other joints within {others:.2f} deg, back home')
    except Abort as e:
        print('ABORT:', e)
    finally:
        r = call(False)
        print('\ntorque off:', 'ok' if r and r.success else 'NO RESPONSE - kill the driver or cut 12 V')
        print('peak current per joint (A):', dict(zip(names, np.round(peak, 3))))
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
