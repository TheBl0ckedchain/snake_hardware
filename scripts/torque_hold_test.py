#!/usr/bin/env python3
"""Stage 1 torque test: enable torque, hold the present pose for 5 s, then torque off.

Nothing should move: on enable the driver sets each goal to the joint's present position.
Torque goes off at once if any joint drifts more than 2 deg, and always at the end (also on Ctrl-C).
Prints per-joint drift and motor current.

Needs the driver running in another terminal:  ros2 launch snake_bringup servos.launch.py
Then run:                                     python3 scripts/torque_hold_test.py
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

HOLD_S, MAX_DRIFT_DEG = 5.0, 2.0


def main():
    rclpy.init()
    n = Node('torque_hold_test')
    last = {}
    n.create_subscription(JointState, '/joint_states', lambda m: last.__setitem__('m', m), 50)
    cli = n.create_client(SetBool, '/dxl_node/enable')
    if not cli.wait_for_service(timeout_sec=5):
        raise SystemExit('dxl_node not running: start  ros2 launch snake_bringup servos.launch.py')

    def call(on):
        f = cli.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        return f.result()

    def sample(t):
        out, end = [], time.time() + t
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)
            if 'm' in last:
                out.append((np.degrees(last['m'].position), np.abs(last['m'].effort)))
        return out

    base = sample(1.0)
    if not base:
        raise SystemExit('no /joint_states received')
    names, p0 = last['m'].name, base[-1][0]
    print('pose before (deg):', np.round(p0, 1))
    try:
        r = call(True)
        print('enable:', r.success, r.message)
        if not r.success:
            return
        pos, cur, t0 = [], [], time.time()
        while time.time() - t0 < HOLD_S:
            for p, c in sample(0.1):
                pos.append(p)
                cur.append(c)
            if pos and np.max(np.abs(pos[-1] - p0)) > MAX_DRIFT_DEG:
                print(f'DRIFT > {MAX_DRIFT_DEG} deg -> aborting:', np.round(pos[-1] - p0, 1))
                break
        P, C = np.array(pos), np.array(cur)
        print(f"\n{'joint':>8} {'max drift':>10} {'mean I':>8} {'peak I':>8}")
        for k, name in enumerate(names):
            print(f'{name:>8} {np.max(np.abs(P[:, k] - p0[k])):>8.2f}deg '
                  f'{C[:, k].mean():>7.3f}A {C[:, k].max():>7.3f}A')
    finally:
        r = call(False)
        print('\ntorque off:', 'ok' if r and r.success else 'NO RESPONSE - kill the driver or cut 12 V')
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
