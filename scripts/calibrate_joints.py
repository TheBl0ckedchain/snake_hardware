#!/usr/bin/env python3
"""Interactive, read-only joint calibration (torque stays off; nothing is written to the servos).

  1. ZERO:   lay the snake perfectly straight (e.g. against a straight edge), press Enter.
             Each joint's reading in that pose becomes its zero offset.
  2. LIMITS: bend joints by hand to their mechanical stops, both ways, then press Enter.
             A live line shows each joint's angle (relative to the new zero) and the min/max seen.

The results go to scripts/joint_calibration.yaml (and the terminal), for snake_driver's dxl.yaml.
Stop dxl_node / robot.launch.py first: this script needs the serial port to itself.

    python3 scripts/calibrate_joints.py
"""
import os
import select
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler

PORT, BAUD, IDS = '/dev/dynamixel', 1000000, [1, 2, 3, 4, 5, 6]
ADDR_TORQUE, ADDR_POS, CENTER, DEG_PER_TICK = 64, 132, 2048, 360 / 4096
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'joint_calibration.yaml')


def main():
    port, ph = PortHandler(PORT), PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(BAUD):
        sys.exit(f'cannot open {PORT} (is dxl_node running?)')
    reader = GroupSyncRead(port, ph, ADDR_POS, 4)
    for i in IDS:
        reader.addParam(i)
        torque, res, _ = ph.read1ByteTxRx(port, i, ADDR_TORQUE)
        if res != COMM_SUCCESS:
            sys.exit(f'id {i} not answering')
        if torque:
            sys.exit(f'id {i} has torque ON; turn it off first (scripts/torque_off.py)')

    def read_deg():
        if reader.txRxPacket() != COMM_SUCCESS:
            return None
        out = []
        for i in IDS:
            v = reader.getData(i, ADDR_POS, 4)
            out.append(((v - (1 << 32)) if v >= 1 << 31 else v) * DEG_PER_TICK - CENTER * DEG_PER_TICK)
        return out

    def enter_pressed():
        return select.select([sys.stdin], [], [], 0)[0] and sys.stdin.readline() is not None

    def fmt(vals):
        return ' '.join(f'j{i}:{v:+6.1f}' for i, v in zip(IDS, vals))

    try:
        print('STEP 1  Lay the snake perfectly straight, hands off, then press Enter.')
        while True:
            q = read_deg()
            if q:
                print('\r  raw  ' + fmt(q) + '   ', end='', flush=True)
            if enter_pressed():
                break
            time.sleep(0.05)
        samples = [s for s in (read_deg() for _ in range(20)) if s]
        zero = [sum(c) / len(c) for c in zip(*samples)]
        print('\n  zero offsets (deg): ' + fmt(zero))

        print('\nSTEP 2  Bend joints by hand to each mechanical stop, both ways (slowly, no force).')
        print('        Press Enter when done.')
        lo, hi = [0.0] * len(IDS), [0.0] * len(IDS)
        while True:
            q = read_deg()
            if q:
                rel = [a - z for a, z in zip(q, zero)]
                lo = [min(a, b) for a, b in zip(lo, rel)]
                hi = [max(a, b) for a, b in zip(hi, rel)]
                print('\r  now ' + fmt(rel) + '  | min ' + ' '.join(f'{v:+5.0f}' for v in lo)
                      + ' | max ' + ' '.join(f'{v:+5.0f}' for v in hi) + ' ', end='', flush=True)
            if enter_pressed():
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print('\naborted; nothing saved')
        return
    finally:
        port.closePort()

    with open(OUT, 'w') as f:
        f.write(f'# written by calibrate_joints.py on {time.strftime("%Y-%m-%d %H:%M")}; degrees, joints 1-6\n')
        f.write(f'zero_offset_deg: [{", ".join(f"{z:.2f}" for z in zero)}]\n')
        f.write(f'min_seen_deg: [{", ".join(f"{v:.1f}" for v in lo)}]\n')
        f.write(f'max_seen_deg: [{", ".join(f"{v:.1f}" for v in hi)}]\n')
    print(f'\n\nsaved {OUT}:')
    print(open(OUT).read())


if __name__ == '__main__':
    main()
