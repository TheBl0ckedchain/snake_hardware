#!/usr/bin/env python3
"""EMERGENCY STOP: torque off on all servos, straight over the bus (no ROS needed).

    python3 ~/snake_hardware/scripts/torque_off.py

Works even while dxl_node is running (the node just sees the joints go limp). It writes only torque
off, at 1 Mbps and at the 57600 factory rate, to every ID it is told about (default 1-6), and retries.
"""
import sys

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT, IDS, ADDR_TORQUE = '/dev/dynamixel', range(1, 7), 64


def main():
    port, ph = PortHandler(PORT), PacketHandler(2.0)
    if not port.openPort():
        sys.exit(f'cannot open {PORT}: cut the 12 V supply instead')
    off = set()
    for baud in (1000000, 57600):
        port.setBaudRate(baud)
        for _ in range(3):
            for i in IDS:
                if i not in off and ph.write1ByteTxRx(port, i, ADDR_TORQUE, 0)[0] == COMM_SUCCESS:
                    off.add(i)
    port.closePort()
    missing = sorted(set(IDS) - off)
    print(f'torque OFF on ids {sorted(off)}' + (f'; NO REPLY from {missing}: cut the 12 V supply' if missing else ''))
    sys.exit(1 if missing else 0)


if __name__ == '__main__':
    main()
