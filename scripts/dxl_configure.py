#!/usr/bin/env python3
"""One-time EEPROM setup of the snake's servos: return delay 0 and baud 1 Mbps.

It writes settings only, never torque or goal registers, so nothing moves. It refuses to touch a servo
whose torque is on (the EEPROM is write-locked then anyway). Safe to rerun: it finds the servos at
any common baud and skips values that are already correct.

Usage: python3 scripts/dxl_configure.py [--port /dev/dynamixel]
"""
import argparse
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_BAUD, ADDR_RETURN_DELAY, ADDR_TORQUE = 8, 9, 64
BAUD_INDEX = {9600: 0, 57600: 1, 115200: 2, 1000000: 3, 2000000: 4, 3000000: 5, 4000000: 6}
TARGET_BAUD, TARGET_DELAY = 1000000, 0
EXPECTED_IDS = list(range(1, 7))


def ping_all(port, ph, baud):
    port.setBaudRate(baud)
    ids, res = ph.broadcastPing(port)
    return sorted(ids) if res == COMM_SUCCESS else []


def check(ph, res, err, what):
    if res != COMM_SUCCESS or err:
        raise IOError(f'{what}: {ph.getTxRxResult(res) if res != COMM_SUCCESS else ph.getRxPacketError(err)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/dynamixel')
    a = ap.parse_args()
    port, ph = PortHandler(a.port), PacketHandler(2.0)
    if not port.openPort():
        raise SystemExit(f'cannot open {a.port}')
    try:
        found = {b: ping_all(port, ph, b) for b in BAUD_INDEX}
        found = {b: ids for b, ids in found.items() if ids}
        print('found:', found or 'nothing')
        for baud, ids in found.items():
            port.setBaudRate(baud)
            for i in ids:
                torque, res, err = ph.read1ByteTxRx(port, i, ADDR_TORQUE)
                check(ph, res, err, f'id {i} read torque')
                if torque:
                    raise SystemExit(f'id {i} has torque ON; refusing to write EEPROM')
                delay, res, err = ph.read1ByteTxRx(port, i, ADDR_RETURN_DELAY)
                check(ph, res, err, f'id {i} read return delay')
                if delay != TARGET_DELAY:
                    check(ph, *ph.write1ByteTxRx(port, i, ADDR_RETURN_DELAY, TARGET_DELAY), f'id {i} write delay')
                    print(f'id {i}: return delay {delay * 2}us -> {TARGET_DELAY * 2}us')
                if baud != TARGET_BAUD:
                    # The servo switches baud right after acking, so the reply may be lost; verify by ping below.
                    ph.write1ByteTxRx(port, i, ADDR_BAUD, BAUD_INDEX[TARGET_BAUD])
                    print(f'id {i}: baud {baud} -> {TARGET_BAUD}')
                    time.sleep(0.05)
        ids = ping_all(port, ph, TARGET_BAUD)
        print(f'@ {TARGET_BAUD}: ids {ids}')
        if ids != EXPECTED_IDS:
            raise SystemExit(f'expected {EXPECTED_IDS}; rerun this script (it finds servos at any baud)')
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
